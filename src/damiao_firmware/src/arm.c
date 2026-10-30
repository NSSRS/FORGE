#include "arm.h"
#include <math.h>
#include <string.h>
static bool bounded(float x, float lo, float hi) { return isfinite(x) && x >= lo && x <= hi; }
static bool positive(float x) { return isfinite(x) && x > 0.0f; }
static unsigned quant(float x, float lo, float hi, unsigned bits) {
    return (unsigned)(((double)x-lo)/(hi-lo)*((1u<<bits)-1));
}
static float unpack(unsigned x, float max, unsigned bits) { return (float)x*(2*max)/((1u<<bits)-1)-max; }
bool dm_pack(const dm_range *r, const dm_command *c, uint8_t d[8]) {
    if (!positive(r->p)||!positive(r->v)||!positive(r->t) ||
        !bounded(c->p,-r->p,r->p)||!bounded(c->v,-r->v,r->v)||
        !bounded(c->t,-r->t,r->t)||!bounded(c->kp,0,500)||!bounded(c->kd,0,5)) return false;
    unsigned p=quant(c->p,-r->p,r->p,16), v=quant(c->v,-r->v,r->v,12);
    unsigned kp=quant(c->kp,0,500,12), kd=quant(c->kd,0,5,12), t=quant(c->t,-r->t,r->t,12);
    d[0]=p>>8; d[1]=p; d[2]=v>>4; d[3]=(v<<4)|(kp>>8);
    d[4]=kp; d[5]=kd>>4; d[6]=(kd<<4)|(t>>8); d[7]=t;
    /* Never let a numeric setpoint alias enable/disable/zero/fault-clear. */
    bool special_prefix=true;
    for(unsigned i=0;i<7;i++) special_prefix &= d[i]==0xff;
    return !(special_prefix && d[7]>=0xfb && d[7]<=0xfe);
}
void arm_init(arm_controller *a, arm_send_fn send, void *context) {
    memset(a,0,sizeof(*a)); a->send=send; a->context=context;
}
static bool busy(const arm_controller *a) {
    for(unsigned i=0;i<ARM_COUNT;i++) if(a->joints[i].enabled || a->joints[i].stopping) return true;
    return false;
}
bool arm_configure(arm_controller *a, unsigned i, const arm_config *c) {
    if(i>=ARM_COUNT || busy(a) || c->bus<1 || c->bus>3 || c->id<1 || c->id>15 ||
       c->feedback_id<16 || c->feedback_id>=0x7ff || !isfinite(c->lo)||!isfinite(c->hi)||c->lo>=c->hi ||
       !positive(c->speed)||!positive(c->torque)||!bounded(c->kp,0,500)||!bounded(c->kd,0,5)) return false;
    for(unsigned j=0;j<ARM_COUNT;j++) if(j!=i && a->joints[j].configured && a->joints[j].cfg.bus==c->bus &&
        (a->joints[j].cfg.id==c->id || a->joints[j].cfg.feedback_id==c->feedback_id)) return false;
    memset(&a->joints[i],0,sizeof(a->joints[i])); a->joints[i].cfg=*c; a->joints[i].configured=true;
    return true;
}
bool arm_probe(arm_controller *a,unsigned i) {
    if(i>=ARM_COUNT || busy(a) || !a->joints[i].configured) return false;
    arm_joint *j=&a->joints[i]; j->registers=0; j->probing=true; j->probe_index=0;
    j->command_seen=false; j->feedback_seen=false; return true;
}
static bool special(arm_controller *a, arm_joint *j, uint8_t code) {
    uint8_t d[8]; memset(d,0xff,8); d[7]=code;
    return a->send(a->context,j->cfg.bus,j->cfg.id,d,8);
}
void arm_stop(arm_controller *a) {
    for(unsigned i=0;i<ARM_COUNT;i++) if(a->joints[i].configured) {
        arm_joint *j=&a->joints[i]; j->enabled=false; j->command_seen=false;
        j->stopping=true; j->probing=false; j->feedback_seen=false;
    }
}
static void fault(arm_controller *a) { a->fault=true; arm_stop(a); }
bool arm_reset(arm_controller *a) {
    if(busy(a)) return false;
    a->fault=false;
    for(unsigned i=0;i<ARM_COUNT;i++) { a->joints[i].registers=0; a->joints[i].command_seen=false; }
    return true;
}
bool arm_set(arm_controller *a,unsigned i,const dm_command *c,uint32_t now) {
    if(i>=ARM_COUNT || a->fault) return false;
    arm_joint *j=&a->joints[i]; uint8_t d[8];
    if(!j->configured || j->registers!=31 || j->stopping ||
       !bounded(c->p,j->cfg.lo,j->cfg.hi)||!bounded(c->v,-j->cfg.speed,j->cfg.speed)||
       !bounded(c->t,-j->cfg.torque,j->cfg.torque)||!bounded(c->kp,0,j->cfg.kp)||!bounded(c->kd,0,j->cfg.kd)||
       !dm_pack(&j->range,c,d)) return false;
    j->cmd=*c; j->command_at=now; j->command_seen=true; return true;
}
bool arm_enable(arm_controller *a,unsigned i,uint32_t now) {
    if(i>=ARM_COUNT || a->fault) return false;
    /* Keep register polling out of the loaded control bus. */
    for(unsigned k=0;k<ARM_COUNT;k++) if(a->joints[k].probing) return false;
    arm_joint *j=&a->joints[i];
    if(!j->configured || j->enabled || j->stopping || j->registers!=31 || !j->command_seen ||
       now-j->command_at>=ARM_LEASE_MS || !j->feedback_seen || now-j->feedback_at>=ARM_FEEDBACK_MS ||
       j->status!=0 || fabsf(j->cmd.p-j->p)>0.05f || j->cmd.v!=0 || j->cmd.t!=0) return false;
    /* Preload a bounded command while disabled before enabling. */
    uint8_t d[8]; if(!dm_pack(&j->range,&j->cmd,d)) return false;
    if(!a->send(a->context,j->cfg.bus,j->cfg.id,d,8)||!special(a,j,0xfc)) { fault(a); return false; }
    j->enabled=true; j->enabled_at=now; return true;
}
void arm_tick(arm_controller *a,uint32_t now) {
    if(now-a->last_tick<ARM_CONTROL_PERIOD_MS) return;
    a->last_tick=now;
    for(unsigned i=0;i<ARM_COUNT;i++) {
        arm_joint *j=&a->joints[i]; if(!j->configured) continue;
        if(j->stopping) { (void)special(a,j,0xfd); continue; }
        if(j->probing) {
            static const uint8_t regs[]={21,22,23,10,9};
            uint8_t d[4]={(uint8_t)j->cfg.id,0,0x33,regs[j->probe_index]};
            if(!a->send(a->context,j->cfg.bus,0x7ff,d,4)) { fault(a); return; }
            j->probe_index=(j->probe_index+1)%5;
            if(j->registers==31) j->probing=false;
        }
        if(!j->enabled) continue;
        if(!j->command_seen || now-j->command_at>=ARM_LEASE_MS || !j->feedback_seen ||
           now-j->feedback_at>=ARM_FEEDBACK_MS || (now-j->enabled_at>=50 && j->status!=1)) { fault(a); return; }
        uint8_t d[8];
        if(!dm_pack(&j->range,&j->cmd,d)||!a->send(a->context,j->cfg.bus,j->cfg.id,d,8)) { fault(a); return; }
    }
}
void arm_receive(arm_controller *a,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t len,uint32_t now) {
    if(len!=8) return;
    for(unsigned i=0;i<ARM_COUNT;i++) {
        arm_joint *j=&a->joints[i]; if(!j->configured || j->cfg.bus!=bus || j->cfg.feedback_id!=id) continue;
        if(j->probing && d[0]==j->cfg.id && d[1]==0 && d[2]==0x33) {
            float f; uint32_t u=(uint32_t)d[4]|(uint32_t)d[5]<<8|(uint32_t)d[6]<<16|(uint32_t)d[7]<<24;
            memcpy(&f,&u,4);
            switch(d[3]) {
            case 21: if(positive(f) && f<=1000 && j->cfg.lo>=-f && j->cfg.hi<=f) { j->range.p=f; j->registers|=1; } break;
            case 22: if(positive(f) && f<=10000 && j->cfg.speed<=f) { j->range.v=f; j->registers|=2; } break;
            case 23: if(positive(f) && f<=10000 && j->cfg.torque<=f) { j->range.t=f; j->registers|=4; } break;
            case 10: if(u==1) j->registers|=8; break; /* vendor CTRL_MODE=1: MIT */
            case 9: if(u>0 && u<=3000) j->registers|=16; break; /* 50 us/count: at most 150 ms */
            default: break;
            }
            return;
        }
        if((d[0]&15)!=j->cfg.id) return;
        /* Register replies must never be treated as motion feedback. */
        if(d[0]==j->cfg.id && d[1]==0 && (d[2]==0x33||d[2]==0x55||d[2]==0xaa)) return;
        j->status=d[0]>>4; j->mos=d[6]; j->rotor=d[7];
        if(j->status>=8) { if(j->enabled) fault(a); return; }
        if(j->status>1) return;
        if(j->stopping && j->status==0) j->stopping=false;
        if((j->registers&7)!=7) return;
        j->p=unpack((unsigned)d[1]<<8|d[2],j->range.p,16);
        j->v=unpack((unsigned)d[3]<<4|d[4]>>4,j->range.v,12);
        j->t=unpack((unsigned)(d[4]&15)<<8|d[5],j->range.t,12);
        j->feedback_at=now; j->feedback_seen=true;
        if(j->enabled && (!bounded(j->p,j->cfg.lo,j->cfg.hi)||fabsf(j->v)>j->cfg.speed*1.2f||
            fabsf(j->t)>j->cfg.torque*1.2f||j->mos>=80||j->rotor>=80)) fault(a);
        return;
    }
}
