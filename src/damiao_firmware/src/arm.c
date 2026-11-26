#include "arm.h"
#include <math.h>
#include <string.h>
/* Counts only completed sends, not controller retries/arbitration wire time. */
static bool tracked_send(arm_controller *a,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t n) {
    bool ok=a->send(a->context,bus,id,d,n);
    if(bus>=1 && bus<=3) {
        if(ok) a->health[bus-1].tx_frames++;
        else a->health[bus-1].tx_failed++;
    }
    return ok;
}
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
bool dm_pack_posvel(float p,float v,uint8_t d[8]) {
    /* arm_set enforces the configured speed envelope; no fixed bench-speed cap. */
    if(!bounded(p,-12.5f,12.5f)||!isfinite(v)||v<0.001f) return false;
    uint32_t words[2]; memcpy(&words[0],&p,4); memcpy(&words[1],&v,4);
    for(unsigned k=0;k<8;k++) d[k]=(uint8_t)(words[k/4]>>(8*(k%4)));
    return true;
}
uint8_t arm_control_mode(const arm_controller *a, const arm_joint *j) {
    bool position_velocity = a->routed_bench ? j->bench_mode == 2 : a->bench_posvel;
    return position_velocity ? 2 : 1;
}
static bool posvel(const arm_controller *a,const arm_joint *j) {
    return arm_control_mode(a, j) == 2;
}
static bool pack_command(const arm_controller *a,const arm_joint *j,const dm_command *c,uint8_t d[8]) {
    if(posvel(a,j)) return c->kp==0 && c->kd==0 && c->t==0 && dm_pack_posvel(c->p,c->v,d);
    return dm_pack(&j->range,c,d);
}
uint16_t arm_setpoint_id(const arm_controller *a, const arm_joint *j) {
    return j->cfg.id+(posvel(a,j)?0x100:0);
}
void arm_init(arm_controller *a, arm_send_fn send, void *context) {
    memset(a,0,sizeof(*a)); a->send=send; a->context=context;
}
static bool busy(const arm_controller *a) {
    for(unsigned i=0;i<ARM_COUNT;i++) if(a->joints[i].enabled || a->joints[i].stopping) return true;
    return false;
}
/* Mode writes and post-configuration reads require a fresh disabled ACK. */
static bool bench_disabled(arm_controller *a,uint32_t now) {
    arm_joint *j=&a->joints[0];
    if(a->routed_bench || !a->bench || busy(a) || !j->configured || j->probing || !j->feedback_seen ||
       now-j->feedback_at>=ARM_FEEDBACK_MS || j->status!=0 || fabsf(j->v)>0.03f) return false;
    for(unsigned i=1;i<ARM_COUNT;i++) if(a->joints[i].configured) return false;
    return true;
}
bool arm_bench_mode(arm_controller *a,uint8_t mode,uint32_t now) {
    if(a->fault || (mode!=1 && mode!=2) || a->diagnostic.state==1 || !bench_disabled(a,now)) return false;
    a->joints[0].registers &= ~8u; a->joints[0].command_seen=false;
    a->diagnostic=(arm_diagnostic){.state=1,.bus=2,.id=1,.reg=10,.opcode=0x55,.started=now};
    uint8_t d[8]={1,0,0x55,10,mode,0,0,0};
    if(!tracked_send(a,2,0x7ff,d,8)) a->diagnostic.state=4;
    return true;
}
/* Routed bench uses explicit per-slot identity/mode; no register traffic while active. */
static bool routed_disabled(arm_controller *a,unsigned i,uint32_t now) {
    if(!a->routed_bench || i>=ARM_COUNT || busy(a)) return false;
    for(unsigned k=0;k<ARM_COUNT;k++) if(a->joints[k].probing) return false;
    arm_joint *j=&a->joints[i];
    return j->configured && j->feedback_seen && now-j->feedback_at<ARM_FEEDBACK_MS &&
        j->status==0 && fabsf(j->v)<=0.03f;
}
arm_request_result arm_prepare_routed_result(arm_controller *a, unsigned i, const arm_config *c,
                                              uint8_t model, uint8_t mode) {
    /* Rated torque ceilings from the P-series manuals: 20 / 12 / 3.5 Nm.
       8009P speed uses 24 V; 4340P retains its 1.5 rad/s commissioning cap.
       These are configuration ceilings, not current clamps or permission to
       apply rated torque to an assembled linkage. Profiles may choose less. */
    const bool manual_travel=(mode & ARM_ROUTED_MANUAL_TRAVEL)!=0;
    const bool position_hold=(mode & ARM_ROUTED_POSITION_HOLD)!=0;
    const bool group_hold=(mode & ARM_ROUTED_GROUP_HOLD)!=0;
    mode &= (uint8_t)~(ARM_ROUTED_MANUAL_TRAVEL | ARM_ROUTED_POSITION_HOLD | ARM_ROUTED_GROUP_HOLD);
    if ((manual_travel && mode != 2) || (position_hold && !manual_travel) ||
        (group_hold && !position_hold)) return ARM_REJECT_INVALID_ARGUMENT;
    const float max_speed=model==1?10.4719755f:model==3?12.5663706f:1.5f;
    /* Operator-authorized gun-load trial, not a new continuous motor rating.
       The explicit configured trip remains instantaneous; no overload timer. */
    const bool load_trial=i>=1 && i<ARM_COUNT && model==(i<3?2:3) &&
        c->bus==2 && c->id==i+1 && c->feedback_id==i+17;
    const float max_torque=model==1?20.0f:model==3?(load_trial?10.0f:3.5f):(load_trial?30.0f:12.0f);
    if (i >= ARM_COUNT) return ARM_REJECT_INVALID_ARGUMENT;
    if (a->id_commissioning || a->range_commissioning) return ARM_REJECT_INVALID_STATE;
    if (a->fault) return ARM_REJECT_FAULT_LATCHED;
    if (busy(a) || a->diagnostic.state == 1 || (a->bench && !a->routed_bench))
        return ARM_REJECT_INVALID_STATE;
    if (model < 1 || model > 3 || (mode != 1 && mode != 2))
        return ARM_REJECT_INVALID_ARGUMENT;
    if (model == 1 ? c->bus != 1 : c->bus == 1) return ARM_REJECT_INVALID_ROUTE;
    if (!isfinite(c->lo) || !isfinite(c->hi) || !isfinite(c->speed) ||
        !isfinite(c->torque) || !isfinite(c->kp) || !isfinite(c->kd))
        return ARM_REJECT_INVALID_ARGUMENT;
    if (!bounded(c->lo, -12.5f, 12.5f) || !bounded(c->hi, -12.5f, 12.5f) ||
        !bounded(c->speed, 0.03f, max_speed) || !bounded(c->torque, 0.01f, max_torque) ||
        c->kp != 0 || c->kd != 0)
        return ARM_REJECT_LIMIT_EXCEEDED;

    for (unsigned k = 0; k < ARM_COUNT; k++) {
        const arm_joint *joint = &a->joints[k];
        if (!joint->configured) continue;
        if (!a->routed_bench || joint->probing) return ARM_REJECT_INVALID_STATE;
        if (k != i && c->bus == 1 && joint->cfg.bus == 1) return ARM_REJECT_INVALID_ROUTE;
        /* A routed session can change bounds and mode within the same route. */
        if (k == i && (joint->cfg.bus != c->bus || joint->cfg.id != c->id ||
                      joint->cfg.feedback_id != c->feedback_id || joint->model != model))
            return ARM_REJECT_INVALID_ROUTE;
    }

    bool old_bench = a->bench, old_routed = a->routed_bench;
    a->bench = true;
    a->routed_bench = true;
    arm_request_result result = arm_configure_result(a, i, c);
    if (result != ARM_REQUEST_OK) {
        a->bench = old_bench;
        a->routed_bench = old_routed;
        return result;
    }
    a->joints[i].bench_mode = mode;
    a->joints[i].model = model;
    a->joints[i].manual_travel = manual_travel;
    a->joints[i].position_hold = position_hold;
    a->joints[i].group_hold = group_hold;
    return ARM_REQUEST_OK;
}
bool arm_prepare_routed(arm_controller *a, unsigned i, const arm_config *c,
                        uint8_t model, uint8_t mode) {
    return arm_prepare_routed_result(a, i, c, model, mode) == ARM_REQUEST_OK;
}
bool arm_routed_write(arm_controller *a,unsigned i,uint8_t reg,uint32_t value,uint32_t now) {
    /* Even after a fault, allow restoration only after acknowledged stationary disable. */
    if(a->id_commissioning || a->range_commissioning || a->diagnostic.state==1 || !routed_disabled(a,i,now) ||
       !((reg==9 && value==2000) || (reg==10 && (value==1 || value==2)))) return false;
    arm_joint *j=&a->joints[i];
    j->registers &= ~(reg==9?16u:8u); j->command_seen=false;
    a->diagnostic=(arm_diagnostic){.state=1,.bus=j->cfg.bus,.id=j->cfg.id,.reg=reg,.opcode=0x55,.started=now};
    uint8_t d[8]={(uint8_t)j->cfg.id,0,0x55,reg,0,0,0,0};
    for(unsigned k=0;k<4;k++) d[4+k]=(uint8_t)(value>>(8*k));
    if(!tracked_send(a,j->cfg.bus,0x7ff,d,8)) a->diagnostic.state=4;
    return true;
}
/* Explicit ID-only session: single configured slot, no motion until MCU reset. */
bool arm_id_operation(arm_controller *a,uint8_t op,uint32_t value,uint32_t now) {
    if(a->fault || a->range_commissioning || a->save_quiet || a->diagnostic.state==1 || !routed_disabled(a,0,now)) return false;
    for(unsigned k=1;k<ARM_COUNT;k++) if(a->joints[k].configured) return false;
    if(op==0 && value==0 && !a->id_commissioning) {
        a->id_commissioning=true; a->id_verified=0; return true;
    }
    if(!a->id_commissioning) return false;
    arm_joint *j=&a->joints[0];
    if(!((op==7 && value>=17 && value<=22) || (op==8 && value>=1 && value<=6) ||
         (op==0xaa && value==0 && a->id_verified==3 && j->cfg.id<=6 &&
          j->cfg.feedback_id==16+j->cfg.id))) return false;
    uint16_t old_id=j->cfg.id;
    uint8_t opcode=op==0xaa?0xaa:0x55, reg=op==0xaa?1:op;
    a->diagnostic=(arm_diagnostic){.state=1,.bus=j->cfg.bus,.id=old_id,
        .reg=reg,.opcode=opcode,.started=now};
    uint8_t d[8]={(uint8_t)old_id,0,opcode,reg,0,0,0,0};
    for(unsigned k=0;k<4;k++) d[4+k]=(uint8_t)(value>>(8*k));
    if(op==7) j->cfg.feedback_id=value;
    if(op==8) j->cfg.id=value;
    if(op!=0xaa) a->id_verified=0;
    j->command_seen=false; j->probing=false;
    if(op==0xaa) {
        /* Disabled before save. Do not emit disable/probe traffic while the
           motor writes flash (manual: up to 30 ms), even if the host polls. */
        a->save_quiet=true; a->save_started=now;
    } else { j->feedback_seen=false; j->stopping=true; }
    if(!tracked_send(a,j->cfg.bus,0x7ff,d,op==0xaa?4:8)) {
        a->diagnostic.state=4; arm_fault(a,5,0,now);
    }
    return true;
}
static float setup_tmax(unsigned slot) { return slot<3?40.0f:12.5f; }
static float original_tmax(unsigned slot) { return slot<3?28.0f:10.0f; }
static bool range_setup(arm_controller *a,unsigned slot,uint8_t op,uint32_t now) {
    /* Only the complete known rig, freshly disabled and stationary. No arbitrary
       register/value/route writes. Motion stays locked even after fault reset. */
    if(slot<1 || slot>=ARM_COUNT || a->fault || a->id_commissioning || a->save_quiet || a->diagnostic.state==1 || op>1) return false;
    for(unsigned k=0;k<ARM_COUNT;k++) {
        arm_joint *j=&a->joints[k];
        if(!routed_disabled(a,k,now) || (j->registers & 15u)!=15u ||
           j->cfg.bus!=(k==0?1:2) || j->cfg.id!=k+1 || j->cfg.feedback_id!=k+17 ||
           j->model!=(k==0?1:k<3?2:3)) return false;
    }
    arm_joint *j=&a->joints[slot];
    const uint8_t bit=(uint8_t)(1u<<slot);
    if(j->cfg.torque>(slot<3?12.0f:3.5f)) return false;
    if(op==0) {
        if((a->range_written_mask & bit) || a->range_written_mask!=a->range_saved_mask ||
           j->range.t!=original_tmax(slot)) return false;
        a->range_commissioning=true; a->range_setup_slot=(uint8_t)slot;
        a->range_written_mask|=bit; a->range_verified=false; a->range_save_attempted=false;
        j->registers &= ~4u;
        for(unsigned k=0;k<ARM_COUNT;k++) a->joints[k].command_seen=false;
    } else {
        if(!a->range_commissioning || a->range_setup_slot!=slot || !a->range_verified || a->range_save_attempted) return false;
        a->range_save_attempted=true;
        a->range_save_attempted_mask|=bit;
        a->save_quiet=true; a->save_started=now;
    }
    uint8_t opcode=op==0?0x55:0xaa, reg=op==0?23:1;
    a->diagnostic=(arm_diagnostic){.state=1,.bus=2,.id=j->cfg.id,.reg=reg,.opcode=opcode,.started=now};
    /* Only the fixed model-specific TMAX is writable; save is four bytes. */
    float target=setup_tmax(slot); uint32_t raw; memcpy(&raw,&target,4);
    uint8_t d[8]={(uint8_t)j->cfg.id,0,opcode,reg,0,0,0,0};
    for(unsigned k=0;k<4;k++) d[4+k]=(uint8_t)(raw>>(8*k));
    if(!tracked_send(a,2,0x7ff,d,op==0?8:4)) {
        a->diagnostic.state=4; arm_fault(a,5,slot,now);
    }
    return true;
}
bool arm_a2_range_setup(arm_controller *a,uint8_t op,uint32_t now) { return range_setup(a,1,op,now); }
bool arm_load_range_setup(arm_controller *a,uint8_t joint,uint8_t op,uint32_t now) {
    return joint>=3 && joint<=6 && range_setup(a,joint-1,op,now);
}
bool arm_read_register(arm_controller *a,uint8_t bus,uint16_t id,uint8_t reg,uint32_t now) {
    if(a->save_quiet || (a->fault && !a->routed_bench) || bus<1 || bus>3 || id>=0x7ff || a->diagnostic.state==1) return false;
    bool legacy=reg==7 || reg==8 || reg==9 || reg==10 || reg==21 || reg==22 || reg==23 || reg==31;
    bool control=reg<=6 || reg==14 || (reg>=24 && reg<=28) || reg==30 ||
                 (reg>=32 && reg<=34) || reg==36 || reg==59 || reg==60;
    if(!legacy && !control) return false;
    if(!legacy && (!a->routed_bench || a->id_commissioning)) return false;
    bool configured=false, allowed=false;
    for(unsigned i=0;i<ARM_COUNT;i++) if(a->joints[i].configured) {
        configured=true;
        /* Expanded reads require every retained route freshly disabled and
           stationary. Never discover these settings on an unverified motor. */
        if(!legacy && !routed_disabled(a,i,now)) return false;
        if(!legacy && a->joints[i].cfg.bus==bus && a->joints[i].cfg.id==id &&
           (reg==59 || reg==60) && a->joints[i].model!=2 && a->joints[i].model!=3) return false;
        if(a->joints[i].cfg.bus==bus && a->joints[i].cfg.id==id && routed_disabled(a,i,now)) allowed=true;
    }
    if(!legacy && (!configured || !allowed)) return false;
    if(configured && !allowed && !(bus==2 && id==1 && bench_disabled(a,now))) return false;
    arm_diagnostic *q=&a->diagnostic;
    *q=(arm_diagnostic){.state=1,.bus=bus,.reg=reg,.opcode=0x33,.id=id,.started=now};
    uint8_t d[4]={(uint8_t)id,(uint8_t)(id>>8),0x33,reg};
    if(!tracked_send(a,bus,0x7ff,d,4)) q->state=4;
    return true;
}
/* Single discovered bench motor only; runtime TIMEOUT=2000 (100 ms), never saved. */
bool arm_bench_timeout(arm_controller *a,uint32_t now) {
    if(a->fault || a->diagnostic.state==1) return false;
    for(unsigned i=0;i<ARM_COUNT;i++) if(a->joints[i].configured) return false;
    a->diagnostic=(arm_diagnostic){.state=1,.bus=2,.id=1,.reg=9,.opcode=0x55,.started=now};
    const uint8_t d[8]={1,0,0x55,9,0xd0,0x07,0,0};
    if(!tracked_send(a,2,0x7ff,d,8)) a->diagnostic.state=4;
    return true;
}
bool arm_prepare_bench(arm_controller *a,float lo,float hi) {
    if(a->routed_bench || a->fault || busy(a) || a->diagnostic.state==1 ||
       !bounded(lo,-12.5f,12.5f) || !bounded(hi,-12.5f,12.5f) || lo>=hi) return false;
    for(unsigned i=0;i<ARM_COUNT;i++) if(a->joints[i].configured && !(a->bench && i==0)) return false;
    a->bench=true; a->bench_posvel=false;
    arm_config c={2,1,0,lo,hi,0.3f,0.5f,2.0f,1.0f};
    return arm_configure(a,0,&c);
}
bool arm_prepare_posvel(arm_controller *a,float lo,float hi) {
    if(!arm_prepare_bench(a,lo,hi)) return false;
    a->bench_posvel=true;
    a->joints[0].cfg.torque=3.0f;
    a->joints[0].cfg.kp=0; a->joints[0].cfg.kd=0;
    return true; /* Wide bounds may probe, but arm_enable still forbids them. */
}
arm_request_result arm_configure_result(arm_controller *a, unsigned i, const arm_config *c) {
    if (a->id_commissioning || a->range_commissioning || a->diagnostic.state == 1)
        return ARM_REJECT_INVALID_STATE;
    if (i >= ARM_COUNT) return ARM_REJECT_INVALID_ARGUMENT;
    if (busy(a)) return ARM_REJECT_INVALID_STATE;
    bool allows_zero_feedback = a->routed_bench || (a->bench && i == 0 && c->bus == 2 && c->id == 1);
    if (c->bus < 1 || c->bus > 3 || c->id < 1 || c->id > 15 || c->feedback_id >= 0x7ff ||
        (c->feedback_id < 16 && !(allows_zero_feedback && c->feedback_id == 0)))
        return ARM_REJECT_INVALID_ROUTE;
    if (!isfinite(c->lo) || !isfinite(c->hi) || !isfinite(c->speed) ||
        !isfinite(c->torque) || !isfinite(c->kp) || !isfinite(c->kd))
        return ARM_REJECT_INVALID_ARGUMENT;
    if (c->lo >= c->hi || c->speed <= 0 || c->torque <= 0 ||
        !bounded(c->kp, 0, 500) || !bounded(c->kd, 0, 5))
        return ARM_REJECT_LIMIT_EXCEEDED;
    for (unsigned j = 0; j < ARM_COUNT; j++) {
        const arm_joint *joint = &a->joints[j];
        if (j != i && joint->configured && joint->cfg.bus == c->bus &&
            (joint->cfg.id == c->id || joint->cfg.feedback_id == c->feedback_id))
            return ARM_REJECT_INVALID_ROUTE;
    }
    memset(&a->joints[i], 0, sizeof(a->joints[i]));
    a->joints[i].cfg = *c;
    a->joints[i].configured = true;
    return ARM_REQUEST_OK;
}
bool arm_configure(arm_controller *a, unsigned i, const arm_config *c) {
    return arm_configure_result(a, i, c) == ARM_REQUEST_OK;
}

arm_request_result arm_probe_result(arm_controller *a, unsigned i) {
    if (i >= ARM_COUNT) return ARM_REJECT_INVALID_ARGUMENT;
    if (a->diagnostic.state == 1 || busy(a) || !a->joints[i].configured)
        return ARM_REJECT_INVALID_STATE;
    arm_joint *joint = &a->joints[i];
    joint->registers = 0;
    joint->probing = true;
    joint->probe_index = 0;
    joint->command_seen = false;
    joint->feedback_seen = false;
    return ARM_REQUEST_OK;
}
bool arm_probe(arm_controller *a, unsigned i) {
    return arm_probe_result(a, i) == ARM_REQUEST_OK;
}
static bool special(arm_controller *a, arm_joint *j, uint8_t code) {
    uint8_t d[8]; memset(d,0xff,8); d[7]=code;
    return tracked_send(a,j->cfg.bus,j->cfg.id,d,8);
}
void arm_stop(arm_controller *a) {
    for(unsigned i=0;i<ARM_COUNT;i++) if(a->joints[i].configured) {
        arm_joint *j=&a->joints[i]; j->enabled=false; j->enable_confirmed=false; j->command_seen=false;
        j->smooth_jog=false;
        j->stopping=true; j->probing=false; j->feedback_seen=false;
    }
}
void arm_fault(arm_controller *a,uint8_t reason,unsigned slot,uint32_t now) {
    if(!a->fault) {
        a->fault_reason=reason; a->fault_slot=slot; a->fault_at=now;
        a->fault_command_age=UINT32_MAX; a->fault_feedback_age=UINT32_MAX;
        if(slot<ARM_COUNT) {
            arm_joint *j=&a->joints[slot]; a->fault_status=j->status;
            a->fault_p=j->p; a->fault_v=j->v; a->fault_t=j->t;
            a->fault_target=j->cmd.p; a->fault_velocity=j->cmd.v;
            a->fault_speed_limit=j->cfg.speed; a->fault_torque_limit=j->cfg.torque;
            a->fault_sample_flags=(j->feedback_seen?1u:0u)|(j->command_seen?2u:0u);
            if(j->command_seen) a->fault_command_age=now-j->command_at;
            if(j->feedback_seen) a->fault_feedback_age=now-j->feedback_at;
        }
    }
    a->fault=true; arm_stop(a);
}
bool arm_reset(arm_controller *a) {
    if(busy(a)) return false;
    a->fault=false; a->fault_reason=0; a->fault_status=0; a->fault_slot=0;
    a->fault_at=0; a->fault_command_age=0; a->fault_feedback_age=0;
    a->fault_sample_flags=0;
    for(unsigned i=0;i<ARM_COUNT;i++) { a->joints[i].registers=0; a->joints[i].command_seen=false; }
    return true;
}
static arm_request_result check_command(const arm_controller *a, unsigned i, const dm_command *c) {
    if (i >= ARM_COUNT) return ARM_REJECT_INVALID_ARGUMENT;
    if (a->id_commissioning || a->range_commissioning) return ARM_REJECT_INVALID_STATE;
    if (a->fault) return ARM_REJECT_FAULT_LATCHED;
    const arm_joint *joint = &a->joints[i];
    if (!joint->configured || joint->registers != 31 || joint->stopping)
        return ARM_REJECT_INVALID_STATE;
    if (!isfinite(c->p) || !isfinite(c->v) || !isfinite(c->t) ||
        !isfinite(c->kp) || !isfinite(c->kd))
        return ARM_REJECT_INVALID_ARGUMENT;
    uint8_t data[8];
    if (!bounded(c->p, joint->cfg.lo, joint->cfg.hi) ||
        !bounded(c->v, -joint->cfg.speed, joint->cfg.speed) ||
        !bounded(c->t, -joint->cfg.torque, joint->cfg.torque) ||
        !bounded(c->kp, 0, joint->cfg.kp) || !bounded(c->kd, 0, joint->cfg.kd) ||
        !pack_command(a, joint, c, data))
        return ARM_REJECT_LIMIT_EXCEEDED;
    if (joint->enabled && !joint->enable_confirmed &&
        (c->p != joint->cmd.p || c->v != (posvel(a, joint) ? joint->cmd.v : 0) ||
         c->t != 0 || c->kp != joint->cmd.kp || c->kd != joint->cmd.kd))
        return ARM_REJECT_INVALID_STATE;
    if (posvel(a, joint) && fabsf(c->p - joint->p) > (a->routed_bench ? 3.2f : 1.0f))
        return ARM_REJECT_LIMIT_EXCEEDED;
    return ARM_REQUEST_OK;
}
arm_request_result arm_set_result(arm_controller *a, unsigned i, const dm_command *c, uint32_t now) {
    arm_request_result result = check_command(a, i, c);
    if (result != ARM_REQUEST_OK) return result;
    arm_joint *joint = &a->joints[i];
    joint->cmd = *c;
    joint->command_at = now;
    joint->command_seen = true;
    joint->smooth_jog = false; /* Release/hold bypasses the jog slew. */
    return ARM_REQUEST_OK;
}
bool arm_set(arm_controller *a, unsigned i, const dm_command *c, uint32_t now) {
    return arm_set_result(a, i, c, now) == ARM_REQUEST_OK;
}

static arm_request_result check_smooth(const arm_controller *a, unsigned i, const dm_command *c) {
    if (i >= ARM_COUNT) return ARM_REJECT_INVALID_ARGUMENT;
    const arm_joint *joint = &a->joints[i];
    /* Only explicit, already-confirmed manual mode-2 holds may ramp. */
    if (!a->routed_bench || !joint->position_hold || !joint->manual_travel ||
        joint->bench_mode != 2 || !joint->enabled || !joint->enable_confirmed)
        return ARM_REJECT_INVALID_STATE;
    if (!isfinite(c->v)) return ARM_REJECT_INVALID_ARGUMENT;
    if (!bounded(c->v, 0.001f, 0.17453294f)) return ARM_REJECT_LIMIT_EXCEEDED;
    return ARM_REQUEST_OK;
}
arm_request_result arm_set_smooth_result(arm_controller *a, unsigned i, const dm_command *c, uint32_t now) {
    arm_request_result smooth = check_smooth(a, i, c);
    if (smooth != ARM_REQUEST_OK) return smooth;
    arm_request_result result = arm_set_result(a, i, c, now);
    if (result == ARM_REQUEST_OK) a->joints[i].smooth_jog = true;
    return result;
}
bool arm_set_smooth(arm_controller *a, unsigned i, const dm_command *c, uint32_t now) {
    return arm_set_smooth_result(a, i, c, now) == ARM_REQUEST_OK;
}

arm_request_result arm_set_group_result(arm_controller *a, uint8_t mask, uint8_t smooth_mask,
                                      const dm_command commands[ARM_COUNT], uint32_t now, uint8_t *failed_slot) {
    *failed_slot = ARM_REJECT_NO_SLOT;
    uint8_t configured = 0;
    for (unsigned i = 0; i < ARM_COUNT; i++)
        if (a->joints[i].configured) configured |= 1u << i;
    if (!mask || mask != configured || (smooth_mask & ~mask)) return ARM_REJECT_INVALID_ARGUMENT;
    if (a->fault) return ARM_REJECT_FAULT_LATCHED;
    if (!a->routed_bench || a->id_commissioning || a->range_commissioning)
        return ARM_REJECT_INVALID_STATE;
    /* No writes or CAN sends during validation, including on the last slot. */
    for (unsigned i = 0; i < ARM_COUNT; i++) if (mask & (1u << i)) {
        *failed_slot = i;
        const arm_joint *joint = &a->joints[i];
        if (!joint->group_hold || joint->bench_mode != 2) return ARM_REJECT_INVALID_STATE;
        arm_request_result result = check_command(a, i, &commands[i]);
        if (result != ARM_REQUEST_OK) return result;
        if (!joint->feedback_seen || now - joint->feedback_at >= ARM_FEEDBACK_MS)
            return ARM_REJECT_STALE_FEEDBACK;
        /* USB is serviced before arm_tick: never revive an expired live lease. */
        if (joint->enabled && (!joint->command_seen || now - joint->command_at >= ARM_LEASE_MS))
            return ARM_REJECT_STALE_PRELOAD;
        if (smooth_mask & (1u << i)) {
            result = check_smooth(a, i, &commands[i]);
            if (result != ARM_REQUEST_OK) return result;
        }
    }
    /* forge_poll dispatch and arm_tick share one main loop, never an ISR.
       All targets become visible to the next tick with the same lease time. */
    for (unsigned i = 0; i < ARM_COUNT; i++) if (mask & (1u << i)) {
        arm_joint *joint = &a->joints[i];
        joint->cmd = commands[i];
        joint->command_at = now;
        joint->command_seen = true;
        joint->smooth_jog = (smooth_mask & (1u << i)) != 0;
    }
    *failed_slot = ARM_REJECT_NO_SLOT;
    return ARM_REQUEST_OK;
}

static arm_request_result check_enable(const arm_controller *a, unsigned i, uint32_t now) {
    if (i >= ARM_COUNT) return ARM_REJECT_INVALID_ARGUMENT;
    if (a->id_commissioning || a->range_commissioning) return ARM_REJECT_INVALID_STATE;
    if (a->fault) return ARM_REJECT_FAULT_LATCHED;
    if (a->diagnostic.state == 1) return ARM_REJECT_INVALID_STATE;
    /* Keep register polling out of the loaded control bus. */
    for (unsigned k = 0; k < ARM_COUNT; k++)
        if (a->joints[k].probing) return ARM_REJECT_INVALID_STATE;
    const arm_joint *joint = &a->joints[i];
    /* Single-axis hold cannot be mixed with another active axis. */
    for (unsigned k = 0; k < ARM_COUNT; k++)
        if (k != i && a->joints[k].enabled &&
            (joint->position_hold || a->joints[k].position_hold) &&
            !(joint->group_hold && a->joints[k].group_hold))
            return ARM_REJECT_INVALID_STATE;
    if (!joint->configured || joint->enabled || joint->stopping || joint->registers != 31)
        return ARM_REJECT_INVALID_STATE;
    if (!joint->command_seen || now - joint->command_at >= ARM_LEASE_MS)
        return ARM_REJECT_STALE_PRELOAD;
    if (!joint->feedback_seen || now - joint->feedback_at >= ARM_FEEDBACK_MS)
        return ARM_REJECT_STALE_FEEDBACK;
    if (joint->status != 0) return ARM_REJECT_INVALID_STATE;
    if (fabsf(joint->cmd.p - joint->p) > 0.05f) return ARM_REJECT_LIMIT_EXCEEDED;
    if ((!posvel(a, joint) && joint->cmd.v != 0) || joint->cmd.t != 0)
        return ARM_REJECT_INVALID_STATE;
    /* Wide probing routes cannot enable without explicit manual-travel policy. */
    if (a->bench && (!posvel(a, joint) || (!a->routed_bench && i != 0)))
        return ARM_REJECT_INVALID_STATE;
    if (a->bench && !(a->routed_bench && joint->manual_travel) &&
        joint->cfg.hi - joint->cfg.lo > (a->routed_bench ? 3.25f : 1.1f))
        return ARM_REJECT_LIMIT_EXCEEDED;
    return ARM_REQUEST_OK;
}
static arm_request_result enable_slot(arm_controller *a, unsigned i, uint32_t now) {
    arm_request_result result = check_enable(a, i, now);
    if (result != ARM_REQUEST_OK) return result;
    arm_joint *joint = &a->joints[i];
    /* Preload a bounded command while disabled before enabling. */
    uint8_t data[8];
    if (!pack_command(a, joint, &joint->cmd, data)) return ARM_REJECT_LIMIT_EXCEEDED;
    if (!tracked_send(a, joint->cfg.bus, arm_setpoint_id(a, joint), data, 8)) {
        arm_fault(a, 5, i, now);
        return ARM_REJECT_CAN_TX_FAILED;
    }
    joint->sent_position = joint->cmd.p;
    joint->smooth_jog = false;
    joint->enable_confirmed = false;
    joint->enabled = true;
    joint->enabled_at = now;
    if (!special(a, joint, 0xfc)) {
        arm_fault(a, 5, i, now);
        return ARM_REJECT_CAN_TX_FAILED;
    }
    if (a->fault) return ARM_REJECT_FAULT_LATCHED;
    if (!joint->enabled) return ARM_REJECT_INVALID_STATE;
    if (a->bench) a->bench_enabled_at = now;
    joint->enabled = true;
    joint->enabled_at = now;
    return ARM_REQUEST_OK;
}
arm_request_result arm_enable_result(arm_controller *a, unsigned i, uint32_t now) {
    if (i >= ARM_COUNT) return ARM_REJECT_INVALID_ARGUMENT;
    for (unsigned k = 0; k < ARM_COUNT; k++)
        if (a->joints[k].configured && a->joints[k].group_hold)
            return ARM_REJECT_INVALID_STATE;
    return enable_slot(a, i, now);
}
bool arm_enable(arm_controller *a, unsigned i, uint32_t now) {
    return arm_enable_result(a, i, now) == ARM_REQUEST_OK;
}

arm_request_result arm_enable_group_result(arm_controller *a, uint8_t mask, uint32_t now,
                                            uint8_t *failed_slot) {
    *failed_slot = ARM_REJECT_NO_SLOT;
    if (mask == 0 || (mask & ~63u)) return ARM_REJECT_INVALID_ARGUMENT;
    if (!a->routed_bench) return ARM_REJECT_INVALID_STATE;
    uint8_t configured = 0, group = 0;
    for (unsigned i = 0; i < ARM_COUNT; i++) {
        if (!a->joints[i].configured) continue;
        configured |= 1u << i;
        if (a->joints[i].group_hold) group |= 1u << i;
    }
    /* Validate the whole group before sending the first enable frame. */
    if (group && (group != configured || mask != group)) return ARM_REJECT_INVALID_STATE;
    for (unsigned i = 0; i < ARM_COUNT; i++) {
        if ((mask & (1u << i)) && a->joints[i].position_hold && !a->joints[i].group_hold &&
            (mask & (mask - 1u))) {
            *failed_slot = i;
            return ARM_REJECT_INVALID_STATE;
        }
    }
    for (unsigned i = 0; i < ARM_COUNT; i++) {
        if (!(mask & (1u << i))) continue;
        arm_request_result result = check_enable(a, i, now);
        if (result != ARM_REQUEST_OK) {
            *failed_slot = i;
            return result;
        }
    }
    for (unsigned i = 0; i < ARM_COUNT; i++) {
        if (!(mask & (1u << i))) continue;
        arm_request_result result = enable_slot(a, i, now);
        if (result != ARM_REQUEST_OK) {
            arm_fault(a, 5, i, now);
            *failed_slot = i;
            return result;
        }
    }
    return ARM_REQUEST_OK;
}
bool arm_enable_group(arm_controller *a, uint8_t mask, uint32_t now) {
    uint8_t failed_slot;
    return arm_enable_group_result(a, mask, now, &failed_slot) == ARM_REQUEST_OK;
}
void arm_tick(arm_controller *a,uint32_t now) {
    if(a->save_quiet) {
        if(now-a->save_started<50u) return;
        a->save_quiet=false;
    }
    if(a->diagnostic.state==1 && now-a->diagnostic.started>=100u) a->diagnostic.state=3;
    if(a->bench && !a->routed_bench && a->joints[0].enabled && now-a->bench_enabled_at>=15000u) { arm_fault(a,10,0,now); return; }
    if(now-a->last_tick<ARM_CONTROL_PERIOD_MS) return;
    bool enabled=false;
    for(unsigned k=0;k<ARM_COUNT;k++) enabled |= a->joints[k].enabled;
    if(enabled) {
        uint32_t gap=now-a->last_tick;
        if(gap>a->max_tick_gap_ms) a->max_tick_gap_ms=gap;
        if(gap>ARM_CONTROL_PERIOD_MS) a->late_ticks++;
    }
    a->last_tick=now;
    for(unsigned i=0;i<ARM_COUNT;i++) {
        arm_joint *j=&a->joints[i]; if(!j->configured) continue;
        if(j->stopping) { (void)special(a,j,0xfd); continue; }
        if(j->probing) {
            static const uint8_t regs[]={21,22,23,10,9};
            uint8_t d[4]={(uint8_t)j->cfg.id,0,0x33,regs[j->probe_index]};
            if(!tracked_send(a,j->cfg.bus,0x7ff,d,4)) { arm_fault(a,5,i,now); return; }
            j->probe_index=(j->probe_index+1)%5;
            if(j->registers==31) j->probing=false;
        }
        if(!j->enabled) continue;
        if(a->routed_bench && !j->position_hold && now-j->enabled_at>=15000u) { arm_fault(a,10,i,now); return; }
        if(!j->command_seen || now-j->command_at>=ARM_LEASE_MS) { arm_fault(a,1,i,now); return; }
        if(!j->feedback_seen || now-j->feedback_at>=ARM_FEEDBACK_MS) { arm_fault(a,2,i,now); return; }
        if(!j->enable_confirmed && now-j->enabled_at>=ARM_ENABLE_CONFIRM_MS) { arm_fault(a,3,i,now); return; }
        uint8_t d[8];
        dm_command command=j->cmd;
        if(j->smooth_jog) {
            /* Slew from the last transmitted target, never from noisy feedback.
               No extrapolation beyond the host endpoint, lease renewal or
               catch-up jump after a late tick. CAN rate remains 500 Hz. */
            float step=command.v*(ARM_CONTROL_PERIOD_MS*0.001f);
            float delta=command.p-j->sent_position;
            if(delta>step) command.p=j->sent_position+step;
            else if(delta< -step) command.p=j->sent_position-step;
        }
        if(!pack_command(a,j,&command,d)) { arm_fault(a,4,i,now); return; }
        if(!tracked_send(a,j->cfg.bus,arm_setpoint_id(a,j),d,8)) { arm_fault(a,5,i,now); return; }
        j->sent_position=command.p;
    }
}
void arm_receive(arm_controller *a,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t len,uint32_t now) {
    if(bus>=1 && bus<=3 && len<=8) {
        arm_rx_trace *r=&a->rx_trace[bus-1];
        r->count++; r->at=now; r->id=id; r->len=len;
        memset(r->data,0,8); memcpy(r->data,d,len);
    }
    if(len!=8 && len!=4) return;
    arm_diagnostic *q=&a->diagnostic;
    if(q->state==1 && now-q->started<100u && bus==q->bus && id<=0x7ff &&
       (len==8 || q->opcode==0xaa) &&
       (d[0]==(uint8_t)q->id || (a->id_commissioning && q->opcode==0x55 && q->reg==8 &&
        d[0]==a->joints[0].cfg.id)) && d[1]==(uint8_t)(q->id>>8) && d[2]==q->opcode && d[3]==q->reg) {
        q->feedback_id=id;
        q->value=len==8?((uint32_t)d[4]|(uint32_t)d[5]<<8|(uint32_t)d[6]<<16|(uint32_t)d[7]<<24):0;
        if(a->range_commissioning && q->opcode==0x33 && q->reg==23 &&
           bus==2 && q->id==a->range_setup_slot+1 && id==a->range_setup_slot+17) {
            float value; memcpy(&value,&q->value,4);
            a->range_verified=value==setup_tmax(a->range_setup_slot);
            if(a->range_verified || value==original_tmax(a->range_setup_slot)) {
                a->joints[a->range_setup_slot].range.t=value;
                a->joints[a->range_setup_slot].registers|=4u;
                a->joints[a->range_setup_slot].feedback_seen=false;
            }
        }
        if(a->range_commissioning && a->range_save_attempted && q->opcode==0xaa && q->reg==1 &&
           bus==2 && q->id==a->range_setup_slot+1 && id==a->range_setup_slot+17 && q->value==0)
            a->range_saved_mask|=(uint8_t)(1u<<a->range_setup_slot);
        if(a->id_commissioning && q->opcode==0x33) {
            arm_joint *j=&a->joints[0];
            if(bus==j->cfg.bus && id==j->cfg.feedback_id && q->id==j->cfg.id) {
                if(q->reg==7 && q->value==j->cfg.feedback_id) a->id_verified|=1;
                if(q->reg==8 && q->value==j->cfg.id) a->id_verified|=2;
            }
        }
        q->state=2;
        return;
    }
    if(len!=8) return;
    for(unsigned i=0;i<ARM_COUNT;i++) {
        arm_joint *j=&a->joints[i]; if(!j->configured || j->cfg.bus!=bus || j->cfg.feedback_id!=id) continue;
        if(j->probing && d[0]==j->cfg.id && d[1]==0 && d[2]==0x33) {
            float f; uint32_t u=(uint32_t)d[4]|(uint32_t)d[5]<<8|(uint32_t)d[6]<<16|(uint32_t)d[7]<<24;
            memcpy(&f,&u,4);
            switch(d[3]) {
            case 21: if(positive(f) && f<=1000 && j->cfg.lo>=-f && j->cfg.hi<=f) { j->range.p=f; j->registers|=1; } break;
            case 22: if(positive(f) && f<=10000 && j->cfg.speed<=f) { j->range.v=f; j->registers|=2; } break;
            case 23:
                /* Extended trips must remain observable at feedback saturation. */
                if(positive(f) && f<=10000 && j->cfg.torque<=f &&
                   !(((j->model==2 && j->cfg.torque>12.0f) ||
                      (j->model==3 && j->cfg.torque>3.5f)) && j->cfg.torque>=f)) {
                    j->range.t=f; j->registers|=4;
                }
                break;
            case 10: if(u==(posvel(a,j)?2u:1u)) j->registers|=8; break;
            case 9: if(u>0 && u<=3000) j->registers|=16; break; /* 50 us/count: at most 150 ms */
            default: break;
            }
            return;
        }
        if((d[0]&15)!=j->cfg.id) return;
        /* Register replies must never be treated as motion feedback. */
        if(d[0]==j->cfg.id && d[1]==0 && (d[2]==0x33||d[2]==0x55||d[2]==0xaa)) return;
        j->status=d[0]>>4; j->mos=d[6]; j->rotor=d[7];
        if(j->status>=8) { if(j->enabled) arm_fault(a,6,i,now); return; }
        if(j->status>1) return;
        if(j->enabled && j->status==1) j->enable_confirmed=true;
        else if(j->enabled && j->enable_confirmed && j->status==0) { arm_fault(a,3,i,now); return; }
        if(j->stopping && j->status==0) j->stopping=false;
        if((j->registers&7)!=7) return;
        j->p=unpack((unsigned)d[1]<<8|d[2],j->range.p,16);
        j->v=unpack((unsigned)d[3]<<4|d[4]>>4,j->range.v,12);
        j->t=unpack((unsigned)(d[4]&15)<<8|d[5],j->range.t,12);
        arm_bus_health *h=&a->health[bus-1]; h->feedback_frames++;
        if(j->enabled && j->feedback_seen) {
            uint32_t gap=now-j->feedback_at;
            if(gap>h->max_feedback_gap_ms) h->max_feedback_gap_ms=gap;
        }
        j->feedback_at=now; j->feedback_seen=true;
        if(j->enabled) {
            if(!bounded(j->p,j->cfg.lo,j->cfg.hi)) arm_fault(a,7,i,now);
            else if(fabsf(j->v)>j->cfg.speed*(a->routed_bench?1.0f:1.2f)) arm_fault(a,8,i,now);
            else if(fabsf(j->t)>j->cfg.torque*(posvel(a,j)?1.0f:1.2f)) arm_fault(a,9,i,now);
            else if(j->mos>=80||j->rotor>=80) arm_fault(a,11,i,now);
        }
        return;
    }
}
