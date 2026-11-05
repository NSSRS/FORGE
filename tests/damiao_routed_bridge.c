/* Independent six-motor plant for routed bench tests. Never opens CAN/USB hardware. */
#include "wire.h"
#include <math.h>
#include <string.h>
typedef struct {
    unsigned bus,id,rx,mode,timeout,status,commands,mode_writes;
    unsigned coasting_replies, disables, saves, stored_id, stored_rx;
    uint32_t save_at; bool saving;
    bool ignore_ids, drop_save;
    float pmax,vmax,tmax,p,target,speed,v,torque;
    bool present, ignore_mode, stalled;
} motor;
static motor motors[6];
static arm_controller a;
static arm_wire w;
static uint32_t now;
static unsigned invalid;
static unsigned quant(float v,float range,unsigned bits) {
    return (unsigned)((v+range)/(2*range)*((1u<<bits)-1));
}
static bool tx(void *ctx,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t n) {
    (void)ctx;
    for(unsigned i=0;i<6;i++) {
        motor *m=&motors[i]; if(!m->present || bus!=m->bus) continue;
        if(m->saving && now-m->save_at<30u) invalid++;
        uint8_t r[8]={0};
        if(id==0x7ff && d[0]==m->id && d[1]==0) {
            memcpy(r,d,4); uint32_t value=0;
            if(d[2]==0x55 && n==8) {
                memcpy(&value,d+4,4);
                if(d[3]==7) { if(!m->ignore_ids) m->rx=value; }
                else if(d[3]==8) { if(!m->ignore_ids) m->id=value; }
                else if(d[3]==9) m->timeout=value;
                else if(d[3]==10) { m->mode_writes++; if(!m->ignore_mode) m->mode=value; }
                else invalid++;
            } else if(d[2]==0x33) {
                if(d[3]==7) value=m->rx;
                else if(d[3]==8) value=m->id;
                else if(d[3]==9) value=m->timeout;
                else if(d[3]==10) value=m->mode;
                else { float f=d[3]==21?m->pmax:d[3]==22?m->vmax:d[3]==23?m->tmax:4.f; memcpy(&value,&f,4); }
            } else if(d[2]==0xaa && d[3]==1 && n==4) {
                if(m->status!=0) { invalid++; continue; }
                m->saves++; m->stored_id=m->id; m->stored_rx=m->rx;
                m->saving=true; m->save_at=now;
                if(!m->drop_save) arm_receive(&a,bus,m->rx,r,4,now);
                continue;
            } else continue;
            memcpy(r+4,&value,4);
        } else if((id==m->id || id==m->id+0x100) && n==8) {
            if(d[0]==255 && d[1]==255 && d[7]==0xfc) m->status=1;
            else if(d[0]==255 && d[1]==255 && d[7]==0xfd) {
                m->status=0; m->disables++;
                m->v=m->coasting_replies ? .1f : 0;
                if(m->coasting_replies) m->coasting_replies--;
            }
            else if(id==m->id+0x100) {
                m->commands++; memcpy(&m->target,d,4); memcpy(&m->speed,d+4,4);
                if(m->mode!=2 || !isfinite(m->target) || !isfinite(m->speed) || m->speed<=0 || m->speed>m->vmax) invalid++;
            } else invalid++;
            unsigned p=quant(m->p,m->pmax,16),v=quant(m->v,m->vmax,12),t=quant(m->torque,m->tmax,12);
            r[0]=m->id|(m->status<<4); r[1]=p>>8; r[2]=p; r[3]=v>>4;
            r[4]=(v<<4)|(t>>8); r[5]=t; r[6]=25; r[7]=25;
        } else continue;
        arm_receive(&a,bus,m->rx,r,8,now);
    }
    return true;
}
void fake_init(void) {
    memset(motors,0,sizeof(motors)); now=0; invalid=0;
    for(unsigned i=0;i<6;i++) {
        motor *m=&motors[i]; m->bus=i?2:1; m->id=i+1; m->rx=17+i;
        m->pmax=12.5f; m->vmax=i==0?45.f:i<3?10.f:30.f;
        m->tmax=i==0?54.f:i<3?28.f:10.f; m->mode=1; m->present=true;
        m->stored_id=m->id; m->stored_rx=m->rx;
    }
    arm_init(&a,tx,0); wire_init(&w,&a);
}
void fake_route(unsigned i,unsigned bus,unsigned id,unsigned rx) { motors[i].bus=bus; motors[i].id=id; motors[i].rx=rx; }
void fake_advance(unsigned ms) {
    for(unsigned t=0;t<ms;t++) {
        now++;
        for(unsigned i=0;i<6;i++) {
            motor *m=&motors[i];
            if(m->status==1 && m->mode==2 && !m->stalled) {
                float step=m->speed*.001f, diff=m->target-m->p;
                float move=diff>step?step:diff< -step?-step:diff;
                m->p+=move; m->v=move*1000;
            }
        }
        arm_tick(&a,now);
    }
}
int fake_exchange(const uint8_t in[64],uint8_t out[64]) {
    int result=0; for(unsigned i=0;i<64;i++) result|=wire_byte(&w,in[i],now,out);
    fake_advance(5); return result;
}
unsigned fake_now(void) { return now; }
unsigned fake_mode(unsigned i) { return motors[i].mode; }
unsigned fake_commands(unsigned i) { return motors[i].commands; }
unsigned fake_status(unsigned i) { return motors[i].status; }
unsigned fake_invalid(void) { return invalid; }
void fake_missing(unsigned i) { motors[i].present=false; }
void fake_stall(unsigned i) { motors[i].stalled=true; }
void fake_ignore_mode(unsigned i) { motors[i].ignore_mode=true; }
void fake_torque(unsigned i,float torque) { motors[i].torque=torque; }
void fake_speed(unsigned i,float speed) { motors[i].stalled=true; motors[i].v=speed; }
void fake_coast(unsigned i,unsigned replies) { motors[i].coasting_replies=replies; }
unsigned fake_disables(unsigned i) { return motors[i].disables; }

unsigned fake_id(unsigned i) { return motors[i].id; }
unsigned fake_rx(unsigned i) { return motors[i].rx; }
unsigned fake_saves(unsigned i) { return motors[i].saves; }
unsigned fake_timeout(unsigned i) { return motors[i].timeout; }
void fake_ignore_ids(unsigned i) { motors[i].ignore_ids=true; }
void fake_drop_save(unsigned i) { motors[i].drop_save=true; }
void fake_power_cycle(void) {
    for(unsigned i=0;i<6;i++) {
        motors[i].id=motors[i].stored_id; motors[i].rx=motors[i].stored_rx;
        motors[i].status=0;
    }
    arm_init(&a,tx,0); wire_init(&w,&a);
}
