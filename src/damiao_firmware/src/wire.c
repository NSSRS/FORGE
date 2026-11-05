#include "wire.h"
#include <string.h>
static uint32_t u32(const uint8_t *d) { return (uint32_t)d[0]|(uint32_t)d[1]<<8|(uint32_t)d[2]<<16|(uint32_t)d[3]<<24; }
static uint16_t u16(const uint8_t *d) { return (uint16_t)(d[0]|d[1]<<8); }
static float f32(const uint8_t *d) { uint32_t u=u32(d); float f; memcpy(&f,&u,4); return f; }
static void put32(uint8_t *d,uint32_t u) { for(unsigned i=0;i<4;i++) d[i]=(uint8_t)(u>>(8*i)); }
static void putf(uint8_t *d,float f) { uint32_t u; memcpy(&u,&f,4); put32(d,u); }
uint32_t wire_crc(const uint8_t *d,size_t n) {
    uint32_t crc=0xffffffff;
    for(size_t i=0;i<n;i++) { crc^=d[i]; for(unsigned b=0;b<8;b++) crc=(crc>>1)^((crc&1)?0xedb88320:0); }
    return ~crc;
}
void wire_init(arm_wire *w,arm_controller *a) { memset(w,0,sizeof(*w)); w->arm=a; }
static bool active(arm_controller *a) { for(unsigned i=0;i<6;i++) if(a->joints[i].enabled) return true; return false; }
static bool dispatch(arm_wire *w,uint32_t now,uint8_t *out) {
    uint8_t type=w->buffer[3], *p=w->buffer+8; unsigned i=p[0];
    uint32_t seq=u32(w->buffer+4); arm_controller *a=w->arm;
    memset(out,0,64); memcpy(out,w->buffer,8); out[3]|=0x80; bool ok=false;
    if(type==1 && !active(a)) { w->session=true; w->sequence=seq; memcpy(out+9,"FORGE-MC02/1",12); put32(out+21,1000u/ARM_CONTROL_PERIOD_MS); put32(out+25,ARM_CAPABILITIES); ok=true; }
    else if(type==6) { arm_stop(a); ok=true; } /* stop always accepted, including replay */
    else if(w->session && seq>w->sequence) {
        w->sequence=seq;
        if(type==2) {
            arm_config c={.bus=p[1],.id=u16(p+2),.feedback_id=u16(p+4),.lo=f32(p+6),.hi=f32(p+10),
                .speed=f32(p+14),.torque=f32(p+18),.kp=f32(p+22),.kd=f32(p+26)};
            ok=!a->bench && arm_configure(a,i,&c);
        } else if(type==3) ok=arm_probe(a,i);
        else if(type==4) { dm_command c={f32(p+1),f32(p+5),f32(p+9),f32(p+13),f32(p+17)}; ok=arm_set(a,i,&c,now); }
        else if(type==5) ok=arm_enable(a,i,now);
        else if(type==7 && i<6) {
            arm_joint *j=&a->joints[i]; ok=true;
            out[9]=a->fault; out[10]=j->enabled; out[11]=j->registers; out[12]=j->status;
            out[13]=j->mos; out[14]=j->rotor; out[15]=j->stopping;
            put32(out+16,j->feedback_seen?now-j->feedback_at:UINT32_MAX);
            putf(out+20,j->p); putf(out+24,j->v); putf(out+28,j->t);
            putf(out+32,j->range.p); putf(out+36,j->range.v); putf(out+40,j->range.t);
            out[44]=a->fault_reason; out[45]=a->fault_status; out[46]=a->fault_slot; out[47]=j->enable_confirmed?1:0;
            put32(out+48,a->fault_command_age); put32(out+52,a->fault_feedback_age); put32(out+56,a->fault_at);
        } else if(type==8) ok=arm_reset(a);
        else if(type==9) ok=arm_read_register(a,p[0],u16(p+1),p[3],now);
        else if(type==11) ok=arm_bench_timeout(a,now);
        else if(type==12) ok=arm_prepare_bench(a,f32(p),f32(p+4));
        /* Type 13 and capability bits 4..6 retired with experimental MIT bench motion. */
        else if(type==14) ok=arm_prepare_posvel(a,f32(p),f32(p+4));
        else if(type==15) ok=arm_bench_mode(a,p[0],now);
        else if(type==16) {
            arm_config c={.bus=p[1],.id=u16(p+2),.feedback_id=u16(p+4),
                .lo=f32(p+8),.hi=f32(p+12),.speed=f32(p+16),.torque=f32(p+20),.kp=0,.kd=0};
            ok=arm_prepare_routed(a,i,&c,p[6],p[7]);
        } else if(type==17) ok=arm_routed_write(a,i,p[1],u32(p+2),now);
        else if(type==18) ok=arm_enable_group(a,p[0],now);
        else if(type==20) ok=arm_id_operation(a,p[0],u32(p+1),now);
        else if(type==22 && p[0]<3) {
            /* Capturing/reading a snapshot generates no CAN traffic. */
            unsigned page=p[0];
            if(page==0) {
                if(++w->snapshot_generation==0) w->snapshot_generation=1;
                for(unsigned pg=0;pg<3;pg++) {
                    uint8_t *b=w->snapshot[pg]; memset(b,0,51);
                    put32(b,w->snapshot_generation); put32(b+4,now);
                    b[8]=a->fault; b[9]=a->fault_reason; b[10]=a->fault_slot;
                    for(unsigned k=0;k<2;k++) {
                        arm_joint *j=&a->joints[pg*2+k]; uint8_t *v=b+11+k*18;
                        putf(v,j->p); putf(v+4,j->v); putf(v+8,j->t);
                        uint32_t age=j->feedback_seen?now-j->feedback_at:UINT32_MAX;
                        uint16_t age16=age<65535?age:65535;
                        v[12]=age16; v[13]=age16>>8;
                        v[14]=(j->configured?1:0)|(j->enabled?2:0)|(j->feedback_seen?4:0)|
                              (j->stopping?8:0)|(j->enable_confirmed?16:0);
                        v[15]=j->status; v[16]=j->mos; v[17]=j->rotor;
                    }
                }
            }
            if(page==0 || (w->snapshot_generation && u32(p+1)==w->snapshot_generation)) {
                memcpy(out+9,w->snapshot[page],51); ok=true;
            }
        }
        else if(type==23 && p[0]>=1 && p[0]<=3) {
            unsigned bus=p[0]-1; arm_bus_health *h=&a->health[bus];
            uint32_t values[]={now,h->tx_frames,h->tx_failed,a->rx_trace[bus].count,
                h->feedback_frames,h->max_feedback_gap_ms,a->max_tick_gap_ms,a->late_ticks,
                h->bus_off_events,h->status_flags,h->tx_error_count,h->rx_error_count};
            for(unsigned k=0;k<12;k++) put32(out+9+k*4,values[k]);
            ok=true;
        }
        else if(type==24) {
            uint32_t configured=0;
            for(unsigned k=0;k<ARM_COUNT;k++) if(a->joints[k].configured) configured |= 1u<<k;
            uint32_t values[]={ARM_BASE_API_VERSION,ARM_CAPABILITIES,ARM_CONTROL_PERIOD_MS,
                ARM_LEASE_MS,ARM_FEEDBACK_MS,ARM_ENABLE_CONFIRM_MS,1000000u,configured,
                a->id_commissioning?1u:0u,FORGE_BUILD_ID};
            for(unsigned k=0;k<10;k++) put32(out+9+k*4,values[k]);
            ok=true;
        }
        else if(type==21 && p[0]>=1 && p[0]<=3) {
            /* Passive last received CAN frame, including rejected payloads. */
            arm_rx_trace *r=&a->rx_trace[p[0]-1]; ok=true;
            put32(out+9,r->count); put32(out+13,r->at);
            out[17]=r->id; out[18]=r->id>>8; out[19]=r->len;
            memcpy(out+20,r->data,8);
        }
        else if(type==19) {
            /* Read-only first-fault record; never clears the fault latch. */
            ok=true; out[9]=a->fault; out[10]=a->fault_reason;
            out[11]=a->fault_slot; out[12]=a->fault_status;
            put32(out+13,a->fault_at); put32(out+17,a->fault_command_age);
            put32(out+21,a->fault_feedback_age);
            putf(out+25,a->fault_p); putf(out+29,a->fault_v); putf(out+33,a->fault_t);
            putf(out+37,a->fault_target); putf(out+41,a->fault_velocity);
            putf(out+45,a->fault_speed_limit); putf(out+49,a->fault_torque_limit);
            out[53]=a->fault_sample_flags;
        } else if(type==10) {
            arm_diagnostic *q=&a->diagnostic; ok=true;
            out[9]=q->state; out[10]=q->bus; out[11]=q->id; out[12]=q->id>>8;
            out[13]=q->reg; out[14]=q->feedback_id; out[15]=q->feedback_id>>8;
            put32(out+16,q->value);
        }
    }
    out[8]=ok?0:1; put32(out+60,wire_crc(out,60)); return true;
}
bool wire_byte(arm_wire *w,uint8_t b,uint32_t now,uint8_t out[64]) {
    if(w->used==0 && b!=0xa5) return false;
    if(w->used==1 && b!=0x5a) { w->used=(b==0xa5)?1:0; return false; }
    w->buffer[w->used++]=b;
    if(w->used<64) return false;
    w->used=0;
    if(w->buffer[2]!=1 || u32(w->buffer+60)!=wire_crc(w->buffer,60)) {
        /* Retain a possible header inside the bad packet to recover byte loss. */
        for(unsigned i=1;i<63;i++) if(w->buffer[i]==0xa5 && w->buffer[i+1]==0x5a) {
            memmove(w->buffer,w->buffer+i,64-i); w->used=64-i; return false;
        }
        if(w->buffer[63]==0xa5) { w->buffer[0]=0xa5; w->used=1; }
        return false;
    }
    return dispatch(w,now,out);
}
