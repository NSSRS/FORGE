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
    if(type==1 && !active(a)) { w->session=true; w->sequence=seq; memcpy(out+9,"FORGE-MC02/1",12); put32(out+21,1000u/ARM_CONTROL_PERIOD_MS); ok=true; }
    else if(type==6) { arm_stop(a); ok=true; } /* stop always accepted, including replay */
    else if(w->session && seq>w->sequence) {
        w->sequence=seq;
        if(type==2) {
            arm_config c={.bus=p[1],.id=u16(p+2),.feedback_id=u16(p+4),.lo=f32(p+6),.hi=f32(p+10),
                .speed=f32(p+14),.torque=f32(p+18),.kp=f32(p+22),.kd=f32(p+26)};
            ok=arm_configure(a,i,&c);
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
        } else if(type==8) ok=arm_reset(a);
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
