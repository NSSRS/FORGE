#include "arm.h"
#include "wire.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned sends; static uint8_t last[8]; static bool fail_send;
static bool send_frame(void *ctx,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t n) {
    (void)ctx; assert(bus==1); assert(id==1||id==2||id==0x7ff); assert(n==4||n==8);
    sends++; memcpy(last,d,n); return !fail_send;
}
static arm_config config={1,1,17,-1,1,2,3,20,1};
static dm_command hold={0,0,5,0.2f,0};
static void reg(arm_controller *a,unsigned r,uint32_t u) {
    uint8_t d[]={1,0,0x33,r,u,u>>8,u>>16,u>>24}; arm_receive(a,1,17,d,8,10);
}
static void regf(arm_controller *a,unsigned r,float f) { uint32_t u; memcpy(&u,&f,4); reg(a,r,u); }
static void ready(arm_controller *a) {
    arm_init(a,send_frame,0); assert(arm_configure(a,0,&config)); assert(arm_probe(a,0));
    regf(a,21,12.5f); regf(a,22,30); regf(a,23,40); reg(a,10,1); reg(a,9,2000);
    assert(a->joints[0].registers==31); arm_tick(a,10);
}
static void feedback(arm_controller *a,uint8_t status,uint32_t now) {
    uint8_t d[]={1|(status<<4),0x7f,0xff,0x7f,0xf7,0xff,25,26};
    arm_receive(a,1,17,d,8,now);
}
static void active(arm_controller *a,uint32_t now) {
    ready(a); feedback(a,0,now); assert(arm_set(a,0,&hold,now)); assert(arm_enable(a,0,now)); feedback(a,1,now+1);
}
static void codec(void) {
    dm_range r={12.5f,30,40}; dm_command c={0,0,0,0,0}; uint8_t d[8];
    const uint8_t zero[]={0x7f,0xff,0x7f,0xf0,0,0,7,0xff};
    assert(dm_pack(&r,&c,d)); assert(memcmp(d,zero,8)==0);
    c=(dm_command){12.5f,30,500,5,40}; assert(dm_pack(&r,&c,d));
    for(unsigned i=0;i<8;i++) assert(d[i]==255);
    c.p=NAN; assert(!dm_pack(&r,&c,d)); c.p=13; assert(!dm_pack(&r,&c,d));
    c.p=0; r.v=0; assert(!dm_pack(&r,&c,d));
}
static void safety(void) {
    arm_controller a; ready(&a);
    assert(!arm_enable(&a,0,10)); feedback(&a,0,20);
    dm_command bad=hold; bad.p=2; assert(!arm_set(&a,0,&bad,20));
    bad=hold; bad.kp=21; assert(!arm_set(&a,0,&bad,20));
    bad=hold; bad.t=NAN; assert(!arm_set(&a,0,&bad,20));
    assert(arm_set(&a,0,&hold,20)); assert(!arm_enable(&a,0,170));
    active(&a,20); assert(!arm_configure(&a,0,&config));
    /* Fresh feedback cannot renew an expired command lease. */
    feedback(&a,1,160); arm_tick(&a,170); assert(a.fault && !a.joints[0].enabled);
    arm_tick(&a,175); assert(last[7]==0xfd); assert(!arm_reset(&a));
    feedback(&a,0,176); assert(arm_reset(&a)); assert(!arm_enable(&a,0,177));
    active(&a,20); assert(arm_set(&a,0,&hold,119)); arm_tick(&a,121); assert(a.fault);
    active(&a,20); feedback(&a,8,30); assert(a.fault);
    active(&a,20); fail_send=true; arm_tick(&a,30); fail_send=false; assert(a.fault);
    active(&a,UINT32_MAX-30); feedback(&a,1,10); arm_tick(&a,15); assert(!a.fault);
    arm_tick(&a,120); assert(a.fault); /* rollover-safe leases */
    ready(&a); arm_config duplicate=config; assert(!arm_configure(&a,1,&duplicate));
    duplicate.id=2; assert(!arm_configure(&a,1,&duplicate));
    duplicate.feedback_id=18; assert(arm_configure(&a,1,&duplicate));
    assert(arm_probe(&a,0)); reg(&a,10,2); reg(&a,9,0); regf(&a,21,NAN);
    assert(a.joints[0].registers==0); reg(&a,9,3001); assert(a.joints[0].registers==0);
    /* Misrouted, truncated feedback cannot make a joint fresh. */
    ready(&a); uint8_t d[]={1,0x7f,0xff,0x7f,0xf7,0xff,25,25};
    arm_receive(&a,2,17,d,8,10); arm_receive(&a,1,18,d,8,10); arm_receive(&a,1,17,d,7,10);
    assert(!a.joints[0].feedback_seen);
    ready(&a); duplicate=config; duplicate.id=2; duplicate.feedback_id=18;
    assert(arm_configure(&a,1,&duplicate)); assert(arm_probe(&a,1));
    feedback(&a,0,20); assert(arm_set(&a,0,&hold,20));
    assert(!arm_enable(&a,0,20)); /* no diagnostics alongside active control */
}
static void packet(uint8_t *d,unsigned type,unsigned seq) {
    memset(d,0,64); d[0]=0xa5; d[1]=0x5a; d[2]=1; d[3]=type; d[4]=seq;
    uint32_t crc=wire_crc(d,60); for(unsigned i=0;i<4;i++) d[60+i]=crc>>(8*i);
}
static bool feed(arm_wire *w,uint8_t *d,uint8_t *out) { bool got=false; for(unsigned i=0;i<64;i++) got|=wire_byte(w,d[i],10,out); return got; }
static void framing(void) {
    assert(wire_crc((const uint8_t *)"123456789",9)==0xcbf43926);
    arm_controller a; arm_init(&a,send_frame,0); arm_wire w; wire_init(&w,&a); uint8_t d[64],out[64];
    packet(d,1,1); assert(feed(&w,d,out)); assert(out[8]==0); assert(!memcmp(out+9,"FORGE-MC02/1",12));
    assert(out[21]==0xf4 && out[22]==1 && out[23]==0 && out[24]==0);
    packet(d,7,2); assert(feed(&w,d,out)); assert(out[8]==0);
    assert(feed(&w,d,out)); assert(out[8]==1); /* replay rejected */
    packet(d,7,3); d[9]^=1; assert(!feed(&w,d,out));
    packet(d,7,4); assert(feed(&w,d,out)); assert(out[8]==0);
    /* A dropped byte followed by a good frame resynchronizes. */
    packet(d,7,5); for(unsigned i=0;i<63;i++) (void)wire_byte(&w,d[i],10,out);
    packet(d,7,6); assert(feed(&w,d,out)); assert(out[8]==0);
    packet(d,6,1); assert(feed(&w,d,out)); assert(out[8]==0); /* stop bypasses sequence */
}
int main(void) { codec(); safety(); framing(); puts("Damiao codec, state machine, watchdog and framing tests passed"); }
