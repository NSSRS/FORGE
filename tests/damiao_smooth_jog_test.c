#include "arm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static float sent;
static unsigned commands;
static bool fail_send;
static float speed;
static bool tx(void *ctx,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t n) {
    (void)ctx; assert(bus==1 && n==8);
    if(fail_send) return false;
    if(id==0x101) { memcpy(&sent,d,4); commands++; }
    return true;
}
static void fresh(arm_controller *a,uint32_t now) {
    a->joints[0].feedback_at=now;
    a->joints[0].feedback_seen=true;
}
static void ready(arm_controller *a,uint32_t now) {
    arm_init(a,tx,0); fail_send=false; commands=0;
    arm_config c={1,1,17,-12.5f,12.5f,1.5f,20,0,0};
    assert(arm_prepare_routed(a,0,&c,1,2|ARM_ROUTED_MANUAL_TRAVEL|ARM_ROUTED_POSITION_HOLD));
    arm_joint *j=&a->joints[0];
    j->registers=31; j->range=(dm_range){12.5f,45,54}; j->p=2.f;
    fresh(a,now);
    dm_command hold={2.f,speed,0,0,0};
    assert(!arm_set_smooth(a,0,&hold,now)); /* No implicit enable. */
    assert(arm_set(a,0,&hold,now)); assert(arm_enable(a,0,now));
    assert(!arm_set_smooth(a,0,&hold,now)); /* Require enabled confirmation. */
    j->enable_confirmed=true; j->status=1; a->last_tick=now;
    assert(sent==2.f);
}
static void continuous_and_release(void) {
    for(int direction=-1;direction<=1;direction+=2) {
        arm_controller a; ready(&a,20);
        dm_command c={2.f+direction*speed*.1f,speed,0,0,0};
        assert(arm_set_smooth(&a,0,&c,20));
        for(uint32_t t=22;t<=120;t+=2) {
            if(t%20==0) { c.p+=direction*speed*.02f; assert(arm_set_smooth(&a,0,&c,t)); }
            fresh(&a,t); float previous=sent; arm_tick(&a,t);
            assert(!a.fault);
            assert(fabsf((sent-previous)-direction*speed*.002f)<3e-7f);
        }
        assert(commands==51); /* One preload plus 50 updates; no CAN-rate increase. */
        assert(fabsf(sent-(2.f+direction*speed*.1f))<1e-5f);
        /* A delayed tick cannot replay all missed travel in one jump. */
        float previous=sent; fresh(&a,160); arm_tick(&a,160);
        assert(fabsf((sent-previous)-direction*speed*.002f)<3e-7f);
        /* Release captures measured position behind the moving target and must
           bypass slew in a single tick, not continue traveling during braking. */
        c.p=2.f-direction*.04f; assert(arm_set(&a,0,&c,161));
        fresh(&a,162); arm_tick(&a,162); assert(sent==c.p && !a.joints[0].smooth_jog);
        for(uint32_t t=164;t<=180;t+=2) { fresh(&a,t); arm_tick(&a,t); assert(sent==c.p); }
        previous=sent; c.p-=direction*.02f; assert(arm_set_smooth(&a,0,&c,180));
        fresh(&a,182); arm_tick(&a,182);
        assert(fabsf((sent-previous)+direction*speed*.002f)<3e-7f);
    }
}
static void endpoint_and_rejections(void) {
    arm_controller a; ready(&a,20);
    dm_command c={2.f+speed*.003f,speed,0,0,0};
    assert(arm_set_smooth(&a,0,&c,20));
    fresh(&a,22); arm_tick(&a,22); assert(sent<c.p);
    for(uint32_t t=24;t<=40;t+=2) { fresh(&a,t); arm_tick(&a,t); assert(sent==c.p); }
    /* Invalid updates never replace the endpoint or extend the lease. */
    dm_command bad=c; bad.v=.175f; assert(!arm_set_smooth(&a,0,&bad,41));
    bad=c; bad.p=NAN; assert(!arm_set_smooth(&a,0,&bad,41));
    bad=c; bad.p=12.6f; assert(!arm_set_smooth(&a,0,&bad,41));
    bad=c; bad.v=-speed; assert(!arm_set_smooth(&a,0,&bad,41));
    bad=c; bad.t=.1f; assert(!arm_set_smooth(&a,0,&bad,41));
    assert(!arm_set_smooth(&a,6,&c,41));
    a.joints[0].position_hold=false; assert(!arm_set_smooth(&a,0,&c,41));
    a.joints[0].position_hold=true;
    assert(a.joints[0].command_at==20 && a.joints[0].cmd.p==c.p);
    fresh(&a,170); arm_tick(&a,170);
    assert(a.fault && a.fault_reason==1 && !a.joints[0].smooth_jog);
    assert(!arm_set_smooth(&a,0,&c,171));
}
static void watchdogs_stop_and_rollover(void) {
    arm_controller a; ready(&a,20);
    dm_command c={2.1f,speed,0,0,0};
    assert(arm_set_smooth(&a,0,&c,20));
    for(uint32_t t=40;t<=120;t+=20) { assert(arm_set_smooth(&a,0,&c,t)); arm_tick(&a,t); }
    assert(a.fault && a.fault_reason==2); /* Fresh commands cannot mask missing feedback. */
    ready(&a,20); assert(arm_set_smooth(&a,0,&c,20));
    arm_stop(&a); unsigned before=commands; arm_tick(&a,22);
    assert(commands==before && !a.joints[0].smooth_jog && !a.joints[0].enabled);
    ready(&a,20); assert(arm_set_smooth(&a,0,&c,20));
    fail_send=true; arm_tick(&a,22); fail_send=false;
    assert(a.fault && a.fault_reason==5 && a.joints[0].sent_position==2.f);
    ready(&a,UINT32_MAX-1); assert(arm_set_smooth(&a,0,&c,UINT32_MAX-1));
    fresh(&a,0); arm_tick(&a,0);
    assert(!a.fault && fabsf(sent-2.f-speed*.002f)<3e-7f);
}
int main(void) {
    for(unsigned multiplier=1;multiplier<=2;multiplier++) {
        speed=0.0872664626f*multiplier;
        continuous_and_release(); endpoint_and_rejections(); watchdogs_stop_and_rollover();
    }
    puts("Smooth jog target cadence, immediate hold and watchdog tests passed");
}
