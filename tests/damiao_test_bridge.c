/* In-process fake CAN peripheral exercising the real C firmware and USB codec. */
#include "wire.h"
#include <string.h>
#include <math.h>
static unsigned commands[6], motor_mode=1, mode_writes, posvel_commands, invalid_commands;
static bool move_motor, ignore_mode_write;
static float shaft, requested, velocity_limit;
static uint32_t bench_timeout, enable_delay, enable_due[6];
static bool enable_waiting[6];
static arm_controller a; static arm_wire w; static uint32_t now; static uint8_t status[6];
static bool tx(void *ctx,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t n) {
    (void)ctx;
    /* One factory-like motor: CAN2, command ID 1, feedback ID 0. */
    if(a.diagnostic.state==1 && bus==2 && id==0x7ff && n==4 && d[0]==1 && d[1]==0 && d[2]==0x33) {
        uint8_t reply[8]={0}; memcpy(reply,d,4);
        uint32_t value=d[3]==8?1:d[3]==9?bench_timeout:d[3]==10?motor_mode:0;
        if(d[3]>=21 && d[3]<=23) { float f=d[3]==21?12.5f:d[3]==22?10.0f:28.0f; memcpy(&value,&f,4); }
        if(d[3]==31) { float f=4; memcpy(&value,&f,4); }
        for(unsigned k=0;k<4;k++) reply[4+k]=value>>(8*k);
        arm_receive(&a,bus,0,reply,8,now);
    }
    if(a.diagnostic.state==1 && bus==2 && id==0x7ff && n==8 && d[0]==1 && d[2]==0x55 && (d[3]==9 || d[3]==10)) {
        if(d[3]==9) bench_timeout=(uint32_t)d[4]|(uint32_t)d[5]<<8;
        else { mode_writes++; if(!ignore_mode_write) motor_mode=d[4]; requested=0; velocity_limit=0; }
        arm_receive(&a,bus,0,d,8,now);
    }
    for(unsigned i=0;i<6;i++) {
        arm_joint *j=&a.joints[i]; if(!j->configured||j->cfg.bus!=bus) continue;
        uint8_t reply[8]={0};
        if(id==0x7ff && n==4 && d[0]==j->cfg.id) {
            memcpy(reply,d,4); uint32_t value=0; float f=0;
            if(d[3]==21) f=12.5f; if(d[3]==22) f=a.bench?10:30; if(d[3]==23) f=a.bench?28:40;
            memcpy(&value,&f,4); if(d[3]==10) value=motor_mode; if(d[3]==9) value=2000;
            if(d[3]==31) { float f=4; memcpy(&value,&f,4); }
        for(unsigned k=0;k<4;k++) reply[4+k]=value>>(8*k);
        } else if((id==j->cfg.id || id==j->cfg.id+0x100) && n==8) {
            if(d[0]==255 && d[1]==255 && d[7]==0xfc) {
                status[i]=enable_delay?0:1; enable_waiting[i]=enable_delay!=0; enable_due[i]=now+enable_delay;
            }
            if(d[0]==255 && d[1]==255 && d[7]==0xfd) { status[i]=0; enable_waiting[i]=false; }
            if(!(d[0]==255 && d[1]==255)) {
                commands[i]++;
                if(id==j->cfg.id+0x100) {
                    posvel_commands++;
                    memcpy(&requested,d,4); memcpy(&velocity_limit,d+4,4);
                    if(motor_mode!=2 || !isfinite(requested) || velocity_limit<=0 || velocity_limit>0.10001f) invalid_commands++;
                } else if(motor_mode!=1) invalid_commands++;
            }
            uint8_t feedback[]={j->cfg.id|(status[i]<<4),0x7f,0xff,0x7f,0xf7,0xff,25,25};
            if(a.bench && move_motor) {
                unsigned pos=(unsigned)((shaft+12.5f)/25.0f*65535);
                feedback[1]=pos>>8; feedback[2]=pos;
            }
            memcpy(reply,feedback,8);
        } else continue;
        arm_receive(&a,bus,j->cfg.feedback_id,reply,8,now);
    }
    return true;
}
void fake_init(void) { motor_mode=1; mode_writes=0; posvel_commands=0; invalid_commands=0; move_motor=false; ignore_mode_write=false; shaft=0; requested=0; velocity_limit=0; now=0; bench_timeout=0; enable_delay=0; memset(enable_waiting,0,sizeof(enable_waiting)); memset(status,0,6); memset(commands,0,sizeof(commands)); arm_init(&a,tx,0); wire_init(&w,&a); }
void fake_advance(unsigned ms) { for(unsigned k=0;k<ms;k++) { ++now;
    for(unsigned i=0;i<6;i++) if(enable_waiting[i] && now>=enable_due[i]) { status[i]=1; enable_waiting[i]=false; }
    if(move_motor && motor_mode==2 && status[0]==1) {
        float step=velocity_limit*.001f, error=requested-shaft;
        shaft += error>step?step:error< -step?-step:error;
    }
    arm_tick(&a,now); } }
int fake_exchange(const uint8_t in[64],uint8_t out[64]) {
    int result=0; for(unsigned i=0;i<64;i++) result|=wire_byte(&w,in[i],now,out);
    fake_advance(5); return result;
}

unsigned fake_commands(unsigned slot) { return slot<6?commands[slot]:0; }
unsigned fake_now(void) { return now; }

void fake_set_enable_delay(unsigned ms) { enable_delay=ms; }

void fake_set_motion(unsigned on) { move_motor=on!=0; }
void fake_ignore_mode_write(unsigned on) { ignore_mode_write=on!=0; }
unsigned fake_mode(void) { return motor_mode; }
unsigned fake_mode_writes(void) { return mode_writes; }
unsigned fake_posvel_commands(void) { return posvel_commands; }
unsigned fake_invalid_commands(void) { return invalid_commands; }
