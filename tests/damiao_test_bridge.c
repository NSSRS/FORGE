/* In-process fake CAN peripheral exercising the real C firmware and USB codec. */
#include "wire.h"
#include <string.h>
static unsigned commands[6];
static arm_controller a; static arm_wire w; static uint32_t now; static uint8_t status[6];
static bool tx(void *ctx,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t n) {
    (void)ctx;
    for(unsigned i=0;i<6;i++) {
        arm_joint *j=&a.joints[i]; if(!j->configured||j->cfg.bus!=bus) continue;
        uint8_t reply[8]={0};
        if(id==0x7ff && n==4 && d[0]==j->cfg.id) {
            memcpy(reply,d,4); uint32_t value=0; float f=0;
            if(d[3]==21) f=12.5f; if(d[3]==22) f=30; if(d[3]==23) f=40;
            memcpy(&value,&f,4); if(d[3]==10) value=1; if(d[3]==9) value=2000;
            for(unsigned k=0;k<4;k++) reply[4+k]=value>>(8*k);
        } else if(id==j->cfg.id && n==8) {
            if(d[0]==255 && d[1]==255 && d[7]==0xfc) status[i]=1;
            if(d[0]==255 && d[1]==255 && d[7]==0xfd) status[i]=0;
            if(!(d[0]==255 && d[1]==255)) commands[i]++;
            uint8_t feedback[]={j->cfg.id|(status[i]<<4),0x7f,0xff,0x7f,0xf7,0xff,25,25};
            memcpy(reply,feedback,8);
        } else continue;
        arm_receive(&a,bus,j->cfg.feedback_id,reply,8,now);
    }
    return true;
}
void fake_init(void) { now=0; memset(status,0,6); memset(commands,0,sizeof(commands)); arm_init(&a,tx,0); wire_init(&w,&a); }
void fake_advance(unsigned ms) { for(unsigned k=0;k<ms;k++) { ++now; arm_tick(&a,now); } }
int fake_exchange(const uint8_t in[64],uint8_t out[64]) {
    int result=0; for(unsigned i=0;i<64;i++) result|=wire_byte(&w,in[i],now,out);
    fake_advance(5); return result;
}

unsigned fake_commands(unsigned slot) { return slot<6?commands[slot]:0; }
