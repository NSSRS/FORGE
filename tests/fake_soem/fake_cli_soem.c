/* Isolated query/assignment transport. No sockets or hardware access. */
#include "ethercat.h"
#include "bridge_pdo.h"
#include <stdio.h>
#include <string.h>

ec_slavet ec_slave[2];
ec_groupt ec_group[1];
int ec_slavecount = 1;
static encos_bridge_outputs_t output;
static encos_bridge_inputs_t input;
static unsigned bus, motor_id, frames[6], config_count, controls, invalid;
static int packed_reply, bad_wkc;

int ec_init(const char *name)
{
    bus = strstr(name, "can2") ? 2 : 1;
    motor_id = strstr(name, "id4") ? 4 : 1;
    packed_reply = strstr(name, "packed-reply") != NULL;
    bad_wkc = strstr(name, "bad-wkc") != NULL;
    memset(&output, 0, sizeof(output)); memset(&input, 0, sizeof(input));
    ec_slave[1] = (ec_slavet){.Obits=688, .Ibits=736, .Obytes=86, .Ibytes=92,
        .outputs=(uint8_t *)&output, .inputs=(uint8_t *)&input};
    ec_group[0] = (ec_groupt){1, 1};
    return 1;
}
int ec_config_init(int unused) { (void)unused; return 1; }
int ec_config_map(void *map) { (void)map; return 178; }
int ec_readPDOmap(uint16_t slave, int *out, int *in)
{ (void)slave; *out=688; *in=736; return 1; }
int ec_configdc(void) { return 1; }
uint16_t ec_statecheck(uint16_t slave, uint16_t state, int timeout)
{ (void)timeout; ec_slave[slave].state=state; return state; }
int ec_send_processdata(void) { return 1; }
int ec_writestate(uint16_t slave) { (void)slave; return 1; }
void ec_close(void)
{
    encos_bridge_outputs_t empty={0};
    printf("FAKE frames=%u,%u,%u,%u,%u,%u config=%u controls=%u invalid=%u cleared=%u\n",
           frames[0],frames[1],frames[2],frames[3],frames[4],frames[5],
           config_count,controls,invalid,memcmp(&output,&empty,sizeof(output))==0);
}

int ec_receive_processdata(int timeout)
{
    (void)timeout;
    if (bad_wkc) return 0;
    if (output.motor_num > 6) { ++invalid; return 0; }
    for (unsigned i=0; i<output.motor_num; ++i) {
        const encos_can_frame_t *f=&output.motor[i];
        encos_can_frame_t empty={0};
        if (!f->dlc) { if (memcmp(f,&empty,sizeof(empty))) ++invalid; continue; }
        ++frames[i];
        if (i/3+1 != bus) continue;
        encos_can_frame_t *r=&input.motor[packed_reply ? 0 : i];
        memset(r,0,sizeof(*r));
        r->id=f->id;
        if (f->id==0x7ff && f->dlc==4 && f->data[3]==0x82) {
            r->dlc=5; r->data[0]=0xff; r->data[1]=0xff; r->data[2]=1;
            r->data[3]=motor_id>>8; r->data[4]=motor_id;
        } else if (f->id==0x7ff && f->dlc==6 && f->data[3]==4) {
            ++config_count;
            if (((unsigned)f->data[0]<<8 | f->data[1])!=motor_id) continue;
            motor_id=(unsigned)f->data[4]<<8 | f->data[5];
            r->dlc=4; r->data[0]=motor_id>>8; r->data[1]=motor_id;
            r->data[2]=1; r->data[3]=4;
        } else if (f->id==motor_id && f->dlc==2 && f->data[0]==0xe0) {
            r->data[0]=0xa0; r->data[1]=f->data[1];
            switch (f->data[1]) {
            case 1: r->dlc=6; r->data[2]=0x41; r->data[3]=0x48; break;
            case 30: r->dlc=8; r->data[2]=1; r->data[5]=2; break;
            case 31: r->dlc=4; r->data[2]=1; r->data[3]=0xf4; break;
            case 37: r->dlc=3; r->data[2]=1; break;
            default: ++invalid;
            }
        } else if (f->dlc==7 || f->dlc==8) ++controls;
    }
    /* Cache persists after output clear, as the CLI must handle bridge caches. */
    return 3;
}
