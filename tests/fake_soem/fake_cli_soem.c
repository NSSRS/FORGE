/* Isolated query/assignment transport. No sockets or hardware access. */
#include "ethercat.h"
#include "bridge_pdo.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ec_slavet ec_slave[2];
ec_groupt ec_group[1];
int ec_slavecount = 1;
static encos_bridge_outputs_t output;
static encos_bridge_inputs_t input;
static unsigned bus, motor_id, frames[6], config_count, controls, invalid;
static int packed_reply, bad_wkc;
static int four_motor_probe, missing_motor, drop_wkc, omit_version, cycles;
static const char *reply_gate;
static int mute_peer, clear_muted;

int ec_init(const char *name)
{
    bus = strstr(name, "can2") ? 2 : 1;
    motor_id = strstr(name, "id4") ? 4 : 1;
    packed_reply = strstr(name, "packed-reply") != NULL;
    bad_wkc = strstr(name, "bad-wkc") != NULL;
    four_motor_probe = strstr(name, "probe") != NULL;
    missing_motor = strstr(name, "missing4") != NULL;
    drop_wkc = strstr(name, "drop-wkc") != NULL;
    omit_version = strstr(name, "no-version") != NULL;
    reply_gate = getenv("ENCOS_TEST_REPLY_GATE");
    mute_peer = strstr(name, "mute-peer") != NULL;
    clear_muted = strstr(name, "clear-muted") != NULL;
    memset(&output, 0, sizeof(output)); memset(&input, 0, sizeof(input));
    ec_slave[1] = (ec_slavet){.Obits=688, .Ibits=736, .Obytes=86, .Ibytes=92,
        .outputs=(uint8_t *)&output, .inputs=(uint8_t *)&input,
        .eep_man=9, .eep_id=0x26483052, .eep_rev=0x20111};
    ec_group[0] = (ec_groupt){1, 1};
    return 1;
}
int ec_config_init(int unused) { (void)unused; return 1; }
int ec_config_map(void *map) { (void)map; return 178; }
int ec_readPDOmap(uint16_t slave, int *out, int *in)
{ (void)slave; *out=688; *in=736; return 1; }
int ec_SDOread(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t complete_access,
               int *size, void *value, int timeout)
{
    (void)timeout;
    if (slave != 1 || subindex || complete_access || index != 0x100a || *size < 4)
        return 0;
    memcpy(value, "5.11", 4);
    *size = 4;
    return 1;
}
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
    if (four_motor_probe && output.motor_num) {
        const unsigned route[6] = {1, 2, 0, 3, 4, 0};
        if (output.motor_num != 5 || output.can_ide) { ++invalid; return 0; }
        if (drop_wkc && ++cycles == 12) return 0;
        input.status[1] ^= 0xc0;
        input.motor_num = 4;
        unsigned muted = 0;
        FILE *gate = reply_gate ? fopen(reply_gate, "r") : NULL;
        if (gate) {
            int digit = fgetc(gate);
            if (digit >= '1' && digit <= '4') muted = (unsigned)(digit - '0');
            fclose(gate);
        }
        for (unsigned i=0; i<6; ++i) {
            const encos_can_frame_t *f = &output.motor[i];
            encos_can_frame_t *reply = &input.motor[5-i]; /* Input order differs. */
            if (!f->dlc) {
                encos_can_frame_t empty = {0};
                if (memcmp(f, &empty, sizeof(empty))) { ++invalid; return 0; }
                if (route[i] % 2 == 0) {
                    memset(reply, 0, sizeof(*reply));
                    if (route[i] == 2) reply->id = 2; /* ID retained, DLC cleared. */
                }
                continue; /* Odd IDs retain their last reply. */
            }
            if (!route[i] || f->id != route[i] || f->dlc != 2 || f->rtr ||
                f->data[0] != 0xe0 || (f->data[1] != 1 && f->data[1] != 30)) {
                ++invalid;
                return 0;
            }
            ++frames[i];
            if (muted && (route[i] == muted || (mute_peer && route[i] == 3))) {
                if (clear_muted) memset(reply, 0, sizeof(*reply));
                continue;
            }
            if (missing_motor && route[i] == 4) continue;
            if (omit_version && route[i] == 1 && f->data[1] == 30) continue;
            reply->id = route[i];
            reply->dlc = f->data[1] == 1 ? 6 : 8;
            reply->data[0] = 0xa0;
            reply->data[1] = f->data[1];
            reply->data[2] = 0x41;
        }
        return 3;
    }
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
