#include "ethercat.h"
#include "bridge_pdo.h"
#include <stdatomic.h>
#include <string.h>

ec_slavet ec_slave[2];
ec_groupt ec_group[1];
static encos_bridge_outputs_t output;
static encos_bridge_inputs_t input;
static float position[6], velocity[6];
static atomic_int bad_wkc, no_feedback, motor_error, controls, zeros, clears;
static int bad_layout;

void fake_fault(int kind)
{
    atomic_store(&bad_wkc, kind == 1);
    atomic_store(&no_feedback, kind == 2);
    atomic_store(&motor_error, kind == 3);
}
int fake_controls(void) { return atomic_load(&controls); }
int fake_zeros(void) { return atomic_load(&zeros); }
int fake_clears(void) { return atomic_load(&clears); }

int ec_init(const char *name)
{
    bad_layout = strcmp(name, "forge-test-bad-layout") == 0;
    memset(&output, 0, sizeof(output)); memset(&input, 0, sizeof(input));
    memset(position, 0, sizeof(position)); memset(velocity, 0, sizeof(velocity));
    fake_fault(0); atomic_store(&controls, 0); atomic_store(&zeros, 0); atomic_store(&clears, 0);
    ec_slave[1] = (ec_slavet){.Obits=688, .Ibits=736, .Obytes=86, .Ibytes=92,
        .outputs=(uint8_t *)&output, .inputs=(uint8_t *)&input};
    ec_group[0] = (ec_groupt){1, 1};
    return 1;
}
int ec_config_init(int unused) { (void)unused; return 1; }
int ec_config_map(void *map) { (void)map; return 178; }
int ec_readPDOmap(uint16_t slave, int *output_bits, int *input_bits)
{ (void)slave; *output_bits = bad_layout ? 8192 : 688; *input_bits = 736; return 1; }
int ec_configdc(void) { return 1; }
uint16_t ec_statecheck(uint16_t slave, uint16_t state, int timeout)
{ (void)timeout; ec_slave[slave].state = state; return state; }
int ec_send_processdata(void) { return 1; }
int ec_writestate(uint16_t slave) { (void)slave; return 1; }
void ec_close(void) {}

static void put_float(uint8_t *data, float value)
{
    uint32_t bits; memcpy(&bits, &value, 4);
    for (int i=0; i<4; ++i) data[i] = (uint8_t)(bits >> (24-8*i));
}
static float get_float(const uint8_t *data)
{
    uint32_t bits = 0;
    for (int i=0; i<4; ++i) bits = (bits << 8) | data[i];
    float value; memcpy(&value, &bits, 4); return value;
}

int ec_receive_processdata(int timeout)
{
    (void)timeout;
    if (atomic_load(&bad_wkc)) return 0;
    memset(&input, 0, sizeof(input));
    if (!output.motor_num) atomic_fetch_add(&clears, 1);
    for (unsigned i=0; i<output.motor_num; ++i) {
        encos_can_frame_t *f = &output.motor[i], *r = &input.motor[i];
        r->id = f->id;
        if (f->dlc == 2 && f->data[0] == 0xe0) {
            r->dlc = 6; r->data[0] = 0xa0; r->data[1] = 1;
            put_float(&r->data[2], position[i]);
        } else {
            atomic_fetch_add(&controls, 1);
            unsigned mode = f->data[0] >> 5;
            unsigned ack = mode == 1 ? (f->data[7] & 3) : (f->data[0] & 3);
            if (mode == 2) {
                velocity[i] = get_float(&f->data[1]);
                if (velocity[i] == 0) atomic_fetch_add(&zeros, 1);
                position[i] += velocity[i] * 6 * .01f;
            } else {
                uint64_t packed = 0;
                for (int b=0; b<8; ++b) packed = (packed << 8) | f->data[b];
                uint32_t bits = (uint32_t)(packed >> 29);
                memcpy(&position[i], &bits, 4);
                velocity[i] = 0;
            }
            r->dlc = 8; r->data[0] = (uint8_t)((ack << 5) | atomic_load(&motor_error));
            put_float(&r->data[1], ack == 2 ? position[i] : velocity[i]);
            r->data[7] = 100;
        }
    }
    if (atomic_load(&no_feedback)) memset(&input, 0, sizeof(input));
    return 3;
}
