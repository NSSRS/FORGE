/* Test-only SOEM surface; never included by the hardware target. */
#ifndef TEST_ETHERCAT_H
#define TEST_ETHERCAT_H
#include <stdint.h>
#define FALSE 0
#define ECT_COEDET_SDOCA 0x20
#define EC_STATE_SAFE_OP 4
#define EC_STATE_OPERATIONAL 8
#define EC_TIMEOUTSTATE 2000
typedef struct {
    uint8_t CoEdetails;
    uint16_t state, Obits, Ibits;
    uint32_t Obytes, Ibytes;
    uint8_t *outputs, *inputs;
} ec_slavet;
typedef struct { int outputsWKC, inputsWKC; } ec_groupt;
extern ec_slavet ec_slave[2];
extern ec_groupt ec_group[1];
int ec_init(const char *name);
int ec_config_init(int unused);
int ec_config_map(void *map);
int ec_readPDOmap(uint16_t slave, int *output_bits, int *input_bits);
int ec_configdc(void);
uint16_t ec_statecheck(uint16_t slave, uint16_t state, int timeout);
int ec_send_processdata(void);
int ec_receive_processdata(int timeout);
int ec_writestate(uint16_t slave);
void ec_close(void);
#endif
