#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ethercat.h"
#include "bridge_pdo.h"
#include "encos_protocol.h"

#define CYCLE_US 1000
#define RESPONSE_TIMEOUT_CYCLES 1000
static uint8_t io_map[4096];

static bool exchange(int expected_wkc)
{
    ec_send_processdata();
    return ec_receive_processdata(EC_TIMEOUTRET) >= expected_wkc;
}

static bool enter_operational(const char *interface_name, int *expected_wkc)
{
    if (!ec_init(interface_name) || ec_config_init(FALSE) != 1) return false;
    ec_slave[1].CoEdetails &= ~ECT_COEDET_SDOCA;
    ec_config_map(io_map);
    ec_configdc();
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[1].Obytes != ENCOS_BRIDGE_OUTPUT_BYTES ||
        ec_slave[1].Ibytes != ENCOS_BRIDGE_INPUT_BYTES) return false;
    *expected_wkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    memset(ec_slave[1].outputs, 0, ENCOS_BRIDGE_OUTPUT_BYTES);
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    exchange(*expected_wkc);
    ec_writestate(0);
    for (int tries = 0; tries < 40; ++tries) {
        exchange(*expected_wkc);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) return true;
    }
    return false;
}

static bool parse_id(const char *text, uint16_t *id)
{
    char *end = NULL;
    unsigned long value = strtoul(text, &end, 0);
    if (text == end || *end != '\0' || value == 0 || value >= ENCOS_DISCOVERY_CAN_ID) return false;
    *id = (uint16_t)value;
    return true;
}

int main(int argc, char **argv)
{
    if (argc != 5 || strcmp(argv[4], "--execute") != 0) {
        fprintf(stderr, "Usage: sudo %s ETHERNET_INTERFACE OLD_ID NEW_ID --execute\n", argv[0]);
        fprintf(stderr, "Disconnect or power off every other motor first. This command changes a persistent CAN ID.\n");
        return 2;
    }
    uint16_t old_id = 0, new_id = 0;
    if (!parse_id(argv[2], &old_id) || !parse_id(argv[3], &new_id) || old_id == new_id) {
        fprintf(stderr, "IDs must be distinct standard CAN IDs from 1 through 0x7FE.\n");
        return 2;
    }
    int expected_wkc = 0;
    if (!enter_operational(argv[1], &expected_wkc)) {
        fprintf(stderr, "Bridge did not reach OPERATIONAL with exactly one EtherCAT bridge.\n");
        ec_close();
        return 1;
    }
    encos_bridge_outputs_t *outputs = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    const encos_bridge_inputs_t *inputs = (const encos_bridge_inputs_t *)ec_slave[1].inputs;
    encos_can_frame_t request = {0};
    request.id = ENCOS_DISCOVERY_CAN_ID;
    request.dlc = 6;
    request.data[0] = (uint8_t)(old_id >> 8);
    request.data[1] = (uint8_t)old_id;
    request.data[2] = 0x00;
    request.data[3] = 0x04;
    request.data[4] = (uint8_t)(new_id >> 8);
    request.data[5] = (uint8_t)new_id;

    /* One PDO cycle only: holding a configuration frame could retransmit it. */
    memset(outputs, 0, sizeof(*outputs));
    outputs->motor_num = 1;
    outputs->can_ide = 0;
    outputs->motor[0] = request;
    if (!exchange(expected_wkc)) goto fail;
    memset(outputs, 0, sizeof(*outputs));
    for (int cycle = 0; cycle < RESPONSE_TIMEOUT_CYCLES; ++cycle) {
        if (!exchange(expected_wkc)) goto fail;
        for (size_t slot = 0; slot < ENCOS_BRIDGE_CHANNELS; ++slot) {
            const encos_can_frame_t *reply = &inputs->motor[slot];
            if (reply->id == ENCOS_DISCOVERY_CAN_ID && reply->rtr == 0 && reply->dlc == 4 &&
                reply->data[0] == (uint8_t)(new_id >> 8) && reply->data[1] == (uint8_t)new_id &&
                reply->data[2] == 0x01 && reply->data[3] == 0x04) {
                printf("PASS: motor CAN ID changed from %u to %u. Power-cycle, then query ID %u.\n", old_id, new_id, new_id);
                exchange(expected_wkc); ec_close(); return 0;
            }
        }
        usleep(CYCLE_US);
    }
    fprintf(stderr, "No matching ID-change acknowledgement. Do not retry with multiple motors connected.\n");
fail:
    memset(outputs, 0, sizeof(*outputs)); exchange(expected_wkc); ec_close(); return 1;
}
