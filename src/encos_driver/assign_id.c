#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ethercat.h"
#include "bridge_pdo.h"
#include "cli_bridge.h"
#include "encos_protocol.h"

#define CYCLE_US 1000
#define RESPONSE_TIMEOUT_CYCLES 1000

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
    uint8_t slot = 0;
    bool execute = false, bus_seen = false;
    if (argc < 5) goto usage;
    for (int arg = 4; arg < argc; ++arg) {
        if (!strcmp(argv[arg], "--execute") && !execute) execute = true;
        else if (!strcmp(argv[arg], "--can-bus") && !bus_seen && arg + 1 < argc) {
            if (!encos_parse_can_bus(argv[++arg], &slot)) goto usage;
            bus_seen = true;
        } else goto usage;
    }
    if (!execute) goto usage;
    uint16_t old_id = 0, new_id = 0;
    if (!parse_id(argv[2], &old_id) || !parse_id(argv[3], &new_id) || old_id == new_id) {
        fprintf(stderr, "IDs must be distinct standard CAN IDs from 1 through 0x7FE.\n");
        return 2;
    }
    int expected_wkc = 0;
    signal(SIGINT, encos_request_stop);
    signal(SIGTERM, encos_request_stop);
    if (!encos_open_cli_bridge(argv[1], &expected_wkc)) {
        fprintf(stderr, "Bridge did not reach OPERATIONAL with exactly one EtherCAT bridge.\n");
        ec_close();
        return 1;
    }
    encos_bridge_outputs_t *outputs = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    const encos_bridge_inputs_t *inputs = (const encos_bridge_inputs_t *)ec_slave[1].inputs;
    uint16_t discovered = 0;
    printf("ID assignment requires every other motor disconnected or powered off; CAN%u, PDO slot %u.\n", slot / 3 + 1, slot);
    if (!encos_discover_motor_at_slot(expected_wkc, slot, &discovered) || discovered != old_id) {
        fprintf(stderr, "Isolated discovery did not match OLD_ID %u; no configuration command sent.\n", old_id);
        goto fail;
    }
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
    outputs->motor_num = slot + 1;
    outputs->can_ide = 0;
    outputs->motor[slot] = request;
    if (encos_stop_requested) goto fail;
    if (!encos_exchange(expected_wkc)) goto fail;
    memset(outputs, 0, sizeof(*outputs));
    for (int cycle = 0; cycle < RESPONSE_TIMEOUT_CYCLES && !encos_stop_requested; ++cycle) {
        if (!encos_exchange(expected_wkc)) goto fail;
        for (size_t reply_slot = 0; reply_slot < ENCOS_BRIDGE_CHANNELS; ++reply_slot) {
            const encos_can_frame_t *reply = &inputs->motor[reply_slot];
            if (reply->id == ENCOS_DISCOVERY_CAN_ID && reply->rtr == 0 && reply->dlc == 4 &&
                reply->data[0] == (uint8_t)(new_id >> 8) && reply->data[1] == (uint8_t)new_id &&
                reply->data[2] == 0x01 && reply->data[3] == 0x04) {
                printf("PASS: motor CAN ID changed from %u to %u. Power-cycle, then query ID %u.\n", old_id, new_id, new_id);
                encos_exchange(expected_wkc);
                ec_close();
                return 0;
            }
        }
        usleep(CYCLE_US);
    }
    fprintf(stderr, "No matching ID-change acknowledgement. Do not retry with multiple motors connected.\n");
fail:
    memset(outputs, 0, sizeof(*outputs));
    encos_exchange(expected_wkc);
    ec_close();
    return 1;
usage:
    fprintf(stderr, "Usage: sudo %s ETHERNET_INTERFACE OLD_ID NEW_ID --execute [--can-bus 1|2]\n", argv[0]);
    fprintf(stderr, "Disconnect or power off every other motor first. This command changes a persistent CAN ID.\n");
    return 2;
}
