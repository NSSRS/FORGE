#include <signal.h>
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

static volatile sig_atomic_t stop_requested;

static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static bool exchange(int expected_wkc)
{
    ec_send_processdata();
    return ec_receive_processdata(EC_TIMEOUTRET) >= expected_wkc;
}

static bool valid_frame(const encos_can_frame_t *frame)
{
    return frame->id <= ENCOS_DISCOVERY_CAN_ID && frame->rtr == 0 &&
           frame->dlc <= ENCOS_CAN_DATA_MAX;
}

typedef bool (*reply_matcher_t)(const encos_can_frame_t *, void *);

static bool send_query_and_wait(encos_bridge_outputs_t *outputs,
                                const encos_bridge_inputs_t *inputs, int expected_wkc,
                                const encos_can_frame_t *request, reply_matcher_t matcher,
                                void *result)
{
    memset(outputs, 0, sizeof(*outputs));
    outputs->motor_num = 1;
    outputs->can_ide = 0; /* Standard 11-bit CAN identifier. */
    outputs->motor[0] = *request;

    /* Hold the read-only request for a few cycles, then make the slot inactive. */
    for (int cycle = 0; cycle < 5 && !stop_requested; ++cycle) {
        if (!exchange(expected_wkc)) {
            fprintf(stderr, "EtherCAT working counter dropped while sending query.\n");
            return false;
        }
        usleep(CYCLE_US);
    }
    memset(outputs, 0, sizeof(*outputs));

    for (int cycle = 0; cycle < RESPONSE_TIMEOUT_CYCLES && !stop_requested; ++cycle) {
        if (!exchange(expected_wkc)) {
            fprintf(stderr, "EtherCAT working counter dropped while awaiting reply.\n");
            return false;
        }
        for (size_t slot = 0; slot < ENCOS_BRIDGE_CHANNELS; ++slot) {
            const encos_can_frame_t *frame = &inputs->motor[slot];
            if (valid_frame(frame) && frame->dlc != 0 && matcher(frame, result)) {
                return true;
            }
        }
        usleep(CYCLE_US);
    }
    return false;
}

static bool match_discovery(const encos_can_frame_t *frame, void *result)
{
    return encos_parse_discovery_reply(frame, result);
}

typedef struct {
    uint16_t motor_id;
    float position_deg;
    uint8_t error;
} position_result_t;

static bool match_position(const encos_can_frame_t *frame, void *opaque)
{
    position_result_t *result = opaque;
    return encos_parse_position_reply(frame, result->motor_id, &result->position_deg,
                                      &result->error);
}

typedef struct {
    uint16_t motor_id;
    encos_version_t version;
    uint8_t error;
} version_result_t;

static bool match_version(const encos_can_frame_t *frame, void *opaque)
{
    version_result_t *result = opaque;
    return encos_parse_version_reply(frame, result->motor_id, &result->version,
                                     &result->error);
}

typedef struct {
    uint16_t motor_id;
    uint16_t timeout_ms;
    uint8_t error;
} timeout_result_t;

static bool match_timeout(const encos_can_frame_t *frame, void *opaque)
{
    timeout_result_t *result = opaque;
    return encos_parse_timeout_reply(frame, result->motor_id, &result->timeout_ms,
                                     &result->error);
}

static int enter_operational(const char *interface_name, int *expected_wkc)
{
    if (!ec_init(interface_name)) {
        fprintf(stderr, "Could not open EtherCAT interface %s. Run with sudo.\n",
                interface_name);
        return 0;
    }
    if (ec_config_init(FALSE) != 1) {
        fprintf(stderr, "Expected exactly one EtherCAT bridge; found %d.\n", ec_slavecount);
        return 0;
    }
    ec_slave[1].CoEdetails &= ~ECT_COEDET_SDOCA;
    static uint8_t io_map[4096];
    ec_config_map(io_map);
    ec_configdc();
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    if (ec_slave[1].Obytes != ENCOS_BRIDGE_OUTPUT_BYTES ||
        ec_slave[1].Ibytes != ENCOS_BRIDGE_INPUT_BYTES) {
        fprintf(stderr, "PDO mismatch: board reports OUT=%u IN=%u; require OUT=86 IN=92.\n",
                ec_slave[1].Obytes, ec_slave[1].Ibytes);
        return 0;
    }

    *expected_wkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    memset(ec_slave[1].outputs, 0, ENCOS_BRIDGE_OUTPUT_BYTES);
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    exchange(*expected_wkc);
    ec_writestate(0);
    for (int tries = 0; tries < 40; ++tries) {
        exchange(*expected_wkc);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
            return 1;
        }
    }
    fprintf(stderr, "Bridge did not reach EtherCAT OPERATIONAL state.\n");
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "Usage: sudo %s ETHERNET_INTERFACE\n", argv[0]);
        return 2;
    }
    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);

    int expected_wkc = 0;
    if (!enter_operational(argv[1], &expected_wkc)) {
        ec_close();
        return 1;
    }
    encos_bridge_outputs_t *outputs = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    const encos_bridge_inputs_t *inputs = (const encos_bridge_inputs_t *)ec_slave[1].inputs;
    printf("Bridge operational: OUT=86 IN=92 WKC=%d. Query-only mode.\n", expected_wkc);

    uint16_t motor_id = 0;
    encos_can_frame_t request = encos_make_discovery_request();
    if (!send_query_and_wait(outputs, inputs, expected_wkc, &request, match_discovery,
                             &motor_id)) {
        fprintf(stderr, "No valid motor-ID reply within 1 second.\n");
        goto fail;
    }
    printf("Motor ID: %u\n", motor_id);

    position_result_t position = {.motor_id = motor_id};
    request = encos_make_query_request(motor_id, 1);
    if (!send_query_and_wait(outputs, inputs, expected_wkc, &request, match_position,
                             &position)) {
        fprintf(stderr, "No valid position reply within 1 second.\n");
        goto fail;
    }
    printf("Position: %.6f deg, motor error: %u\n", position.position_deg, position.error);

    version_result_t version = {.motor_id = motor_id};
    request = encos_make_query_request(motor_id, 30);
    if (!send_query_and_wait(outputs, inputs, expected_wkc, &request, match_version,
                             &version)) {
        fprintf(stderr, "No valid version reply within 1 second.\n");
        goto fail;
    }
    printf("Hardware version: %u.%u.%u; software version: %u.%u.%u; motor error: %u\n",
           version.version.hardware[0], version.version.hardware[1],
           version.version.hardware[2], version.version.software[0],
           version.version.software[1], version.version.software[2], version.error);

    timeout_result_t timeout = {.motor_id = motor_id};
    request = encos_make_query_request(motor_id, 31);
    if (!send_query_and_wait(outputs, inputs, expected_wkc, &request, match_timeout,
                             &timeout)) {
        fprintf(stderr, "No valid CAN-timeout reply within 1 second.\n");
        goto fail;
    }
    printf("CAN timeout: %u ms; motor error: %u\n", timeout.timeout_ms, timeout.error);

    memset(outputs, 0, sizeof(*outputs));
    exchange(expected_wkc);
    ec_close();
    return 0;

fail:
    memset(outputs, 0, sizeof(*outputs));
    exchange(expected_wkc);
    ec_close();
    return 1;
}
