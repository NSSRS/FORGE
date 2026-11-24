#include "cli_bridge.h"
#include "encos_protocol.h"
#include "ethercat.h"
#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/file.h>

/* Same interface lock as MotorBus; released by the kernel on CLI exit. */
static int owner_fd = -1;

bool encos_exchange(int expected_wkc)
{
    ec_send_processdata();
    return ec_receive_processdata(EC_TIMEOUTRET) >= expected_wkc;
}

bool encos_open_cli_bridge(const char *interface_name, int *expected_wkc)
{
    if (!interface_name || !*interface_name || strlen(interface_name) >= 64) return false;
    for (const char *p = interface_name; *p; ++p)
        if (!isalnum((unsigned char)*p) && !strchr("_.:-", *p)) return false;
    if (owner_fd >= 0) return false;
    char lock_path[128];
    snprintf(lock_path, sizeof(lock_path), "/tmp/forge-encos-%s.lock", interface_name);
    owner_fd = open(lock_path, O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (owner_fd < 0 || flock(owner_fd, LOCK_EX | LOCK_NB)) {
        fprintf(stderr, "EtherCAT interface %s is owned or its lock is unavailable. Stop the existing owner first.\n", interface_name);
        if (owner_fd >= 0) close(owner_fd);
        owner_fd = -1;
        return false;
    }
    if (!ec_init(interface_name)) {
        fprintf(stderr, "Could not open EtherCAT interface %s. Run with sudo.\n",
                interface_name);
        return false;
    }
    if (ec_config_init(FALSE) != 1) {
        fprintf(stderr, "Expected exactly one EtherCAT bridge; found %d.\n", ec_slavecount);
        return false;
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
        return false;
    }

    *expected_wkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    memset(ec_slave[1].outputs, 0, ENCOS_BRIDGE_OUTPUT_BYTES);
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    encos_exchange(*expected_wkc);
    ec_writestate(0);
    for (int tries = 0; tries < 40; ++tries) {
        encos_exchange(*expected_wkc);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
            return true;
        }
    }
    fprintf(stderr, "Bridge did not reach EtherCAT OPERATIONAL state.\n");
    return false;
}

bool encos_parse_motor_ids(const char *text, uint16_t ids[ENCOS_MAX_CONTROLLED_MOTORS],
                           size_t *count)
{
    char buffer[64];
    if (text == NULL || count == NULL || strlen(text) >= sizeof(buffer)) return false;
    strcpy(buffer, text);
    *count = 0;
    for (char *token = strtok(buffer, ","); token != NULL; token = strtok(NULL, ",")) {
        char *end = NULL;
        unsigned long value = strtoul(token, &end, 10);
        if (*token == '\0' || *end != '\0' || value == 0 ||
            value >= ENCOS_DISCOVERY_CAN_ID || *count >= ENCOS_MAX_CONTROLLED_MOTORS) return false;
        for (size_t index = 0; index < *count; ++index)
            if (ids[index] == value) return false;
        ids[(*count)++] = (uint16_t)value;
    }
    return *count != 0;
}

volatile sig_atomic_t encos_stop_requested;

void encos_request_stop(int signal_number)
{
    (void)signal_number;
    encos_stop_requested = 1;
}

bool encos_parse_can_bus(const char *text, uint8_t *slot)
{
    if (!text || !slot || (strcmp(text, "1") && strcmp(text, "2"))) return false;
    *slot = strcmp(text, "1") == 0 ? 0 : 3;
    return true;
}

bool encos_query_reply_at_slot(int expected_wkc, uint8_t slot,
                              const encos_can_frame_t *request,
                              encos_reply_matcher_t matcher, void *result)
{
    if (slot >= ENCOS_BRIDGE_CHANNELS) return false;
    encos_bridge_outputs_t *outputs = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    const encos_bridge_inputs_t *inputs = (const encos_bridge_inputs_t *)ec_slave[1].inputs;
    memset(outputs, 0, sizeof(*outputs));
    outputs->motor_num = slot + 1;
    outputs->can_ide = 0; /* Standard 11-bit CAN identifier. */
    outputs->motor[slot] = *request;

    /* Hold the read-only request for a few cycles, then make the slot inactive. */
    for (int cycle = 0; cycle < 5 && !encos_stop_requested; ++cycle) {
        if (!encos_exchange(expected_wkc)) {
            fprintf(stderr, "EtherCAT working counter dropped while sending query.\n");
            return false;
        }
        usleep(1000);
    }
    memset(outputs, 0, sizeof(*outputs));

    for (int cycle = 0; cycle < 1000 && !encos_stop_requested; ++cycle) {
        if (!encos_exchange(expected_wkc)) {
            fprintf(stderr, "EtherCAT working counter dropped while awaiting reply.\n");
            return false;
        }
        /* Input entries have no verified source-bus tag. Match the protocol
         * reply across all entries; discovery still requires physical isolation. */
        for (size_t reply_slot = 0; reply_slot < ENCOS_BRIDGE_CHANNELS; ++reply_slot) {
            const encos_can_frame_t *frame = &inputs->motor[reply_slot];
            if (matcher(frame, result)) {
                return true;
            }
        }
        usleep(1000);
    }
    return false;
}

bool encos_query_reply(int expected_wkc, const encos_can_frame_t *request,
                       encos_reply_matcher_t matcher, void *result)
{
    return encos_query_reply_at_slot(expected_wkc, 0, request, matcher, result);
}

static bool match_discovery(const encos_can_frame_t *frame, void *result)
{
    return encos_parse_discovery_reply(frame, result);
}

static bool match_position(const encos_can_frame_t *frame, void *opaque)
{
    encos_position_reply_t *result = opaque;
    return encos_parse_position_reply(frame, result->motor_id, &result->position_deg,
                                      &result->error);
}

bool encos_discover_motor(int expected_wkc, uint16_t *motor_id)
{
    return encos_discover_motor_at_slot(expected_wkc, 0, motor_id);
}

bool encos_discover_motor_at_slot(int expected_wkc, uint8_t slot, uint16_t *motor_id)
{
    encos_can_frame_t request = encos_make_discovery_request();
    return encos_query_reply_at_slot(expected_wkc, slot, &request, match_discovery, motor_id);
}

bool encos_query_position(int expected_wkc, encos_position_reply_t *position)
{
    return encos_query_position_at_slot(expected_wkc, 0, position);
}

bool encos_query_position_at_slot(int expected_wkc, uint8_t slot,
                                  encos_position_reply_t *position)
{
    encos_can_frame_t request = encos_make_query_request(position->motor_id, 1);
    return encos_query_reply_at_slot(expected_wkc, slot, &request, match_position, position);
}
