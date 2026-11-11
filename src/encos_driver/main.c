#include <signal.h>
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

typedef struct {
    uint16_t motor_id;
    bool released;
    uint8_t error;
} brake_result_t;

static bool match_brake_status(const encos_can_frame_t *frame, void *opaque)
{
    brake_result_t *result = opaque;
    return encos_parse_brake_status_reply(frame, result->motor_id, &result->released,
                                          &result->error);
}


static const char *motor_error_name(uint8_t error)
{
    switch (error) {
    case 0: return "none";
    case 1: return "overtemperature";
    case 2: return "overcurrent";
    case 3: return "overvoltage";
    case 4: return "undervoltage";
    case 5: return "encoder fault";
    case 6: return "brake overvoltage";
    case 7: return "driver fault";
    default: return "unknown motor error";
    }
}

int main(int argc, char **argv)
{
    uint16_t motor_ids[ENCOS_MAX_CONTROLLED_MOTORS] = {0};
    size_t motor_count = 0;
    const bool discover = argc == 2;
    if (!discover && (argc != 4 || strcmp(argv[2], "--motor-ids") != 0 ||
                      !encos_parse_motor_ids(argv[3], motor_ids, &motor_count))) {
        fprintf(stderr, "Usage: sudo %s ETHERNET_INTERFACE [--motor-ids ID[,ID...]]\n", argv[0]);
        fprintf(stderr, "Use one to four explicit CAN IDs when more than one motor is powered.\n");
        return 2;
    }
    signal(SIGINT, encos_request_stop);
    signal(SIGTERM, encos_request_stop);

    int expected_wkc = 0;
    if (!encos_open_cli_bridge(argv[1], &expected_wkc)) {
        ec_close();
        return 1;
    }
    encos_bridge_outputs_t *outputs = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    printf("Bridge operational: OUT=86 IN=92 WKC=%d. Query-only mode.\n", expected_wkc);

    if (discover) {
        if (!encos_discover_motor(expected_wkc, &motor_ids[0])) {
            fprintf(stderr, "No valid motor-ID reply within 1 second.\n");
            goto fail;
        }
        motor_count = 1;
    }
    printf("Configured motor count: %zu\n", motor_count);
    int result = 0;
    for (size_t index = 0; index < motor_count; ++index) {
        const uint16_t motor_id = motor_ids[index];
        encos_position_reply_t position = {.motor_id = motor_id};
        version_result_t version = {.motor_id = motor_id};
        timeout_result_t timeout = {.motor_id = motor_id};
        brake_result_t brake = {.motor_id = motor_id};
        if (!encos_query_position(expected_wkc, &position)) {
            fprintf(stderr, "Motor %u: no valid position reply.\n", motor_id);
            result = 1;
            continue;
        }
        if (position.error != 0) {
            fprintf(stderr, "Motor %u: error %u (%s) in position reply.\n", motor_id,
                    position.error, motor_error_name(position.error));
            result = 1;
            continue;
        }
        encos_can_frame_t request = encos_make_query_request(motor_id, 30);
        if (!encos_query_reply(expected_wkc, &request, match_version, &version)) {
            fprintf(stderr, "Motor %u: no valid version reply.\n", motor_id);
            result = 1;
            continue;
        }
        if (version.error != 0) {
            fprintf(stderr, "Motor %u: error %u (%s) in version reply.\n", motor_id,
                    version.error, motor_error_name(version.error));
            result = 1;
            continue;
        }
        request = encos_make_query_request(motor_id, 31);
        if (!encos_query_reply(expected_wkc, &request, match_timeout, &timeout)) {
            fprintf(stderr, "Motor %u: no valid timeout reply.\n", motor_id);
            result = 1;
            continue;
        }
        if (timeout.error != 0) {
            fprintf(stderr, "Motor %u: error %u (%s) in timeout reply.\n", motor_id,
                    timeout.error, motor_error_name(timeout.error));
            result = 1;
            continue;
        }
        request = encos_make_query_request(motor_id, 37);
        const bool brake_known = encos_query_reply(expected_wkc, &request, match_brake_status, &brake) &&
                                 brake.error == 0;
        printf("Motor %u: position %.6f deg; HW %u.%u.%u; SW %u.%u.%u; CAN timeout %u ms; brake %s.\n",
               motor_id, position.position_deg, version.version.hardware[0], version.version.hardware[1],
               version.version.hardware[2], version.version.software[0], version.version.software[1],
               version.version.software[2], timeout.timeout_ms,
               brake_known ? (brake.released ? "released" : "engaged") : "unknown");
    }
    memset(outputs, 0, sizeof(*outputs)); encos_exchange(expected_wkc); ec_close(); return result;
fail:
    memset(outputs, 0, sizeof(*outputs)); encos_exchange(expected_wkc); ec_close(); return 1;
}
