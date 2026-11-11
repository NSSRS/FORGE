#include <errno.h>
#include <math.h>
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

#define QUERY_CYCLE_US 1000
#define MOTION_CYCLE_US 1000
#define SPEED_CEILING_RPM 5.0f
#define SETTLE_CYCLES 500
#define END_HOLD_CYCLES 500
#define RAMP_CYCLES 2000
#define LOG_EVERY_CYCLES 10

typedef struct {
    float initial_position;
    encos_type2_feedback_t feedback;
    bool feedback_seen;
    float maximum_excursion;
} motor_state_t;

static void clear_outputs(encos_bridge_outputs_t *outputs, int expected_wkc)
{
    if (outputs == NULL) {
        return;
    }
    memset(outputs, 0, sizeof(*outputs));
    for (int cycle = 0; cycle < 5; ++cycle) {
        encos_exchange(expected_wkc);
        usleep(QUERY_CYCLE_US);
    }
}

static bool find_type2_feedback(const encos_bridge_inputs_t *inputs, uint16_t motor_id,
                                encos_type2_feedback_t *feedback)
{
    for (size_t slot = 0; slot < ENCOS_BRIDGE_CHANNELS; ++slot) {
        if (encos_parse_type2_feedback(&inputs->motor[slot], motor_id, feedback)) {
            return true;
        }
    }
    return false;
}

static float target_for_cycle(int cycle, float initial_position, float move_degrees,
                              int ramp_cycles)
{
    if (cycle < SETTLE_CYCLES) {
        return initial_position;
    }
    cycle -= SETTLE_CYCLES;
    if (cycle < ramp_cycles) {
        return initial_position + move_degrees * (cycle + 1) / ramp_cycles;
    }
    cycle -= ramp_cycles;
    if (cycle < END_HOLD_CYCLES) {
        return initial_position + move_degrees;
    }
    cycle -= END_HOLD_CYCLES;
    if (cycle < ramp_cycles) {
        return initial_position + move_degrees * (1.0f - (float)(cycle + 1) / ramp_cycles);
    }
    return initial_position;
}

static bool parse_current_limit(const char *text, float *current_limit)
{
    char *end = NULL;
    errno = 0;
    const float value = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(value) ||
        value < 0.1f || value > 6.5f) {
        return false;
    }
    *current_limit = value;
    return true;
}

int main(int argc, char **argv)
{
    bool discover = argc == 5 && strcmp(argv[4], "--execute") == 0;
    uint16_t motor_ids[ENCOS_MAX_CONTROLLED_MOTORS] = {0};
    motor_state_t motors[ENCOS_MAX_CONTROLLED_MOTORS] = {0};
    size_t motor_count = 0;
    if (!discover && (argc != 7 || strcmp(argv[4], "--motor-ids") != 0 ||
                      strcmp(argv[6], "--execute") != 0 ||
                      !encos_parse_motor_ids(argv[5], motor_ids, &motor_count))) {
        fprintf(stderr, "Usage: sudo %s IFACE CURRENT_A MOVE_DEG [--motor-ids ID[,ID...]] --execute\n", argv[0]);
        fprintf(stderr, "Explicit IDs support one to four motors; legacy form discovers one motor.\n");
        return 2;
    }
    float current_limit = 0.0f;
    if (!parse_current_limit(argv[2], &current_limit)) { fprintf(stderr, "Invalid current limit.\n"); return 2; }
    char *move_end = NULL; errno = 0; const float move_degrees = strtof(argv[3], &move_end);
    if (errno != 0 || move_end == argv[3] || *move_end != '\0' || !isfinite(move_degrees) ||
        move_degrees < 0.1f || move_degrees > 45.0f) { fprintf(stderr, "Invalid move; required +0.1 to +45 degrees.\n"); return 2; }
    signal(SIGINT, encos_request_stop); signal(SIGTERM, encos_request_stop);
    int expected_wkc = 0;
    if (!encos_open_cli_bridge(argv[1], &expected_wkc)) { ec_close(); return 1; }
    encos_bridge_outputs_t *outputs = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    const encos_bridge_inputs_t *inputs = (const encos_bridge_inputs_t *)ec_slave[1].inputs;
    if (discover) {
        if (!encos_discover_motor(expected_wkc, &motor_ids[0])) {
            fprintf(stderr, "Could not discover one motor. Use --motor-ids with multiple powered motors.\n"); goto fail;
        }
        motor_count = 1;
    }
    for (size_t i = 0; i < motor_count; ++i) {
        encos_position_reply_t reply = {.motor_id = motor_ids[i]};
        if (!encos_query_position(expected_wkc, &reply) || reply.error) {
            fprintf(stderr, "Motor %u has no fault-free initial position (error %u).\n",
                    motor_ids[i], reply.error);
            goto fail;
        }
        motors[i].initial_position = reply.position_deg;
        printf("Motor %u initial position %.6f deg.\n", motor_ids[i], motors[i].initial_position);
    }
    printf("Test: %zu motors, +%.1f deg over %.1f s, return over %.1f s; target 1000 Hz; speed 5 rpm; current %.2f A.\n", motor_count, move_degrees, RAMP_CYCLES * MOTION_CYCLE_US / 1000000.0f, RAMP_CYCLES * MOTION_CYCLE_US / 1000000.0f, current_limit);
    printf("time_s,motor_id,target_deg,position_deg,current_a,temperature_c,error\n");
    const int total = SETTLE_CYCLES + RAMP_CYCLES + END_HOLD_CYCLES + RAMP_CYCLES + END_HOLD_CYCLES;
    int result = 0;
    for (int cycle = 0; cycle < total && !encos_stop_requested; ++cycle) {
        memset(outputs, 0, sizeof(*outputs)); outputs->motor_num = (uint8_t)motor_count; outputs->can_ide = 0;
        for (size_t i = 0; i < motor_count; ++i) {
            const float target = target_for_cycle(cycle, motors[i].initial_position, move_degrees, RAMP_CYCLES);
            if (!encos_make_servo_position_request(motor_ids[i], target, SPEED_CEILING_RPM, current_limit, 2, &outputs->motor[i])) {
                fprintf(stderr, "Internal command validation failed.\n"); result = 1; break;
            }
        }
        if (result || !encos_exchange(expected_wkc)) { fprintf(stderr, "EtherCAT exchange failed; stopping output.\n"); result = 1; break; }
        for (size_t i = 0; i < motor_count; ++i) {
            encos_type2_feedback_t latest = {0};
            const float target = target_for_cycle(cycle, motors[i].initial_position, move_degrees, RAMP_CYCLES);
            if (find_type2_feedback(inputs, motor_ids[i], &latest)) {
                motors[i].feedback = latest; motors[i].feedback_seen = true;
                const float excursion = fabsf(latest.position_deg - motors[i].initial_position);
                if (excursion > motors[i].maximum_excursion) motors[i].maximum_excursion = excursion;
                const float margin = fmaxf(0.5f, current_limit * 0.25f);
                if (latest.error != 0 || latest.temperature_c > 70.0f || fabsf(latest.current_a) > current_limit + margin ||
                    excursion > move_degrees + 2.0f || fabsf(latest.position_deg - target) > 5.0f) {
                    fprintf(stderr, "Motor %u feedback limit exceeded; stopping output.\n", motor_ids[i]); result = 1; break;
                }
            } else if (cycle >= 500) { fprintf(stderr, "Motor %u has no type-2 feedback within 0.5 seconds; stopping output.\n", motor_ids[i]); result = 1; break; }
            if (cycle == SETTLE_CYCLES + 1000 && motors[i].maximum_excursion < 0.1f) { fprintf(stderr, "Motor %u did not move.\n", motor_ids[i]); result = 1; break; }
            if (cycle % LOG_EVERY_CYCLES == 0 && motors[i].feedback_seen) printf("%.3f,%u,%.6f,%.6f,%.3f,%.1f,%u\n", cycle * MOTION_CYCLE_US / 1000000.0f, motor_ids[i], target, latest.position_deg, latest.current_a, latest.temperature_c, latest.error);
        }
        if (result) break;
        fflush(stdout); usleep(MOTION_CYCLE_US);
    }
    if (encos_stop_requested) { fprintf(stderr, "Interrupted; stopping output.\n"); result = 1; }
    for (size_t i = 0; i < motor_count; ++i)
        if (!motors[i].feedback_seen || motors[i].maximum_excursion < move_degrees * 0.8f || fabsf(motors[i].feedback.position_deg - motors[i].initial_position) > 1.0f) result = 1;
    clear_outputs(outputs, expected_wkc); ec_close();
    if (!result) printf("PASS: %zu bounded motions completed and returned near initial positions.\n", motor_count);
    return result;
fail:
    clear_outputs(outputs, expected_wkc); ec_close(); return 1;
}
