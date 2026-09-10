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
#include "encos_protocol.h"

#define QUERY_CYCLE_US 1000
#define MOTION_CYCLE_US 10000
#define SPEED_CEILING_RPM 1.0f
#define SETTLE_CYCLES 50
#define END_HOLD_CYCLES 50
#define RAMP_CYCLES 1000

static volatile sig_atomic_t stop_requested;
static uint8_t io_map[4096];

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

static bool enter_operational(const char *interface_name, int *expected_wkc)
{
    if (!ec_init(interface_name)) {
        fprintf(stderr, "Could not open EtherCAT interface %s. Run with sudo.\n", interface_name);
        return false;
    }
    if (ec_config_init(FALSE) != 1) {
        fprintf(stderr, "Expected exactly one EtherCAT bridge; found %d.\n", ec_slavecount);
        return false;
    }
    ec_slave[1].CoEdetails &= ~ECT_COEDET_SDOCA;
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
    exchange(*expected_wkc);
    ec_writestate(0);
    for (int tries = 0; tries < 40; ++tries) {
        exchange(*expected_wkc);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
            return true;
        }
    }
    fprintf(stderr, "Bridge did not reach EtherCAT OPERATIONAL state.\n");
    return false;
}

static void clear_outputs(encos_bridge_outputs_t *outputs, int expected_wkc)
{
    if (outputs == NULL) {
        return;
    }
    memset(outputs, 0, sizeof(*outputs));
    for (int cycle = 0; cycle < 5; ++cycle) {
        exchange(expected_wkc);
        usleep(QUERY_CYCLE_US);
    }
}

static bool find_discovery(const encos_bridge_inputs_t *inputs, uint16_t *motor_id)
{
    for (size_t slot = 0; slot < ENCOS_BRIDGE_CHANNELS; ++slot) {
        if (encos_parse_discovery_reply(&inputs->motor[slot], motor_id)) {
            return true;
        }
    }
    return false;
}

static bool find_position_query(const encos_bridge_inputs_t *inputs, uint16_t motor_id,
                                float *position_deg, uint8_t *error)
{
    for (size_t slot = 0; slot < ENCOS_BRIDGE_CHANNELS; ++slot) {
        if (encos_parse_position_reply(&inputs->motor[slot], motor_id, position_deg, error)) {
            return true;
        }
    }
    return false;
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

static bool pulse_query(encos_bridge_outputs_t *outputs, int expected_wkc,
                        const encos_can_frame_t *request)
{
    memset(outputs, 0, sizeof(*outputs));
    outputs->motor_num = 1;
    outputs->can_ide = 0;
    outputs->motor[0] = *request;
    for (int cycle = 0; cycle < 5; ++cycle) {
        if (!exchange(expected_wkc)) {
            return false;
        }
        usleep(QUERY_CYCLE_US);
    }
    memset(outputs, 0, sizeof(*outputs));
    return true;
}

static bool read_motor_id(encos_bridge_outputs_t *outputs,
                          const encos_bridge_inputs_t *inputs, int expected_wkc,
                          uint16_t *motor_id)
{
    const encos_can_frame_t request = encos_make_discovery_request();
    if (!pulse_query(outputs, expected_wkc, &request)) {
        return false;
    }
    for (int cycle = 0; cycle < 1000 && !stop_requested; ++cycle) {
        if (!exchange(expected_wkc)) {
            return false;
        }
        if (find_discovery(inputs, motor_id)) {
            return true;
        }
        usleep(QUERY_CYCLE_US);
    }
    return false;
}

static bool read_position(encos_bridge_outputs_t *outputs,
                          const encos_bridge_inputs_t *inputs, int expected_wkc,
                          uint16_t motor_id, float *position_deg)
{
    const encos_can_frame_t request = encos_make_query_request(motor_id, 1);
    if (!pulse_query(outputs, expected_wkc, &request)) {
        return false;
    }
    for (int cycle = 0; cycle < 1000 && !stop_requested; ++cycle) {
        uint8_t error = 0;
        if (!exchange(expected_wkc)) {
            return false;
        }
        if (find_position_query(inputs, motor_id, position_deg, &error)) {
            if (error != 0) {
                fprintf(stderr, "Motor reported error %u in initial position reply.\n", error);
                return false;
            }
            return true;
        }
        usleep(QUERY_CYCLE_US);
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
    if (argc != 5 || strcmp(argv[4], "--execute") != 0) {
        fprintf(stderr, "Usage: sudo %s ETHERNET_INTERFACE CURRENT_LIMIT_A MOVE_DEG --execute\n", argv[0]);
        fprintf(stderr, "Current must be 0.1 to 6.5 A and approved for the motor.\n");
        return 2;
    }
    float current_limit = 0.0f;
    if (!parse_current_limit(argv[2], &current_limit)) {
        fprintf(stderr, "Invalid current limit; required range is 0.1 to 6.5 A.\n");
        return 2;
    }
    char *move_end = NULL;
    errno = 0;
    const float move_degrees = strtof(argv[3], &move_end);
    if (errno != 0 || move_end == argv[3] || *move_end != '\0' ||
        !isfinite(move_degrees) || move_degrees < 0.1f || move_degrees > 45.0f) {
        fprintf(stderr, "Invalid move; required range is +0.1 to +45.0 degrees.\n");
        return 2;
    }
    const int ramp_cycles = RAMP_CYCLES;
    const int total_motion_cycles = SETTLE_CYCLES + ramp_cycles + END_HOLD_CYCLES +
                                    ramp_cycles + END_HOLD_CYCLES;
    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);

    int expected_wkc = 0;
    if (!enter_operational(argv[1], &expected_wkc)) {
        ec_close();
        return 1;
    }
    encos_bridge_outputs_t *outputs = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    const encos_bridge_inputs_t *inputs = (const encos_bridge_inputs_t *)ec_slave[1].inputs;

    uint16_t motor_id = 0;
    float initial_position = 0.0f;
    if (!read_motor_id(outputs, inputs, expected_wkc, &motor_id) ||
        !read_position(outputs, inputs, expected_wkc, motor_id, &initial_position)) {
        fprintf(stderr, "Could not establish fault-free motor telemetry. Motion not started.\n");
        clear_outputs(outputs, expected_wkc);
        ec_close();
        return 1;
    }

    printf("Motor %u initial position %.6f deg.\n", motor_id, initial_position);
    printf("Test: +%.1f deg over %.1f s, hold 0.5 s, return over %.1f s; speed 1 rpm; current %.2f A.\n",
           move_degrees, ramp_cycles / 100.0f, ramp_cycles / 100.0f, current_limit);
    printf("time_s,target_deg,position_deg,current_a,temperature_c,error\n");

    encos_type2_feedback_t feedback = {0};
    bool feedback_seen = false;
    float maximum_excursion = 0.0f;
    int result = 0;
    for (int cycle = 0; cycle < total_motion_cycles && !stop_requested; ++cycle) {
        const float target = target_for_cycle(cycle, initial_position, move_degrees,
                                              ramp_cycles);
        encos_can_frame_t command = {0};
        if (!encos_make_servo_position_request(motor_id, target, SPEED_CEILING_RPM,
                                               current_limit, 2, &command)) {
            fprintf(stderr, "Internal command validation failed.\n");
            result = 1;
            break;
        }
        memset(outputs, 0, sizeof(*outputs));
        outputs->motor_num = 1;
        outputs->can_ide = 0;
        outputs->motor[0] = command;
        if (!exchange(expected_wkc)) {
            fprintf(stderr, "EtherCAT working counter dropped; stopping output.\n");
            result = 1;
            break;
        }

        encos_type2_feedback_t latest = {0};
        if (find_type2_feedback(inputs, motor_id, &latest)) {
            feedback = latest;
            feedback_seen = true;
            const float excursion = fabsf(feedback.position_deg - initial_position);
            if (excursion > maximum_excursion) {
                maximum_excursion = excursion;
            }
            const float current_margin = fmaxf(0.5f, current_limit * 0.25f);
            if (feedback.error != 0 || feedback.temperature_c > 70.0f ||
                fabsf(feedback.current_a) > current_limit + current_margin ||
                excursion > move_degrees + 2.0f ||
                fabsf(feedback.position_deg - target) > 5.0f) {
                fprintf(stderr, "Feedback limit exceeded; stopping output.\n");
                result = 1;
                break;
            }
        } else if (cycle >= 50) {
            fprintf(stderr, "No type-2 motor feedback within 0.5 seconds; stopping output.\n");
            result = 1;
            break;
        }

        if (cycle == SETTLE_CYCLES + 100 && maximum_excursion < 0.1f) {
            fprintf(stderr, "No measured motion after 1 second of ramp; stopping output.\n");
            result = 1;
            break;
        }
        if (cycle % 10 == 0 && feedback_seen) {
            printf("%.2f,%.6f,%.6f,%.3f,%.1f,%u\n", cycle / 100.0f, target,
                   feedback.position_deg, feedback.current_a, feedback.temperature_c,
                   feedback.error);
            fflush(stdout);
        }
        usleep(MOTION_CYCLE_US);
    }

    if (stop_requested) {
        fprintf(stderr, "Interrupted; stopping output.\n");
        result = 1;
    }
    if (result == 0 && (!feedback_seen || maximum_excursion < move_degrees * 0.8f ||
                        fabsf(feedback.position_deg - initial_position) > 1.0f)) {
        fprintf(stderr, "Motion/return acceptance check failed.\n");
        result = 1;
    }

    clear_outputs(outputs, expected_wkc);
    ec_close();
    if (result == 0) {
        printf("PASS: bounded motion completed and returned near the initial position.\n");
    }
    return result;
}
