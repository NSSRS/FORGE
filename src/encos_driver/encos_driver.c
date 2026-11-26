#define _POSIX_C_SOURCE 200809L
#include "encos_driver.h"
#include "encos_protocol.h"
#include "ethercat.h"
#include <ctype.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <time.h>
#include <unistd.h>

#define MAX_MOTORS 4
#define PERIOD_S 0.01
#define TIMEOUT_S 0.25
#define VELOCITY_SLEW_RPM_S 60.0f

typedef struct {
    encos_motor_config_t config;
    encos_motor_state_t state;
    unsigned slot;
    int mode;
    float target, sent_rpm;
    double command_time, feedback_time;
    double position_observed_s, velocity_observed_s, electrical_observed_s;
    unsigned next_ack, last_control_reply;
    bool request_position;
} motor_t;

struct encos_driver {
    pthread_t thread;
    pthread_mutex_t mutex;
    motor_t motors[MAX_MOTORS];
    size_t count;
    int fault, expected_wkc, lock_fd;
    int last_wkc;
    uint64_t exchanges, failed_exchanges;
    double max_cycle_gap_s;
    uint8_t io_map[4096];
};

static pthread_mutex_t owner_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool owned;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void sleep_until(double deadline)
{
    struct timespec ts = {.tv_sec = (time_t)deadline,
                         .tv_nsec = (long)((deadline - (time_t)deadline) * 1e9)};
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

static int exchange(encos_driver_t *d)
{
    ec_send_processdata();
    d->last_wkc = ec_receive_processdata(2000);
    d->exchanges++;
    if (d->last_wkc < d->expected_wkc) d->failed_exchanges++;
    return d->last_wkc >= d->expected_wkc;
}

static motor_t *find_motor(encos_driver_t *d, uint32_t id)
{
    for (size_t i = 0; i < d->count; ++i)
        if (d->motors[i].config.motor_id == id) return &d->motors[i];
    return NULL;
}

enum { QUERY_POSITION = 1, CONTROL_POSITION = 2, CONTROL_VELOCITY = 3 };

static unsigned other_ack(unsigned ack)
{
    return ack == CONTROL_POSITION ? CONTROL_VELOCITY : CONTROL_POSITION;
}

typedef struct {
    encos_motor_state_t state;
    unsigned reply_mask;
    bool occupied;
} feedback_sample_t;

static bool within_limits(const motor_t *m, const encos_motor_state_t *state)
{
    const unsigned id = m->config.motor_id;
    const float current_limit = m->config.max_current_a + fmaxf(0.5f, m->config.max_current_a * 0.25f);
    if (state->error) {
        fprintf(stderr, "ENCOS fault: motor %u reported error %u.\n", id, state->error);
    } else if (state->temperature_c > 70) {
        fprintf(stderr, "ENCOS fault: motor %u temperature %.2f C exceeds 70 C.\n", id, state->temperature_c);
    } else if (fabsf(state->current_a) > current_limit) {
        fprintf(stderr, "ENCOS fault: motor %u current %.3f A exceeds %.3f A.\n", id, state->current_a, current_limit);
    } else if (m->mode == 1 && (state->position_deg < m->config.min_position_deg ||
                                 state->position_deg > m->config.max_position_deg)) {
        fprintf(stderr, "ENCOS fault: motor %u position %.3f deg outside [%.3f, %.3f].\n",
                id, state->position_deg, m->config.min_position_deg, m->config.max_position_deg);
    } else if (fabsf(state->velocity_rpm) > m->config.max_velocity_rpm + 1.0f) {
        fprintf(stderr, "ENCOS fault: motor %u velocity %.3f RPM exceeds %.3f RPM.\n",
                id, state->velocity_rpm, m->config.max_velocity_rpm);
    } else {
        return true;
    }
    return false;
}

static unsigned decode_feedback(const motor_t *m, const encos_can_frame_t *frame, encos_motor_state_t *state)
{
    unsigned id = m->config.motor_id;
    uint8_t error;
    float position;
    encos_type2_feedback_t p;
    encos_type3_feedback_t v;
    if (!m->mode && encos_parse_position_reply(frame, id, &position, &error)) {
        state->position_deg = position;
        state->error = error;
        state->flags |= 1;
        return QUERY_POSITION;
    }
    if (m->mode && encos_parse_type2_feedback(frame, id, &p)) {
        state->position_deg = p.position_deg;
        state->current_a = p.current_a;
        state->temperature_c = p.temperature_c;
        state->error = p.error;
        state->flags |= 5;
        return CONTROL_POSITION;
    }
    if (m->mode && encos_parse_type3_feedback(frame, id, &v)) {
        state->velocity_rpm = v.velocity_rpm;
        state->current_a = v.current_a;
        state->temperature_c = v.temperature_c;
        state->error = v.error;
        state->flags |= 6;
        return CONTROL_VELOCITY;
    }
    return 0;
}

static void accept_observation(motor_t *m, const feedback_sample_t *sample, double now)
{
    m->state = sample->state;
    m->state.observed_monotonic_s = now;
    if (sample->reply_mask & ((1u << QUERY_POSITION) | (1u << CONTROL_POSITION))) m->position_observed_s = now;
    if (sample->reply_mask & (1u << CONTROL_VELOCITY)) m->velocity_observed_s = now;
    if (m->mode) m->electrical_observed_s = now;
}

static void read_feedback(encos_driver_t *d, double now)
{
    const encos_bridge_inputs_t *in = (const void *)ec_slave[1].inputs;
    if (in->can_ide != 0 || in->motor_num > ENCOS_BRIDGE_CHANNELS) return;
    feedback_sample_t samples[MAX_MOTORS] = {0};
    for (size_t i = 0; i < d->count; ++i) samples[i].state = d->motors[i].state;
    for (size_t slot = 0; slot < ENCOS_BRIDGE_CHANNELS; ++slot) {
        const encos_can_frame_t *frame = &in->motor[slot];
        if (!frame->dlc) continue;
        motor_t *m = find_motor(d, frame->id);
        if (!m) continue;
        feedback_sample_t *sample = &samples[m - d->motors];
        sample->occupied = true;
        unsigned reply = decode_feedback(m, frame, &sample->state);
        if (!reply) continue;
        sample->reply_mask |= 1u << reply;
        /* Check every decoded record, including duplicates and ambiguous images. */
        if (!within_limits(m, &sample->state)) {
            accept_observation(m, sample, now);
            d->fault = 3;
            return;
        }
    }
    for (size_t i = 0; i < d->count; ++i) {
        motor_t *m = &d->motors[i];
        const feedback_sample_t *sample = &samples[i];
        if (!m->mode) {
            bool seen = sample->reply_mask == (1u << QUERY_POSITION);
            bool requested = m->request_position;
            /* Keep an outstanding query through delayed/unrelated replies;
             * re-arm a completed one only when this ID clears. */
            if (seen) m->request_position = false;
            else if (!sample->occupied) m->request_position = true;
            /* An idle query is armed by an empty observation for this ID.
             * Wait for the prior reply to clear before asking again. */
            if (!requested || !seen) continue;
        } else {
            /* Credit one unambiguous control reply type per image. Reordering
             * two retained entries in the same image cannot manufacture progress. */
            unsigned reply = 0;
            if (sample->reply_mask == (1u << CONTROL_POSITION)) reply = CONTROL_POSITION;
            else if (sample->reply_mask == (1u << CONTROL_VELOCITY)) reply = CONTROL_VELOCITY;
            if (!reply || reply == m->last_control_reply) {
                /* Recover from skipped samples or repeated ACKs without adding
                 * a CAN frame. Healthy traffic keeps its pipelined alternation. */
                if (m->last_control_reply && (!reply || now - m->feedback_time >= 2 * PERIOD_S))
                    m->next_ack = other_ack(m->last_control_reply);
                continue;
            }
            m->last_control_reply = reply;
        }
        accept_observation(m, sample, now);
        m->feedback_time = now;
    }
}

static void check_deadlines(encos_driver_t *d, double now)
{
    for (size_t i = 0; !d->fault && i < d->count; ++i) {
        motor_t *m = &d->motors[i];
        if (!m->mode) continue;
        if (now - m->command_time > TIMEOUT_S) {
            fprintf(stderr, "ENCOS fault: motor %u command age %.6f s exceeds %.2f s.\n",
                    m->config.motor_id, now - m->command_time, TIMEOUT_S);
            d->fault = 1;
        } else if (now - m->feedback_time > TIMEOUT_S) {
            fprintf(stderr, "ENCOS fault: motor %u feedback age %.6f s exceeds %.2f s.\n",
                    m->config.motor_id, now - m->feedback_time, TIMEOUT_S);
            d->fault = 3;
        }
    }
}

static void *run(void *context)
{
    encos_driver_t *d = context;
    double previous = now_s(), next = previous, stop_started = 0;
    encos_bridge_outputs_t *out = (encos_bridge_outputs_t *)ec_slave[1].outputs;
    for (;;) {
        pthread_mutex_lock(&d->mutex);
        double now = now_s(), dt = now - previous;
        previous = now;
        if (dt > d->max_cycle_gap_s) d->max_cycle_gap_s = dt;
        if (!d->fault && dt > 0.1) {
            fprintf(stderr, "ENCOS fault: native EtherCAT loop gap %.6f s exceeds 0.1 s.\n", dt);
            d->fault = 2;
        }
        check_deadlines(d, now);
        if (d->fault && stop_started == 0) stop_started = now;
        memset(out, 0, sizeof(*out));
        if (stop_started && now - stop_started >= 0.5) {
            for (int i = 0; i < 5; ++i) exchange(d);
            pthread_mutex_unlock(&d->mutex);
            break;
        }
        /* Motor_Num spans through the highest addressed passage. Unused
         * records stay empty (DLC=0) after memset above. Do not fill gaps with
         * queries: their replies can replace an active motor's control ACK. */
        for (size_t i = 0; i < d->count; ++i)
            if (out->motor_num <= d->motors[i].slot)
                out->motor_num = (uint8_t)(d->motors[i].slot + 1);
        for (size_t i = 0; i < d->count; ++i) {
            motor_t *m = &d->motors[i];
            if (!m->mode) {
                if (m->request_position)
                    out->motor[m->slot] = encos_make_query_request(m->config.motor_id, 1);
            } else {
                uint8_t ack = (uint8_t)m->next_ack;
                m->next_ack = other_ack(ack);
                if (d->fault || m->mode == 2) {
                    if (d->fault) {
                        m->sent_rpm = 0.0f;
                    } else {
                        const float step = VELOCITY_SLEW_RPM_S * (float)dt;
                        m->sent_rpm += fmaxf(-step,
                            fminf(step, m->target - m->sent_rpm));
                    }
                    encos_make_servo_velocity_request(m->config.motor_id, m->sent_rpm,
                        m->config.max_current_a, ack, &out->motor[m->slot]);
                } else {
                    encos_make_servo_position_request(m->config.motor_id, m->target,
                        m->config.max_velocity_rpm, m->config.max_current_a,
                        ack, &out->motor[m->slot]);
                }
            }
        }
        if (!exchange(d) && !d->fault) {
            fprintf(stderr, "ENCOS fault: EtherCAT working counter dropped below %d.\n",
                    d->expected_wkc);
            d->fault = 2;
        }
        if (!d->fault) {
            double observed = now_s();
            check_deadlines(d, observed);
            if (!d->fault) read_feedback(d, observed);
        }
        pthread_mutex_unlock(&d->mutex);
        next += PERIOD_S;
        if (next < now_s()) next = now_s() + PERIOD_S;
        sleep_until(next);
    }
    return NULL;
}

encos_driver_t *encos_driver_open(const char *interface_name,
                                const encos_motor_config_t *config, size_t count)
{
    return encos_driver_open_mapped(interface_name, config, count, NULL);
}

encos_driver_t *encos_driver_open_mapped(const char *interface_name,
    const encos_motor_config_t *config, size_t count, const uint8_t *slots)
{
    if (!interface_name || !*interface_name || strlen(interface_name) > 63 ||
        !config || count == 0 || count > MAX_MOTORS) return NULL;
    for (const char *p = interface_name; *p; ++p)
        if (!isalnum((unsigned char)*p) && !strchr("_.:-", *p)) return NULL;
    for (size_t i = 0; i < count; ++i) {
        const encos_motor_config_t *c = &config[i];
        if (slots) {
            if (slots[i] >= ENCOS_BRIDGE_CHANNELS) return NULL;
            for (size_t j = 0; j < i; ++j) if (slots[i] == slots[j]) return NULL;
        }
        if (c->motor_id == 0 || c->motor_id >= 0x7ff ||
            !isfinite(c->min_position_deg) || !isfinite(c->max_position_deg) ||
            c->min_position_deg >= c->max_position_deg ||
            !isfinite(c->max_velocity_rpm) || c->max_velocity_rpm <= 0 || c->max_velocity_rpm > 3276.7f ||
            !isfinite(c->max_current_a) || c->max_current_a < 0.1f ||
            c->max_current_a > 409.5f) return NULL;
        for (size_t j = 0; j < i; ++j) if (c->motor_id == config[j].motor_id) return NULL;
    }
    pthread_mutex_lock(&owner_mutex);
    if (owned) { pthread_mutex_unlock(&owner_mutex); return NULL; }
    owned = true;
    pthread_mutex_unlock(&owner_mutex);
    encos_driver_t *d = calloc(1, sizeof(*d));
    bool connected = false;
    if (!d) goto release;
    d->lock_fd = -1;
    char lock_path[128];
    snprintf(lock_path, sizeof(lock_path), "/tmp/forge-encos-%s.lock", interface_name);
    d->lock_fd = open(lock_path, O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (d->lock_fd < 0 || flock(d->lock_fd, LOCK_EX | LOCK_NB)) goto fail;
    if (!ec_init(interface_name)) goto fail;
    connected = true;
    if (ec_config_init(FALSE) != 1) goto fail;
    ec_slave[1].CoEdetails &= ~ECT_COEDET_SDOCA;
    /* Reject unexpected layouts before SOEM maps into the bounded IO buffer. */
    int output_bits = 0, input_bits = 0;
    if (!ec_readPDOmap(1, &output_bits, &input_bits) ||
        output_bits != ENCOS_BRIDGE_OUTPUT_BYTES * 8 ||
        input_bits != ENCOS_BRIDGE_INPUT_BYTES * 8) goto fail;
    ec_config_map(d->io_map);
    ec_configdc();
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[1].Obytes != ENCOS_BRIDGE_OUTPUT_BYTES ||
        ec_slave[1].Ibytes != ENCOS_BRIDGE_INPUT_BYTES) goto fail;
    d->expected_wkc = ec_group[0].outputsWKC * 2 + ec_group[0].inputsWKC;
    memset(ec_slave[1].outputs, 0, ENCOS_BRIDGE_OUTPUT_BYTES);
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    exchange(d);
    ec_writestate(0);
    for (int tries = 0; tries < 40; ++tries) {
        exchange(d);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) break;
    }
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) goto fail;
    d->count = count;
    for (size_t i = 0; i < count; ++i) {
        d->motors[i].config = config[i];
        d->motors[i].next_ack = CONTROL_POSITION;
        d->motors[i].slot = slots ? slots[i] : (unsigned)i;
    }
    if (pthread_mutex_init(&d->mutex, NULL)) goto fail;
    if (pthread_create(&d->thread, NULL, run, d)) {
        pthread_mutex_destroy(&d->mutex);
        goto fail;
    }
    return d;
fail:
    if (connected) ec_close();
    if (d->lock_fd >= 0) close(d->lock_fd);
    free(d);
release:
    pthread_mutex_lock(&owner_mutex);
    owned = false;
    pthread_mutex_unlock(&owner_mutex);
    return NULL;
}

int encos_driver_command(encos_driver_t *d, uint16_t id, int mode, float target)
{
    if (!d || !isfinite(target) || (mode != 1 && mode != 2)) return -1;
    pthread_mutex_lock(&d->mutex);
    motor_t *m = find_motor(d, id);
    int result = -1;
    double now = now_s();
    /* Setters and the worker latch the same first expired deadline. */
    check_deadlines(d, now);
    if (m && !d->fault && (m->state.flags & 1) && now - m->feedback_time <= TIMEOUT_S &&
        ((mode == 1 && target >= m->config.min_position_deg && target <= m->config.max_position_deg) ||
         (mode == 2 && fabsf(target) <= m->config.max_velocity_rpm))) {
        if (m->mode != mode)
            m->sent_rpm = (m->state.flags & 2) ? m->state.velocity_rpm : 0.0f;
        if (!m->mode) m->feedback_time = now;
        m->mode = mode;
        m->target = target;
        m->command_time = now;
        result = 0;
    }
    pthread_mutex_unlock(&d->mutex);
    return result;
}

int encos_driver_read(encos_driver_t *d, uint16_t id, encos_motor_state_t *state)
{
    if (!d || !state) return -1;
    pthread_mutex_lock(&d->mutex);
    motor_t *m = find_motor(d, id);
    if (m) *state = m->state;
    pthread_mutex_unlock(&d->mutex);
    return m ? 0 : -1;
}

int encos_driver_snapshot(encos_driver_t *d, encos_snapshot_t *snapshot, size_t size)
{
    if (!d || !snapshot || size != sizeof(*snapshot)) return -1;
    pthread_mutex_lock(&d->mutex);
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->api_version = 1;
    snapshot->motor_count = (uint32_t)d->count;
    snapshot->fault = (uint32_t)d->fault;
    snapshot->exchanges = d->exchanges;
    snapshot->failed_exchanges = d->failed_exchanges;
    snapshot->last_wkc = d->last_wkc;
    snapshot->expected_wkc = d->expected_wkc;
    snapshot->captured_monotonic_s = now_s();
    snapshot->max_cycle_gap_s = d->max_cycle_gap_s;
    for (size_t i = 0; i < d->count; ++i) {
        const motor_t *m = &d->motors[i];
        encos_motor_telemetry_t *out = &snapshot->motors[i];
        out->state = m->state;
        out->motor_id = m->config.motor_id;
        out->slot = m->slot;
        out->position_observed_s = m->position_observed_s;
        out->velocity_observed_s = m->velocity_observed_s;
        out->electrical_observed_s = m->electrical_observed_s;
    }
    pthread_mutex_unlock(&d->mutex);
    return 0;
}

int encos_driver_status(encos_driver_t *d)
{
    if (!d) return -1;
    pthread_mutex_lock(&d->mutex);
    int fault = d->fault;
    pthread_mutex_unlock(&d->mutex);
    return fault;
}

void encos_driver_close(encos_driver_t *d)
{
    if (!d) return;
    pthread_mutex_lock(&d->mutex);
    if (!d->fault) d->fault = 4;
    pthread_mutex_unlock(&d->mutex);
    pthread_join(d->thread, NULL);
    ec_close();
    close(d->lock_fd);
    pthread_mutex_destroy(&d->mutex);
    free(d);
    pthread_mutex_lock(&owner_mutex);
    owned = false;
    pthread_mutex_unlock(&owner_mutex);
}
