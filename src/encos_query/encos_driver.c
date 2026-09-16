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
    int mode;
    float target, sent_rpm;
    double command_time, feedback_time;
    unsigned ack;
} motor_t;

struct encos_driver {
    pthread_t thread;
    pthread_mutex_t mutex;
    motor_t motors[MAX_MOTORS];
    size_t count;
    int fault, expected_wkc, lock_fd;
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
    return ec_receive_processdata(2000) >= d->expected_wkc;
}

static motor_t *find_motor(encos_driver_t *d, uint16_t id)
{
    for (size_t i = 0; i < d->count; ++i)
        if (d->motors[i].config.motor_id == id) return &d->motors[i];
    return NULL;
}

static void read_feedback(encos_driver_t *d, motor_t *m, double now)
{
    const encos_bridge_inputs_t *in = (const encos_bridge_inputs_t *)ec_slave[1].inputs;
    for (size_t slot = 0; slot < ENCOS_BRIDGE_CHANNELS; ++slot) {
        encos_type2_feedback_t p;
        encos_type3_feedback_t v;
        float position;
        uint8_t error;
        const encos_can_frame_t *f = &in->motor[slot];
        bool seen = false;
        if (!m->mode && encos_parse_position_reply(f, m->config.motor_id, &position, &error)) {
            m->state.position_deg = position;
            m->state.error = error;
            m->state.flags |= 1;
            seen = true;
        } else if (m->mode && encos_parse_type2_feedback(f, m->config.motor_id, &p)) {
            m->state.position_deg = p.position_deg;
            m->state.current_a = p.current_a;
            m->state.temperature_c = p.temperature_c;
            m->state.error = p.error;
            m->state.flags |= 5;
            seen = true;
        } else if (m->mode && encos_parse_type3_feedback(f, m->config.motor_id, &v)) {
            m->state.velocity_rpm = v.velocity_rpm;
            m->state.current_a = v.current_a;
            m->state.temperature_c = v.temperature_c;
            m->state.error = v.error;
            m->state.flags |= 6;
            seen = true;
        }
        if (!seen) continue;
        /* Observation time is NOT a proven CAN reception timestamp: the bridge
         * may retain frames. See docs/PYTHON_CONTROL.md before hardware use. */
        m->state.observed_monotonic_s = m->feedback_time = now;
        const float current_limit = m->config.max_current_a +
                                    fmaxf(0.5f, m->config.max_current_a * 0.25f);
        if (m->state.error) {
            fprintf(stderr, "ENCOS fault: motor %u reported error %u.\n",
                    m->config.motor_id, m->state.error);
            d->fault = 3;
            return;
        }
        if (m->state.temperature_c > 70) {
            fprintf(stderr, "ENCOS fault: motor %u temperature %.2f C exceeds 70 C.\n",
                    m->config.motor_id, m->state.temperature_c);
            d->fault = 3;
            return;
        }
        if (fabsf(m->state.current_a) > current_limit) {
            fprintf(stderr, "ENCOS fault: motor %u current %.3f A exceeds %.3f A.\n",
                    m->config.motor_id, m->state.current_a, current_limit);
            d->fault = 3;
            return;
        }
        if (m->mode == 1 &&
            (m->state.position_deg < m->config.min_position_deg ||
             m->state.position_deg > m->config.max_position_deg)) {
            fprintf(stderr, "ENCOS fault: motor %u position %.3f deg outside [%.3f, %.3f].\n",
                    m->config.motor_id, m->state.position_deg,
                    m->config.min_position_deg, m->config.max_position_deg);
            d->fault = 3;
            return;
        }
        if (fabsf(m->state.velocity_rpm) > m->config.max_velocity_rpm + 1.0f) {
            fprintf(stderr, "ENCOS fault: motor %u velocity %.3f RPM exceeds %.3f RPM.\n",
                    m->config.motor_id, m->state.velocity_rpm,
                    m->config.max_velocity_rpm + 1.0f);
            d->fault = 3;
            return;
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
        if (!d->fault && dt > 0.1) {
            fprintf(stderr, "ENCOS fault: native EtherCAT loop gap %.6f s exceeds 0.1 s.\n", dt);
            d->fault = 2;
        }
        for (size_t i = 0; i < d->count; ++i) {
            motor_t *m = &d->motors[i];
            if (!d->fault && m->mode && now - m->command_time > TIMEOUT_S) {
                fprintf(stderr, "ENCOS fault: motor %u command age %.6f s exceeds %.2f s.\n",
                        m->config.motor_id, now - m->command_time, TIMEOUT_S);
                d->fault = 1;
            }
            if (!d->fault && m->mode && now - m->feedback_time > TIMEOUT_S) {
                fprintf(stderr, "ENCOS fault: motor %u feedback age %.6f s exceeds %.2f s.\n",
                        m->config.motor_id, now - m->feedback_time, TIMEOUT_S);
                d->fault = 3;
            }
        }
        if (d->fault && stop_started == 0) stop_started = now;
        memset(out, 0, sizeof(*out));
        if (stop_started && now - stop_started >= 0.5) {
            for (int i = 0; i < 5; ++i) exchange(d);
            pthread_mutex_unlock(&d->mutex);
            break;
        }
        out->motor_num = (uint8_t)d->count;
        for (size_t i = 0; i < d->count; ++i) {
            motor_t *m = &d->motors[i];
            if (!m->mode) {
                out->motor[i] = encos_make_query_request(m->config.motor_id, 1);
            } else {
                m->ack = m->ack == 2 ? 3 : 2;
                if (d->fault || m->mode == 2) {
                    if (d->fault) {
                        m->sent_rpm = 0.0f;
                    } else {
                        const float step = VELOCITY_SLEW_RPM_S * (float)dt;
                        m->sent_rpm += fmaxf(-step,
                            fminf(step, m->target - m->sent_rpm));
                    }
                    encos_make_servo_velocity_request(m->config.motor_id, m->sent_rpm,
                        m->config.max_current_a, (uint8_t)m->ack, &out->motor[i]);
                } else {
                    encos_make_servo_position_request(m->config.motor_id, m->target,
                        m->config.max_velocity_rpm, m->config.max_current_a,
                        (uint8_t)m->ack, &out->motor[i]);
                }
            }
        }
        if (!exchange(d) && !d->fault) {
            fprintf(stderr, "ENCOS fault: EtherCAT working counter dropped below %d.\n",
                    d->expected_wkc);
            d->fault = 2;
        }
        if (!d->fault)
            for (size_t i = 0; i < d->count && !d->fault; ++i)
                read_feedback(d, &d->motors[i], now_s());
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
    if (!interface_name || !*interface_name || strlen(interface_name) > 63 ||
        !config || count == 0 || count > MAX_MOTORS) return NULL;
    for (const char *p = interface_name; *p; ++p)
        if (!isalnum((unsigned char)*p) && !strchr("_.:-", *p)) return NULL;
    for (size_t i = 0; i < count; ++i) {
        const encos_motor_config_t *c = &config[i];
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
    for (size_t i = 0; i < count; ++i) d->motors[i].config = config[i];
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
    /* A late setter cannot revive an expired command before the worker checks. */
    for (size_t i = 0; i < d->count; ++i)
        if (!d->fault && d->motors[i].mode && now - d->motors[i].command_time > TIMEOUT_S)
            d->fault = 1;
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
