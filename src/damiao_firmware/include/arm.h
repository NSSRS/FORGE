#ifndef FORGE_ARM_H
#define FORGE_ARM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define ARM_COUNT 6
#define ARM_CONTROL_PERIOD_MS 2u /* 500 Hz CAN updates; host lease is independent */
#define ARM_LEASE_MS 150u
#define ARM_FEEDBACK_MS 100u
/* All units are output-shaft rad, rad/s and Nm. Never ENCOS units/packets. */
typedef struct { float p, v, t; } dm_range;
typedef struct { float p, v, kp, kd, t; } dm_command;
typedef struct { uint8_t bus; uint16_t id, feedback_id; float lo, hi, speed, torque, kp, kd; } arm_config;
typedef bool (*arm_send_fn)(void *, uint8_t, uint16_t, const uint8_t *, uint8_t);
typedef struct {
    arm_config cfg; dm_range range; dm_command cmd;
    float p, v, t; uint32_t feedback_at, command_at, enabled_at;
    uint8_t registers, status, mos, rotor, probe_index;
    bool configured, probing, feedback_seen, command_seen, enabled, stopping;
} arm_joint;
typedef struct {
    arm_joint joints[ARM_COUNT]; arm_send_fn send; void *context;
    bool fault; uint32_t last_tick;
} arm_controller;
bool dm_pack(const dm_range *, const dm_command *, uint8_t out[8]);
void arm_init(arm_controller *, arm_send_fn, void *);
bool arm_configure(arm_controller *, unsigned, const arm_config *);
bool arm_probe(arm_controller *, unsigned);
bool arm_set(arm_controller *, unsigned, const dm_command *, uint32_t);
bool arm_enable(arm_controller *, unsigned, uint32_t);
void arm_stop(arm_controller *);
bool arm_reset(arm_controller *);
void arm_tick(arm_controller *, uint32_t);
void arm_receive(arm_controller *, uint8_t, uint16_t, const uint8_t *, uint8_t, uint32_t);
#endif
