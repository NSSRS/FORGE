#ifndef FORGE_ARM_H
#define FORGE_ARM_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define ARM_COUNT 6
#define ARM_CAP_RATED_4340_TORQUE (1u << 13) /* Rated 12 Nm configuration ceiling. */
#define ARM_CAP_CONTROL_READBACK (1u << 14) /* Disabled-only control/protection reads. */
#define ARM_CAP_MANUAL_TRAVEL (1u << 15) /* Explicit mode-2 manual travel, no startup corridor. */
#define ARM_ROUTED_MANUAL_TRAVEL 0x80u /* USB configure mode flag; never sent to motor. */
#define ARM_CAPABILITIES (8079u | ARM_CAP_RATED_4340_TORQUE | ARM_CAP_CONTROL_READBACK | ARM_CAP_MANUAL_TRAVEL)
#define ARM_BASE_API_VERSION 1u
#ifndef FORGE_BUILD_ID
#define FORGE_BUILD_ID 0u /* native tests/nonstandard builds */
#endif
#define ARM_CONTROL_PERIOD_MS 2u /* 500 Hz CAN updates; host lease is independent */
#define ARM_LEASE_MS 150u
#define ARM_FEEDBACK_MS 100u
#define ARM_ENABLE_CONFIRM_MS 250u
/* All units are output-shaft rad, rad/s and Nm. Never ENCOS units/packets. */
typedef struct { float p, v, t; } dm_range;
typedef struct { float p, v, kp, kd, t; } dm_command;
typedef struct { uint8_t bus; uint16_t id, feedback_id; float lo, hi, speed, torque, kp, kd; } arm_config;
typedef bool (*arm_send_fn)(void *, uint8_t, uint16_t, const uint8_t *, uint8_t);
typedef struct {
    arm_config cfg; dm_range range; dm_command cmd;
    float p, v, t; uint32_t feedback_at, command_at, enabled_at;
    uint8_t registers, status, mos, rotor, probe_index, bench_mode, model;
    bool configured, probing, feedback_seen, command_seen, enabled, stopping, enable_confirmed, manual_travel;
} arm_joint;
/* Diagnostics: unconfigured bus or fresh, acknowledged-disabled bench motor. */
typedef struct {
    uint8_t state, bus, reg, opcode; /* 0 idle, 1 pending, 2 reply, 3 timeout, 4 TX failure */
    uint16_t id, feedback_id; uint32_t value, started;
} arm_diagnostic;
typedef struct {
    uint32_t count, at; uint16_t id; uint8_t len, data[8];
} arm_rx_trace;
typedef struct {
    uint32_t tx_frames, tx_failed, feedback_frames, max_feedback_gap_ms;
    uint32_t bus_off_events, status_flags, tx_error_count, rx_error_count;
} arm_bus_health;
typedef struct {
    arm_bus_health health[3];
    uint32_t max_tick_gap_ms, late_ticks;
    arm_rx_trace rx_trace[3];
    bool save_quiet; uint32_t save_started;
    arm_diagnostic diagnostic;
    arm_joint joints[ARM_COUNT]; arm_send_fn send; void *context;
    uint8_t fault_reason, fault_status, fault_slot;
    uint32_t fault_command_age, fault_feedback_age, fault_at;
    /* First-fault sample survives subsequent disable feedback. */
    float fault_p, fault_v, fault_t, fault_target, fault_velocity, fault_speed_limit, fault_torque_limit;
    uint8_t fault_sample_flags;
    bool id_commissioning; uint8_t id_verified;
    bool fault, bench, bench_posvel, routed_bench; uint32_t last_tick, bench_enabled_at;
} arm_controller;
bool dm_pack(const dm_range *, const dm_command *, uint8_t out[8]);
void arm_init(arm_controller *, arm_send_fn, void *);
bool arm_prepare_routed(arm_controller *, unsigned, const arm_config *, uint8_t, uint8_t);
bool arm_id_operation(arm_controller *, uint8_t, uint32_t, uint32_t);
bool arm_routed_write(arm_controller *, unsigned, uint8_t, uint32_t, uint32_t);
bool arm_bench_timeout(arm_controller *, uint32_t);
bool arm_prepare_bench(arm_controller *, float, float);
bool arm_prepare_posvel(arm_controller *, float, float);
bool arm_bench_mode(arm_controller *, uint8_t, uint32_t);
bool dm_pack_posvel(float, float, uint8_t out[8]);
bool arm_read_register(arm_controller *, uint8_t, uint16_t, uint8_t, uint32_t);
bool arm_configure(arm_controller *, unsigned, const arm_config *);
bool arm_probe(arm_controller *, unsigned);
bool arm_set(arm_controller *, unsigned, const dm_command *, uint32_t);
bool arm_enable_group(arm_controller *, uint8_t, uint32_t);
bool arm_enable(arm_controller *, unsigned, uint32_t);
void arm_stop(arm_controller *);
void arm_fault(arm_controller *, uint8_t, unsigned, uint32_t);
bool arm_reset(arm_controller *);
void arm_tick(arm_controller *, uint32_t);
void arm_receive(arm_controller *, uint8_t, uint16_t, const uint8_t *, uint8_t, uint32_t);
#endif
