#ifndef ENCOS_DRIVER_H
#define ENCOS_DRIVER_H
#include <stddef.h>
#include <stdint.h>

/* One process-wide SOEM session, up to four motors in configuration order.
 * Contiguous slots 0-2 route to CAN1; slot 3 (fourth motor) routes to CAN2. */
typedef struct {
    uint16_t motor_id;
    float min_position_deg, max_position_deg;
    float max_velocity_rpm, max_current_a, max_acceleration_rpm_s;
} encos_motor_config_t;

typedef struct {
    float position_deg, velocity_rpm, current_a, temperature_c;
    double observed_monotonic_s;
    uint32_t flags; /* bit 0 position; bit 1 velocity; bit 2 current/temperature */
    uint32_t error;
} encos_motor_state_t;

typedef struct encos_driver encos_driver_t;
/* Returns NULL on invalid config/open failure; never issues motion on open. */
encos_driver_t *encos_driver_open(const char *interface_name,
                                const encos_motor_config_t *config, size_t count);
/* mode 1 position (degrees), mode 2 velocity (RPM). Refresh each active motor
 * within 250 ms, including a stationary position hold. Failure latches globally. */
int encos_driver_command(encos_driver_t *driver, uint16_t id, int mode, float target);
int encos_driver_read(encos_driver_t *driver, uint16_t id, encos_motor_state_t *state);
/* 0 running; 1 command timeout; 2 EtherCAT; 3 feedback/limit; 4 closing. */
int encos_driver_status(encos_driver_t *driver);
/* Best-effort zero-speed commands for 500ms then empty PDOs; not a brake. */
void encos_driver_close(encos_driver_t *driver);
#endif
