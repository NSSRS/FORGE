#ifndef ENCOS_PROTOCOL_H
#define ENCOS_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "bridge_pdo.h"

#define ENCOS_DISCOVERY_CAN_ID 0x7ffu

typedef struct {
    uint8_t hardware[3];
    uint8_t software[3];
} encos_version_t;

typedef struct {
    float position_deg;
    float current_a;
    float temperature_c;
    uint8_t error;
} encos_type2_feedback_t;

typedef struct {
    float velocity_rpm;
    float current_a;
    float temperature_c;
    uint8_t error;
} encos_type3_feedback_t;

bool encos_make_servo_velocity_request(uint16_t motor_id, float velocity_rpm,
                                      float current_ceiling_a, uint8_t feedback_type,
                                      encos_can_frame_t *frame);
bool encos_parse_type3_feedback(const encos_can_frame_t *frame, uint16_t motor_id,
                                encos_type3_feedback_t *feedback);

encos_can_frame_t encos_make_discovery_request(void);
encos_can_frame_t encos_make_query_request(uint16_t motor_id, uint8_t query_code);
bool encos_make_servo_position_request(uint16_t motor_id, float position_deg,
                                       float speed_ceiling_rpm, float current_ceiling_a,
                                       uint8_t feedback_type, encos_can_frame_t *frame);

bool encos_parse_discovery_reply(const encos_can_frame_t *frame, uint16_t *motor_id);
bool encos_parse_position_reply(const encos_can_frame_t *frame, uint16_t motor_id,
                                float *position_deg, uint8_t *error);
bool encos_parse_version_reply(const encos_can_frame_t *frame, uint16_t motor_id,
                               encos_version_t *version, uint8_t *error);
bool encos_parse_timeout_reply(const encos_can_frame_t *frame, uint16_t motor_id,
                               uint16_t *timeout_ms, uint8_t *error);
bool encos_parse_type2_feedback(const encos_can_frame_t *frame, uint16_t motor_id,
                                encos_type2_feedback_t *feedback);

#endif
