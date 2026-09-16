#include "encos_protocol.h"

#include <math.h>
#include <string.h>

static uint32_t read_be_u32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

static void write_be_u64(uint64_t value, uint8_t *data)
{
    for (int index = 7; index >= 0; --index) {
        data[index] = (uint8_t)(value & 0xffu);
        value >>= 8;
    }
}

/* ENCOS V1.19EAP section 9.1.3: seven bytes, float RPM, 0.1 A ceiling. */
bool encos_make_servo_velocity_request(uint16_t motor_id, float velocity_rpm,
                                      float current_ceiling_a, uint8_t feedback_type,
                                      encos_can_frame_t *frame)
{
    if (!frame || motor_id == 0 || motor_id >= ENCOS_DISCOVERY_CAN_ID ||
        !isfinite(velocity_rpm) || !isfinite(current_ceiling_a) ||
        current_ceiling_a < 0 || current_ceiling_a > 6553.5f || feedback_type > 3)
        return false;
    uint32_t bits;
    memcpy(&bits, &velocity_rpm, sizeof(bits));
    uint32_t current = (uint32_t)lroundf(current_ceiling_a * 10.0f);
    *frame = (encos_can_frame_t){.id = motor_id, .dlc = 7};
    frame->data[0] = 0x40u | feedback_type;
    for (int i = 0; i < 4; ++i) frame->data[1+i] = (uint8_t)(bits >> (24-8*i));
    frame->data[5] = (uint8_t)(current >> 8);
    frame->data[6] = (uint8_t)current;
    return true;
}

bool encos_parse_type3_feedback(const encos_can_frame_t *frame, uint16_t motor_id,
                                encos_type3_feedback_t *feedback)
{
    if (!frame || !feedback || frame->id != motor_id || frame->rtr != 0 ||
        frame->dlc != 8 || (frame->data[0] >> 5) != 3) return false;
    uint32_t bits = read_be_u32(&frame->data[1]);
    memcpy(&feedback->velocity_rpm, &bits, sizeof(bits));
    if (!isfinite(feedback->velocity_rpm)) return false;
    feedback->current_a = (int16_t)(((uint16_t)frame->data[5] << 8) | frame->data[6]) / 100.0f;
    feedback->temperature_c = ((int)frame->data[7] - 50) / 2.0f;
    feedback->error = frame->data[0] & 0x1f;
    return true;
}

static bool query_header_matches(const encos_can_frame_t *frame, uint16_t motor_id,
                                 uint8_t query_code, uint8_t expected_dlc, uint8_t *error)
{
    if (frame == NULL || frame->id != motor_id || frame->rtr != 0 ||
        frame->dlc != expected_dlc || (frame->data[0] >> 5) != 5 ||
        frame->data[1] != query_code) {
        return false;
    }
    if (error != NULL) {
        *error = frame->data[0] & 0x1f;
    }
    return true;
}

encos_can_frame_t encos_make_discovery_request(void)
{
    encos_can_frame_t frame = {0};
    frame.id = ENCOS_DISCOVERY_CAN_ID;
    frame.dlc = 4;
    frame.data[0] = 0xff;
    frame.data[1] = 0xff;
    frame.data[2] = 0x00;
    frame.data[3] = 0x82;
    return frame;
}

encos_can_frame_t encos_make_query_request(uint16_t motor_id, uint8_t query_code)
{
    encos_can_frame_t frame = {0};
    frame.id = motor_id;
    frame.dlc = 2;
    frame.data[0] = 0xe0;
    frame.data[1] = query_code;
    return frame;
}

bool encos_make_servo_position_request(uint16_t motor_id, float position_deg,
                                       float speed_ceiling_rpm, float current_ceiling_a,
                                       uint8_t feedback_type, encos_can_frame_t *frame)
{
    if (frame == NULL || motor_id == 0 || motor_id >= ENCOS_DISCOVERY_CAN_ID ||
        !isfinite(position_deg) || !isfinite(speed_ceiling_rpm) ||
        !isfinite(current_ceiling_a) || speed_ceiling_rpm < 0.0f ||
        speed_ceiling_rpm > 3276.7f || current_ceiling_a < 0.0f ||
        current_ceiling_a > 409.5f || feedback_type > 3) {
        return false;
    }

    const uint32_t speed = (uint32_t)lroundf(speed_ceiling_rpm * 10.0f);
    const uint32_t current = (uint32_t)lroundf(current_ceiling_a * 10.0f);
    uint32_t position_bits = 0;
    memcpy(&position_bits, &position_deg, sizeof(position_bits));
    const uint64_t payload = ((uint64_t)1u << 61) |
                             ((uint64_t)position_bits << 29) |
                             ((uint64_t)speed << 14) |
                             ((uint64_t)current << 2) | feedback_type;

    *frame = (encos_can_frame_t){0};
    frame->id = motor_id;
    frame->dlc = 8;
    write_be_u64(payload, frame->data);
    return true;
}

bool encos_parse_discovery_reply(const encos_can_frame_t *frame, uint16_t *motor_id)
{
    if (frame == NULL || motor_id == NULL || frame->id != ENCOS_DISCOVERY_CAN_ID ||
        frame->rtr != 0 || frame->dlc != 5 || frame->data[0] != 0xff ||
        frame->data[1] != 0xff || frame->data[2] != 0x01) {
        return false;
    }
    const uint16_t id = (uint16_t)(((uint16_t)frame->data[3] << 8) | frame->data[4]);
    if (id == 0 || id >= ENCOS_DISCOVERY_CAN_ID) {
        return false;
    }
    *motor_id = id;
    return true;
}

bool encos_parse_position_reply(const encos_can_frame_t *frame, uint16_t motor_id,
                                float *position_deg, uint8_t *error)
{
    if (position_deg == NULL || !query_header_matches(frame, motor_id, 1, 6, error)) {
        return false;
    }
    const uint32_t bits = read_be_u32(&frame->data[2]);
    memcpy(position_deg, &bits, sizeof(bits));
    return isfinite(*position_deg);
}

bool encos_parse_version_reply(const encos_can_frame_t *frame, uint16_t motor_id,
                               encos_version_t *version, uint8_t *error)
{
    if (version == NULL || !query_header_matches(frame, motor_id, 30, 8, error)) {
        return false;
    }
    memcpy(version->hardware, &frame->data[2], 3);
    memcpy(version->software, &frame->data[5], 3);
    return true;
}

bool encos_parse_timeout_reply(const encos_can_frame_t *frame, uint16_t motor_id,
                               uint16_t *timeout_ms, uint8_t *error)
{
    if (timeout_ms == NULL || !query_header_matches(frame, motor_id, 31, 4, error)) {
        return false;
    }
    *timeout_ms = (uint16_t)(((uint16_t)frame->data[2] << 8) | frame->data[3]);
    return true;
}

bool encos_parse_brake_status_reply(const encos_can_frame_t *frame, uint16_t motor_id,
                                    bool *released, uint8_t *error)
{
    if (released == NULL || !query_header_matches(frame, motor_id, 37, 3, error) ||
        frame->data[2] > 1) {
        return false;
    }
    *released = frame->data[2] == 1;
    return true;
}


bool encos_parse_type2_feedback(const encos_can_frame_t *frame, uint16_t motor_id,
                                encos_type2_feedback_t *feedback)
{
    if (frame == NULL || feedback == NULL || frame->id != motor_id || frame->rtr != 0 ||
        frame->dlc != 8 || (frame->data[0] >> 5) != 2) {
        return false;
    }
    const uint32_t position_bits = read_be_u32(&frame->data[1]);
    memcpy(&feedback->position_deg, &position_bits, sizeof(position_bits));
    if (!isfinite(feedback->position_deg)) {
        return false;
    }
    const int16_t current_raw = (int16_t)(((uint16_t)frame->data[5] << 8) |
                                          frame->data[6]);
    feedback->current_a = current_raw / 100.0f;
    feedback->temperature_c = ((int)frame->data[7] - 50) / 2.0f;
    feedback->error = frame->data[0] & 0x1f;
    return true;
}
