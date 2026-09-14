#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "encos_protocol.h"

#define CHECK(condition) do { if (!(condition)) { fprintf(stderr, "check failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); exit(EXIT_FAILURE); } } while (0)

static void test_layout_and_requests(void)
{
    CHECK(sizeof(encos_bridge_outputs_t) == 86);
    CHECK(sizeof(encos_bridge_inputs_t) == 92);

    encos_can_frame_t frame = encos_make_discovery_request();
    const uint8_t discovery[] = {0xff, 0xff, 0x00, 0x82};
    CHECK(frame.id == 0x7ff && frame.dlc == 4);
    CHECK(memcmp(frame.data, discovery, sizeof(discovery)) == 0);

    frame = encos_make_query_request(0x123, 30);
    CHECK(frame.id == 0x123 && frame.dlc == 2);
    CHECK(frame.data[0] == 0xe0 && frame.data[1] == 30);
}

static void test_replies(void)
{
    encos_can_frame_t frame = {.id = 0x7ff, .dlc = 5,
                               .data = {0xff, 0xff, 0x01, 0x01, 0x23}};
    uint16_t id = 0;
    CHECK(encos_parse_discovery_reply(&frame, &id));
    CHECK(id == 0x123);

    frame = (encos_can_frame_t){.id = 0x123, .dlc = 6,
                                .data = {0xa0, 0x01, 0x41, 0x48, 0x00, 0x00}};
    float position = 0.0f;
    uint8_t error = 99;
    CHECK(encos_parse_position_reply(&frame, id, &position, &error));
    CHECK(fabsf(position - 12.5f) < 0.0001f && error == 0);

    frame = (encos_can_frame_t){.id = 0x123, .dlc = 8,
                                .data = {0xa0, 30, 1, 2, 3, 4, 5, 6}};
    encos_version_t version = {0};
    CHECK(encos_parse_version_reply(&frame, id, &version, &error));
    CHECK(version.hardware[0] == 1 && version.hardware[2] == 3);
    CHECK(version.software[0] == 4 && version.software[2] == 6);

    frame = (encos_can_frame_t){.id = 0x123, .dlc = 4,
                                .data = {0xa3, 31, 0x01, 0xf4}};
    uint16_t timeout_ms = 0;
    CHECK(encos_parse_timeout_reply(&frame, id, &timeout_ms, &error));
    CHECK(timeout_ms == 500 && error == 3);

    frame.dlc = 5;
    CHECK(!encos_parse_timeout_reply(&frame, id, &timeout_ms, &error));
}

static void test_servo_position_and_feedback(void)
{
    encos_can_frame_t frame = {0};
    const uint8_t zero_vector[] = {0x20, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0xca};
    CHECK(encos_make_servo_position_request(1, 0.0f, 20.0f, 5.0f, 2, &frame));
    CHECK(frame.id == 1 && frame.dlc == 8);
    CHECK(memcmp(frame.data, zero_vector, sizeof(zero_vector)) == 0);

    const uint8_t ninety_vector[] = {0x28, 0x56, 0x80, 0x00, 0x00, 0x32, 0x00, 0xca};
    CHECK(encos_make_servo_position_request(1, 90.0f, 20.0f, 5.0f, 2, &frame));
    CHECK(memcmp(frame.data, ninety_vector, sizeof(ninety_vector)) == 0);
    CHECK(!encos_make_servo_position_request(1, 0.0f, -1.0f, 5.0f, 2, &frame));

    frame = (encos_can_frame_t){.id = 1,
                                .dlc = 8,
                                .data = {0x40, 0x42, 0xf0, 0x00, 0x00,
                                         0xff, 0x85, 0x78}};
    encos_type2_feedback_t feedback = {0};
    CHECK(encos_parse_type2_feedback(&frame, 1, &feedback));
    CHECK(fabsf(feedback.position_deg - 120.0f) < 0.0001f);
    CHECK(fabsf(feedback.current_a - (-1.23f)) < 0.0001f);
    CHECK(fabsf(feedback.temperature_c - 35.0f) < 0.0001f);
    CHECK(feedback.error == 0);
}

static void test_velocity(void)
{
    encos_can_frame_t frame;
    const uint8_t manual[] = {0x41, 0x42, 0x48, 0, 0, 0, 0x64};
    CHECK(encos_make_servo_velocity_request(1, 50, 10, 1, &frame));
    CHECK(frame.id == 1 && frame.dlc == 7 && !frame.rtr);
    CHECK(!memcmp(frame.data, manual, sizeof(manual)));
    CHECK(encos_make_servo_velocity_request(1, -50, 10, 3, &frame));
    CHECK(frame.data[0] == 0x43 && frame.data[1] == 0xc2);
    CHECK(!encos_make_servo_velocity_request(0, 1, 1, 1, &frame));
    CHECK(!encos_make_servo_velocity_request(0x7ff, 1, 1, 1, &frame));
    CHECK(!encos_make_servo_velocity_request(1, NAN, 1, 1, &frame));
    CHECK(!encos_make_servo_velocity_request(1, INFINITY, 1, 1, &frame));
    CHECK(!encos_make_servo_velocity_request(1, 1, -1, 1, &frame));
    CHECK(!encos_make_servo_velocity_request(1, 1, 6554, 1, &frame));
    CHECK(!encos_make_servo_velocity_request(1, 1, 1, 4, &frame));
    CHECK(!encos_make_servo_velocity_request(1, 1, 1, 1, NULL));
    frame = (encos_can_frame_t){.id = 1, .dlc = 8,
        .data = {0x63, 0xc2, 0x48, 0, 0, 0xff, 0x85, 49}};
    encos_type3_feedback_t v;
    CHECK(encos_parse_type3_feedback(&frame, 1, &v));
    CHECK(v.velocity_rpm == -50 && fabsf(v.current_a + 1.23f) < 1e-5f);
    CHECK(v.temperature_c == -0.5f && v.error == 3);
    CHECK(!encos_parse_type3_feedback(&frame, 2, &v));
    frame.dlc = 7;
    CHECK(!encos_parse_type3_feedback(&frame, 1, &v));
    frame.dlc = 8; frame.rtr = 1;
    CHECK(!encos_parse_type3_feedback(&frame, 1, &v));
    frame.rtr = 0; frame.data[1] = 0x7f; frame.data[2] = 0x80;
    CHECK(!encos_parse_type3_feedback(&frame, 1, &v));
}

int main(void)
{
    test_layout_and_requests();
    test_replies();
    test_servo_position_and_feedback();
    test_velocity();
    puts("encos protocol tests passed");
    return 0;
}
