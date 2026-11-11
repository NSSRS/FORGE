#ifndef ENCOS_CLI_BRIDGE_H
#define ENCOS_CLI_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <signal.h>
#include "bridge_pdo.h"
#include <stdint.h>

#define ENCOS_MAX_CONTROLLED_MOTORS 4u

bool encos_open_cli_bridge(const char *interface_name, int *expected_wkc);
bool encos_exchange(int expected_wkc);
bool encos_parse_motor_ids(const char *text, uint16_t ids[ENCOS_MAX_CONTROLLED_MOTORS],
                           size_t *count);

typedef struct {
    uint16_t motor_id;
    float position_deg;
    uint8_t error;
} encos_position_reply_t;

extern volatile sig_atomic_t encos_stop_requested;
void encos_request_stop(int signal_number);
typedef bool (*encos_reply_matcher_t)(const encos_can_frame_t *, void *);
bool encos_query_reply(int expected_wkc, const encos_can_frame_t *request,
                       encos_reply_matcher_t matcher, void *result);
bool encos_discover_motor(int expected_wkc, uint16_t *motor_id);
bool encos_query_position(int expected_wkc, encos_position_reply_t *position);

#endif
