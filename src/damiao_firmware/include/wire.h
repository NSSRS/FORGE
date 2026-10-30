#ifndef FORGE_WIRE_H
#define FORGE_WIRE_H
#include "arm.h"
#define WIRE_SIZE 64
/* Request/response: A5 5A, version=1, type, LE sequence, 52-byte payload,
 * CRC32/ISO-HDLC over bytes 0..59. Response type=request|0x80.
 * Payload[0] is result (0 success); remaining bytes are response data. */
typedef struct { arm_controller *arm; uint8_t buffer[64]; unsigned used; uint32_t sequence; bool session; } arm_wire;
uint32_t wire_crc(const uint8_t *, size_t);
void wire_init(arm_wire *, arm_controller *);
bool wire_byte(arm_wire *, uint8_t, uint32_t, uint8_t response[64]);
#endif
