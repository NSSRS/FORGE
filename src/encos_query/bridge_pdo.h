#ifndef ENCOS_BRIDGE_PDO_H
#define ENCOS_BRIDGE_PDO_H

#include <stdint.h>

#define ENCOS_BRIDGE_CHANNELS 6u
#define ENCOS_CAN_DATA_MAX 8u
#define ENCOS_BRIDGE_OUTPUT_BYTES 86u
#define ENCOS_BRIDGE_INPUT_BYTES 92u

#pragma pack(push, 1)
typedef struct {
    uint32_t id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[ENCOS_CAN_DATA_MAX];
} encos_can_frame_t;

/* EtherCAT RxPDO 0x1601: host outputs consumed by the bridge. */
typedef struct {
    uint8_t motor_num;
    uint8_t can_ide;
    encos_can_frame_t motor[ENCOS_BRIDGE_CHANNELS];
} encos_bridge_outputs_t;

/* EtherCAT TxPDO 0x1a00 followed by the six-byte TxPDO 0x1a02. */
typedef struct {
    uint8_t motor_num;
    uint8_t can_ide;
    encos_can_frame_t motor[ENCOS_BRIDGE_CHANNELS];
    uint8_t status[6];
} encos_bridge_inputs_t;
#pragma pack(pop)

_Static_assert(sizeof(encos_can_frame_t) == 14, "CAN PDO record must be 14 bytes");
_Static_assert(sizeof(encos_bridge_outputs_t) == ENCOS_BRIDGE_OUTPUT_BYTES,
               "bridge output PDO must be 86 bytes");
_Static_assert(sizeof(encos_bridge_inputs_t) == ENCOS_BRIDGE_INPUT_BYTES,
               "bridge input PDO must be 92 bytes");

#endif
