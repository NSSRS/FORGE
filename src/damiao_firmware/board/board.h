#ifndef FORGE_BOARD_H
#define FORGE_BOARD_H
#include <stdint.h>
void forge_init(void);
void forge_poll(void);
void forge_usb_receive(const uint8_t *, uint32_t);
#endif
