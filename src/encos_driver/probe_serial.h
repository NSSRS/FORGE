#ifndef ENCOS_PROBE_SERIAL_H
#define ENCOS_PROBE_SERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>

typedef struct {
    int fd;
    unsigned baud;
    uint64_t bytes;
    struct termios saved;
} probe_serial_t;

bool probe_serial_open(probe_serial_t *serial, const char *path, const char *baud);
bool probe_serial_sample(probe_serial_t *serial, FILE *report);
bool probe_serial_close(probe_serial_t *serial);

#endif
