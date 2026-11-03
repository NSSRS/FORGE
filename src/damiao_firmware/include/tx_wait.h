#ifndef FORGE_TX_WAIT_H
#define FORGE_TX_WAIT_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t (*now)(void *);
    bool (*pending)(void *);
    bool (*sent)(void *);
    void (*cancel)(void *);
    void (*halt)(void *);
} tx_wait_ops;
/* Return success only after on-wire completion. Never leave a stale TX pending. */
bool tx_wait(const tx_wait_ops *, void *, uint32_t);
#endif
