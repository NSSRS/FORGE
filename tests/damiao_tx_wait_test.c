#include "tx_wait.h"
#include <assert.h>
#include <stdio.h>
typedef struct { uint32_t clock, checks, finish_at; bool success, canceled, halted, stuck; } fake;
static uint32_t now(void *p) { fake *f=p; return f->clock++; }
static bool pending(void *p) {
    fake *f=p; f->checks++;
    if(f->halted || (f->canceled && !f->stuck)) return false;
    return f->clock < f->finish_at || f->stuck;
}
static bool sent(void *p) { return ((fake *)p)->success; }
static void cancel(void *p) { ((fake *)p)->canceled=true; }
static void halt(void *p) { ((fake *)p)->halted=true; }
static const tx_wait_ops ops={now,pending,sent,cancel,halt};
int main(void) {
    fake f={.finish_at=2,.success=true};
    assert(tx_wait(&ops,&f,5)); assert(!f.canceled && f.checks>1);
    /* Queued/canceled transmission without an on-wire success is not success. */
    f=(fake){.finish_at=0,.success=false}; assert(!tx_wait(&ops,&f,2));
    f=(fake){.finish_at=100,.success=true}; assert(!tx_wait(&ops,&f,2)); assert(f.canceled && !f.halted);
    f=(fake){.finish_at=100,.stuck=true}; assert(!tx_wait(&ops,&f,2)); assert(f.canceled && f.halted);
    f=(fake){.clock=UINT32_MAX-1,.stuck=true}; assert(!tx_wait(&ops,&f,2)); assert(f.halted);
    puts("Bounded CAN completion/cancellation tests passed");
}
