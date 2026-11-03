#include "tx_wait.h"
bool tx_wait(const tx_wait_ops *ops,void *ctx,uint32_t budget_ms) {
    uint32_t start=ops->now(ctx);
    while(ops->pending(ctx)) {
        if(ops->now(ctx)-start>=budget_ms) {
            ops->cancel(ctx);
            start=ops->now(ctx);
            while(ops->pending(ctx)) {
                if(ops->now(ctx)-start>=budget_ms) { ops->halt(ctx); break; }
            }
            return false;
        }
    }
    return ops->sent(ctx);
}
