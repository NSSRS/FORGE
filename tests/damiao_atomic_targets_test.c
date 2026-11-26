/* Real wire/parser and controller, with no hardware or transport threads. */
#include "wire.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned sends;
static float sent[ARM_COUNT];
static bool tx(void *ctx, uint8_t bus, uint16_t id, const uint8_t *d, uint8_t n) {
    (void)ctx; (void)bus;
    sends++;
    if (id >= 0x101 && id <= 0x106 && n == 8) memcpy(&sent[id - 0x101], d, 4);
    return true;
}
static void put32(uint8_t *p, uint32_t n) {
    for (unsigned i = 0; i < 4; i++) p[i] = n >> (8 * i);
}
static void putf(uint8_t *p, float f) { uint32_t n; memcpy(&n, &f, 4); put32(p, n); }
static void crc(uint8_t *p) { put32(p + 60, wire_crc(p, 60)); }
static void packet(uint8_t *p, uint32_t seq, uint8_t mask) {
    memset(p, 0, 64); p[0] = 0xa5; p[1] = 0x5a; p[2] = 1; p[3] = 30;
    put32(p + 4, seq); p[8] = mask;
    for (unsigned i = 0; i < ARM_COUNT; i++) {
        putf(p + 12 + 8 * i, .1f * (i + 1)); putf(p + 16 + 8 * i, .05f);
    }
    crc(p);
}
static bool feed(arm_wire *w, const uint8_t *p, unsigned n, uint32_t now, uint8_t *out) {
    bool reply = false;
    for (unsigned i = 0; i < n; i++) reply |= wire_byte(w, p[i], now, out);
    return reply;
}
static void ready(arm_controller *a, arm_wire *w, uint32_t now, uint8_t mask) {
    arm_init(a, tx, NULL); wire_init(w, a); sends = 0;
    /* Model the existing, independently tested routed configuration/readback. */
    a->routed_bench = a->bench = true; a->last_tick = now;
    for (unsigned i = 0; i < ARM_COUNT; i++) if (mask & (1u << i)) {
        arm_joint *j = &a->joints[i];
        j->cfg = (arm_config){i ? 2 : 1, i + 1, i + 17, -1, 1, 1, 2, 0, 0};
        j->range = (dm_range){12.5f, 30, 40}; j->registers = 31; j->bench_mode = 2;
        j->configured = j->manual_travel = j->position_hold = j->group_hold = true;
        j->feedback_seen = j->command_seen = true; j->feedback_at = j->command_at = now;
        j->cmd.v = .05f;
    }
    uint8_t hello[64] = {0xa5, 0x5a, 1, 1}, out[64]; put32(hello + 4, 1); crc(hello);
    assert(feed(w, hello, 64, now, out) && out[8] == 0);
}
static void unchanged(const arm_controller *a, const arm_controller *before) {
    assert(memcmp(a, before, sizeof(*a)) == 0); assert(sends == 0);
}
static void fragmented_and_replayed(void) {
    for (unsigned cut = 0; cut < 64; cut++) {
        arm_controller a; arm_wire w; uint8_t p[64], out[64]; ready(&a, &w, 20, 63);
        arm_controller before = a; packet(p, 2, 63);
        assert(!feed(&w, p, cut, 21, out)); unchanged(&a, &before);
        assert(feed(&w, p + cut, 64 - cut, 22, out) && out[8] == 0);
        for (unsigned i = 0; i < ARM_COUNT; i++) {
            assert(a.joints[i].command_at == 22 && a.joints[i].command_seen);
            assert(a.joints[i].cmd.p == .1f * (i + 1));
            assert(!a.joints[i].enabled); /* A batch never enables a motor. */
        }
        before = a;
        assert(feed(&w, p, 64, 23, out) && out[9] == ARM_REJECT_STALE_SEQUENCE);
        unchanged(&a, &before);
    }
}
static void rejected_batches(void) {
    for (unsigned kind = 0; kind < 16; kind++) {
        arm_controller a; arm_wire w; uint8_t p[64], out[64]; ready(&a, &w, 20, 63);
        packet(p, 2, 63);
        switch (kind) {
        case 0: putf(p + 52, NAN); break; /* Last slot, no earlier target may change. */
        case 1: putf(p + 52, 1.1f); break;
        case 2: putf(p + 56, 0); break;
        case 3: putf(p + 56, -.05f); break;
        case 4: p[8] = 31; break; /* Omitted configured motor. */
        case 5: p[8] = 0xff; break;
        case 6: p[9] = 0x80; break;
        case 7: p[10] = 1; break;
        case 8: a.joints[5].feedback_seen = false; break;
        case 9: a.joints[5].registers = 15; break;
        case 10: a.joints[5].stopping = true; break;
        case 11: a.joints[5].bench_mode = 1; break;
        case 12: a.fault = true; break;
        case 13: a.range_commissioning = true; break;
        case 14: a.joints[5].enabled = true; break; /* Unconfirmed changed preload. */
        case 15: p[9] = 32; break; /* Smoothing is not allowed while disabled. */
        }
        crc(p); arm_controller before = a;
        assert(feed(&w, p, 64, 21, out) && out[8] == 1); unchanged(&a, &before);
    }
    arm_controller a; arm_wire w; uint8_t p[64], out[64]; ready(&a, &w, 20, 63);
    packet(p, 2, 63); p[60] ^= 1; arm_controller before = a;
    assert(!feed(&w, p, 64, 21, out)); unchanged(&a, &before);
}
static void deadlines_and_sparse_slots(void) {
    arm_controller a; arm_wire w; uint8_t p[64], out[64];
    ready(&a, &w, 20, 63); packet(p, 2, 63);
    assert(!feed(&w, p, 40, 20, out));
    for (unsigned i = 0; i < ARM_COUNT; i++) a.joints[i].feedback_at = 170;
    arm_controller before = a;
    assert(feed(&w, p + 40, 24, 170, out) && out[9] == ARM_REJECT_STALE_PRELOAD);
    unchanged(&a, &before); /* Even disabled preloads cannot finish an old frame. */
    ready(&a, &w, 20, 63); packet(p, 2, 63); before = a;
    assert(feed(&w, p, 64, 120, out) && out[9] == ARM_REJECT_STALE_FEEDBACK);
    unchanged(&a, &before);
    ready(&a, &w, 20, 63); packet(p, 2, 63);
    for (unsigned i = 0; i < ARM_COUNT; i++) {
        a.joints[i].enabled = a.joints[i].enable_confirmed = true;
        a.joints[i].status = 1; a.joints[i].feedback_at = 170;
    }
    before = a;
    assert(feed(&w, p, 64, 170, out) && out[9] == ARM_REJECT_STALE_PRELOAD);
    unchanged(&a, &before);
    arm_tick(&a, 170); assert(a.fault && a.fault_reason == 1);
    for (unsigned i = 0; i < ARM_COUNT; i++) assert(!a.joints[i].enabled);

    ready(&a, &w, UINT32_MAX - 10, 62); packet(p, 2, 62);
    assert(!feed(&w, p, 40, UINT32_MAX - 5, out));
    assert(feed(&w, p + 40, 24, 5, out) && out[8] == 0);
    assert(!a.joints[0].command_seen);
    for (unsigned i = 1; i < ARM_COUNT; i++) assert(a.joints[i].command_at == 5);
}
static void commit_precedes_tick(void) {
    arm_controller a; arm_wire w; uint8_t p[64], out[64]; ready(&a, &w, 20, 63);
    for (unsigned i = 0; i < ARM_COUNT; i++) {
        a.joints[i].enabled = a.joints[i].enable_confirmed = true; a.joints[i].status = 1;
    }
    packet(p, 2, 63); assert(feed(&w, p, 64, 21, out) && out[8] == 0 && sends == 0);
    arm_tick(&a, 22); assert(!a.fault && sends == 6);
    for (unsigned i = 0; i < ARM_COUNT; i++) assert(sent[i] == .1f * (i + 1));
    packet(p, 3, 63); p[9] = 32; crc(p);
    assert(feed(&w, p, 64, 23, out) && out[8] == 0);
    for (unsigned i = 0; i < ARM_COUNT; i++) assert(a.joints[i].smooth_jog == (i == 5));
}
int main(void) {
    fragmented_and_replayed(); rejected_batches(); deadlines_and_sparse_slots(); commit_precedes_tick();
    puts("atomic group target tests passed"); return 0;
}
