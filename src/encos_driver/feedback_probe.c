/* Query-only characterization of the four-motor ENCOS process image. */
#include "cli_bridge.h"
#include "encos_protocol.h"
#include "ethercat.h"
#include "probe_serial.h"
#include <inttypes.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static const unsigned slots[] = {0, 1, 3, 4};
enum stage { POSITION, PAUSE_QUERY, VERSION };
static const char *const stage_names[] = {"position", "pause", "version"};

static int64_t clock_ns(clockid_t clock_id)
{
    struct timespec now;
    clock_gettime(clock_id, &now);
    return (int64_t)now.tv_sec * 1000000000 + now.tv_nsec;
}

static void hex(FILE *report, const void *data, size_t size)
{
    const uint8_t *bytes = data;
    for (size_t i = 0; i < size; ++i) fprintf(report, "%02x", bytes[i]);
}

/* Keep device-provided strings byte-exact even for non-ASCII firmware. */
static void identity_string(FILE *report, uint16_t index)
{
    uint8_t value[128];
    int size = sizeof(value);
    if (ec_SDOread(1, index, 0, FALSE, &size, value, 20000) > 0 &&
        size > 0 && size <= (int)sizeof(value)) {
        fputc('"', report);
        hex(report, value, size);
        fputc('"', report);
    } else {
        fputs("null", report);
    }
}

typedef struct {
    unsigned position_mask, version_mask, identical_mask, empty_mask;
} observations_t;

static bool read_observations(const encos_bridge_inputs_t *input, observations_t *obs,
                              encos_can_frame_t last_positions[4], bool save_positions)
{
    if (input->motor_num > ENCOS_BRIDGE_CHANNELS || input->can_ide != 0) {
        fprintf(stderr, "Unexpected input record count or CAN identifier format.\n");
        return false;
    }
    /* The six returned entries may be packed in a different order. */
    for (unsigned i = 0; i < ENCOS_BRIDGE_CHANNELS; ++i) {
        const encos_can_frame_t *frame = &input->motor[i];
        if (frame->dlc > ENCOS_CAN_DATA_MAX) {
            fprintf(stderr, "Invalid input DLC in slot %u.\n", i);
            return false;
        }
        if (frame->id < 1 || frame->id > 4) continue;
        unsigned bit = 1u << (frame->id - 1);
        if (!frame->dlc) {
            obs->empty_mask |= bit;
            continue;
        }
        uint8_t error = 0;
        float angle;
        encos_version_t version;
        if (encos_parse_position_reply(frame, frame->id, &angle, &error)) {
            encos_can_frame_t *last = &last_positions[frame->id - 1];
            if (last->dlc == frame->dlc && !memcmp(last->data, frame->data, frame->dlc))
                obs->identical_mask |= bit;
            if (save_positions) *last = *frame;
            obs->position_mask |= bit;
        } else if (encos_parse_version_reply(frame, frame->id, &version, &error)) {
            obs->version_mask |= bit;
        }
        if (error) {
            fprintf(stderr, "Motor %u reports error %u.\n", frame->id, error);
            return false;
        }
    }
    return true;
}

static bool sample_image(FILE *report, int wkc, int64_t *now)
{
    if (encos_stop_requested || ec_send_processdata() <= 0) return false;
    int actual_wkc = ec_receive_processdata(EC_TIMEOUTRET);
    *now = clock_ns(CLOCK_MONOTONIC);
    fprintf(report, "{\"event\":\"pdo\",\"epoch_ns\":%" PRId64 ",\"mono_ns\":%" PRId64
            ",\"wkc\":%d,\"input_hex\":\"", clock_ns(CLOCK_REALTIME), *now, actual_wkc);
    hex(report, ec_slave[1].inputs, ENCOS_BRIDGE_INPUT_BYTES);
    fputs("\"}\n", report);
    if (actual_wkc != wkc)
        fprintf(stderr, "EtherCAT WKC %d, expected %d.\n", actual_wkc, wkc);
    return actual_wkc == wkc && !ferror(report);
}

static bool run_stage(FILE *report, int wkc, probe_serial_t *serial, enum stage stage,
                      unsigned target, unsigned duration_ms, encos_can_frame_t last_positions[4],
                      bool *all_expected)
{
    encos_bridge_outputs_t *out = (void *)ec_slave[1].outputs;
    const encos_bridge_inputs_t *in = (const void *)ec_slave[1].inputs;
    memset(out, 0, sizeof(*out));
    out->motor_num = slots[3] + 1; /* Keep fixed routing even when ID 4 is paused. */
    for (unsigned id = 1; id <= 4; ++id) {
        if (stage == PAUSE_QUERY && id == target) continue;
        uint8_t query = stage == VERSION && id == target ? 30 : 1;
        out->motor[slots[id - 1]] = encos_make_query_request(id, query);
    }
    const unsigned bit = target ? 1u << (target - 1) : 0;
    const unsigned required_positions = stage == POSITION ? 15u : 15u & ~bit;
    unsigned observed_positions = 0, observed_versions = 0;
    unsigned late_frames = 0, late_position = 0, late_identical = 0, late_empty = 0;
    unsigned toggle_edges = 0, state_edges = 0;
    int previous_status = -1;
    int64_t first = 0, elapsed = 0;
    fprintf(report, "{\"event\":\"stage\",\"stage\":\"%s\",\"id\":%u,\"output_hex\":\"",
            stage_names[stage], target);
    hex(report, out, sizeof(*out));
    fputs("\"}\n", report);
    do {
        int64_t now;
        if (!sample_image(report, wkc, &now)) return false;
        if (!first) first = now;
        elapsed = now - first;
        observations_t obs = {0};
        if (!read_observations(in, &obs, last_positions, stage == POSITION) ||
            !probe_serial_sample(serial, report) || ferror(report))
            return false;
        /* In each phase count only the settled response. In particular a
         * pre-pause position is not evidence that a peer continued to report. */
        if (elapsed >= 250000000) {
            ++late_frames;
            observed_positions |= obs.position_mask & ~obs.version_mask;
            observed_versions |= obs.version_mask & ~obs.position_mask;
            if (obs.position_mask & bit) ++late_position;
            if (obs.identical_mask & bit) ++late_identical;
            if (obs.empty_mask & bit) ++late_empty;
            int status = in->status[1];
            if (previous_status >= 0) {
                if ((status ^ previous_status) & 0x80) ++toggle_edges;
                if ((status ^ previous_status) & 0x40) ++state_edges;
            }
            previous_status = status;
        }
        if (elapsed < (int64_t)duration_ms * 1000000) usleep(5000);
    } while (elapsed < (int64_t)duration_ms * 1000000);
    bool complete = (observed_positions & required_positions) == required_positions &&
                    (stage != VERSION || (observed_versions & bit));
    fprintf(report, "{\"event\":\"stage_result\",\"stage\":\"%s\",\"id\":%u,"
            "\"late_frames\":%u,\"late_target_position_frames\":%u,"
            "\"late_target_matching_frames\":%u,\"late_target_zero_dlc_frames\":%u,"
            "\"late_toggle_edges\":%u,\"late_state_edges\":%u,\"position_mask\":%u,\"version_mask\":%u,"
            "\"expected_records_seen\":%s}\n", stage_names[stage], target, late_frames,
            late_position, late_identical, late_empty, toggle_edges, state_edges,
            observed_positions, observed_versions, complete ? "true" : "false");
    if (stage == PAUSE_QUERY)
        printf("ID %u: position visible in %u/%u PDOs after 250 ms without a host query "
               "(%u match pre-pause bytes), Toggle edges %u.\n",
               target, late_position, late_frames, late_identical, toggle_edges);
    if (!complete) {
        fprintf(stderr, "Missing or ambiguous %s records (target %u, position mask 0x%x, "
                "version mask 0x%x).\n", stage_names[stage], target, observed_positions, observed_versions);
        *all_expected = false;
    }
    return fflush(report) == 0;
}

/* Read operator event markers without pausing EtherCAT between keystrokes. */
static int enter_pressed(void)
{
    struct pollfd fd = {.fd = STDIN_FILENO, .events = POLLIN};
    int ready = poll(&fd, 1, 0);
    if (ready <= 0) return ready;
    char c;
    if (read(fd.fd, &c, 1) != 1) return -1;
    return c == '\n';
}

static void loss_marker(FILE *report, unsigned id, const char *stage, int64_t now)
{
    fprintf(report, "{\"event\":\"loss_stage\",\"id\":%u,\"stage\":\"%s\","
            "\"mono_ns\":%" PRId64 ",\"epoch_ns\":%" PRId64 "}\n",
            id, stage, now, clock_ns(CLOCK_REALTIME));
    fflush(report);
}

static bool run_reply_loss(FILE *report, int wkc, unsigned target)
{
    enum { BASELINE, WAIT_OFF, OBSERVE_OFF, WAIT_ON, RECOVERY } phase = BASELINE;
    const int64_t ms = 1000000, deadline = 250 * ms;
    int64_t since = clock_ns(CLOCK_MONOTONIC), previous = since, next_query = since;
    int64_t updated[4] = {0};
    uint8_t accepted[4] = {0}, query = 0;
    unsigned changes[4] = {0}, recovered_from = 0;
    unsigned offline_samples = 0, visible = 0, late_changes = 0;
    unsigned target_index = target - 1, target_bit = 1u << target_index;
    encos_bridge_outputs_t *out = (void *)ec_slave[1].outputs;
    encos_can_frame_t last_positions[4] = {0};
    memset(out, 0, sizeof(*out));
    out->motor_num = slots[3] + 1;
    loss_marker(report, target, "baseline", since);
    for (;;) {
        if (clock_ns(CLOCK_MONOTONIC) >= next_query) {
            query = query == 1 ? 30 : 1;
            for (unsigned i = 0; i < 4; ++i)
                out->motor[slots[i]] = encos_make_query_request(i + 1, query);
            next_query = clock_ns(CLOCK_MONOTONIC) + 50 * ms;
            fprintf(report, "{\"event\":\"loss_query\",\"id\":%u,\"output_hex\":\"", target);
            hex(report, out, sizeof(*out));
            fputs("\"}\n", report);
        }
        int64_t now;
        observations_t obs = {0};
        if (!sample_image(report, wkc, &now) ||
            !read_observations((const void *)ec_slave[1].inputs, &obs, last_positions, false))
            return false;
        if (now - previous > deadline) {
            fprintf(stderr, "Sampling gap exceeded 250 ms.\n");
            return false;
        }
        previous = now;
        unsigned matching = query == 1 ? obs.position_mask & ~obs.version_mask
                                       : obs.version_mask & ~obs.position_mask;
        bool baseline_ready = true;
        unsigned changed = 0;
        for (unsigned i = 0; i < 4; ++i) {
            if (phase != BASELINE && i != target_index && now - updated[i] > deadline) {
                fprintf(stderr, "Peer ID %u stopped advancing during the test of ID %u.\n",
                        i + 1, target);
                return false;
            }
            if ((matching & (1u << i)) && accepted[i] != query) {
                accepted[i] = query;
                updated[i] = now;
                ++changes[i];
                changed |= 1u << i;
            }
            if (changes[i] < 3 || now - updated[i] > deadline) baseline_ready = false;
        }
        int64_t elapsed = now - since;
        if ((phase == BASELINE || phase == RECOVERY) && elapsed > 5000 * ms) {
            fprintf(stderr, "Missing %s reply-type changes for ID %u or its peers.\n",
                    phase == BASELINE ? "baseline" : "recovery", target);
            return false;
        }
        if (phase == BASELINE && baseline_ready) {
            printf("All four responding. Leave every connector seated. Switch off only ID %u "
                   "with its installed power switch or suppress its replies with the test fixture. "
                   "Press Enter when done, or Ctrl-C if neither is available.\n", target);
            fflush(stdout);
            loss_marker(report, target, "await_suppression", now);
            phase = WAIT_OFF;
            since = now;
        } else if (phase == WAIT_OFF || phase == WAIT_ON) {
            int enter = enter_pressed();
            if (enter < 0 || elapsed > 120000 * ms) {
                fprintf(stderr, "Operator marker missing or timed out.\n");
                return false;
            }
            if (enter) {
                bool off = phase == WAIT_OFF;
                since = now;
                loss_marker(report, target, off ? "operator_off" : "operator_on", now);
                phase = off ? OBSERVE_OFF : RECOVERY;
                recovered_from = changes[target_index];
            }
        } else if (phase == OBSERVE_OFF && elapsed >= deadline) {
            ++offline_samples;
            if ((obs.position_mask | obs.version_mask) & target_bit) ++visible;
            if (changed & target_bit) ++late_changes;
            if (elapsed >= 1250 * ms) {
                fprintf(report, "{\"event\":\"loss_window\",\"id\":%u,\"samples\":%u,"
                        "\"visible_target_samples\":%u,\"target_type_changes\":%u}\n",
                        target, offline_samples, visible, late_changes);
                printf("ID %u: %u late reply-type changes; target record visible in %u/%u "
                       "samples. Restore ID %u with the same switch or fixture, then press Enter.\n",
                       target, late_changes, visible, offline_samples, target);
                fflush(stdout);
                loss_marker(report, target, "await_restoration", now);
                phase = WAIT_ON;
                since = now;
            }
        } else if (phase == RECOVERY && changes[target_index] - recovered_from >= 2) {
            loss_marker(report, target, "recovery_observed", now);
            if (late_changes) {
                fprintf(stderr, "ID %u kept advancing after the suppression marker.\n", target);
                return false;
            }
            printf("ID %u: host observed isolated loss and recovery with all four polled.\n", target);
            return true;
        }
        usleep(5000);
    }
}

int main(int argc, char **argv)
{
    bool loss_test = argc == 5 && !strcmp(argv[3], "--reply-loss");
    bool debug = argc == 6 && !strcmp(argv[3], "--debug-port");
    if ((argc != 3 && !debug && !loss_test) ||
        (loss_test && strcmp(argv[4], "all") &&
         (strlen(argv[4]) != 1 || !strchr("1234", argv[4][0])))) {
        fprintf(stderr, "Usage: %s INTERFACE NEW_REPORT.jsonl "
                "[--reply-loss {1|2|3|4|all} | --debug-port TTY BAUD]\n", argv[0]);
        return 2;
    }
    FILE *report = fopen(argv[2], "wx");
    if (!report) { perror(argv[2]); return 1; }
    signal(SIGINT, encos_request_stop);
    signal(SIGTERM, encos_request_stop);
    probe_serial_t serial = {.fd = -1};
    bool ok = false;
    int wkc = 0;
    fprintf(report, "{\"event\":\"start\",\"schema\":\"forge.encos_feedback_probe\","
            "\"version\":1,\"mode\":\"%s\",\"serial_requested\":%s,\"debug_tty_path_hex\":",
            loss_test ? "continuous_poll_reply_loss" : "query_pause", debug ? "true" : "false");
    if (debug) {
        fputc('"', report);
        hex(report, argv[4], strlen(argv[4]));
        fputc('"', report);
    } else {
        fputs("null", report);
    }
    fputs(",\"can_freshness_verified\":false}\n", report);
    if (debug && !probe_serial_open(&serial, argv[4], argv[5])) goto finish;
    if (encos_stop_requested || !encos_open_cli_bridge(argv[1], &wkc)) goto finish;
    fprintf(report, "{\"event\":\"bridge\",\"interface\":\"%s\",\"vendor_id\":%u,\"product_code\":%u,"
            "\"revision\":%u,\"expected_wkc\":%d,\"hardware_1009_hex\":", argv[1], ec_slave[1].eep_man,
            ec_slave[1].eep_id, ec_slave[1].eep_rev, wkc);
    identity_string(report, 0x1009);
    fputs(",\"software_100a_hex\":", report);
    identity_string(report, 0x100a);
    fputs("}\n", report);
    encos_can_frame_t last_positions[4] = {0};
    bool all_expected = true;
    if (loss_test) {
        unsigned only = !strcmp(argv[4], "all") ? 0 : (unsigned)(argv[4][0] - '0');
        ok = wkc > 0;
        for (unsigned id = 1; ok && id <= 4; ++id)
            if (!only || id == only) ok = run_reply_loss(report, wkc, id);
    } else {
        ok = wkc > 0 && run_stage(report, wkc, &serial, POSITION, 0, 500, last_positions,
                                  &all_expected) && all_expected;
        for (unsigned id = 1; ok && id <= 4; ++id)
            ok = run_stage(report, wkc, &serial, PAUSE_QUERY, id, 800, last_positions, &all_expected) &&
                 run_stage(report, wkc, &serial, VERSION, id, 500, last_positions, &all_expected) &&
                 run_stage(report, wkc, &serial, POSITION, id, 500, last_positions, &all_expected);
    }
    ok = ok && all_expected;
    memset(ec_slave[1].outputs, 0, ENCOS_BRIDGE_OUTPUT_BYTES);
    if (!encos_exchange(wkc)) {
        fprintf(stderr, "Final empty output exchange failed.\n");
        ok = false;
    }
    ec_close();
finish:
    if (serial.fd >= 0) {
        printf("Debug port: %" PRIu64 " bytes received at %u baud.\n", serial.bytes, serial.baud);
        if (!probe_serial_close(&serial)) ok = false;
    }
    ok = ok && !encos_stop_requested;
    fprintf(report, "{\"event\":\"end\",\"completed\":%s,\"interrupted\":%s,"
            "\"debug_rx_bytes\":%" PRIu64 ",\"debug_baud\":%u,\"can_freshness_verified\":false}\n",
            ok ? "true" : "false", encos_stop_requested ? "true" : "false", serial.bytes, serial.baud);
    if (ferror(report)) ok = false;
    if (fclose(report)) { perror("report"); ok = false; }
    printf("Query-only experiment %s. Report: %s\n", ok ? "completed" : "incomplete", argv[2]);
    return ok ? 0 : 1;
}
