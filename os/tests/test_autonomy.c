#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fattos/fattos.h"
#include "fattos/fattos_board.h"
#include "fattos/fattos_bt.h"
#include "fattos/fattos_command.h"
#include "fattos/fattos_log.h"
#include "fattos/fattos_motion.h"
#include "fattos/fattos_nav.h"
#include "fattos/fattos_rc.h"
#include "fattos/fattos_safety.h"
#include "fattos/fattos_telemetry.h"
#include "fattos/fattos_vehicle.h"
#include "fattos_host.h"

/* Terminating check: the Windows debug-CRT assert can hang CI on a dialog. */
#undef assert
#define assert(cond)                                                            \
    do {                                                                        \
        if (!(cond)) {                                                          \
            fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,    \
                    #cond);                                                     \
            exit(1);                                                            \
        }                                                                       \
    } while (0)

/* ---- Behavior tree ------------------------------------------------------ */
static int bb_multi_ticks;

static FattosBTStatus bb_true(void *bb)
{
    (void)bb;
    return FATTOS_BT_SUCCESS;
}
static FattosBTStatus bb_false(void *bb)
{
    (void)bb;
    return FATTOS_BT_FAILURE;
}
static FattosBTStatus bb_multi(void *bb)
{
    (void)bb;
    return ++bb_multi_ticks >= 3 ? FATTOS_BT_SUCCESS : FATTOS_BT_RUNNING;
}
static int bb_quick_runs;
static FattosBTStatus bb_quick(void *bb)
{
    (void)bb;
    bb_quick_runs++;
    return FATTOS_BT_SUCCESS;
}

static void test_bt(void)
{
    /* Root sequence: condition, then a 3-tick action (memory resume). */
    FattosBTNode nodes[5];
    memset(nodes, 0, sizeof(nodes));
    nodes[0] = (FattosBTNode){FATTOS_BT_SEQUENCE, 1, 2, 0, NULL, NULL};
    nodes[1] = (FattosBTNode){FATTOS_BT_CONDITION, -1, 0, 0, bb_true, NULL};
    nodes[2] = (FattosBTNode){FATTOS_BT_ACTION, -1, 0, 0, bb_multi, NULL};
    FattosBT bt;
    assert(fattos_bt_init(&bt, nodes, 3U, NULL) == FATTOS_OK);

    bb_multi_ticks = 0;
    assert(fattos_bt_tick(&bt) == FATTOS_BT_RUNNING);
    assert(fattos_bt_tick(&bt) == FATTOS_BT_RUNNING);
    assert(fattos_bt_tick(&bt) == FATTOS_BT_SUCCESS); /* resumed, then done */
    assert(bb_multi_ticks == 3);
    assert(bt.ticks == 3U);

    /* Selector picks the first child that succeeds. */
    FattosBTNode sel[3];
    memset(sel, 0, sizeof(sel));
    sel[0] = (FattosBTNode){FATTOS_BT_SELECTOR, 1, 2, 0, NULL, NULL};
    sel[1] = (FattosBTNode){FATTOS_BT_CONDITION, -1, 0, 0, bb_false, NULL};
    sel[2] = (FattosBTNode){FATTOS_BT_ACTION, -1, 0, 0, bb_quick, NULL};
    FattosBT bt2;
    assert(fattos_bt_init(&bt2, sel, 3U, NULL) == FATTOS_OK);
    bb_quick_runs = 0;
    assert(fattos_bt_tick(&bt2) == FATTOS_BT_SUCCESS);
    assert(bb_quick_runs == 1);

    /* Inverter flips a failing condition into success. */
    FattosBTNode inv[3];
    memset(inv, 0, sizeof(inv));
    inv[0] = (FattosBTNode){FATTOS_BT_SEQUENCE, 1, 1, 0, NULL, NULL};
    inv[1] = (FattosBTNode){FATTOS_BT_INVERTER, 2, 1, 0, NULL, NULL};
    inv[2] = (FattosBTNode){FATTOS_BT_CONDITION, -1, 0, 0, bb_false, NULL};
    FattosBT bt3;
    assert(fattos_bt_init(&bt3, inv, 3U, NULL) == FATTOS_OK);
    assert(fattos_bt_tick(&bt3) == FATTOS_BT_SUCCESS);

    /* Structural validation: out-of-range children and leafless actions. */
    FattosBTNode bad[1];
    memset(bad, 0, sizeof(bad));
    bad[0] = (FattosBTNode){FATTOS_BT_SEQUENCE, 5, 2, 0, NULL, NULL};
    assert(fattos_bt_init(&bt3, bad, 1U, NULL) == FATTOS_ERR_INVALID_ARG);
    bad[0] = (FattosBTNode){FATTOS_BT_ACTION, -1, 0, 0, NULL, NULL};
    assert(fattos_bt_init(&bt3, bad, 1U, NULL) == FATTOS_ERR_INVALID_ARG);
    assert(fattos_bt_tick(NULL) == FATTOS_BT_FAILURE);
}

/* ---- Motion profile ----------------------------------------------------- */
static void test_motion(void)
{
    FattosMotionProfile motion;
    assert(fattos_motion_init(&motion, 1.0f, 0.5f) == FATTOS_OK);
    assert(fattos_motion_done(&motion));

    /* 0 -> 4 m at vmax 1, a 0.5: accel 2 s (1 m), cruise 2 s (2 m),
       brake 2 s (1 m) -> 6 s total. */
    fattos_motion_set_target(&motion, 4.0f);
    float peak = 0.0f;
    int steps = 0;
    while (!fattos_motion_done(&motion) && steps < 20000) {
        (void)fattos_motion_step(&motion, 0.001f);
        if (fabsf(motion.velocity) > peak) {
            peak = fabsf(motion.velocity);
        }
        steps++;
    }
    assert(fattos_motion_done(&motion));
    assert(fabsf(motion.position - 4.0f) < 1e-3f);
    assert(motion.velocity == 0.0f);
    assert(peak <= 1.001f);               /* speed limit respected */
    assert(steps > 5900 && steps < 6100); /* ~6 s trajectory */

    /* Retarget mid-profile: brake, reverse, stop exactly on target. */
    fattos_motion_set_target(&motion, 10.0f);
    for (int i = 0; i < 1000; ++i) {
        (void)fattos_motion_step(&motion, 0.001f); /* 1 s accelerating */
    }
    assert(!fattos_motion_done(&motion));
    fattos_motion_set_target(&motion, 2.0f);
    steps = 0;
    while (!fattos_motion_done(&motion) && steps < 20000) {
        (void)fattos_motion_step(&motion, 0.001f);
        assert(fabsf(motion.velocity) <= 1.001f);
        steps++;
    }
    assert(fabsf(motion.position - 2.0f) < 1e-3f);

    assert(fattos_motion_init(&motion, 0.0f, 1.0f) == FATTOS_ERR_INVALID_ARG);
}

/* ---- Vehicle state manager ---------------------------------------------- */
static void heartbeat_both(FattosSafety *safety, uint8_t a, uint8_t b)
{
    (void)fattos_safety_heartbeat(safety, a);
    (void)fattos_safety_heartbeat(safety, b);
    (void)fattos_safety_poll(safety);
}

static void test_vehicle(void)
{
    FattosSafety safety;
    uint8_t ch_a = 0U;
    uint8_t ch_b = 0U;
    assert(fattos_safety_init(&safety) == FATTOS_OK);
    assert(fattos_safety_register_channel(&safety, "a", 0U, 50U, &ch_a) ==
           FATTOS_OK);
    assert(fattos_safety_register_channel(&safety, "b", 1U, 50U, &ch_b) ==
           FATTOS_OK);

    FattosVehicle vehicle;
    assert(fattos_vehicle_init(&vehicle, &safety, 100U, 50U) == FATTOS_OK);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_DISARMED);
    assert(!fattos_vehicle_can_actuate(&vehicle)); /* never before arming */

    /* Arming is deliberate: conditions must hold for arm_hold_ms. */
    fattos_vehicle_set_prearm(&vehicle, true);
    fattos_vehicle_set_rc(&vehicle, true, true);
    for (int i = 0; i < 100; ++i) {
        fattos_vehicle_set_rc(&vehicle, true, true); /* continuous RC */
        heartbeat_both(&safety, ch_a, ch_b);
        assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_DISARMED);
        fattos_host_advance(1U);
    }
    fattos_vehicle_set_rc(&vehicle, true, true);
    heartbeat_both(&safety, ch_a, ch_b);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_ARMED);
    assert(fattos_vehicle_can_actuate(&vehicle));

    /* Pilot disarm. */
    fattos_vehicle_set_rc(&vehicle, true, false);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_DISARMED);
    assert(!fattos_vehicle_can_actuate(&vehicle));

    /* RC loss while armed -> FAILSAFE, and recovery requires re-arming. */
    fattos_vehicle_set_rc(&vehicle, true, true);
    for (int i = 0; i < 101; ++i) {
        fattos_vehicle_set_rc(&vehicle, true, true); /* continuous RC */
        heartbeat_both(&safety, ch_a, ch_b);
        (void)fattos_vehicle_update(&vehicle);
        fattos_host_advance(1U);
    }
    fattos_vehicle_set_rc(&vehicle, true, true);
    heartbeat_both(&safety, ch_a, ch_b);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_ARMED);
    fattos_vehicle_set_rc(&vehicle, false, true); /* signal gone */
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_FAILSAFE);
    assert(!fattos_vehicle_can_actuate(&vehicle));
    fattos_vehicle_set_rc(&vehicle, true, true); /* signal back */
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_DISARMED);
    assert(strcmp(fattos_vehicle_state_name(vehicle.state), "DISARMED") == 0);

    /* Re-arm, then heartbeat timeout -> FAILSAFE (supervisor fault). */
    for (int i = 0; i < 101; ++i) {
        fattos_vehicle_set_rc(&vehicle, true, true);
        heartbeat_both(&safety, ch_a, ch_b);
        (void)fattos_vehicle_update(&vehicle);
        fattos_host_advance(1U);
    }
    fattos_vehicle_set_rc(&vehicle, true, true);
    heartbeat_both(&safety, ch_a, ch_b);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_ARMED);
    fattos_host_advance(60U); /* miss both heartbeats */
    (void)fattos_safety_poll(&safety);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_FAILSAFE);
    assert(!fattos_vehicle_can_actuate(&vehicle));

    /* E-stop latches all the way down and only explicit clear recovers. */
    heartbeat_both(&safety, ch_a, ch_b);
    assert(fattos_vehicle_estop(&vehicle) == FATTOS_OK);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_ESTOP);
    assert(fattos_vehicle_estop_clear(&vehicle) == FATTOS_OK);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_DISARMED);
    assert(!fattos_vehicle_can_actuate(&vehicle));
}

/* ---- SBUS RC decoder ---------------------------------------------------- */
static size_t sbus_pack(uint8_t out[25], const uint16_t channels[16],
                        uint8_t flags)
{
    memset(out, 0, 25);
    out[0] = 0x0FU;
    for (size_t bit = 0U; bit < 176U; ++bit) {
        uint16_t value = channels[bit / 11U];
        if ((value >> (bit % 11U)) & 1U) {
            out[1U + bit / 8U] |= (uint8_t)(1U << (bit % 8U));
        }
    }
    out[23] = flags;
    out[24] = 0x00U; /* valid SBUS tail */
    return 25U;
}

static void test_rc(void)
{
    FattosRCState rc;
    fattos_rc_init(&rc);
    assert(!fattos_rc_signal_ok(&rc, 0U, 50U));

    uint16_t channels[16];
    for (int i = 0; i < 16; ++i) {
        channels[i] = 992U;
    }
    channels[0] = 172U;
    channels[1] = 1811U;
    channels[5] = 1811U; /* arm switch high */
    uint8_t frame[25];
    (void)sbus_pack(frame, channels, 0U);

    for (size_t i = 0U; i < sizeof(frame); ++i) {
        FattosStatus st = fattos_rc_push(&rc, frame[i]);
        assert(st == (i == sizeof(frame) - 1U ? FATTOS_OK : FATTOS_ERR_EMPTY));
    }
    assert(rc.frames_ok == 1U);
    assert(fattos_rc_channel_raw(&rc, 0U) == 172U);
    assert(fattos_rc_channel_raw(&rc, 1U) == 1811U);
    assert(fabsf(fattos_rc_channel_norm(&rc, 0U) + 1.0f) < 1e-3f);
    assert(fabsf(fattos_rc_channel_norm(&rc, 1U) - 1.0f) < 1e-3f);
    assert(fabsf(fattos_rc_channel_throttle(&rc, 5U) - 1.0f) < 1e-3f);
    assert(fattos_rc_signal_ok(&rc, rc.last_frame_ms, 50U));
    assert(!fattos_rc_signal_ok(&rc, rc.last_frame_ms + 60U, 50U)); /* stale */

    /* Failsafe flag kills the signal immediately. */
    (void)sbus_pack(frame, channels, 0x08U);
    for (size_t i = 0U; i < sizeof(frame); ++i) {
        (void)fattos_rc_push(&rc, frame[i]);
    }
    assert(rc.failsafe);
    assert(!fattos_rc_signal_ok(&rc, rc.last_frame_ms, 50U));

    /* Bad footer rejected; decoder resyncs on the next good frame. */
    rc.failsafe = false;
    (void)sbus_pack(frame, channels, 0U);
    frame[24] = 0xA5U;
    for (size_t i = 0U; i < sizeof(frame); ++i) {
        (void)fattos_rc_push(&rc, frame[i]);
    }
    assert(rc.frames_bad == 1U);
    (void)sbus_pack(frame, channels, 0U);
    for (size_t i = 0U; i < sizeof(frame); ++i) {
        (void)fattos_rc_push(&rc, frame[i]);
    }
    assert(rc.frames_ok == 3U);
}

/* ---- Command registry --------------------------------------------------- */
static float g_cmd_target;

static uint8_t cmd_set_target(void *ctx, const uint8_t *args, uint8_t args_len,
                              uint8_t *reply, uint8_t *reply_len,
                              uint8_t reply_cap)
{
    (void)ctx;
    if (args_len != 2U || reply_cap < 2U) {
        return FATTOS_CMD_STATUS_BAD_ARGS;
    }
    int16_t value = (int16_t)((uint16_t)args[0] | ((uint16_t)args[1] << 8));
    g_cmd_target = (float)value / 1000.0f;
    reply[0] = args[0];
    reply[1] = args[1];
    *reply_len = 2U;
    return FATTOS_CMD_STATUS_OK;
}

static uint8_t cmd_ping(void *ctx, const uint8_t *args, uint8_t args_len,
                        uint8_t *reply, uint8_t *reply_len, uint8_t reply_cap)
{
    (void)ctx;
    (void)args;
    (void)args_len;
    if (reply_cap < 1U) {
        return FATTOS_CMD_STATUS_FAILED;
    }
    reply[0] = 0x7AU;
    *reply_len = 1U;
    return FATTOS_CMD_STATUS_OK;
}

static void pump_frame(const uint8_t *frame, size_t len,
                       FattosTelemetryDecoder *decoder,
                       FattosTelemetryFrame *out)
{
    for (size_t i = 0U; i < len; ++i) {
        if (fattos_telemetry_decoder_push(decoder, frame[i], out) == FATTOS_OK) {
            return;
        }
    }
}

static void test_commands(void)
{
    FattosCommandRegistry registry;
    assert(fattos_cmd_registry_init(&registry) == FATTOS_OK);
    assert(fattos_cmd_register(&registry, 0x10U, cmd_set_target, NULL) ==
           FATTOS_OK);
    assert(fattos_cmd_register(&registry, 0x11U, cmd_ping, NULL) == FATTOS_OK);

    FattosTelemetryDecoder uplink;
    fattos_telemetry_decoder_init(&uplink);
    FattosTelemetryDecoder downlink;
    fattos_telemetry_decoder_init(&downlink);
    FattosTelemetryFrame received;
    uint8_t wire[64];
    uint8_t reply[64];
    size_t reply_len = 0U;

    /* SET_TARGET(0x1234) -> ack with echoed payload. */
    uint8_t payload[3] = {0x10U, 0x34U, 0x12U};
    size_t len = fattos_telemetry_encode(FATTOS_CMD_REQUEST, 7U, payload, 3U,
                                         wire, sizeof(wire));
    pump_frame(wire, len, &uplink, &received);
    assert(fattos_cmd_dispatch(&registry, &received, 7U, reply, sizeof(reply),
                               &reply_len) == FATTOS_OK);
    assert(reply_len > 0U);
    pump_frame(reply, reply_len, &downlink, &received);
    assert(received.type == FATTOS_CMD_REPLY);
    assert(received.payload[0] == 0x10U);
    assert(received.payload[1] == FATTOS_CMD_STATUS_OK);
    assert(received.payload[2] == 0x34U && received.payload[3] == 0x12U);
    assert(fabsf(g_cmd_target - 4.660f) < 1e-3f); /* 0x1234 / 1000 */

    /* Unknown id -> explicit NACK. */
    payload[0] = 0x99U;
    len = fattos_telemetry_encode(FATTOS_CMD_REQUEST, 8U, payload, 1U, wire,
                                  sizeof(wire));
    pump_frame(wire, len, &uplink, &received);
    assert(fattos_cmd_dispatch(&registry, &received, 8U, reply, sizeof(reply),
                               &reply_len) == FATTOS_OK);
    pump_frame(reply, reply_len, &downlink, &received);
    assert(received.payload[1] == FATTOS_CMD_STATUS_UNKNOWN);
    assert(registry.unknown == 1U);

    /* Empty command payload -> malformed NACK, not silence. */
    len = fattos_telemetry_encode(FATTOS_CMD_REQUEST, 9U, NULL, 0U, wire,
                                  sizeof(wire));
    pump_frame(wire, len, &uplink, &received);
    assert(fattos_cmd_dispatch(&registry, &received, 9U, reply, sizeof(reply),
                               &reply_len) == FATTOS_OK);
    pump_frame(reply, reply_len, &downlink, &received);
    assert(received.payload[1] == FATTOS_CMD_STATUS_BAD_ARGS);

    /* Non-command frames pass through untouched. */
    uint8_t other[2] = {1U, 2U};
    len = fattos_telemetry_encode(0x10U, 1U, other, 2U, wire, sizeof(wire));
    pump_frame(wire, len, &uplink, &received);
    assert(fattos_cmd_dispatch(&registry, &received, 1U, reply, sizeof(reply),
                               &reply_len) == FATTOS_ERR_EMPTY);
    assert(registry.handled == 1U);
}

/* ---- Logger ------------------------------------------------------------- */
static void test_log(void)
{
    FattosLogger log;
    fattos_log_init(&log, FATTOS_LOG_INFO);

    fattos_log(&log, FATTOS_LOG_DEBUG, "filtered %d", 1);
    assert(log.filtered == 1U);
    fattos_log(&log, FATTOS_LOG_INFO, "hello %d", 42);
    assert(log.count == 1U && log.total == 1U);
    const FattosLogEntry *entry = fattos_log_at(&log, 0U);
    assert(entry != NULL && entry->level == FATTOS_LOG_INFO);
    assert(strcmp(entry->text, "hello 42") == 0);
    assert(fattos_log_at(&log, 1U) == NULL);

    /* Ring overflow drops the oldest, counted, never blocks. */
    for (int i = 0; i < 20; ++i) {
        fattos_log(&log, FATTOS_LOG_ERROR, "e%d", i);
    }
    assert(log.dropped == (20U + 1U) - FATTOS_LOG_RING);
    assert(log.count == FATTOS_LOG_RING);
    entry = fattos_log_at(&log, 0U); /* newest */
    assert(strcmp(entry->text, "e19") == 0);
    entry = fattos_log_at(&log, FATTOS_LOG_RING - 1U); /* oldest surviving */
    assert(strcmp(entry->text, "e4") == 0);
    assert(strcmp(fattos_log_level_name(FATTOS_LOG_WARN), "WARN") == 0);
}

/* ---- Board manifest ----------------------------------------------------- */
static void test_board(void)
{
    static const FattosDevice devices[] = {
        {"left_motor", FATTOS_DEV_MOTOR, 0, 0, 60, 0.0f, 0.0f},
        {"right_motor", FATTOS_DEV_MOTOR, 1, 0, 60, 0.0f, 0.0f},
        {"drive", FATTOS_DEV_ODOMETRY, 0, 0, 1000, 0.03f, 0.15f},
        {"imu", FATTOS_DEV_IMU, 1, 0x68U, 0, 0.0f, 0.0f},
        {"rc", FATTOS_DEV_RC, 3, 0, 0, 0.0f, 0.0f},
    };
    FattosBoard board;
    fattos_board_init(&board, "test-bot", devices,
                      sizeof(devices) / sizeof(devices[0]));
    assert(board.count == 5U);
    assert(fattos_board_find(&board, "drive") == &devices[2]);
    assert(fattos_board_find(&board, "nope") == NULL);
    assert(fattos_board_find_kind(&board, FATTOS_DEV_MOTOR) == &devices[0]);
    assert(fattos_board_find_kind(&board, FATTOS_DEV_ESTOP) == NULL);

    /* The OS derives its tables from the manifest. */
    FattosSafety safety;
    assert(fattos_board_setup_safety(&board, &safety) == FATTOS_OK);
    assert(safety.count == 2U);
    assert(fattos_safety_find(&safety, "left_motor") == 0);
    assert(fattos_safety_find(&safety, "ghost") == -1);

    FattosOdometry odom;
    assert(fattos_board_setup_odometry(&board, &odom) == FATTOS_OK);
    assert(fabsf(odom.wheel_radius - 0.03f) < 1e-6f);
    assert(fabsf(odom.wheel_base - 0.15f) < 1e-6f);
    assert(odom.ticks_per_rev == 1000.0f);

    FattosRCState rc;
    assert(fattos_board_setup_rc(&board, &rc) == FATTOS_OK);
    FattosVehicle vehicle;
    assert(fattos_board_setup_vehicle(&board, &vehicle, &safety, 100U, 100U) ==
           FATTOS_OK);

    /* Named writes: only declared motors, always through the supervisor. */
    assert(fattos_board_write_motor(&board, &safety, "left_motor", 500U) ==
           FATTOS_OK);
    assert(fattos_board_write_motor(&board, &safety, "drive", 500U) ==
           FATTOS_ERR_INVALID_ARG); /* not an actuator */
    assert(fattos_board_write_motor(&board, &safety, "ghost", 500U) ==
           FATTOS_ERR_INVALID_ARG); /* not declared */

    float accel[3];
    float gyro[3];
    assert(fattos_board_read_imu(&board, accel, gyro) == FATTOS_OK);

    /* A manifest without actuators is rejected up front. */
    static const FattosDevice no_motors[] = {
        {"rc", FATTOS_DEV_RC, 3, 0, 0, 0.0f, 0.0f}};
    FattosBoard bad;
    fattos_board_init(&bad, "bad", no_motors, 1U);
    FattosSafety empty;
    assert(fattos_board_setup_safety(&bad, &empty) ==
           FATTOS_ERR_INVALID_ARG);
}

/* ---- Slew-rate actuator protection -------------------------------------- */
static void test_slew(void)
{
    FattosSafety safety;
    uint8_t id = 0U;
    assert(fattos_safety_init(&safety) == FATTOS_OK);
    assert(fattos_safety_register_channel(&safety, "m", 0U, 50U, &id) ==
           FATTOS_OK);
    assert(fattos_safety_set_slew(&safety, id, 100U) == FATTOS_OK);
    assert(fattos_safety_last_duty(&safety, id) == 0U);

    /* 0 -> 600 capped at +100 per command. */
    assert(fattos_safety_command(&safety, id, 600U) == FATTOS_OK);
    assert(fattos_safety_last_duty(&safety, id) == 100U);
    assert(fattos_safety_command(&safety, id, 250U) == FATTOS_OK);
    assert(fattos_safety_last_duty(&safety, id) == 200U);
    /* Downward also limited: 200 -> 50 becomes 100. */
    assert(fattos_safety_command(&safety, id, 50U) == FATTOS_OK);
    assert(fattos_safety_last_duty(&safety, id) == 100U);

    /* Slew disabled -> direct. */
    assert(fattos_safety_set_slew(&safety, id, 0U) == FATTOS_OK);
    assert(fattos_safety_command(&safety, id, 400U) == FATTOS_OK);
    assert(fattos_safety_last_duty(&safety, id) == 400U);

    /* Faults bypass slew: immediate zero, not a ramp. */
    fattos_host_advance(60U);
    assert(fattos_safety_command(&safety, id, 400U) == FATTOS_ERR_FAULT);
    assert(fattos_safety_last_duty(&safety, id) == 0U);
    assert(fattos_safety_set_slew(&safety, 99U, 10U) ==
           FATTOS_ERR_INVALID_ARG);
    assert(fattos_safety_last_duty(&safety, 99U) == 0U);
}

/* ---- Vehicle fresh supervisor evaluation (safety regression) ------------ */
static void test_vehicle_fresh_eval(void)
{
    FattosSafety safety;
    uint8_t ch = 0U;
    assert(fattos_safety_init(&safety) == FATTOS_OK);
    assert(fattos_safety_register_channel(&safety, "m", 0U, 50U, &ch) ==
           FATTOS_OK);
    FattosVehicle vehicle;
    assert(fattos_vehicle_init(&vehicle, &safety, 10U, 50U) == FATTOS_OK);
    fattos_vehicle_set_prearm(&vehicle, true);

    for (int i = 0; i < 12; ++i) {
        fattos_vehicle_set_rc(&vehicle, true, true);
        (void)fattos_safety_heartbeat(&safety, ch);
        (void)fattos_vehicle_update(&vehicle);
        fattos_host_advance(1U);
    }
    fattos_vehicle_set_rc(&vehicle, true, true);
    (void)fattos_safety_heartbeat(&safety, ch);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_ARMED);

    /* Stop heartbeats and call NO safety_poll: the vehicle gate must
       notice the stale supervisor within one update (fresh evaluation,
       not cached state) and fail safe. */
    fattos_host_advance(60U);
    assert(fattos_vehicle_update(&vehicle) == FATTOS_VEHICLE_FAILSAFE);
    assert(!fattos_vehicle_can_actuate(&vehicle));
    /* The poll inside update already drove the fail-safe duty. */
    assert(!fattos_safety_is_safe(&safety));
}

/* ---- Go-to-goal navigation ---------------------------------------------- */
static void test_nav(void)
{
    FattosNav nav;
    assert(fattos_nav_init(&nav, 1.0f, 0.4f, 0.05f) == FATTOS_OK);
    assert(fattos_nav_init(NULL, 1.0f, 0.4f, 0.05f) ==
           FATTOS_ERR_INVALID_ARG);
    float v = -1.0f;
    float h = -1.0f;

    /* Goal due north while facing east: bearing pi/2, speed faded for turn. */
    assert(!fattos_nav_step(&nav, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, &v, &h));
    assert(fabsf(h - 1.5707963f) < 1e-3f);
    assert(v > 0.0f && v < 0.4f);

    /* Head-on far goal: capped at max_linear. */
    assert(!fattos_nav_step(&nav, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, &v, &h));
    assert(fabsf(v - 0.4f) < 1e-4f);
    assert(fabsf(h) < 1e-4f);

    /* Speed scales with remaining distance below the cap: 0.25 m -> 0.25. */
    assert(!fattos_nav_step(&nav, 0.0f, 0.0f, 0.0f, 0.25f, 0.0f, &v, &h));
    assert(fabsf(v - 0.25f) < 1e-4f);

    /* Arrival inside the radius: speed zero, counted once. */
    assert(fattos_nav_step(&nav, 0.98f, 0.0f, 0.0f, 1.0f, 0.0f, &v, &h));
    assert(v == 0.0f && nav.goals_reached == 1U);
    assert(fattos_nav_step(&nav, 0.985f, 0.0f, 0.0f, 1.0f, 0.0f, &v, &h));
    assert(nav.goals_reached == 1U); /* no double count */
    assert(fattos_nav_step(NULL, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, &v, &h) ==
           false);

    /* Wrap helper: folds into (-pi, pi]. */
    assert(fabsf(fattos_wrap_pi(3.0f * 3.14159265f) - 3.14159265f) < 1e-3f);
    assert(fabsf(fattos_wrap_pi(-2.5f * 3.14159265f) + 1.5707963f) < 1e-3f);
}

int main(void)
{
    assert(fattos_init() == FATTOS_OK);

    test_bt();
    test_motion();
    test_vehicle();
    test_rc();
    test_commands();
    test_log();
    test_board();
    test_slew();
    test_vehicle_fresh_eval();
    test_nav();

    puts("FattOS autonomy tests passed");
    return 0;
}