#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fattos/fattos.h"
#include "fattos/fattos_ai.h"
#include "fattos/fattos_board.h"
#include "fattos/fattos_bt.h"
#include "fattos/fattos_command.h"
#include "fattos/fattos_features.h"
#include "fattos/fattos_imu.h"
#include "fattos/fattos_log.h"
#include "fattos/fattos_nav.h"
#include "fattos/fattos_nn.h"
#include "fattos/fattos_pid.h"
#include "fattos/fattos_platform.h"
#include "fattos/fattos_safety.h"
#include "fattos/fattos_telemetry.h"
#include "fattos/fattos_timer.h"
#include "fattos/fattos_vehicle.h"
#include "fattos/fattos_watchdog.h"
#include "fattos_host.h"

/*
 * FattOS host demo: a complete simulated autonomous robot.
 *
 * THIS IS THE WHOLE PITCH IN ONE FILE:
 *   1. The board manifest declares WHAT IS CONNECTED WHERE — pins,
 *      motors, geometry, sensors. The OS derives its safety table,
 *      odometry, vehicle gate and RC input from it. The app never
 *      touches raw pins for managed I/O.
 *   2. Everything else is supervision + behavior: SBUS arming, a behavior
 *      tree gating go-to-goal navigation, yaw-hold PID with feedforward,
 *      magnetometer-corrected fusion, NN inference, ground-station
 *      commands that retarget the goal mid-run, framed telemetry, logs.
 *   3. Deterministic: 3 s of simulated time, PASS/FAIL at the end.
 */

#define SIM_MS          3000U
#define CONTROL_DT      0.010f
#define IMU_DT          0.005f
#define MAX_WHEEL_SPEED 0.5f
#define GCS_GOAL_X      0.35f /* command retargets goal.x at t=500 ms */
#define GYRO_BIAS       0.01f /* simulated z-gyro bias (rad/s) */

/* ---- 1. THE BOARD MANIFEST: what is connected where --------------------- */
static const FattosDevice k_devices[] = {
    /* name        kind                 pin pin2 param  scale  offset */
    {"left_motor", FATTOS_DEV_MOTOR,    0,  0,   60,    0.0f,  0.0f},
    {"right_motor", FATTOS_DEV_MOTOR,   1,  0,   60,    0.0f,  0.0f},
    {"drive", FATTOS_DEV_ODOMETRY,      0,  0,   1000,  0.03f, 0.15f},
    {"imu", FATTOS_DEV_IMU,             1,  0x68, 0,    0.0f,  0.0f},
    {"rc", FATTOS_DEV_RC,               3,  0,   0,     0.0f,  0.0f},
    {"estop_in", FATTOS_DEV_ESTOP,      13, 0,   0,     0.0f,  0.0f},
};
static const FattosBoard k_board = {"fattos-demo-bot", k_devices,
                                    sizeof(k_devices) / sizeof(k_devices[0])};

/* ---- Managed OS state (built from the manifest) ------------------------- */
static FattosSafety g_safety;
static FattosVehicle g_vehicle;
static FattosOdometry g_odom;
static FattosRCState g_rc;
static FattosIMUState g_imu;
static FattosPID g_heading;
static FattosNav g_nav;
static FattosTimer g_beat_timer;
static FattosTelemetryDecoder g_down;   /* pose uplink */
static FattosTelemetryDecoder g_up;     /* command request */
static FattosTelemetryDecoder g_ack;    /* command reply */
static FattosFeatureWindow g_features;
static FattosCommandRegistry g_cmd;
static FattosLogger g_log;
static float g_feature_storage[16U * 3U];

/* Simulated plant. */
static float g_true_x;
static float g_true_y;
static float g_true_yaw;
static float g_true_yaw_rate;
static float g_meas_yaw;
static float g_res_left;
static float g_res_right;
static float g_wheel_left;
static float g_wheel_right;

/* Behavior state. */
typedef struct {
    bool rc_ok;
    bool armed;
    bool arrived;       /* nav reports arrival */
    float nav_linear;   /* speed from go-to-goal */
    float linear;       /* chosen by the tree, consumed by control */
    float goal_x;
    float goal_y;
    float yaw_target;   /* bearing to goal, owned by nav */
} RobotBrain;

static RobotBrain g_brain = {false, false, false, 0.0f, 0.0f,
                             0.30f, 0.10f, 0.0f};
static FattosBTNode g_bt_nodes[5];
static FattosBT g_bt;

/* Bookkeeping. */
static int g_left_id = -1;
static int g_right_id = -1;
static uint8_t g_wdt_control;
static uint8_t g_wdt_fusion;
static uint8_t g_task_fusion;
static uint8_t g_task_control;
static uint8_t g_task_nn;
static uint8_t g_task_telemetry;
static uint8_t g_task_diag;
static uint8_t g_task_timers;
static uint8_t g_task_rc;
static uint32_t g_nn_inferences;
static int g_nn_class = -1;
static uint32_t g_timer_fires;
static uint32_t g_fault_hook_calls;
static FattosVehicleState g_last_vehicle_state = FATTOS_VEHICLE_INIT;
static int16_t g_telemetry_payload[4];
static uint8_t g_sbus_frame[FATTOS_RC_FRAME_LEN];
static bool g_cmd_sent;

/* ---- Safety-first integration: overruns latch the e-stop ---------------- */
static void overrun_hook(uint8_t task_id, uint32_t exec_ticks,
                         uint32_t period_ticks)
{
    (void)task_id;
    (void)exec_ticks;
    (void)period_ticks;
    g_fault_hook_calls++;
    (void)fattos_vehicle_estop(&g_vehicle);
}

/* ---- INT8 model (deterministic, built once at startup) ------------------ */
static int8_t g_w1[8U * 6U];
static int32_t g_b1[8U];
static int8_t g_w3[4U * 8U];
static int32_t g_b3[4U];
static FattosNNLayer g_layers[3];
static FattosNNModel g_model;

static FattosStatus build_model(void)
{
    for (size_t o = 0U; o < 8U; ++o) {
        g_b1[o] = (int32_t)(o * 3U) - 4;
        for (size_t i = 0U; i < 6U; ++i) {
            g_w1[o * 6U + i] = (int8_t)(((o * 3U + i * 5U) % 7U) - 3U);
        }
    }
    for (size_t o = 0U; o < 4U; ++o) {
        g_b3[o] = (int32_t)o * 5 - 6;
        for (size_t i = 0U; i < 8U; ++i) {
            g_w3[o * 8U + i] = (int8_t)(((o * 5U + i * 3U) % 7U) - 3U);
        }
    }
    int32_t mult = 0;
    int32_t shift = 0;
    if (fattos_nn_make_scale(0.25f, &mult, &shift) != FATTOS_OK) {
        return FATTOS_ERR_PLATFORM;
    }
    g_layers[0] = (FattosNNLayer){FATTOS_NN_OP_FULLY_CONNECTED, g_w1, g_b1,
                                  6U, 8U, mult, shift};
    g_layers[1] = (FattosNNLayer){FATTOS_NN_OP_RELU, NULL, NULL, 8U, 8U, 0, 0};
    g_layers[2] = (FattosNNLayer){FATTOS_NN_OP_FULLY_CONNECTED, g_w3, g_b3,
                                  8U, 4U, mult, shift};
    g_model.layers = g_layers;
    g_model.layer_count = 3U;
    return fattos_nn_bind(&g_model);
}

/* ---- Behavior tree leaves ---------------------------------------------- */
static FattosBTStatus bb_armed_check(void *bbv)
{
    RobotBrain *bb = (RobotBrain *)bbv;
    return (bb->rc_ok && bb->armed) ? FATTOS_BT_SUCCESS : FATTOS_BT_FAILURE;
}

static FattosBTStatus bb_drive(void *bbv)
{
    RobotBrain *bb = (RobotBrain *)bbv;
    if (bb->arrived) {
        return FATTOS_BT_FAILURE; /* nav done: fall through to hold */
    }
    bb->linear = bb->nav_linear;
    return FATTOS_BT_SUCCESS;
}

static FattosBTStatus bb_hold(void *bbv)
{
    ((RobotBrain *)bbv)->linear = 0.0f;
    return FATTOS_BT_SUCCESS;
}

static void build_behavior_tree(void)
{
    g_bt_nodes[0] = (FattosBTNode){FATTOS_BT_SELECTOR, 1, 2, 0, NULL, NULL};
    g_bt_nodes[1] = (FattosBTNode){FATTOS_BT_SEQUENCE, 2, 2, 0, NULL, NULL};
    g_bt_nodes[2] = (FattosBTNode){FATTOS_BT_CONDITION, -1, 0, 0,
                                   bb_armed_check, NULL};
    g_bt_nodes[3] = (FattosBTNode){FATTOS_BT_ACTION, -1, 0, 0, bb_drive, NULL};
    g_bt_nodes[4] = (FattosBTNode){FATTOS_BT_ACTION, -1, 0, 0, bb_hold, NULL};
    (void)fattos_bt_init(&g_bt, g_bt_nodes, 5U, &g_brain);
}

/* ---- Ground-station commands: retarget the goal ------------------------ */
static uint8_t cmd_goal_x(void *ctx, const uint8_t *args, uint8_t args_len,
                          uint8_t *reply, uint8_t *reply_len,
                          uint8_t reply_cap)
{
    (void)ctx;
    if (args_len != 2U || reply_cap < 2U) {
        return FATTOS_CMD_STATUS_BAD_ARGS;
    }
    int16_t mm = (int16_t)((uint16_t)args[0] | ((uint16_t)args[1] << 8));
    g_brain.goal_x = (float)mm / 1000.0f;
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

/* ---- SBUS frame builder (what a receiver would stream) ----------------- */
static void sbus_build(uint8_t out[FATTOS_RC_FRAME_LEN])
{
    uint16_t channels[16];
    for (int i = 0; i < 16; ++i) {
        channels[i] = 992U;
    }
    channels[5] = 1811U; /* arm switch high */
    memset(out, 0, FATTOS_RC_FRAME_LEN);
    out[0] = 0x0FU;
    for (size_t bit = 0U; bit < 176U; ++bit) {
        if ((channels[bit / 11U] >> (bit % 11U)) & 1U) {
            out[1U + bit / 8U] |= (uint8_t)(1U << (bit % 8U));
        }
    }
    out[24] = 0x00U; /* valid tail */
}

/* ---- Tasks -------------------------------------------------------------- */
static void rc_task(void *context)
{
    (void)context;
    for (size_t i = 0U; i < FATTOS_RC_FRAME_LEN; ++i) {
        (void)fattos_rc_push(&g_rc, g_sbus_frame[i]);
    }
}

static void fusion_task(void *context)
{
    (void)context;
    float gyro[3] = {0.0f, 0.0f, g_true_yaw_rate + GYRO_BIAS};
    float accel[3] = {0.0f, 0.0f, 1.0f};
    float mag[3] = {cosf(g_true_yaw), -sinf(g_true_yaw), 0.0f};
    fattos_imu_update_mag(&g_imu, gyro, accel, mag, IMU_DT);

    float roll;
    float pitch;
    if (fattos_imu_euler(&g_imu, &roll, &pitch, &g_meas_yaw) == FATTOS_OK) {
        (void)roll;
        (void)pitch;
    }
    (void)fattos_features_push(&g_features, accel);
    (void)fattos_wdt_feed(g_wdt_fusion);
}

static void control_task(void *context)
{
    (void)context;

    /* 1. Vehicle state machine (supervisor evaluated freshly inside). */
    bool rc_ok = fattos_rc_signal_ok(&g_rc, fattos_millis(), 100U);
    bool arm_switch = g_rc.valid && fattos_rc_channel_raw(&g_rc, 5U) > 1300U;
    fattos_vehicle_set_rc(&g_vehicle, rc_ok, arm_switch);
    fattos_vehicle_set_prearm(&g_vehicle, true);
    FattosVehicleState vs = fattos_vehicle_update(&g_vehicle);
    if (vs != g_last_vehicle_state) {
        fattos_log(&g_log, FATTOS_LOG_INFO, "vehicle %s->%s",
                   fattos_vehicle_state_name(g_last_vehicle_state),
                   fattos_vehicle_state_name(vs));
        g_last_vehicle_state = vs;
    }

    /* 2. Go-to-goal navigation: heading target + speed envelope. */
    g_brain.arrived = fattos_nav_step(&g_nav, g_odom.x, g_odom.y,
                                      g_odom.theta, g_brain.goal_x,
                                      g_brain.goal_y, &g_brain.nav_linear,
                                      &g_brain.yaw_target);

    /* 3. Behavior tree: may we drive at all? */
    g_brain.rc_ok = rc_ok;
    g_brain.armed = fattos_vehicle_can_actuate(&g_vehicle);
    (void)fattos_bt_tick(&g_bt);

    /* 4. Yaw-hold on the nav heading; bumpless while disabled. */
    float linear = g_brain.armed ? g_brain.linear : 0.0f;
    float angular = 0.0f;
    if (g_brain.armed) {
        angular = fattos_pid_step_ff(&g_heading, g_brain.yaw_target,
                                     g_meas_yaw, CONTROL_DT, 0.0f);
    } else {
        fattos_pid_seed(&g_heading, g_meas_yaw, g_meas_yaw, 0.0f);
    }
    (void)fattos_diff_mix(linear, angular, MAX_WHEEL_SPEED, &g_wheel_left,
                          &g_wheel_right);

    /* 5. Encoder tick deltas + plant integration. */
    float mpt = 2.0f * 3.14159265358979f * g_odom.wheel_radius /
                g_odom.ticks_per_rev;
    float ticks_l = g_wheel_left * CONTROL_DT / mpt + g_res_left;
    float ticks_r = g_wheel_right * CONTROL_DT / mpt + g_res_right;
    int32_t dl = (int32_t)ticks_l;
    int32_t dr = (int32_t)ticks_r;
    g_res_left = ticks_l - (float)dl;
    g_res_right = ticks_r - (float)dr;
    fattos_odometry_update(&g_odom, dl, dr);

    float v = 0.5f * (g_wheel_left + g_wheel_right);
    float w = (g_wheel_right - g_wheel_left) / g_odom.wheel_base;
    g_true_x += v * cosf(g_true_yaw) * CONTROL_DT;
    g_true_y += v * sinf(g_true_yaw) * CONTROL_DT;
    g_true_yaw += w * CONTROL_DT;
    g_true_yaw = fattos_wrap_pi(g_true_yaw);
    g_true_yaw_rate = w;

    /* 6. Heartbeats, then named writes through the manifest+supervisor. */
    (void)fattos_safety_heartbeat(&g_safety, (uint8_t)g_left_id);
    (void)fattos_safety_heartbeat(&g_safety, (uint8_t)g_right_id);
    uint16_t duty_l =
        (uint16_t)(fabsf(g_wheel_left) * 1000.0f / MAX_WHEEL_SPEED);
    uint16_t duty_r =
        (uint16_t)(fabsf(g_wheel_right) * 1000.0f / MAX_WHEEL_SPEED);
    (void)fattos_board_write_motor(&k_board, &g_safety, "left_motor", duty_l);
    (void)fattos_board_write_motor(&k_board, &g_safety, "right_motor", duty_r);

    (void)fattos_wdt_feed(g_wdt_control);
}

static void nn_task(void *context)
{
    (void)context;
    float vector[6];
    int8_t inputs[6];
    int8_t outputs[4];
    if (fattos_features_vector(&g_features, vector) != FATTOS_OK ||
        fattos_features_quantize(vector, 6U, 0.2f, inputs) != FATTOS_OK) {
        return;
    }
    FattosAIIO io = {inputs, sizeof(inputs), outputs, sizeof(outputs)};
    if (fattos_ai_infer(&io) == FATTOS_OK) {
        g_nn_inferences++;
        g_nn_class = fattos_nn_argmax(outputs, 4U);
    }
}

static void telemetry_task(void *context)
{
    (void)context;
    /* Mid-run ground-station command: retarget goal.x to 0.35 m. */
    if (!g_cmd_sent && fattos_millis() >= 500U) {
        g_cmd_sent = true;
        uint16_t mm = (uint16_t)(GCS_GOAL_X * 1000.0f);
        uint8_t payload[3] = {0x10U, (uint8_t)mm, (uint8_t)(mm >> 8)};
        uint8_t wire[32];
        size_t len = fattos_telemetry_encode(FATTOS_CMD_REQUEST, 1U, payload,
                                             3U, wire, sizeof(wire));
        FattosTelemetryFrame frame;
        for (size_t i = 0U; i < len; ++i) {
            if (fattos_telemetry_decoder_push(&g_up, wire[i], &frame) ==
                FATTOS_OK) {
                uint8_t reply[32];
                size_t reply_len = 0U;
                if (fattos_cmd_dispatch(&g_cmd, &frame, 1U, reply,
                                        sizeof(reply), &reply_len) ==
                        FATTOS_OK &&
                    reply_len > 0U) {
                    for (size_t j = 0U; j < reply_len; ++j) {
                        (void)fattos_telemetry_decoder_push(&g_ack, reply[j],
                                                            &frame);
                    }
                }
            }
        }
    }

    g_telemetry_payload[0] = (int16_t)(g_odom.x * 1000.0f);
    g_telemetry_payload[1] = (int16_t)(g_odom.y * 1000.0f);
    g_telemetry_payload[2] = (int16_t)(g_odom.theta * 1000.0f);
    g_telemetry_payload[3] = (int16_t)(g_meas_yaw * 1000.0f);

    static uint8_t seq;
    uint8_t frame[32];
    size_t len = fattos_telemetry_encode(0x10U, seq++, g_telemetry_payload,
                                         sizeof(g_telemetry_payload), frame,
                                         sizeof(frame));
    FattosTelemetryFrame decoded;
    for (size_t i = 0U; i < len; ++i) {
        (void)fattos_telemetry_decoder_push(&g_down, frame[i], &decoded);
    }
}

static void diagnostics_task(void *context)
{
    (void)context;
    (void)fattos_safety_poll(&g_safety);
    (void)fattos_wdt_poll();
    (void)fattos_wdt_kick_hardware();
}

static void beat_timer_cb(void *context)
{
    (void)context;
    g_timer_fires++;
}

/* ---- Main --------------------------------------------------------------- */
int main(void)
{
    if (fattos_init() != FATTOS_OK || fattos_ai_init() != FATTOS_OK) {
        fputs("FattOS initialization failed\n", stderr);
        return 1;
    }
    fattos_log_init(&g_log, FATTOS_LOG_DEBUG);
    fattos_set_fault_hook(overrun_hook);

    /* Everything below is DERIVED from the manifest — no raw pin logic. */
    if (fattos_board_setup_safety(&k_board, &g_safety) != FATTOS_OK ||
        fattos_board_setup_odometry(&k_board, &g_odom) != FATTOS_OK ||
        fattos_board_setup_rc(&k_board, &g_rc) != FATTOS_OK ||
        fattos_board_setup_vehicle(&k_board, &g_vehicle, &g_safety, 100U,
                                   100U) != FATTOS_OK) {
        fputs("Board manifest setup failed\n", stderr);
        return 1;
    }
    g_left_id = fattos_safety_find(&g_safety, "left_motor");
    g_right_id = fattos_safety_find(&g_safety, "right_motor");

    /* Actuator protection: at most 100 permille change per control cycle. */
    (void)fattos_safety_set_slew(&g_safety, (uint8_t)g_left_id, 100U);
    (void)fattos_safety_set_slew(&g_safety, (uint8_t)g_right_id, 100U);

    fattos_imu_init(&g_imu);
    if (fattos_pid_init(&g_heading, 1.0f, 0.1f, 0.05f, -0.5f, 0.5f, 0.6f) !=
            FATTOS_OK ||
        fattos_nav_init(&g_nav, 1.0f, 0.4f, 0.05f) != FATTOS_OK ||
        fattos_features_init(&g_features, g_feature_storage, 16U, 3U) !=
            FATTOS_OK ||
        fattos_cmd_registry_init(&g_cmd) != FATTOS_OK ||
        build_model() != FATTOS_OK) {
        fputs("Subsystem initialization failed\n", stderr);
        return 1;
    }
    fattos_telemetry_decoder_init(&g_down);
    fattos_telemetry_decoder_init(&g_up);
    fattos_telemetry_decoder_init(&g_ack);
    build_behavior_tree();
    sbus_build(g_sbus_frame);
    (void)fattos_cmd_register(&g_cmd, 0x10U, cmd_goal_x, NULL);
    (void)fattos_cmd_register(&g_cmd, 0x11U, cmd_ping, NULL);
    fattos_log(&g_log, FATTOS_LOG_INFO, "boot v1.0 board=%s",
               k_board.name);

    if (fattos_wdt_register("control", 60U, &g_wdt_control) != FATTOS_OK ||
        fattos_wdt_register("fusion", 30U, &g_wdt_fusion) != FATTOS_OK) {
        fputs("Watchdog registration failed\n", stderr);
        return 1;
    }

    if (fattos_task_create("rc_inject", rc_task, NULL, 20U, 4U, &g_task_rc) !=
            FATTOS_OK ||
        fattos_task_create("imu_fusion", fusion_task, NULL, 5U, 3U,
                           &g_task_fusion) != FATTOS_OK ||
        fattos_task_create("control", control_task, NULL, 10U, 5U,
                           &g_task_control) != FATTOS_OK ||
        fattos_task_create("nn_infer", nn_task, NULL, 50U, 2U, &g_task_nn) !=
            FATTOS_OK ||
        fattos_task_create("telemetry", telemetry_task, NULL, 100U, 1U,
                           &g_task_telemetry) != FATTOS_OK ||
        fattos_task_create("diagnostics", diagnostics_task, NULL, 100U, 1U,
                           &g_task_diag) != FATTOS_OK ||
        fattos_timers_register(10U, 1U, &g_task_timers) != FATTOS_OK) {
        fputs("Task registration failed\n", stderr);
        return 1;
    }

    if (fattos_timer_init(&g_beat_timer, "beat", beat_timer_cb, NULL) !=
            FATTOS_OK ||
        fattos_timer_start(&g_beat_timer, 25U, true) != FATTOS_OK) {
        fputs("Timer setup failed\n", stderr);
        return 1;
    }

    /* Bounded drain: a stuck callback can never spin this loop forever. */
    for (uint32_t tick = 0U; tick < SIM_MS; ++tick) {
        fattos_host_tick();
        uint32_t dispatched = 0U;
        if (fattos_run_pending(16U, &dispatched) != FATTOS_OK) {
            fputs("Scheduler error\n", stderr);
            return 1;
        }
    }

    float roll;
    float pitch;
    float fused_yaw = 0.0f;
    (void)fattos_imu_euler(&g_imu, &roll, &pitch, &fused_yaw);
    FattosSafetyState safety_state = fattos_safety_poll(&g_safety);
    bool wdt_healthy = fattos_wdt_is_healthy();
    float goal_dist = sqrtf((g_odom.x - g_brain.goal_x) *
                                (g_odom.x - g_brain.goal_x) +
                            (g_odom.y - g_brain.goal_y) *
                                (g_odom.y - g_brain.goal_y));

    printf("FattOS robot demo complete at %lu ms (board: %s)\n",
           (unsigned long)fattos_millis(), k_board.name);
    puts("Task schedule (runs / exec_max / jitter / overruns / missed):");
    const uint8_t ids[7] = {g_task_fusion, g_task_control, g_task_nn,
                            g_task_telemetry, g_task_diag, g_task_timers,
                            g_task_rc};
    for (size_t i = 0U; i < 7U; ++i) {
        FattosTaskInfo info;
        if (fattos_task_get_info(ids[i], &info) == FATTOS_OK) {
            printf("  %-11s %5lu %4lu %5lu %5lu %5lu\n", info.name,
                   (unsigned long)info.run_count,
                   (unsigned long)info.exec_max_ticks,
                   (unsigned long)info.jitter_max_ticks,
                   (unsigned long)info.overrun_count,
                   (unsigned long)info.missed_releases);
        }
    }

    printf("  vehicle: %s  rc: %lu frames  bt: %lu ticks\n",
           fattos_vehicle_state_name(g_vehicle.state),
           (unsigned long)g_rc.frames_ok, (unsigned long)g_bt.ticks);
    printf("  nav: %s  goal (%.2f, %.2f)  dist %.3f m  reached %lu\n",
           g_nav.arrived ? "ARRIVED" : "en route", (double)g_brain.goal_x,
           (double)g_brain.goal_y, (double)goal_dist,
           (unsigned long)g_nav.goals_reached);
    printf("  safety: %s  watchdog: %s  estop hooks: %lu\n",
           safety_state == FATTOS_SAFETY_OK ? "OK" : "FAULTED",
           wdt_healthy ? "OK" : "STARVED", (unsigned long)g_fault_hook_calls);
    printf("  pose: odom (%.3f, %.3f, %.3f)  true (%.3f, %.3f, %.3f)\n",
           (double)g_odom.x, (double)g_odom.y, (double)g_odom.theta,
           (double)g_true_x, (double)g_true_y, (double)g_true_yaw);
    printf("  nn: %lu inferences, class %d | gcs: %lu cmd, %lu ack\n",
           (unsigned long)g_nn_inferences, g_nn_class,
           (unsigned long)g_cmd.handled, (unsigned long)g_ack.frames_ok);
    printf("  telemetry: %lu ok, %lu rejected | timers: %lu\n",
           (unsigned long)g_down.frames_ok,
           (unsigned long)g_down.frames_rejected,
           (unsigned long)g_timer_fires);
    puts("  log:");
    for (size_t i = 0U; i < g_log.count && i < 6U; ++i) {
        const FattosLogEntry *e = fattos_log_at(&g_log, i);
        printf("    [%lu ms] %-5s %s\n", (unsigned long)e->timestamp_ms,
               fattos_log_level_name(e->level), e->text);
    }

    /* End-to-end acceptance — what "works" means here. */
    float pose_error =
        fabsf(g_odom.x - g_true_x) + fabsf(g_odom.y - g_true_y);
    bool ok = safety_state == FATTOS_SAFETY_OK && wdt_healthy &&
              g_down.frames_rejected == 0U && g_down.frames_ok > 25U &&
              g_nn_inferences >= 50U && g_timer_fires > 0U &&
              g_fault_hook_calls == 0U && pose_error < 0.05f &&
              g_vehicle.state == FATTOS_VEHICLE_ARMED &&
              g_nav.arrived && g_nav.goals_reached >= 1U &&
              goal_dist <= 0.06f &&
              g_rc.frames_ok > 40U && g_bt.ticks > 200U &&
              g_cmd.handled == 1U && g_ack.frames_ok == 1U &&
              g_log.count > 0U;
    printf("demo checks: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}