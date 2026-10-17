#ifndef FATTOS_BOARD_H
#define FATTOS_BOARD_H

#include <stddef.h>
#include <stdint.h>
#include "fattos.h"
#include "fattos_odometry.h"
#include "fattos_rc.h"
#include "fattos_safety.h"
#include "fattos_vehicle.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Board manifest — declare ONCE, in your application, what is connected
 * where; FattOS derives its own tables from it. The application never
 * touches raw pins for I/O the OS manages:
 *
 *   static const FattosDevice devices[] = {
 *       {"left_motor",  FATTOS_DEV_MOTOR,    .pin = 0, .param = 60},
 *       {"right_motor", FATTOS_DEV_MOTOR,    .pin = 1, .param = 60},
 *       {"drive",       FATTOS_DEV_ODOMETRY, .param = 1000,
 *        .scale = 0.03f, .offset = 0.15f},          // radius, wheel base
 *       {"imu",         FATTOS_DEV_IMU,      .pin = 1},
 *       {"rc",          FATTOS_DEV_RC,       .pin = 3},
 *   };
 *   static const FattosBoard board = {"my-robot", devices, 5};
 *
 *   fattos_board_setup_safety(&board, &safety);      // motors -> supervisor
 *   fattos_board_setup_odometry(&board, &odom);      // geometry -> odom
 *   fattos_board_setup_vehicle(&board, &veh, &safety, 100, 100);
 *   fattos_board_setup_rc(&board, &rc);
 *   fattos_board_write_motor(&board, &safety, "left_motor", 400);
 *
 * The manifest is plain data: grep-able, diff-able, and exactly what an
 * AI agent needs to know the robot's wiring without reading schematics.
 */
typedef enum {
    FATTOS_DEV_NONE = 0,
    FATTOS_DEV_MOTOR = 1,    /* PWM out: pin = channel, param = timeout ms */
    FATTOS_DEV_ODOMETRY = 2, /* param = ticks/rev, scale = wheel radius m,
                                offset = wheel base m */
    FATTOS_DEV_IMU = 3,      /* pin = bus, pin2 = address/aux */
    FATTOS_DEV_RC = 4,       /* SBUS input: pin = uart */
    FATTOS_DEV_ESTOP = 5     /* GPIO input (optional hardware e-stop) */
} FattosDeviceKind;

typedef struct {
    const char *name;   /* unique id used by lookups and safety channels */
    uint8_t kind;       /* FattosDeviceKind */
    uint16_t pin;       /* primary pin / PWM channel / bus index */
    uint16_t pin2;      /* secondary pin (SCL, address, dir) */
    uint32_t param;     /* kind-specific integer (see enum comments) */
    float scale;        /* kind-specific float */
    float offset;       /* kind-specific float */
} FattosDevice;

typedef struct {
    const char *name;
    const FattosDevice *devices; /* usually a static const array */
    size_t count;
} FattosBoard;

void fattos_board_init(FattosBoard *board, const char *name,
                       const FattosDevice *devices, size_t count);

const FattosDevice *fattos_board_find(const FattosBoard *board,
                                      const char *name);
const FattosDevice *fattos_board_find_kind(const FattosBoard *board,
                                           FattosDeviceKind kind);

/* Derived setup — the OS builds its tables from the manifest. */
/* Every FATTOS_DEV_MOTOR becomes a safety channel (heartbeat + slew gate). */
FattosStatus fattos_board_setup_safety(const FattosBoard *board,
                                       FattosSafety *safety);
/* The odometry device supplies wheel geometry and encoder resolution. */
FattosStatus fattos_board_setup_odometry(const FattosBoard *board,
                                         FattosOdometry *odom);
/* Requires the RC device; arm_hold/rc loss are explicit parameters. */
FattosStatus fattos_board_setup_vehicle(const FattosBoard *board,
                                        FattosVehicle *vehicle,
                                        FattosSafety *safety,
                                        uint32_t arm_hold_ms,
                                        uint32_t rc_timeout_ms);
FattosStatus fattos_board_setup_rc(const FattosBoard *board,
                                   FattosRCState *rc);

/* Named actuator write: validates the device kind, then routes through the
   safety supervisor (fresh state check, clamp, slew, fail-safe). */
FattosStatus fattos_board_write_motor(const FattosBoard *board,
                                      FattosSafety *safety, const char *name,
                                      uint16_t duty_permille);

/* Board-managed IMU read through the platform HAL (accel in g, gyro rad/s). */
FattosStatus fattos_board_read_imu(const FattosBoard *board, float accel_g[3],
                                   float gyro_rads[3]);

#ifdef __cplusplus
}
#endif

#endif