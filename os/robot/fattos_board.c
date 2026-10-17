#include "fattos/fattos_board.h"
#include "fattos/fattos_platform.h"
#include <string.h>

void fattos_board_init(FattosBoard *board, const char *name,
                       const FattosDevice *devices, size_t count)
{
    if (board == NULL) {
        return;
    }
    board->name = name;
    board->devices = devices;
    board->count = (devices != NULL) ? count : 0U;
}

const FattosDevice *fattos_board_find(const FattosBoard *board,
                                      const char *name)
{
    if (board == NULL || name == NULL || board->devices == NULL) {
        return NULL;
    }
    for (size_t i = 0U; i < board->count; ++i) {
        if (board->devices[i].name != NULL &&
            strcmp(board->devices[i].name, name) == 0) {
            return &board->devices[i];
        }
    }
    return NULL;
}

const FattosDevice *fattos_board_find_kind(const FattosBoard *board,
                                           FattosDeviceKind kind)
{
    if (board == NULL || board->devices == NULL) {
        return NULL;
    }
    for (size_t i = 0U; i < board->count; ++i) {
        if (board->devices[i].kind == (uint8_t)kind) {
            return &board->devices[i];
        }
    }
    return NULL;
}

FattosStatus fattos_board_setup_safety(const FattosBoard *board,
                                       FattosSafety *safety)
{
    if (board == NULL || safety == NULL || board->devices == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    FattosStatus status = fattos_safety_init(safety);
    if (status != FATTOS_OK) {
        return status;
    }
    size_t motors = 0U;
    for (size_t i = 0U; i < board->count; ++i) {
        const FattosDevice *dev = &board->devices[i];
        if (dev->kind != FATTOS_DEV_MOTOR) {
            continue;
        }
        if (dev->pin > 255U) {
            return FATTOS_ERR_INVALID_ARG;
        }
        uint32_t timeout = dev->param > 0U ? dev->param : 50U;
        status = fattos_safety_register_channel(safety, dev->name,
                                                (uint8_t)dev->pin, timeout,
                                                NULL);
        if (status != FATTOS_OK) {
            return status;
        }
        motors++;
    }
    /* A robot manifest without actuators is a wiring error, not a runtime
       surprise — reject it here. */
    return motors > 0U ? FATTOS_OK : FATTOS_ERR_INVALID_ARG;
}

FattosStatus fattos_board_setup_odometry(const FattosBoard *board,
                                         FattosOdometry *odom)
{
    if (board == NULL || odom == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    const FattosDevice *dev = fattos_board_find_kind(board, FATTOS_DEV_ODOMETRY);
    if (dev == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    return fattos_odometry_init(odom, dev->scale, dev->offset,
                                (float)dev->param);
}

FattosStatus fattos_board_setup_vehicle(const FattosBoard *board,
                                        FattosVehicle *vehicle,
                                        FattosSafety *safety,
                                        uint32_t arm_hold_ms,
                                        uint32_t rc_timeout_ms)
{
    if (board == NULL || vehicle == NULL || safety == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (fattos_board_find_kind(board, FATTOS_DEV_RC) == NULL) {
        return FATTOS_ERR_INVALID_ARG; /* no pilot link declared: cannot arm */
    }
    if (safety->count == 0U) {
        return FATTOS_ERR_INVALID_ARG; /* safety table not derived yet */
    }
    return fattos_vehicle_init(vehicle, safety, arm_hold_ms, rc_timeout_ms);
}

FattosStatus fattos_board_setup_rc(const FattosBoard *board,
                                   FattosRCState *rc)
{
    if (board == NULL || rc == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (fattos_board_find_kind(board, FATTOS_DEV_RC) == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    fattos_rc_init(rc);
    return FATTOS_OK;
}

FattosStatus fattos_board_write_motor(const FattosBoard *board,
                                      FattosSafety *safety, const char *name,
                                      uint16_t duty_permille)
{
    if (board == NULL || safety == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    const FattosDevice *dev = fattos_board_find(board, name);
    if (dev == NULL || dev->kind != FATTOS_DEV_MOTOR) {
        return FATTOS_ERR_INVALID_ARG; /* not a declared actuator */
    }
    int id = fattos_safety_find(safety, name);
    if (id < 0) {
        return FATTOS_ERR_NOT_INITIALIZED; /* safety table not derived */
    }
    return fattos_safety_command(safety, (uint8_t)id, duty_permille);
}

FattosStatus fattos_board_read_imu(const FattosBoard *board, float accel_g[3],
                                   float gyro_rads[3])
{
    if (board == NULL || accel_g == NULL || gyro_rads == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (fattos_board_find_kind(board, FATTOS_DEV_IMU) == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    return fattos_platform_imu_read6(accel_g, gyro_rads);
}