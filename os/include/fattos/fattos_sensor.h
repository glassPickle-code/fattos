#ifndef FATTOS_SENSOR_H
#define FATTOS_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include "fattos.h"
#include "fattos_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Sensor subsystem — "declare the part, the OS knows the part."
 *
 * A board manifest lists sensors by KIND (FATTOS_SENSOR_MPU6050, ...) with
 * bus + address. fattos_sensor_setup() looks up the driver in the registry,
 * probes WHO_AM_I on the bus, applies the chip's configuration from plain
 * parameters (data rate, full-scale range), and starts it. Drivers convert
 * raw counts to SI units and implement self-test and auto-calibration, so
 * application code only ever reads FattosSensorSample:
 *
 *   static const FattosSensorEntry sensors[] = {
 *       {"imu", FATTOS_SENSOR_MPU6050, 0, 0x68, 200, 4.0f, 500.0f},
 *       {"mag", FATTOS_SENSOR_QMC5883, 0, 0x0D,  50, 0.0f,   0.0f},
 *       {"baro", FATTOS_SENSOR_BMP280, 0, 0x76,  25, 0.0f,   0.0f},
 *       {"tof", FATTOS_SENSOR_VL53L0X, 0, 0x29,  20, 0.0f,   0.0f},
 *   };
 *   fattos_sensor_setup(sensors, 4, buses, 1);
 *   ...
 *   FattosSensorSample s;
 *   if (fattos_sensor_read("imu", &s) == FATTOS_OK) { use s.accel_g ... }
 *
 * Every sensor carries health (last update, error counts, plausibility
 * bounds) that feeds vehicle prearm automatically.
 */

#define FATTOS_SENSOR_NAME_MAX 12U
#define FATTOS_MAX_SENSORS 8U

/* Known parts. The registry maps these to drivers; adding a driver adds
   one entry here, one struct, and no call-site changes. */
typedef enum {
    FATTOS_SENSOR_NONE = 0,
    FATTOS_SENSOR_MPU6050, /* 6-axis IMU, I2C */
    FATTOS_SENSOR_QMC5883, /* magnetometer, I2C */
    FATTOS_SENSOR_BMP280,  /* barometer, I2C */
    FATTOS_SENSOR_VL53L0X, /* time-of-flight rangefinder, I2C */
    FATTOS_SENSOR_COUNT    /* number of known kinds */
} FattosSensorKind;

const char *fattos_sensor_kind_name(FattosSensorKind kind);

/* What a driver produces: SI units, ready to use. Not every field is
   populated; check `kind` and the valid bitmask. */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t valid; /* bitmask: 1=accel 2=gyro 4=mag 8=pressure 16=temp 32=range */
    float accel_g[3];    /* accelerometer, g */
    float gyro_rads[3];  /* gyroscope, rad/s */
    float mag_ut[3];     /* magnetometer, microtesla */
    float pressure_pa;   /* barometric pressure, Pa */
    float temp_c;        /* die temperature, C */
    float range_m;       /* rangefinder distance, meters */
} FattosSensorSample;

/* Manifest entry: name + part + where it lives + plain configuration. */
typedef struct {
    const char *name;         /* unique, used by fattos_sensor_read */
    uint8_t kind;             /* FattosSensorKind */
    uint8_t bus;              /* index into the bus table passed to setup */
    uint8_t addr7;            /* 7-bit I2C address */
    uint16_t rate_hz;         /* requested output data rate */
    float range_g;            /* IMU accel full-scale (g), 0 = driver default */
    float range_dps;          /* IMU gyro full-scale (deg/s), 0 = driver default */
} FattosSensorEntry;

/* Per-sensor health, exposed for telemetry and prearm decisions. */
typedef enum {
    FATTOS_SENSOR_OK = 0,
    FATTOS_SENSOR_MISSING,  /* not in manifest / probe failed at setup */
    FATTOS_SENSOR_ERROR,    /* bus errors on recent reads */
    FATTOS_SENSOR_STALE,    /* no fresh sample within 3x its period */
    FATTOS_SENSOR_PLAUSIBLE /* sample out of physical bounds */
} FattosSensorHealth;

const char *fattos_sensor_health_name(FattosSensorHealth health);

/* Probe + configure + start every sensor in the manifest. Returns the
   number of sensors that came up OK; a sensor that fails probe is marked
   MISSING and does not abort the others (one bad part must not brick
   the robot — prearm handles the policy). buses may be NULL when the
   manifest is empty. */
size_t fattos_sensor_setup(const FattosSensorEntry *entries, size_t count,
                           const FattosBus *buses, size_t bus_count);

/* Read the latest converted sample by name. FATTOS_ERR_INVALID_ARG for an
   unknown name, FATTOS_ERR_TIMEOUT when the sensor is stale or faulted
   (the sample is still filled with the last good data). */
FattosStatus fattos_sensor_read(const char *name, FattosSensorSample *out);

/* Poll all sensors: reads those due at their configured rate, applies
   auto-calibration, checks plausibility, updates health. Call from a
   periodic task; returns the number of sensors read this call. */
size_t fattos_sensor_poll(uint32_t now_ms);

/* True when every sensor declared in the manifest is present, fresh and
   plausible — wire straight into fattos_vehicle_set_prearm(). */
bool fattos_sensors_prearm_ok(void);

/* Health table access for telemetry/diagnostics. */
size_t fattos_sensor_count(void);
FattosStatus fattos_sensor_get_info(size_t index, const char **out_name,
                                    uint8_t *out_kind, uint8_t *out_health,
                                    uint32_t *out_errors);

/* Auto-calibration: for the IMU, collect gyro bias while the vehicle is
   known to be stationary (call at boot, before arming). Runs in bounded
   time, reports through the normal sample path once done. */
typedef enum {
    FATTOS_CAL_IDLE = 0,
    FATTOS_CAL_GYRO_BIAS, /* averaging gyro bias at rest */
    FATTOS_CAL_DONE,
    FATTOS_CAL_FAILED
} FattosCalState;

FattosStatus fattos_sensor_calibrate(const char *name, FattosCalState what,
                                     uint16_t samples);
FattosCalState fattos_sensor_cal_state(const char *name);

/* Test/host hook: replace the bus table (e.g. with a simulated bus) and
   reset all driver state. Passing NULL clears everything. */
void fattos_sensor_reset(void);

#ifdef __cplusplus
}
#endif

#endif
