#ifndef FATTOS_SIM_BUS_H
#define FATTOS_SIM_BUS_H

#include <stdbool.h>
#include <stdint.h>
#include "fattos/fattos_bus.h"
#include "fattos/fattos_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Simulated I2C bus carrying emulated presence of real chips (WHO_AM_I
 * replies, register maps, raw ADC counts). Tests and host builds drive the
 * exact same driver code an MCU runs — only the bus vtable is swapped.
 *
 *   fattos_sim_reset();
 *   fattos_sim_install(FATTOS_SENSOR_MPU6050, 0x68);
 *   fattos_sim_imu_values(accel_g, gyro_dps);
 *   const FattosBus buses[] = { {"i2c0", fattos_sim_bus()} };
 *   fattos_sensor_setup(manifest, n, buses, 1);
 *
 * Removing a chip mid-test (fattos_sim_set_present(..., false)) makes its
 * address NACK, which exercises probe-failure and bus-error health paths.
 */

/* Attach an emulated chip at addr7. */
void fattos_sim_install(FattosSensorKind kind, uint8_t addr7);
/* Detach (NACK) or re-attach without losing configuration. */
void fattos_sim_set_present(FattosSensorKind kind, bool present);
/* Forget everything: no chips, no config. */
void fattos_sim_reset(void);

/* Physical setpoints: the emulated chips convert these to raw counts using
   the full-scale ranges the *driver actually programmed*. */
void fattos_sim_imu_values(const float accel_g[3], const float gyro_dps[3]);
void fattos_sim_mag_values(const float mag_ut[3]);
void fattos_sim_baro_values(float pressure_pa, float temp_c);
void fattos_sim_tof_value(float range_m);

/* Ready-to-use FattosBus (one simulated I2C bus covers every test chip). */
const FattosBus *fattos_sim_bus(void);

#ifdef __cplusplus
}
#endif

#endif