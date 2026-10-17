#ifndef FATTOS_SENSOR_INTERNAL_H
#define FATTOS_SENSOR_INTERNAL_H

#include <stdint.h>

/*
 * Shared between the BMP280 driver and the host-side chip emulator: the
 * datasheet integer compensation, in one place, so tests can encode raw
 * ADC counts that mean a known pressure/temperature.
 *
 * calib layout (24 bytes as read from register 0x88):
 *   [0]=dig_T1(u16) [1]=dig_T2(s16) [2]=dig_T3(s16)
 *   [3]=dig_P1(u16) [4..11]=dig_P2..dig_P9(s16)
 */
void fattos_bmp280_compensate(const uint16_t calib[12], uint32_t adc_t,
                              uint32_t adc_p, float *temp_c,
                              float *pressure_pa);

#endif
