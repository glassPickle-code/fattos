#ifndef FATTOS_PLATFORM_H
#define FATTOS_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

FattosStatus fattos_platform_init(void);
FattosStatus fattos_platform_timer_start(uint32_t tick_hz);
void fattos_platform_gpio_write(uint16_t pin, bool high);
bool fattos_platform_gpio_read(uint16_t pin);
FattosStatus fattos_platform_pwm_write(uint8_t channel, uint16_t duty_permille);
FattosStatus fattos_platform_read_imu(int16_t *ax, int16_t *ay, int16_t *az);
/* Board-managed 6-axis sample for the device layer: accel in g, gyro in
   rad/s, already scaled by the port. Ports with raw-only sensors convert
   here; board manifests reference this through FATTOS_DEV_IMU. */
FattosStatus fattos_platform_imu_read6(float accel_g[3], float gyro_rads[3]);

void fattos_platform_idle(void);

/* Critical section: disable interrupts, return opaque key, restore on unlock.
   MCU ports must save/restore the full interrupt state (e.g. PRIMASK).
   Nested lock/unlock pairs with the same key must be balanced by the caller. */
uint32_t fattos_platform_irq_lock(void);
void fattos_platform_irq_unlock(uint32_t key);

/* Kick the hardware watchdog. Kernel calls this only while all health
   checks pass; ports may reset the MCU when this is starved. */
FattosStatus fattos_platform_watchdog_kick(void);

#ifdef __cplusplus
}
#endif

#endif
