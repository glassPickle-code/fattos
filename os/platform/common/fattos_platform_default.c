#include "fattos/fattos_platform.h"

/*
 * Safe defaults for a buildable host starter. MCU ports should replace this
 * file with board-specific drivers. No hardware outputs are enabled here.
 */
FattosStatus fattos_platform_init(void)
{
    return FATTOS_OK;
}

FattosStatus fattos_platform_timer_start(uint32_t tick_hz)
{
    return tick_hz == FATTOS_TICK_HZ ? FATTOS_OK : FATTOS_ERR_INVALID_ARG;
}

void fattos_platform_gpio_write(uint16_t pin, bool high)
{
    (void)pin;
    (void)high;
}

bool fattos_platform_gpio_read(uint16_t pin)
{
    (void)pin;
    return false;
}

FattosStatus fattos_platform_pwm_write(uint8_t channel, uint16_t duty_permille)
{
    (void)channel;
    if (duty_permille > 1000U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    /* Default is intentionally inert. */
    return FATTOS_OK;
}

FattosStatus fattos_platform_read_imu(int16_t *ax, int16_t *ay, int16_t *az)
{
    if (ax == NULL || ay == NULL || az == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    *ax = 0;
    *ay = 0;
    *az = 0;
    return FATTOS_OK;
}

FattosStatus fattos_platform_imu_read6(float accel_g[3], float gyro_rads[3])
{
    if (accel_g == NULL || gyro_rads == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 3; ++i) {
        accel_g[i] = 0.0f;
        gyro_rads[i] = 0.0f;
    }
    /* Default is intentionally inert; board ports read the real IMU. */
    return FATTOS_OK;
}

void fattos_platform_idle(void)
{
    /* MCU port may use __WFI() here after checking interrupt semantics. */
}

uint32_t fattos_platform_irq_lock(void)
{
    /* Host default is single-threaded. MCU ports must disable interrupts
       (e.g. read+clear PRIMASK on Cortex-M) and return the saved state. */
    return 0U;
}

void fattos_platform_irq_unlock(uint32_t key)
{
    /* MCU ports restore the interrupt state saved by irq_lock(). */
    (void)key;
}

FattosStatus fattos_platform_watchdog_kick(void)
{
    /* Default build has no hardware watchdog. */
    return FATTOS_OK;
}
