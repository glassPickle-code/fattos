#include "fattos_sim_bus.h"
#include "fattos_sensor_internal.h"

#include <math.h>
#include <string.h>

/*
 * Emulated I2C chips behind the FattosBus vtable. Register-level behavior
 * matches the datasheets closely enough to exercise the real drivers:
 * WHO_AM_I values, config registers the drivers write, and data registers
 * built from the physical setpoints via the datasheet scaling.
 */

typedef struct {
    bool installed;
    uint8_t kind;
    uint8_t addr7;
    uint8_t regs[256]; /* config register file (what the driver wrote) */
    float accel_g[3];
    float gyro_dps[3];
    float mag_ut[3];
    float baro_pa;
    float baro_c;
    /* BMP280 raw counts solved so the driver's own compensation yields the
       physical setpoints above (see solve_adc). */
    uint32_t adc_t;
    uint32_t adc_p;
    uint16_t tof_mm;
} SimChip;

/* BMP280 calibration the emulator publishes at 0x88 (typical Bosch values;
   only self-consistency with the shared compensation matters). */
static const uint16_t g_bmp_calib[12] = {
    27504u, (uint16_t)26435, (uint16_t)(int16_t)-1000,
    36477u, (uint16_t)(int16_t)-10685, (uint16_t)3024,
    (uint16_t)2855, (uint16_t)140,     (uint16_t)(int16_t)-7,
    (uint16_t)15500, (uint16_t)(int16_t)-14600, (uint16_t)6000
};

static SimChip g_chips[FATTOS_SENSOR_COUNT];

static SimChip *chip_at(uint8_t addr7)
{
    for (int k = 1; k < (int)FATTOS_SENSOR_COUNT; k++) {
        if (g_chips[k].installed && g_chips[k].addr7 == addr7) {
            return &g_chips[k];
        }
    }
    return NULL;
}

/*
 * Solve for the raw 20-bit ADC count that the datasheet compensation maps
 * to the desired physical value. Temperature compensation is monotonic
 * increasing in adc_t; pressure is monotonic decreasing in adc_p for a
 * fixed adc_t (t_fine). 24 bisection steps cover the 20-bit range.
 */
static uint32_t solve_temp(float target_c)
{
    uint32_t lo = 200000U, hi = 900000U;
    for (int i = 0; i < 24 && hi > lo + 1U; i++) {
        uint32_t mid = lo + (hi - lo) / 2U;
        float t, p;
        fattos_bmp280_compensate(g_bmp_calib, mid, 700000U, &t, &p);
        if (t < target_c) { lo = mid; } else { hi = mid; }
    }
    return lo;
}

static uint32_t solve_press(float target_pa, uint32_t adc_t)
{
    uint32_t lo = 1U, hi = 1048500U;
    for (int i = 0; i < 24 && hi > lo + 1U; i++) {
        uint32_t mid = lo + (hi - lo) / 2U;
        float t, p;
        fattos_bmp280_compensate(g_bmp_calib, adc_t, mid, &t, &p);
        /* pressure decreases as adc_p increases */
        if (p > target_pa) { lo = mid; } else { hi = mid; }
    }
    return lo;
}

static void baro_resolve(SimChip *c)
{
    c->adc_t = solve_temp(c->baro_c);
    c->adc_p = solve_press(c->baro_pa, c->adc_t);
}

static void put_s16_be(uint8_t *dst, int32_t v)
{
    dst[0] = (uint8_t)((uint32_t)v >> 8);
    dst[1] = (uint8_t)((uint32_t)v & 0xFF);
}

static void fill_mpu_data(SimChip *c, uint8_t *out)
{
    float fs_g = 2.0f << ((c->regs[0x1CU] >> 3) & 3U);
    float fs_d = 250.0f << ((c->regs[0x1BU] >> 3) & 3U);

    for (int i = 0; i < 3; i++) {
        int32_t v = (int32_t)(c->accel_g[i] / (fs_g / 32768.0f));
        put_s16_be(&out[i * 2], v);
    }
    put_s16_be(&out[6], (int32_t)((25.0f - 36.53f) * 340.0f)); /* die temp */
    for (int i = 0; i < 3; i++) {
        int32_t v = (int32_t)(c->gyro_dps[i] / (fs_d / 32768.0f));
        put_s16_be(&out[8 + i * 2], v);
    }
}

static void fill_mag_data(SimChip *c, uint8_t *out)
{
    /* driver's scale: 100/1200 uT per LSB => counts = uT * 12 */
    for (int i = 0; i < 3; i++) {
        int32_t v = (int32_t)lroundf(c->mag_ut[i] * 12.0f);
        put_s16_be(&out[i * 2], v);
    }
}

static FattosStatus bus_i2c_write_read(void *ctx, uint8_t addr7,
                                       const uint8_t *wr, size_t wr_len,
                                       uint8_t *rd, size_t rd_len)
{
    (void)ctx;
    SimChip *c = chip_at(addr7);
    if (c == NULL) { return FATTOS_ERR_PLATFORM; } /* NACK: nobody home */

    if (wr_len > 0U) {
        uint8_t reg = wr[0];
        if (wr_len > 1U) {
            memcpy(&c->regs[reg], &wr[1],
                   wr_len - 1U < sizeof(c->regs) - reg
                       ? wr_len - 1U
                       : sizeof(c->regs) - reg);
        }
        if (rd_len == 0U) { return FATTOS_OK; }
    } else if (rd_len == 0U) {
        return FATTOS_OK;
    }

    uint8_t reg = (wr != NULL && wr_len > 0U) ? wr[0] : (uint8_t)0U;
    if (rd == NULL) { return FATTOS_ERR_INVALID_ARG; }
    memset(rd, 0, rd_len);

    switch ((FattosSensorKind)c->kind) {
    case FATTOS_SENSOR_MPU6050: {
        if (reg == 0x75U && rd_len >= 1U) {
            rd[0] = 0x68U;
        } else if (reg == 0x3BU && rd_len >= 14U) {
            fill_mpu_data(c, rd);
        } else if (rd_len >= 1U && reg < 0x6BU) {
            rd[0] = c->regs[reg];
        }
        return FATTOS_OK;
    }
    case FATTOS_SENSOR_QMC5883: {
        if (reg == 0x0DU && rd_len >= 1U) {
            rd[0] = 0xFFU;
        } else if (reg == 0x00U && rd_len >= 6U) {
            fill_mag_data(c, rd);
        } else if (rd_len >= 1U) {
            rd[0] = c->regs[reg];
        }
        return FATTOS_OK;
    }
    case FATTOS_SENSOR_BMP280: {
        if (reg == 0xD0U && rd_len >= 1U) {
            rd[0] = 0x58U;
        } else if (reg == 0x88U) {
            size_t n = rd_len < 24U ? rd_len : 24U;
            for (size_t i = 0; i < n; i++) {
                rd[i] = (uint8_t)((g_bmp_calib[i / 2U] >>
                                   ((i % 2U == 0U) ? 0U : 8U)) & 0xFFU);
            }
        } else if (reg == 0xF7U && rd_len >= 6U) {
            rd[0] = (uint8_t)(c->adc_p >> 12);
            rd[1] = (uint8_t)(c->adc_p >> 4);
            rd[2] = (uint8_t)((c->adc_p & 0x0FU) << 4);
            rd[3] = (uint8_t)(c->adc_t >> 12);
            rd[4] = (uint8_t)(c->adc_t >> 4);
            rd[5] = (uint8_t)((c->adc_t & 0x0FU) << 4);
        } else if (rd_len >= 1U) {
            rd[0] = c->regs[reg];
        }
        return FATTOS_OK;
    }
    case FATTOS_SENSOR_VL53L0X: {
        if (reg == 0xC0U && rd_len >= 1U) {
            rd[0] = 0xEEU;
        } else if (reg == 0x1EU && rd_len >= 2U) {
            rd[0] = (uint8_t)(c->tof_mm & 0xFFU);
            rd[1] = (uint8_t)(c->tof_mm >> 8);
        } else if (rd_len >= 1U) {
            rd[0] = c->regs[reg];
        }
        return FATTOS_OK;
    }
    default:
        return FATTOS_ERR_PLATFORM;
    }
}

static const FattosBusOps g_sim_ops = {
    .i2c_write_read = bus_i2c_write_read,
    .spi_transfer = NULL,
    .uart_write = NULL,
    .uart_read = NULL,
    .can_send = NULL,
    .can_recv = NULL,
    .ctx = NULL
};

static const FattosBus g_sim_bus = { "sim-i2c0", &g_sim_ops };

const FattosBus *fattos_sim_bus(void)
{
    return &g_sim_bus;
}

void fattos_sim_reset(void)
{
    memset(g_chips, 0, sizeof(g_chips));
}

void fattos_sim_install(FattosSensorKind kind, uint8_t addr7)
{
    if (kind == FATTOS_SENSOR_NONE || kind >= FATTOS_SENSOR_COUNT) {
        return;
    }
    SimChip *c = &g_chips[kind];
    c->installed = true;
    c->kind = (uint8_t)kind;
    c->addr7 = addr7;
    c->baro_pa = 101325.0f;
    c->baro_c = 25.0f;
    c->tof_mm = 500U;
    c->accel_g[0] = c->accel_g[1] = 0.0f;
    c->accel_g[2] = 1.0f; /* rest: gravity along +Z */
    c->mag_ut[0] = 20.0f;
    c->mag_ut[1] = 5.0f;
    c->mag_ut[2] = 40.0f;
    baro_resolve(c);
}

void fattos_sim_set_present(FattosSensorKind kind, bool present)
{
    if (kind == FATTOS_SENSOR_NONE || kind >= FATTOS_SENSOR_COUNT) {
        return;
    }
    g_chips[kind].installed = present;
}

void fattos_sim_imu_values(const float accel_g[3], const float gyro_dps[3])
{
    SimChip *c = &g_chips[FATTOS_SENSOR_MPU6050];
    for (int i = 0; i < 3; i++) {
        c->accel_g[i] = accel_g[i];
        c->gyro_dps[i] = gyro_dps[i];
    }
}

void fattos_sim_mag_values(const float mag_ut[3])
{
    SimChip *c = &g_chips[FATTOS_SENSOR_QMC5883];
    for (int i = 0; i < 3; i++) { c->mag_ut[i] = mag_ut[i]; }
}

void fattos_sim_baro_values(float pressure_pa, float temp_c)
{
    SimChip *c = &g_chips[FATTOS_SENSOR_BMP280];
    c->baro_pa = pressure_pa;
    c->baro_c = temp_c;
    baro_resolve(c);
}

void fattos_sim_tof_value(float range_m)
{
    SimChip *c = &g_chips[FATTOS_SENSOR_VL53L0X];
    int32_t mm = (int32_t)lroundf(range_m * 1000.0f);
    if (mm < 0) { mm = 0; }
    if (mm > 65535) { mm = 65535; }
    c->tof_mm = (uint16_t)mm;
}