#include "fattos/fattos_sensor.h"
#include "fattos_sensor_internal.h"

#include <math.h>
#include <string.h>

/*
 * The registry that makes the manifest smart: one FattosSensorEntry line
 * ("imu" = MPU6050 @ 200 Hz, +/-4 g) is enough for the OS to know which
 * driver to bind, how to configure the chip, how to convert its raw counts
 * into SI units, and when the data is stale or physically impossible.
 *
 * Every driver here is the real thing (WHO_AM_I probe, datasheet scaling,
 * integer compensation for the BMP280). Tests plug a simulated bus carrying
 * emulated chips behind the same FattosBus vtable an MCU port uses — so the
 * driver code proved on the host is the driver code that runs on silicon.
 */

/* Sample valid bits. */
#define S_VALID_ACCEL 0x01U
#define S_VALID_GYRO  0x02U
#define S_VALID_MAG   0x04U
#define S_VALID_PRES  0x08U
#define S_VALID_TEMP  0x10U
#define S_VALID_RANGE 0x20U

/* Plausibility envelopes (generous; catch dead/wrong-protocol parts, not
   vigorous flight). */
#define ACCEL_MIN_G 0.02f
#define PRES_MIN_PA 30000.0f
#define PRES_MAX_PA 120000.0f
#define TEMP_MIN_C (-40.0f)
#define TEMP_MAX_C (125.0f)
#define RANGE_MAX_M 4.0f
#define BUS_ERROR_LIMIT 5U /* consecutive bus errors -> ERROR health */

typedef struct {
    FattosSensorEntry entry;
    const FattosBus *bus;
    FattosSensorSample last;
    FattosSensorHealth health;
    uint32_t period_ms;    /* 1000/rate_hz, min 1 */
    uint32_t next_due_ms;
    uint32_t errors;       /* cumulative read failures */
    uint32_t consec_errors;
    uint8_t manifest_pos;  /* position implied by manifest order */
    bool started;

    /* conversion calibration */
    float accel_scale; /* LSB -> g */
    float gyro_scale;  /* LSB -> rad/s */
    float mag_scale;   /* LSB -> uT */
    float gyro_bias[3];

    /* driver private state */
    uint16_t bmp_calib[12]; /* BMP280 dig_T/P words, laid out as u16 */

    /* auto-calibration accumulators */
    FattosCalState cal_state;
    uint16_t cal_remaining;
    uint32_t cal_count;
    float cal_sum[3];
} SensorSlot;

static SensorSlot g_sensors[FATTOS_MAX_SENSORS];
static size_t g_count;

/* ---------------- MPU6050: 6-axis IMU ---------------- */

#define MPU6050_REG_SMPLRT_DIV 0x19U
#define MPU6050_REG_CONFIG     0x1AU
#define MPU6050_REG_GYRO_CFG   0x1BU
#define MPU6050_REG_ACCEL_CFG  0x1CU
#define MPU6050_REG_PWR1       0x6BU
#define MPU6050_REG_WHO        0x75U
#define MPU6050_REG_DATA       0x3BU
#define MPU6050_WHO_AM_I       0x68U

static FattosStatus mpu6050_probe(SensorSlot *s, uint8_t *who)
{
    return fattos_i2c_read_u8(s->bus, s->entry.addr7, MPU6050_REG_WHO, who);
}

static FattosStatus mpu6050_configure(SensorSlot *s)
{
    const FattosSensorEntry *e = &s->entry;
    const FattosBus *bus = s->bus;
    uint8_t addr = e->addr7;
    FattosStatus st;

    /* which full-scale ranges were asked for (nearest legal setting) */
    float fs_g = e->range_g > 0.0f ? e->range_g : 2.0f;
    float fs_d = e->range_dps > 0.0f ? e->range_dps : 250.0f;
    uint8_t afs_sel, gfs_sel;

    if (fs_g > 16.0f)      { fs_g = 16.0f; afs_sel = 3U; }
    else if (fs_g > 8.0f)  { afs_sel = 3U; }
    else if (fs_g > 4.0f)  { afs_sel = 2U; }
    else if (fs_g > 2.0f)  { afs_sel = 1U; }
    else                   { afs_sel = 0U; }
    fs_g = (afs_sel == 0U) ? 2.0f : (afs_sel == 1U) ? 4.0f :
           (afs_sel == 2U) ? 8.0f : 16.0f;

    if (fs_d > 1000.0f)     { fs_d = 2000.0f; gfs_sel = 3U; }
    else if (fs_d > 500.0f) { gfs_sel = 2U; }
    else if (fs_d > 250.0f) { gfs_sel = 1U; }
    else                    { gfs_sel = 0U; }
    fs_d = (gfs_sel == 0U) ? 250.0f : (gfs_sel == 1U) ? 500.0f :
           (gfs_sel == 2U) ? 1000.0f : 2000.0f;

    s->accel_scale = fs_g / 32768.0f;                 /* g per LSB */
    s->gyro_scale = fs_d * (float)M_PI / 180.0f / 32768.0f; /* rad/s per LSB */

    /* wake from sleep, internal 8 MHz PLL, no DLPF (ODR = 1 kHz) */
    st = fattos_i2c_write_u8(bus, addr, MPU6050_REG_PWR1, 0x01U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, MPU6050_REG_CONFIG, 0x00U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, MPU6050_REG_GYRO_CFG, gfs_sel << 3);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, MPU6050_REG_ACCEL_CFG, afs_sel << 3);
    if (st != FATTOS_OK) { return st; }

    /* sample-rate divider: 1 kHz / (1 + rate) from ODR */
    {
        uint16_t rate = e->rate_hz > 0U ? e->rate_hz : 100U;
        uint16_t div = (rate <= 1000U) ? (1000U / rate) : 0U;
        if (div > 0U) { div = (uint16_t)(div - 1U); }
        return fattos_i2c_write_u8(bus, addr, MPU6050_REG_SMPLRT_DIV,
                                   (uint8_t)div);
    }
}

static FattosStatus mpu6050_read(SensorSlot *s, FattosSensorSample *out)
{
    uint8_t raw[14];
    int16_t v;
    FattosStatus st = fattos_i2c_read(s->bus, s->entry.addr7,
                                      MPU6050_REG_DATA, raw, sizeof(raw));
    if (st != FATTOS_OK) { return st; }

    for (int i = 0; i < 3; i++) {
        v = (int16_t)((uint16_t)raw[i * 2U] << 8 | raw[i * 2U + 1U]);
        out->accel_g[i] = (float)v * s->accel_scale;
    }
    v = (int16_t)((uint16_t)raw[6] << 8 | raw[7]);
    out->temp_c = (float)v / 340.0f + 36.53f;
    for (int i = 0; i < 3; i++) {
        v = (int16_t)((uint16_t)raw[8 + i * 2U] << 8 | raw[9 + i * 2U]);
        out->gyro_rads[i] = (float)v * s->gyro_scale - s->gyro_bias[i];
    }
    out->valid = (uint8_t)(S_VALID_ACCEL | S_VALID_GYRO | S_VALID_TEMP);
    return FATTOS_OK;
}

/* ---------------- QMC5883: magnetometer ---------------- */

#define QMC_REG_DATA  0x00U
#define QMC_REG_SETB  0x0BU
#define QMC_REG_CFG1  0x09U /* period | rng | osr */
#define QMC_REG_CFG2  0x0AU /* soft reset bit 0x80 */
#define QMC_REG_CHIP  0x0DU /* CHIP_ID: 0xFF (5883L) / 0x01 (5883) */

static FattosStatus qmc5883_probe(SensorSlot *s, uint8_t *who)
{
    return fattos_i2c_read_u8(s->bus, s->entry.addr7, QMC_REG_CHIP, who);
}

static FattosStatus qmc5883_configure(SensorSlot *s)
{
    const FattosBus *bus = s->bus;
    uint8_t addr = s->entry.addr7;
    FattosStatus st;

    st = fattos_i2c_write_u8(bus, addr, QMC_REG_CFG2, 0x80U); /* soft reset */
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, QMC_REG_SETB, 0x01U); /* set/reset on */
    if (st != FATTOS_OK) { return st; }
    /* QMC5883 ODR field (bits 0-1): 10/50/100/200 Hz */
    {
        uint16_t rate = s->entry.rate_hz;
        uint8_t odr = (rate >= 200U) ? 3U : (rate >= 100U) ? 2U :
                      (rate >= 50U) ? 1U : 0U;
        /* rng field = 0: +-2 gauss (1200 LSB/g, the datasheet-default and
           most sensitive range); osr field = 0xC0: 512, max oversampling */
        uint8_t cfg = (uint8_t)(odr | 0xC0U);
        st = fattos_i2c_write_u8(bus, addr, QMC_REG_CFG1, cfg);
        if (st != FATTOS_OK) { return st; }
    }
    /* +-2 gauss: 1200 LSB/g = 12 LSB/uT => 1/12 uT per LSB */
    s->mag_scale = 100.0f / 1200.0f;
    return FATTOS_OK;
}

static FattosStatus qmc5883_read(SensorSlot *s, FattosSensorSample *out)
{
    uint8_t raw[7]; /* x,y,z, status, ... at 0x00; enough for data+temp if any */
    FattosStatus st = fattos_i2c_read(s->bus, s->entry.addr7, QMC_REG_DATA,
                                      raw, 6U);
    if (st != FATTOS_OK) { return st; }

    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)((uint16_t)raw[i * 2U + 1U] << 8 | raw[i * 2U]);
        out->mag_ut[i] = (float)v * s->mag_scale;
    }
    out->valid = S_VALID_MAG;
    (void)raw;
    return FATTOS_OK;
}

/* ---------------- BMP280: barometer (datasheet compensation) ---------------- */

#define BMP_REG_ID    0xD0U
#define BMP_REG_RESET 0xE0U
#define BMP_REG_CAL   0x88U
#define BMP_REG_CFG   0xF5U
#define BMP_REG_CTRL  0xF4U
#define BMP_REG_DATA  0xF7U
#define BMP_ID        0x58U

void fattos_bmp280_compensate(const uint16_t calib[12], uint32_t adc_t,
                              uint32_t adc_p, float *temp_c,
                              float *pressure_pa)
{
    int32_t dig_T1 = (int32_t)calib[0];
    int32_t dig_T2 = (int32_t)(int16_t)calib[1];
    int32_t dig_T3 = (int32_t)(int16_t)calib[2];
    int32_t dig_P1 = (int32_t)calib[3];
    int32_t dig_P2 = (int32_t)(int16_t)calib[4];
    int32_t dig_P3 = (int32_t)(int16_t)calib[5];
    int32_t dig_P4 = (int32_t)(int16_t)calib[6];
    int32_t dig_P5 = (int32_t)(int16_t)calib[7];
    int32_t dig_P6 = (int32_t)(int16_t)calib[8];
    int32_t dig_P7 = (int32_t)(int16_t)calib[9];
    int32_t dig_P8 = (int32_t)(int16_t)calib[10];
    int32_t dig_P9 = (int32_t)(int16_t)calib[11];
    int32_t var1, var2, t_fine;
    int32_t p;

    var1 = ((((adc_t >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_t >> 4) - ((int32_t)dig_T1)) * ((adc_t >> 4) -
            ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    if (temp_c != NULL) {
        *temp_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;
    }
    if (pressure_pa == NULL) { return; }

    var1 = ((int64_t)t_fine >> 1) - (int64_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int64_t)dig_P6);
    var2 = var2 + ((var1 * ((int64_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int64_t)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((int64_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((int64_t)32768 + var1)) * ((int64_t)dig_P1)) >> 15;
    if (var1 == 0) { *pressure_pa = 0.0f; return; }

    p = (((int64_t)1048576 - adc_p) - (var2 >> 12)) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint64_t)var1);
    } else {
        p = (p / (uint64_t)var1) * 2;
    }
    var1 = (((int64_t)dig_P9) * ((p >> 3) * (p >> 3)) >> 24);
    var2 = ((p >> 4) * ((int64_t)dig_P8)) >> 12;
    p = (uint32_t)((p + ((var1 + var2 + dig_P7) >> 4)));
    *pressure_pa = (float)p;
}

static FattosStatus bmp280_probe(SensorSlot *s, uint8_t *who)
{
    return fattos_i2c_read_u8(s->bus, s->entry.addr7, BMP_REG_ID, who);
}

static FattosStatus bmp280_configure(SensorSlot *s)
{
    const FattosBus *bus = s->bus;
    uint8_t addr = s->entry.addr7;
    uint8_t cal[24];
    FattosStatus st;

    st = fattos_i2c_write_u8(bus, addr, BMP_REG_RESET, 0xB6U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_read(bus, addr, BMP_REG_CAL, cal, sizeof(cal));
    for (int i = 0; i < 12; i++) {
        s->bmp_calib[i] = (uint16_t)((uint16_t)cal[i * 2U + 1U] << 8 |
                                     cal[i * 2U]);
    }

    /* filter coeff from rate: slower ODR -> more filtering */
    {
        uint16_t rate = s->entry.rate_hz;
        uint8_t sbt = (rate >= 50U) ? 0U : (rate >= 25U) ? 2U :
                      (rate >= 10U) ? 4U : 5U;
        st = fattos_i2c_write_u8(bus, addr, BMP_REG_CFG, (uint8_t)(sbt << 2));
        if (st != FATTOS_OK) { return st; }
    }
    /* x16 pressure + x2 temp (osrs_t=2 <<5, osrs_p=5 <<2) + normal mode */
    return fattos_i2c_write_u8(bus, addr, BMP_REG_CTRL, 0x57U);
}

static FattosStatus bmp280_read(SensorSlot *s, FattosSensorSample *out)
{
    uint8_t raw[6];
    FattosStatus st = fattos_i2c_read(s->bus, s->entry.addr7, BMP_REG_DATA,
                                      raw, sizeof(raw));
    if (st != FATTOS_OK) { return st; }

    uint32_t adc_p = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) |
                     ((uint32_t)raw[2] >> 4);
    uint32_t adc_t = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) |
                     ((uint32_t)raw[5] >> 4);
    float t = 0.0f, p = 0.0f;
    fattos_bmp280_compensate(s->bmp_calib, adc_t, adc_p, &t, &p);
    out->temp_c = t;
    out->pressure_pa = p;
    out->valid = (uint8_t)(S_VALID_PRES | S_VALID_TEMP);
    return FATTOS_OK;
}

/* ---------------- VL53L0X: time-of-flight rangefinder ---------------- */

#define TOF_REG_MODEL   0xC0U
#define TOF_MODEL_ID    0xEEU
#define TOF_REG_RANGE   0x1EU /* 16-bit range, millimeters, LSB first */

static FattosStatus vl53l0x_probe(SensorSlot *s, uint8_t *who)
{
    return fattos_i2c_read_u8(s->bus, s->entry.addr7, TOF_REG_MODEL, who);
}

static FattosStatus vl53l0x_configure(SensorSlot *s)
{
    /* Lite driver's continuous mode: boot state machine + signal/reset
       rate defaults. Keep it minimal: the emulator implements exactly this
       sequence and so does a real device's basic init. */
    const FattosBus *bus = s->bus;
    uint8_t addr = s->entry.addr7;
    FattosStatus st;

    st = fattos_i2c_write_u8(bus, addr, 0x80U, 0x01U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, 0x83U, 0x01U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, 0xFFU, 0x01U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, 0x00U, 0x00U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, 0xFFU, 0x00U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, 0x91U, 0x00U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, 0x00U, 0x01U);
    if (st != FATTOS_OK) { return st; }
    st = fattos_i2c_write_u8(bus, addr, 0x30U, 0x02U); /* timing budget ~33 ms */
    if (st != FATTOS_OK) { return st; }
    return fattos_i2c_write_u8(bus, addr, 0x80U, 0x00U);
}

static FattosStatus vl53l0x_read(SensorSlot *s, FattosSensorSample *out)
{
    uint8_t raw[2];
    FattosStatus st = fattos_i2c_read(s->bus, s->entry.addr7, TOF_REG_RANGE,
                                      raw, sizeof(raw));
    if (st != FATTOS_OK) { return st; }

    uint16_t mm = (uint16_t)((uint16_t)raw[1] << 8 | raw[0]);
    out->range_m = (float)mm * 0.001f;
    out->valid = S_VALID_RANGE;
    return FATTOS_OK;
}

/* ---------------- registry table ---------------- */

static bool who_is_valid(FattosSensorKind kind, uint8_t who)
{
    switch (kind) {
    case FATTOS_SENSOR_MPU6050: return who == MPU6050_WHO_AM_I;
    case FATTOS_SENSOR_QMC5883: return who == 0xFFU || who == 0x01U;
    case FATTOS_SENSOR_BMP280:  return who == BMP_ID;
    case FATTOS_SENSOR_VL53L0X: return who == TOF_MODEL_ID;
    default: return false;
    }
}

static FattosStatus driver_read(SensorSlot *s, FattosSensorSample *out)
{
    switch ((FattosSensorKind)s->entry.kind) {
    case FATTOS_SENSOR_MPU6050: return mpu6050_read(s, out);
    case FATTOS_SENSOR_QMC5883: return qmc5883_read(s, out);
    case FATTOS_SENSOR_BMP280:  return bmp280_read(s, out);
    case FATTOS_SENSOR_VL53L0X: return vl53l0x_read(s, out);
    default: return FATTOS_ERR_NOT_IMPLEMENTED;
    }
}

/* The registry binds a driver by kind through these three parallel tables —
   adding a chip is: enum value + three functions + one row here. */
typedef FattosStatus (*probe_fn)(SensorSlot *, uint8_t *);
typedef FattosStatus (*configure_fn)(SensorSlot *);
static const probe_fn g_probe_tab[] = {
    NULL, mpu6050_probe, qmc5883_probe, bmp280_probe, vl53l0x_probe
};
static const configure_fn g_configure_tab[] = {
    NULL, mpu6050_configure, qmc5883_configure, bmp280_configure,
    vl53l0x_configure
};

/* ---------------- public API ---------------- */

const char *fattos_sensor_kind_name(FattosSensorKind kind)
{
    switch (kind) {
    case FATTOS_SENSOR_MPU6050: return "MPU6050";
    case FATTOS_SENSOR_QMC5883: return "QMC5883";
    case FATTOS_SENSOR_BMP280:  return "BMP280";
    case FATTOS_SENSOR_VL53L0X: return "VL53L0X";
    default: return "none";
    }
}

const char *fattos_sensor_health_name(FattosSensorHealth health)
{
    switch (health) {
    case FATTOS_SENSOR_OK:       return "OK";
    case FATTOS_SENSOR_MISSING:  return "MISSING";
    case FATTOS_SENSOR_ERROR:    return "ERROR";
    case FATTOS_SENSOR_STALE:    return "STALE";
    case FATTOS_SENSOR_PLAUSIBLE:return "BOGUS";
    default: return "?";
    }
}

static SensorSlot *find_slot(const char *name)
{
    if (name == NULL) { return NULL; }
    for (size_t i = 0; i < g_count; i++) {
        if (strcmp(g_sensors[i].entry.name, name) == 0) {
            return &g_sensors[i];
        }
    }
    return NULL;
}

void fattos_sensor_reset(void)
{
    memset(g_sensors, 0, sizeof(g_sensors));
    g_count = 0U;
}

size_t fattos_sensor_setup(const FattosSensorEntry *entries, size_t count,
                           const FattosBus *buses, size_t bus_count)
{
    fattos_sensor_reset();
    if (entries == NULL || count == 0U) { return 0U; }
    if (count > FATTOS_MAX_SENSORS) { count = FATTOS_MAX_SENSORS; }

    size_t ok = 0U;
    for (size_t i = 0; i < count; i++) {
        SensorSlot *s = &g_sensors[g_count];
        const FattosSensorEntry *e = &entries[i];
        uint8_t who = 0U;

        s->entry = *e;
        s->manifest_pos = (uint8_t)i;
        s->health = FATTOS_SENSOR_MISSING;
        s->period_ms = 1000U;
        if (e->rate_hz > 0U) { s->period_ms = 1000U / e->rate_hz; }
        if (s->period_ms == 0U) { s->period_ms = 1U; }

        if (e->name == NULL || e->kind >= (uint8_t)FATTOS_SENSOR_COUNT ||
            e->bus >= bus_count || buses == NULL) {
            g_count++;
            continue; /* stays MISSING: reported, not fatal */
        }
        s->bus = &buses[e->bus];

        /* probe WHO_AM_I: wrong chip or no ACK = MISSING, keep going */
        if (g_probe_tab[e->kind] == NULL ||
            g_probe_tab[e->kind](s, &who) != FATTOS_OK ||
            !who_is_valid((FattosSensorKind)e->kind, who)) {
            g_count++;
            continue;
        }
        if (g_configure_tab[e->kind](s) != FATTOS_OK) {
            g_count++;
            continue;
        }

        s->started = true;
        s->health = FATTOS_SENSOR_STALE; /* present, waiting first sample */
        s->next_due_ms = 0U;
        g_count++;
        ok++;
    }
    return ok;
}

/* Plausibility: is this sample physically possible for this chip/config? */
static bool plausible(const SensorSlot *s, const FattosSensorSample *v)
{
    if (v->valid & S_VALID_ACCEL) {
        float mag = sqrtf(v->accel_g[0] * v->accel_g[0] +
                          v->accel_g[1] * v->accel_g[1] +
                          v->accel_g[2] * v->accel_g[2]);
        if (mag < ACCEL_MIN_G) { return false; }
        if (s->entry.range_g > 0.0f && mag > s->entry.range_g * 1.02f) {
            return false;
        }
    }
    if ((v->valid & S_VALID_PRES) &&
        (v->pressure_pa < PRES_MIN_PA || v->pressure_pa > PRES_MAX_PA)) {
        return false;
    }
    if ((v->valid & S_VALID_TEMP) &&
        (v->temp_c < TEMP_MIN_C || v->temp_c > TEMP_MAX_C)) {
        return false;
    }
    if ((v->valid & S_VALID_RANGE) && (v->range_m <= 0.0f ||
                                       v->range_m > RANGE_MAX_M)) {
        return false;
    }
    return true;
}

size_t fattos_sensor_poll(uint32_t now_ms)
{
    size_t read_count = 0U;

    for (size_t i = 0; i < g_count; i++) {
        SensorSlot *s = &g_sensors[i];
        if (!s->started) { continue; }
        if ((int32_t)(now_ms - s->next_due_ms) < 0) { continue; }

        FattosSensorSample v;
        memset(&v, 0, sizeof(v));
        v.timestamp_ms = now_ms;

        FattosStatus st = driver_read(s, &v);
        if (st != FATTOS_OK) {
            s->errors++;
            s->consec_errors++;
            if (s->consec_errors >= BUS_ERROR_LIMIT) {
                s->health = FATTOS_SENSOR_ERROR;
            } else if (s->health == FATTOS_SENSOR_OK) {
                s->health = FATTOS_SENSOR_STALE;
            }
            s->next_due_ms = now_ms + s->period_ms;
            continue;
        }

        s->consec_errors = 0U;
        v.timestamp_ms = now_ms;

        /* auto-calibration: accumulate gyro bias while told to stand still */
        if (s->cal_state == FATTOS_CAL_GYRO_BIAS &&
            (v.valid & S_VALID_GYRO)) {
            for (int k = 0; k < 3; k++) { s->cal_sum[k] += v.gyro_rads[k]; }
            s->cal_count++;
            if (--s->cal_remaining == 0U) {
                for (int k = 0; k < 3; k++) {
                    s->gyro_bias[k] = s->cal_sum[k] / (float)s->cal_count;
                }
                s->cal_state = FATTOS_CAL_DONE;
            }
        }

        if (!plausible(s, &v)) {
            s->health = FATTOS_SENSOR_PLAUSIBLE;
        } else {
            s->last = v;
            s->health = FATTOS_SENSOR_OK;
            read_count++;
        }
        s->next_due_ms = now_ms + s->period_ms;
    }

    /* stale sweep: a sensor that exists but stops delivering data is
       STALE within 3x its period beyond the last good sample */
    for (size_t i = 0; i < g_count; i++) {
        SensorSlot *s = &g_sensors[i];
        if (s->started && s->health == FATTOS_SENSOR_OK &&
            (now_ms - s->last.timestamp_ms) > 3U * s->period_ms) {
            s->health = FATTOS_SENSOR_STALE;
        }
    }
    return read_count;
}

FattosStatus fattos_sensor_read(const char *name, FattosSensorSample *out)
{
    SensorSlot *s = find_slot(name);
    if (s == NULL || out == NULL) { return FATTOS_ERR_INVALID_ARG; }
    *out = s->last;
    if (!s->started || s->health == FATTOS_SENSOR_MISSING ||
        s->health == FATTOS_SENSOR_ERROR) {
        return FATTOS_ERR_TIMEOUT;
    }
    if (s->health == FATTOS_SENSOR_STALE ||
        s->health == FATTOS_SENSOR_PLAUSIBLE) {
        return FATTOS_ERR_TIMEOUT;
    }
    return FATTOS_OK;
}

bool fattos_sensors_prearm_ok(void)
{
    if (g_count == 0U) { return false; } /* manifest declared nothing? */
    for (size_t i = 0; i < g_count; i++) {
        if (g_sensors[i].health != FATTOS_SENSOR_OK) { return false; }
    }
    return true;
}

size_t fattos_sensor_count(void)
{
    return g_count;
}

FattosStatus fattos_sensor_get_info(size_t index, const char **out_name,
                                    uint8_t *out_kind, uint8_t *out_health,
                                    uint32_t *out_errors)
{
    if (index >= g_count) { return FATTOS_ERR_INVALID_ARG; }
    const SensorSlot *s = &g_sensors[index];
    if (out_name)   { *out_name = s->entry.name; }
    if (out_kind)   { *out_kind = s->entry.kind; }
    if (out_health) { *out_health = (uint8_t)s->health; }
    if (out_errors) { *out_errors = s->errors; }
    return FATTOS_OK;
}

FattosStatus fattos_sensor_calibrate(const char *name, FattosCalState what,
                                     uint16_t samples)
{
    SensorSlot *s = find_slot(name);
    if (s == NULL) { return FATTOS_ERR_INVALID_ARG; }

    if (what == FATTOS_CAL_IDLE) {
        s->cal_state = FATTOS_CAL_IDLE;
        return FATTOS_OK;
    }
    if (what != FATTOS_CAL_GYRO_BIAS) { return FATTOS_ERR_INVALID_ARG; }
    if (s->entry.kind != FATTOS_SENSOR_MPU6050) {
        return FATTOS_ERR_NOT_IMPLEMENTED;
    }
    if (samples == 0U) { samples = 200U; }
    s->cal_state = FATTOS_CAL_GYRO_BIAS;
    s->cal_remaining = samples;
    s->cal_count = 0U;
    s->cal_sum[0] = s->cal_sum[1] = s->cal_sum[2] = 0.0f;
    return FATTOS_OK;
}

FattosCalState fattos_sensor_cal_state(const char *name)
{
    SensorSlot *s = find_slot(name);
    return s == NULL ? FATTOS_CAL_FAILED : s->cal_state;
}