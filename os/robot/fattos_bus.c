#include "fattos/fattos_bus.h"

/*
 * Thin, NULL-safe wrappers over the per-bus vtable. Every helper answers
 * FATTOS_ERR_NOT_IMPLEMENTED (not a crash, not silence) when a board did
 * not wire that bus class, so a misconfigured manifest fails loudly at
 * the first read instead of returning phantom data.
 */

FattosStatus fattos_i2c_write(const FattosBus *bus, uint8_t addr7,
                              const uint8_t *data, size_t len)
{
    if (bus == NULL || bus->ops == NULL || bus->ops->i2c_write_read == NULL ||
        (len > 0U && data == NULL)) {
        return FATTOS_ERR_NOT_IMPLEMENTED;
    }
    return bus->ops->i2c_write_read(bus->ops->ctx, addr7, data, len, NULL, 0U);
}

FattosStatus fattos_i2c_read(const FattosBus *bus, uint8_t addr7, uint8_t reg,
                             uint8_t *data, size_t len)
{
    if (bus == NULL || bus->ops == NULL || bus->ops->i2c_write_read == NULL ||
        data == NULL || len == 0U) {
        return FATTOS_ERR_NOT_IMPLEMENTED;
    }
    return bus->ops->i2c_write_read(bus->ops->ctx, addr7, &reg, 1U, data, len);
}

FattosStatus fattos_i2c_write_u8(const FattosBus *bus, uint8_t addr7,
                                 uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    return fattos_i2c_write(bus, addr7, buf, sizeof(buf));
}

FattosStatus fattos_i2c_read_u8(const FattosBus *bus, uint8_t addr7,
                                uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    return fattos_i2c_read(bus, addr7, reg, value, 1U);
}

FattosStatus fattos_spi_transfer(const FattosBus *bus, uint16_t cs_pin,
                                 const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (bus == NULL || bus->ops == NULL || bus->ops->spi_transfer == NULL ||
        len == 0U) {
        return FATTOS_ERR_NOT_IMPLEMENTED;
    }
    return bus->ops->spi_transfer(bus->ops->ctx, cs_pin, tx, rx, len);
}

FattosStatus fattos_uart_write(const FattosBus *bus, uint8_t port,
                               const uint8_t *buf, size_t len)
{
    if (bus == NULL || bus->ops == NULL || bus->ops->uart_write == NULL ||
        (len > 0U && buf == NULL)) {
        return FATTOS_ERR_NOT_IMPLEMENTED;
    }
    return bus->ops->uart_write(bus->ops->ctx, port, buf, len);
}

int fattos_uart_read(const FattosBus *bus, uint8_t port, uint8_t *buf,
                     size_t cap)
{
    if (bus == NULL || bus->ops == NULL || bus->ops->uart_read == NULL ||
        buf == NULL || cap == 0U) {
        return -1;
    }
    return bus->ops->uart_read(bus->ops->ctx, port, buf, cap);
}

FattosStatus fattos_can_send(const FattosBus *bus, uint32_t id,
                             const uint8_t *data, size_t len)
{
    if (bus == NULL || bus->ops == NULL || bus->ops->can_send == NULL ||
        len > 8U || (len > 0U && data == NULL)) {
        return FATTOS_ERR_NOT_IMPLEMENTED;
    }
    return bus->ops->can_send(bus->ops->ctx, id, data, len);
}

int fattos_can_recv(const FattosBus *bus, uint32_t *id, uint8_t *data,
                    size_t cap)
{
    if (bus == NULL || bus->ops == NULL || bus->ops->can_recv == NULL ||
        id == NULL || cap < 8U) {
        return -1;
    }
    return bus->ops->can_recv(bus->ops->ctx, id, data, cap);
}
