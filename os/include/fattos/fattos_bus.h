#ifndef FATTOS_BUS_H
#define FATTOS_BUS_H

#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Peripheral bus layer — one vtable per bus instance, so a board can mix
 * several I2C buses, an SPI bus and a UART without any global state.
 *
 * Drivers talk to FattosBus only; they never call the platform HAL
 * directly. An MCU port fills FattosBusOps with its peripheral drivers
 * (or wraps vendor HAL calls); tests and host builds plug in a simulated
 * bus. A bus entry with NULL ops for a class means "not wired": helpers
 * return FATTOS_ERR_NOT_IMPLEMENTED, never a crash.
 *
 *   static const FattosBusOps my_i2c0_ops = { .i2c_write_read = ..., .ctx = &hi2c1 };
 *   static const FattosBus i2c0 = {"i2c0", &my_i2c0_ops};
 *   fattos_i2c_read_u8(&i2c0, 0x68, 0x75, &who);
 */
typedef struct {
    /* I2C combined transaction: write wr_len bytes, repeated-start read
       rd_len bytes. The register access pattern every sensor uses. */
    FattosStatus (*i2c_write_read)(void *ctx, uint8_t addr7,
                                   const uint8_t *wr, size_t wr_len,
                                   uint8_t *rd, size_t rd_len);
    /* SPI full-duplex transfer, chip select managed by the port around
       the call. tx may be NULL to clock in dummy bytes; rx may be NULL
       to discard. */
    FattosStatus (*spi_transfer)(void *ctx, uint16_t cs_pin, const uint8_t *tx,
                                 uint8_t *rx, size_t len);
    /* UART write; read returns bytes read (0 = nothing pending, <0 error). */
    FattosStatus (*uart_write)(void *ctx, uint8_t port, const uint8_t *buf,
                               size_t len);
    int (*uart_read)(void *ctx, uint8_t port, uint8_t *buf, size_t cap);
    /* CAN frame send/receive; recv returns bytes stored (0 empty, <0 error). */
    FattosStatus (*can_send)(void *ctx, uint32_t id, const uint8_t *data,
                             size_t len);
    int (*can_recv)(void *ctx, uint32_t *id, uint8_t *data, size_t cap);
    void *ctx; /* bus handle passed to every op (I2C/SPI peripheral, ...) */
} FattosBusOps;

typedef struct {
    const char *name;           /* diagnostics, e.g. "i2c0" */
    const FattosBusOps *ops;    /* NULL = bus not present */
} FattosBus;

/* Register helpers built on i2c_write_read. addr7 is the 7-bit address. */
FattosStatus fattos_i2c_write(const FattosBus *bus, uint8_t addr7,
                              const uint8_t *data, size_t len);
FattosStatus fattos_i2c_read(const FattosBus *bus, uint8_t addr7, uint8_t reg,
                             uint8_t *data, size_t len);
FattosStatus fattos_i2c_write_u8(const FattosBus *bus, uint8_t addr7,
                                 uint8_t reg, uint8_t value);
FattosStatus fattos_i2c_read_u8(const FattosBus *bus, uint8_t addr7,
                                uint8_t reg, uint8_t *value);

/* SPI / UART / CAN convenience wrappers (NULL-safe ops, FATTOS_ERR_NOT_IMPLEMENTED
   when the bus does not provide that class). */
FattosStatus fattos_spi_transfer(const FattosBus *bus, uint16_t cs_pin,
                                 const uint8_t *tx, uint8_t *rx, size_t len);
FattosStatus fattos_uart_write(const FattosBus *bus, uint8_t port,
                               const uint8_t *buf, size_t len);
int fattos_uart_read(const FattosBus *bus, uint8_t port, uint8_t *buf,
                     size_t cap);
FattosStatus fattos_can_send(const FattosBus *bus, uint32_t id,
                             const uint8_t *data, size_t len);
int fattos_can_recv(const FattosBus *bus, uint32_t *id, uint8_t *data,
                    size_t cap);

#ifdef __cplusplus
}
#endif

#endif
