#ifndef FATTOS_CRC_H
#define FATTOS_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Check value for
   "123456789" is 0x29B1. Used in telemetry frames. */
uint16_t fattos_crc16(const void *data, size_t length);

/* CRC-32 (reflected, poly 0xEDB88320, init 0xFFFFFFFF, final xor).
   Check value for "123456789" is 0xCBF43926. */
uint32_t fattos_crc32(const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
