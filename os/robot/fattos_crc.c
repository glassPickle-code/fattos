#include "fattos/fattos_crc.h"

uint16_t fattos_crc16(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = 0xFFFFU;
    if (bytes == NULL) {
        return crc;
    }
    for (size_t i = 0U; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)bytes[i] << 8);
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint32_t fattos_crc32(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFU;
    if (bytes == NULL) {
        return crc;
    }
    for (size_t i = 0U; i < length; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            uint32_t mask = (crc & 1U) != 0U ? 0xFFFFFFFFU : 0U;
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}
