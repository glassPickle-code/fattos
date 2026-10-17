#ifndef FATTOS_HOST_H
#define FATTOS_HOST_H

#include <stdint.h>

/* Advance simulated MCU time by one tick. */
void fattos_host_tick(void);
void fattos_host_advance(uint32_t ticks);

#endif
