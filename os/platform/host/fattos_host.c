#include "fattos_host.h"
#include "fattos/fattos.h"

/* Host tests/demo explicitly advance ticks for deterministic behavior. */
void fattos_host_tick(void)
{
    fattos_tick_isr();
}

void fattos_host_advance(uint32_t ticks)
{
    for (uint32_t i = 0U; i < ticks; ++i) {
        fattos_host_tick();
    }
}
