# STM32 port checklist

This starter does not contain a fake generic startup file because startup,
clock setup, interrupt vectors, memory layout, and peripheral configuration
depend on the exact MCU and board.

1. Use the matching vendor CMSIS device headers and startup assembly/C file.
2. Add a linker script matching the selected Flash and SRAM sizes.
3. Implement `fattos_platform_init()` to initialize board clocks and safe GPIO
   states, or call it after vendor startup if clocks are configured there.
4. Implement `fattos_platform_timer_start(FATTOS_TICK_HZ)` using a hardware
   timer that fires at exactly `FATTOS_TICK_HZ` (default 1 kHz). The public
   API is in milliseconds and converts with `fattos_ms_to_ticks()`, so any
   tick rate works — but task and timeout resolution is one tick, so 1 kHz
   is the recommended rate for control loops.
5. In the timer ISR, call `fattos_tick_isr()` and clear the timer interrupt flag.
6. Implement PWM with explicit channel bounds and `duty_permille <= 1000`.
7. Implement `fattos_platform_irq_lock()/unlock()` as a true save-and-restore
   of the interrupt state (read+clear PRIMASK, restore on unlock). Queues,
   the memory pool, timer/watchdog registries and safety heartbeats depend
   on it.
8. Implement `fattos_platform_watchdog_kick()` to service an IWDG/WWDG and
   ensure starving it resets the board.
9. Add sensor drivers and error handling; do not use fake zero data in production.
10. Measure callback execution time and worst-case interrupt latency and
    compare against the scheduler's `exec_max_ticks`/`missed_releases`
    counters on target.
11. Connect `fattos_wdt_kick_hardware()` from a periodic task and add
    independent hardware motor-disable circuitry.
12. Add target-specific build and flashing instructions only after the board is known.

Do not enable motors until timing, reset behavior, and emergency-stop behavior
have been tested on the actual hardware.
