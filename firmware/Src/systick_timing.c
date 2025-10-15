/*
 * systick_timing.c
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * SysTick timer configuration for millisecond timing
 * Provides HAL-free timing functions for discharge test
 */

#include "systick_timing.h"


/* ==================== Timing Variables ==================== */

// Global millisecond counter
static volatile uint32_t system_tick_ms = 0;

/* ==================== Functions ==================== */

/**
 * Initialize SysTick timer for 1ms interrupts
 * Call this after Clock_Init() in your main
 */
void SysTick_Init(void)
{
    uint32_t clock_freq = 16000000;  // 16MHz HSI16

    // Configure SysTick to interrupt every 1ms
    // SysTick counts down from reload value to 0
    // For 1ms at 16MHz: need to count 16,000 ticks

    // Disable SysTick during configuration
    SYST_CSR = 0;

    // Set reload value: (16,000,000 / 1,000) - 1 = 15,999
    SYST_RVR = (clock_freq / 1000) - 1;

    // Clear current value
    SYST_CVR = 0;

    // Configure and enable SysTick:
    // - Enable counter
    // - Enable interrupt
    // - Use processor clock (16MHz)
    SYST_CSR = SYST_CSR_ENABLE | SYST_CSR_TICKINT | SYST_CSR_CLKSOURCE;
}

/**
 * SysTick interrupt handler - called every 1ms
 * This MUST be named exactly "SysTick_Handler" to override the default
 *
 * NOTE: Make sure this function name matches what's in your startup file!
 */
void SysTick_Handler(void)
{
    system_tick_ms++;
}

/**
 * Get current system time in milliseconds
 * This is your replacement for HAL_GetTick()
 */
uint32_t GetTick_ms(void)
{
    return system_tick_ms;
}

/**
 * Blocking delay in milliseconds
 * WARNING: This blocks execution - only use for short delays!
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = GetTick_ms();
    while ((GetTick_ms() - start) < ms) {
        // Busy wait
    }
}

/**
 * Get elapsed time in milliseconds since a start time
 * Handles uint32_t overflow (wraps at ~49.7 days)
 */
uint32_t GetElapsedTime_ms(uint32_t start_time)
{
    return GetTick_ms() - start_time;
}

/**
 * Check if a timeout has occurred
 * Returns true if timeout_ms has elapsed since start_time
 */
bool CheckTimeout_ms(uint32_t start_time, uint32_t timeout_ms)
{
    return (GetElapsedTime_ms(start_time) >= timeout_ms);
}
