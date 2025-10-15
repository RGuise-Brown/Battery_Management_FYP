/*
 * systick_timing.h
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 */

#ifndef SYSTICK_TIMING_H_
#define SYSTICK_TIMING_H_

#include <stdint.h>
#include <stdbool.h>

/* ==================== SysTick Register Definitions ==================== */

// SysTick Base Address
#define SYSTICK_BASE    0xE000E010UL

// SysTick Registers (using volatile pointers)
#define SYST_CSR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))  // Control and Status
#define SYST_RVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))  // Reload Value
#define SYST_CVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))  // Current Value

// SysTick Control and Status Register bits
#define SYST_CSR_ENABLE     (1UL << 0)  // Enable counter
#define SYST_CSR_TICKINT    (1UL << 1)  // Enable interrupt
#define SYST_CSR_CLKSOURCE  (1UL << 2)  // Use processor clock


/**
 * Initialize SysTick timer for 1ms interrupts
 * Must be called after Clock_Init(CLOCK_16MHZ)
 */
void SysTick_Init(void);

/**
 * Get current system time in milliseconds
 * This is your replacement for HAL_GetTick()
 */
uint32_t GetTick_ms(void);

/**
 * Blocking delay in milliseconds
 * WARNING: This blocks execution - only use for short delays!
 */
void delay_ms(uint32_t ms);

/**
 * Get elapsed time in milliseconds since a start time
 */
uint32_t GetElapsedTime_ms(uint32_t start_time);

/**
 * Check if a timeout has occurred
 */
bool CheckTimeout_ms(uint32_t start_time, uint32_t timeout_ms);

#endif /* SYSTICK_TIMING_H_ */
