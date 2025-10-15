/*
 * discharge_cycle_test.h
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Automated discharge cycle test for BMS validation
 */

#ifndef DISCHARGE_CYCLE_TEST_H_
#define DISCHARGE_CYCLE_TEST_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the discharge test
 * Call this once before starting the test
 */
void DischargeTest_Init(void);

/**
 * Start the discharge test
 * Call this after you've started the DC load sequence
 */
void DischargeTest_Start(void);

/**
 * Main test update function
 * Call this frequently in your main loop (e.g., every 100ms)
 * This function is non-blocking and handles all timing internally
 */
void DischargeTest_Update(void);

/**
 * Stop the discharge test
 * Called automatically when test completes or can be called manually
 *
 * @param reason: String describing why test was stopped
 */
void DischargeTest_Stop(const char* reason);

/**
 * Request test abort
 * Safe to call from interrupt or button press
 * Test will stop at next Update() call
 */
void DischargeTest_RequestAbort(void);

/**
 * Check if test is currently running
 *
 * @return: true if test is running, false otherwise
 */
bool DischargeTest_IsRunning(void);

/**
 * Get current cycle number
 *
 * @return: Current cycle number (1-based)
 */
uint16_t DischargeTest_GetCycleNumber(void);

#endif /* DISCHARGE_CYCLE_TEST_H_ */
