/*
 * rapid_transmission_test.h
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Test B: Rapid Multi-Transmission Test
 */

#ifndef RAPID_TRANSMISSION_TEST_H_
#define RAPID_TRANSMISSION_TEST_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the rapid transmission test
 */
void RapidTransmissionTest_Init(void);

/**
 * Start the rapid transmission test
 * Battery should be at ~50% SOC
 * DC load should be ready for manual control
 */
void RapidTransmissionTest_Start(void);

/**
 * Update function - call frequently in main loop
 * Guides user through applying/removing load at correct times
 */
void RapidTransmissionTest_Update(void);

/**
 * Stop the test
 * Called automatically or can be called manually
 */
void RapidTransmissionTest_Stop(const char* reason);

/**
 * Check if test is running
 */
bool RapidTransmissionTest_IsRunning(void);

/**
 * Get current transmission number (1-3)
 */
uint8_t RapidTransmissionTest_GetCurrentTxNumber(void);

/**
 * Request test abort
 * Safe to call from interrupt
 */
void RapidTransmissionTest_RequestAbort(void);

#endif /* RAPID_TRANSMISSION_TEST_H_ */
