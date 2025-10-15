/*
 * temp_threshold_test.h
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Test A: Temperature Threshold Crossing Test
 */

#ifndef TEMP_THRESHOLD_TEST_H_
#define TEMP_THRESHOLD_TEST_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the temperature threshold test
 */
void TempThresholdTest_Init(void);

/**
 * Start the temperature threshold test
 * Call this after moving battery to opposite temperature
 */
void TempThresholdTest_Start(void);

/**
 * Update function - call frequently in main loop
 * Takes readings every 10 minutes automatically
 */
void TempThresholdTest_Update(void);

/**
 * Stop the test
 * Called automatically or can be called manually
 */
void TempThresholdTest_Stop(const char* reason);

/**
 * Check if test is running
 */
bool TempThresholdTest_IsRunning(void);

/**
 * Get current reading count
 */
uint16_t TempThresholdTest_GetReadingCount(void);

#endif /* TEMP_THRESHOLD_TEST_H_ */
