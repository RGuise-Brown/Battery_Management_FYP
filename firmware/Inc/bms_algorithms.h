/*
 * bms_algorithms.h
 *
 *  Created on: Oct 11, 2025
 *      Author: rachaelguise-brown
 */

#ifndef BMS_ALGORITHMS_H_
#define BMS_ALGORITHMS_H_

#include <stdint.h>
#include <stdbool.h>
#include "battery_monitor.h"

/* ============================================================================
 * CONFIGURATION CONSTANTS (Reference - defined in .c file)
 * ============================================================================ */

/* ============================================================================
 * CONFIGURATION CONSTANTS - TWO TEMPERATURE REGIONS ONLY
 * ============================================================================ */

// Battery capacity at different temperatures (VERIFIED EXPERIMENTAL DATA)
// Only two regions are defined - values that can be tested with available equipment
// - Freezer: -20°C to -30°C → 8.3 Ah capacity
// - Room temp: 20°C to 25°C → 10.5 Ah capacity
#define CAPACITY_VERY_COLD_UAH  8300000    // μAh at T < 0°C (freezer-verified)
#define CAPACITY_WARM_UAH       10500000   // μAh at T ≥ 0°C (room-temp-verified)

// Temperature threshold (in decidegrees = 0.1°C)
// Simple boundary at 0°C - the critical point between the two regions
#define TEMP_COLD_THRESH_DC     0          // 0.0°C - boundary between cold and warm

// Additional reference threshold for very cold conditions
#define TEMP_VERY_COLD_THRESH_DC  -250     // -25.0°C (bottom of freezer range)

// Transmission current
#define TX_CURRENT_MA          1000         // 1000mA pulse - for tests

// Voltage thresholds
#define VOLTAGE_CRITICAL_MV    2000        // 2.5V absolute minimum under load
#define VOLTAGE_WARNING_MV     2500        // 2.7V warning threshold

// SOC threshold (in per-mille)
#define SOC_LOW_THRESH_PM      300         // 30.0%

// Impedance threshold (mOhm)
#define IMPEDANCE_HIGH 			1500 		//1.5Ohms

// Function 3 - Maximum allowed voltage drop during transmission
#define MAX_VOLTAGE_DROP_MV    1000         // 500mV max drop
                                           // TODO: Adjust based on observations

// Temperature history configuration
#define TEMP_HISTORY_SIZE       20      // Store last 20 readings
#define TEMP_TREND_COOLING_FAST -20     // -2.0°C (in decidegrees)
#define TEMP_TREND_COOLING      -5      // -0.5°C
#define TEMP_TREND_WARMING      5       // +0.5°C
#define TEMP_TREND_WARMING_FAST 20      // +2.0°C

// Temperature averaging
#define TEMP_AVG_SAMPLES    5       // Number of samples to average
#define TEMP_SAMPLE_DELAY   100     // Delay between samples in ms


/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * Battery State Structure - ALL INTEGERS
 *
 * Units:
 * - voltage_mv: millivolts (mV) - e.g., 3450 = 3.45V
 * - temperature_dc: decidegrees Celsius (0.1°C) - e.g., -223 = -22.3°C
 * - accumulated_uah: microamp-hours (μAh) - MATCHES BQ35100 - e.g., 5400000 = 5.4Ah
 * - current_ma: milliamps (mA)
 * - impedance_mohm: milliohms (mΩ)
 * - timestamp_ms: milliseconds
 */
typedef struct {
    uint16_t voltage_mv;           // Battery voltage in mV
    int16_t temperature_dc;        // Temperature in 0.1°C (decidegrees)
    int32_t accumulated_uah;      // Accumulated capacity in μAh (MATCHES BQ35100!)
    int16_t current_ma;            // Current in mA (signed, negative = discharge)
    uint16_t impedance_mohm;       // Battery impedance in mΩ
    uint32_t timestamp_ms;         // Timestamp in milliseconds
} BatteryState_Int_t;

/**
 * Decision Enumeration
 *
 * Describes the BMS decision for transmission or operation
 */
typedef enum {
    DECISION_GO,        // Safe to proceed with transmission
    DECISION_WARN,      // Proceed with caution, log warning
    DECISION_BLOCK,     // Do not proceed with transmission
    DECISION_ABORT      // Immediately terminate active transmission
} Decision_t;

/**
 * BMS Result Structure - ALL INTEGERS
 *
 * Units:
 * - predicted_voltage_mv: millivolts (mV)
 * - adjusted_soc_pm: per-mille (0-1000 represents 0-100%)
 *   e.g., 755 = 75.5%, 1000 = 100%, 234 = 23.4%
 * - available_capacity_uah: microamp-hours (μAh) - MATCHES BQ35100
 */
typedef struct {
    Decision_t decision;                // The BMS decision
    char message[128];                  // Human-readable explanation
    uint16_t predicted_voltage_mv;      // Predicted voltage under load (mV)
    uint16_t adjusted_soc_pm;           // Adjusted SOC in per-mille (0-1000)
    uint32_t available_capacity_uah;    // Available capacity at current temp (μAh)
} BmsResult_Int_t;

// Temperature history structures
typedef struct {
    int16_t temperature_dc;     // Temperature in decidegrees (0.1°C)
    uint32_t timestamp_ms;      // When this reading was taken
} TempReading_t;

typedef enum {
    TREND_INSUFFICIENT_DATA,
    TREND_COOLING_FAST,
    TREND_COOLING,
    TREND_STABLE,
    TREND_WARMING,
    TREND_WARMING_FAST
} TempTrend_t;

typedef struct {
    TempTrend_t status;
    int16_t trend_dc_per_hour;      // Temperature change per hour (decidegrees)
    int16_t predicted_temp_1hr_dc;  // Predicted temp in 1 hour (decidegrees)
    char message[128];
    bool capacity_will_decrease;    // True if getting colder
    bool capacity_will_increase;    // True if getting warmer
} TempTrendResult_t;

/* ============================================================================
 * CORE BMS FUNCTIONS (ALL INTEGER OPERATIONS)
 * ============================================================================ */

/**
 * Function 1: Update State of Charge with Temperature Compensation
 *
 * Adjusts the SOC calculation based on temperature-dependent battery capacity.
 * This is the MOST IMPORTANT function based on characterization data showing
 * 20.9% capacity variation between cold and warm conditions.
 *
 * @param state: Current battery state (input)
 * @param result: BMS result with adjusted SOC and capacity (output)
 *
 * Temperature-Capacity Relationship (VERIFIED DATA ONLY):
 * - T < 0°C:  8.3 Ah capacity  (tested in freezer at -20°C to -30°C)
 * - T ≥ 0°C: 10.5 Ah capacity (tested at room temp 20°C to 25°C)
 *
 * The 0°C threshold is the critical boundary between these two regions.
 * No interpolation is used - only experimentally verified values.
 *
 * Example:
 *   state.temperature_dc = -223;  // -22.3°C (freezer)
 *   state.accumulated_uah = 5400000; // 5.4Ah used
 *   bms_update_soc_int(&state, &result);
 *   // result.adjusted_soc_pm = 349 (34.9% remaining)
 *   // result.available_capacity_uah = 8300000 (8.3Ah)
 *   // result.message = "SOC: 34.9% | Available: 8300 mAh @ -22.3°C (COLD)"
 */
void bms_update_soc_int(const BatteryState_Int_t *state,
                        BmsResult_Int_t *result);

/**
 * Function 2: Transmission Go/No-Go Decision
 *
 * Determines whether a satellite transmission should proceed based on:
 * - Current battery voltage
 * - Temperature and temperature-adjusted SOC
 * - Battery impedance
 * - Predicted voltage drop during 500mA pulse
 *
 * @param state: Current battery state (input)
 * @param result: BMS decision and reasoning (output)
 *
 * Decision Logic:
 * - BLOCK if: Predicted voltage < 2500mV OR (Cold + Low SOC)
 * - WARN if: Marginal conditions (cold + low voltage, high impedance, low SOC)
 * - GO if: All conditions acceptable
 *
 * Example:
 *   state.voltage_mv = 3450;         // 3.45V
 *   state.temperature_dc = -223;     // -22.3°C
 *   state.impedance_mohm = 850;      // 850mΩ
 *   bms_transmission_decision_int(&state, &result);
 *   // result.decision = DECISION_WARN or DECISION_BLOCK
 *   // result.message = "WARN: Marginal - Cold (-22C), Predicted V: 3025mV"
 */
void bms_transmission_decision_int(const BatteryState_Int_t *state,
                                   BmsResult_Int_t *result);

/**
 * Function 3: Simple Voltage Drop Monitoring (SIMPLIFIED - NO LOOKUP TABLE)
 *
 * Monitors voltage during transmission and alerts if drop exceeds threshold.
 * This simplified version doesn't use a lookup table - just checks against
 * a specified maximum voltage drop.
 *
 * @param voltage_before_mv: Resting voltage before transmission (mV)
 * @param voltage_during_mv: Voltage measured during transmission (mV)
 * @param max_allowed_drop_mv: Maximum acceptable voltage drop (mV)
 * @param result: BMS decision (continue or abort) (output)
 *
 * Decision Logic:
 * - ABORT if: Voltage < 2500mV OR drop > max_allowed_drop_mv
 * - WARN if: Approaching critical voltage (< 2700mV)
 * - GO if: Voltage drop within acceptable range
 *
 * Example:
 *   voltage_before_mv = 3450;        // 3.45V before pulse
 *   voltage_during_mv = 3080;        // 3.08V during pulse
 *   max_allowed_drop_mv = 500;       // Max 500mV drop allowed
 *   bms_monitor_transmission_int(voltage_before_mv, voltage_during_mv,
 *                                max_allowed_drop_mv, &result);
 *   // Actual drop is 370mV, less than 500mV max -> DECISION_GO
 */
void bms_monitor_transmission_int(uint16_t voltage_before_mv,
                                  uint16_t voltage_during_mv,
                                  uint16_t max_allowed_drop_mv,
                                  BmsResult_Int_t *result);


// ============================================================================
// TEMPERATURE TREND MONITORING AND AVERAGING
// ============================================================================

/**
 * Function 4: Temperature Trend Monitoring
 *
 * Analyzes temperature history to predict when the battery will cross the
 * critical 0°C threshold, which causes a 20.9% capacity change.
 *
 * TWO-REGION MODEL:
 * The only significant capacity change occurs when crossing 0°C. This function
 * specifically monitors for this transition:
 * - Warming from cold (< 0°C) to warm (≥ 0°C): +20.9% capacity increase
 * - Cooling from warm (≥ 0°C) to cold (< 0°C): -20.9% capacity decrease
 *
 * Requires periodic temperature logging using bms_add_temp_reading().
 *
 * @param result: Output with trend analysis and predictions
 *
 * Trend Analysis:
 * - Compares average of 5 oldest readings vs 5 newest readings
 * - Calculates temperature change rate (°C per hour)
 * - Predicts temperature 1 hour in the future
 * - Determines if 0°C threshold will be crossed
 *
 * Trend Statuses:
 * - COOLING_FAST:    > 2.0°C/hr drop → May cross 0°C threshold soon
 * - COOLING:         > 0.5°C/hr drop → Monitor for 0°C approach
 * - STABLE:          ±0.5°C/hr      → No threshold crossing expected
 * - WARMING:         > 0.5°C/hr rise → Monitor for 0°C approach
 * - WARMING_FAST:    > 2.0°C/hr rise → May cross 0°C threshold soon
 * - INSUFFICIENT_DATA: < 10 readings → Need more temperature history
 *
 * Usage Pattern:
 *   1. Periodically (every 10-30 min) add temperature readings:
 *      bms_add_temp_reading(temp_dc, timestamp_ms);
 *
 *   2. Before transmissions or periodically, check trend:
 *      TempTrendResult_t trend;
 *      bms_monitor_temperature_trend(&trend);
 *
 *      if (trend.capacity_will_decrease) {
 *          // Approaching 0°C from warm side - capacity will drop 20.9%
 *          // Consider delaying non-critical transmissions
 *      }
 *
 *      if (trend.capacity_will_increase) {
 *          // Approaching 0°C from cold side - capacity will gain 20.9%
 *          // Good time for transmissions
 *      }
 *
 * Example Output (crossing threshold):
 *   Current temp: 5°C, Trend: -1.5°C/hr
 *   status = TREND_COOLING
 *   trend_dc_per_hour = -15 (-1.5°C/hr)
 *   predicted_temp_1hr_dc = 35 (3.5°C in 1 hour, still above 0°C)
 *   capacity_will_decrease = false (not crossing yet)
 *   message = "Cooling (-1.5°C). No capacity change expected."
 *
 *   But after a few more hours at this rate, will cross 0°C and trigger:
 *   capacity_will_decrease = true
 *   message = "Cooling (-1.5°C). Approaching 0°C threshold."
 *
 * Practical Applications:
 * - Predict the critical 0°C threshold crossing
 * - Delay transmissions if rapid cooling toward 0°C detected
 * - Schedule transmissions when warming toward 0°C (capacity improving)
 * - Provide early warning of the major capacity change event
 * - Log trends for mission analysis
 */
void bms_monitor_temperature_trend(TempTrendResult_t *result);

/**
 * Add Temperature Reading to History
 *
 * Call this periodically (every 10-30 minutes) to build temperature history
 * for trend analysis. Stores readings in a circular buffer of 20 entries.
 *
 * @param temperature_dc: Temperature in decidegrees (0.1°C)
 * @param timestamp_ms: Timestamp when reading was taken (milliseconds)
 *
 * Example:
 *   // Every 10 minutes in your main loop
 *   BatteryMonitorData_t data;
 *   BatteryMonitor_ReadAll(&data);
 *
 *   int16_t temp_dc = data.adc_temp_tenths_c;  // Already in decidegrees
 *   uint32_t timestamp = get_system_time_ms();
 *
 *   bms_add_temp_reading(temp_dc, timestamp);
 *
 * Note: Requires at least 10 readings before trend analysis is available.
 *       Full accuracy achieved with 20 readings (circular buffer fills).
 */
void bms_add_temp_reading(int16_t temperature_dc, uint32_t timestamp_ms);

/**
 * Get Averaged Temperature Reading
 *
 * Takes multiple temperature samples and averages them to reduce noise.
 * Temperature sensors can be noisy - use this instead of single readings
 * for more stable and accurate temperature data.
 *
 * @param monitor_data: Your battery monitor data structure
 * @return: Averaged temperature in decidegrees (0.1°C)
 *
 * Configuration:
 * - TEMP_AVG_SAMPLES: Number of samples to average (default: 5)
 * - TEMP_SAMPLE_DELAY: Delay between samples in ms (default: 100ms)
 *
 * Total time: ~500ms for 5 samples with 100ms delays
 *
 * Example:
 *   BatteryMonitorData_t monitor_data;
 *   int16_t stable_temp_dc = bms_get_averaged_temperature(&monitor_data);
 *   // stable_temp_dc is now averaged from 5 samples, reducing noise
 */
int16_t bms_get_averaged_temperature(BatteryMonitorData_t *monitor_data);

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

/**
 * Convert from BatteryMonitorData_t to BatteryState_Int_t
 *
 * Converts your existing battery monitoring data structure into the format
 * required by the BMS algorithms. Handles unit conversions and selects the
 * best available temperature source.
 *
 * @param monitor: Your BatteryMonitorData_t structure (input)
 * @param state: BatteryState_Int_t for BMS algorithms (output)
 *
 * Note: Requires battery_monitor.h to be included in your .c file
 */
void convert_monitor_to_bms_int(const BatteryMonitorData_t *monitor,
                                BatteryState_Int_t *state);

/**
 * Print BMS Decision Results
 *
 * Formats and displays the BMS decision result using your UART functions.
 * This is a reference implementation - customize for your display needs.
 *
 * @param result: BMS result to display
 */
void print_bms_decision_int(const BmsResult_Int_t *result);

/**
 * Periodic Battery Health Check
 *
 * Checks battery health and displays status. Call every 8 hours or as needed.
 *
 * Note: Requires battery_monitor.h and uart.h
 */
void bms_periodic_health_check_int(void);

/**
 * Protected Transmission with BMS
 *
 * Complete example of protected transmission with pre-flight checks and
 * real-time monitoring.
 *
 * @param data: Data to transmit
 * @param length: Length of data
 * @return: true if transmission succeeded, false if blocked/aborted/failed
 *
 * Note: Requires battery_monitor.h and uart.h
 */
bool bms_protected_transmission_int(uint8_t *data, uint16_t length);

/**
 * Test All BMS Functions
 *
 * Comprehensive test suite for all 3 BMS functions. Add this to your
 * main loop for testing.
 *
 * Note: Requires battery_monitor.h and uart.h
 */
void test_bms_functions_int(void);


#endif /* BMS_ALGORITHMS_H_ */
