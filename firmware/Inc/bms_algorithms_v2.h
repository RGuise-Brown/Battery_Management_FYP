/*
 * bms_algorithms_v2.h
 *
 *  Created on: Oct 16, 2025
 *      Author: rachaelguise-brown
 *
 *  BMS algorithms (v2) — uses INA226 + TMP36 (SystemMonitorData_t)
 *  Keeps temperature-based SOC behavior from the original algorithm,
 *  but sources voltage/current/capacity from SystemMonitor.
 *
 *  Function 2 left as placeholder (no impedance available).
 */

#ifndef BMS_ALGORITHMS_V2_H_
#define BMS_ALGORITHMS_V2_H_

#include <stdint.h>
#include <stdbool.h>
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
#define CAPACITY_VERY_COLD_UAH  8700000    // μAh at T < 0°C (freezer-verified)
#define CAPACITY_WARM_UAH       9000000   // μAh at T ≥ 0°C (room-temp-verified)

// Temperature threshold (in decidegrees = 0.1°C)
// Simple boundary at 0°C - the critical point between the two regions
#define TEMP_COLD_THRESH_DC     0          // 0.0°C - boundary between cold and warm

// Additional reference threshold for very cold conditions
#define TEMP_VERY_COLD_THRESH_DC  -250     // -25.0°C (bottom of freezer range)

// Transmission current
#define TX_CURRENT_MA          1000         // 1000mA pulse - for tests

// Voltage thresholds
#define VOLTAGE_CRITICAL_MV    100        // 2.5V absolute minimum under load
#define VOLTAGE_WARNING_MV     2000        // 2.7V warning threshold

// SOC threshold (in per-mille)
#define SOC_LOW_THRESH_PM      300         // 30.0%

// Impedance threshold (mOhm)
#define IMPEDANCE_HIGH 			1500 		//1.5Ohms

// Function 3 - Maximum allowed voltage drop during transmission
#define MAX_VOLTAGE_DROP_MV    2000         // 500mV max drop
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
 * VOLTAGE-BASED SOC THRESHOLDS (FOR TEMPERATURE ≤ 0°C)
 * ============================================================================
 */
#define SOC_100_75_VOLT_MV   3030   // >3.03V → 100–75%
#define SOC_75_50_VOLT_MV    2500   // 3.03–2.50V → 75–50%
#define SOC_50_25_VOLT_MV    1820   // 2.50–1.82V → 50–25%
#define SOC_25_0_VOLT_MV     0      // <1.82V → 25–0%

typedef struct {
    uint8_t last_soc_band;  // 4 = 100–75%, 3 = 75–50%, 2 = 50–25%, 1 = 25–0%
} VoltageSocState_t;

typedef struct
{
    bool active;              // true if temperature ≤ 0°C
    bool in_range;            // true if SOC is within expected range
    uint16_t soc_lower_pm;    // lower limit of SOC range
    uint16_t soc_upper_pm;    // upper limit of SOC range
} BmsVoltageCheck_t;


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
#include "system_monitoring.h"     // for SystemMonitorData_t

/* Primary v2 functions (suffix _v2) */

/**
 * Function 1 (v2): Temperature-Based SOC Adjustment
 *
 * Uses SystemMonitorData_t's accumulated_uAh (μAh, positive = used)
 * to compute adjusted SOC. Conversion to BatteryState_Int_t inverts
 * the sign to match the original bms expectations (accumulated_uah
 * negative for discharge).
 */
void bms_update_soc_v2(const SystemMonitorData_t *monitor, BmsResult_Int_t *result);

/**
 * Function 2 (v2): Voltage-Based SOC Cross-Check (Cold-Region Only)
 *
 * At temperatures ≤ 0 °C, this function estimates the battery’s SOC
 *   (state-of-charge) based on voltage bands, because voltage-capacity
 *   correlation is strong in the cold region.
 *
 *   The function defines four voltage ranges corresponding to SOC bands:
 *       > 3.03 V   → 100 – 75 %
 *       3.03–2.50 V → 75 – 50 %
 *       2.50–1.82 V → 50 – 25 %
 *       < 1.82 V   → 25 – 0 %
 *
 *   Once the SOC drops to a lower band, it is latched there and will not
 *   increase again even if voltage later rises (prevents false recovery).
 */

void BMS_Function2_CompareSocVoltage(const SystemMonitorData_t *sysData,
                                     const BmsResult_Int_t *integrationResult,
                                     BmsResult_Int_t *voltageResult);

/**
 * Function 3 (v2): Real-Time Transmission Monitoring
 *
 * Same semantics as original; uses measured voltages (mV).
 */
void bms_monitor_transmission_v2(uint16_t voltage_before_mv,
                                 uint16_t voltage_during_mv,
                                 uint16_t max_allowed_drop_mv,
                                 BmsResult_Int_t *result);

/**
 * Function 4 (v2): Temperature Trend Monitoring
 *
 * Same as original implementation (two-region model).
 */
void bms_monitor_temperature_trend_v2(TempTrendResult_t *result);
void bms_add_temp_reading_v2(int16_t temperature_dc, uint32_t timestamp_ms);

/**
 * Convert SystemMonitorData_t -> BatteryState_Int_t
 *
 * Prepares the BatteryState_Int_t (used by other BMS functions) from the
 * system monitoring structure. Sets impedance_mohm = 0 (not available).
 */
void convert_system_to_bms_int(const SystemMonitorData_t *monitor,
                               BatteryState_Int_t *state);

/**
 * Print BMS decision (v2)
 */
void print_bms_decision_v2(const BmsResult_Int_t *result);

#endif /* BMS_ALGORITHMS_V2_H_ */
