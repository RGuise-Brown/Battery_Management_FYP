/*
 * bms_algorithms_full.c
 *
 *  Created on: Oct 11, 2025
 *      Author: rachaelguise-brown
 *
 *
 * Intelligent Battery Management System
 * Three Core Functions for Li-SOCl2 Primary Battery
 *
 * Based on characterization data:
 * - Capacity at T < -20°C: 8.3 Ah
 * - Capacity at T > 0°C: 10.5 Ah
 * - High current transmission: 500mA pulses
 * - Critical voltage threshold: 2.5V under load
 */



#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "bms_algorithms_full.h"
#include "uart.h"

#define VOLTAGE_DROP_MULT      3      // 1.5x multiplier (stored as x2)
#define VOLTAGE_DROP_DIV       2      // divisor for multiplier


/**
 * FUNCTION 1: Temperature-Based SOC Adjustment
 *
 * Adjusts State of Charge based on temperature-dependent capacity
 * This is the MOST IMPORTANT function based on your characterization
 *
 * @param state: Current battery state
 * @param result: Output structure containing adjusted SOC and available capacity
 */
void bms_update_soc_int(const BatteryState_Int_t *state, BmsResult_Int_t *result) {
    uint32_t available_uah;
    int16_t temp_dc = state->temperature_dc;

    // Check which temperature range we're in and use that capacity
    // NO interpolation - just discrete values

    if (temp_dc < TEMP_VERY_COLD_THRESH_DC) {
        // Range 1: Below -20°C
        available_uah = CAPACITY_VERY_COLD_UAH;
    }
    else if (temp_dc < TEMP_COLD_THRESH_DC) {
        // Range 2: -20°C to 0°C
        available_uah = CAPACITY_COLD_UAH;
    }
    else if (temp_dc < TEMP_NORMAL_THRESH_DC) {
        // Range 3: 0°C to 20°C
        available_uah = CAPACITY_NORMAL_UAH;
    }
    else {
        // Range 4: Above 20°C
        available_uah = CAPACITY_WARM_UAH;
    }

    // Calculate SOC: remaining / available * 1000 (per-mille)
    uint32_t remaining_uah;
    if (available_uah > state->accumulated_uah) {
        remaining_uah = available_uah - state->accumulated_uah;
    } else {
        remaining_uah = 0;  // Battery empty
    }

    // Calculate SOC in per-mille (0-1000)
    uint16_t adjusted_soc_pm;
    if (available_uah > 0) {
        uint64_t soc_calc = ((uint64_t)remaining_uah * 1000) / available_uah;
        adjusted_soc_pm = (uint16_t)soc_calc;
    } else {
        adjusted_soc_pm = 0;
    }

    // Clamp to valid range
    if (adjusted_soc_pm > 1000) {
        adjusted_soc_pm = 1000;
    }

    // Store results
    result->adjusted_soc_pm = adjusted_soc_pm;
    result->available_capacity_uah = available_uah;

    // Generate message
    uint16_t soc_pct = adjusted_soc_pm / 10;
    uint16_t soc_dec = adjusted_soc_pm % 10;
    int16_t temp_whole = temp_dc / 10;
    uint16_t temp_dec = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    uint32_t available_mah = available_uah / 1000;  // For display only

    snprintf(result->message, sizeof(result->message),
             "SOC: %u.%u%% | Available: %lu mAh @ %d.%uC",
             soc_pct, soc_dec, (unsigned long)available_mah, temp_whole, temp_dec);
}

/**
 * FUNCTION 2: Transmission Go/No-Go Decision
 *
 * Determines if a satellite transmission should proceed based on:
 * - Current battery state (voltage, temperature, SOC)
 * - Predicted voltage drop during transmission pulse
 * - Historical characterization data
 *
 * @param state: Current battery state
 * @param result: Output structure with decision and reasoning
 */
void bms_transmission_decision_int(const BatteryState_Int_t *state, BmsResult_Int_t *result) {
    // First, get temperature-adjusted SOC
    bms_update_soc_int(state, result);

    // Predict voltage drop during 500mA transmission pulse
    // V_drop = I × R (in mV = mA × mΩ / 1000)
    uint32_t voltage_drop_uv = (uint32_t)TX_CURRENT_MA * state->impedance_mohm;  // μV
    uint16_t voltage_drop_mv = (uint16_t)(voltage_drop_uv / 1000);  // Convert to mV

    uint16_t predicted_voltage_mv = (state->voltage_mv > voltage_drop_mv) ?
                                    (state->voltage_mv - voltage_drop_mv) : 0;

    result->predicted_voltage_mv = predicted_voltage_mv;

    // Decision logic

    // CRITICAL BLOCK: Predicted voltage below absolute minimum
    if (predicted_voltage_mv < VOLTAGE_CRITICAL_MV) {
        result->decision = DECISION_BLOCK;
        snprintf(result->message, sizeof(result->message),
                 "BLOCK: Predicted voltage %umV below critical (%umV)",
                 predicted_voltage_mv, VOLTAGE_CRITICAL_MV);
        return;
    }

    // CRITICAL BLOCK: Cold temperature + Low SOC combination
    if (state->temperature_dc < TEMP_COLD_THRESH_DC && result->adjusted_soc_pm < SOC_LOW_THRESH_PM) {
        result->decision = DECISION_BLOCK;
        int16_t temp_whole = state->temperature_dc / 10;
        uint16_t soc_pct = result->adjusted_soc_pm / 10;
        uint16_t soc_dec = result->adjusted_soc_pm % 10;
        snprintf(result->message, sizeof(result->message),
                 "BLOCK: Cold (%dC) + Low SOC (%u.%u%%) - High failure risk",
                 temp_whole, soc_pct, soc_dec);
        return;
    }

    // WARNING: Cold temperature + Marginal voltage
    if (state->temperature_dc < TEMP_VERY_COLD_THRESH_DC && predicted_voltage_mv < VOLTAGE_WARNING_MV) {
        result->decision = DECISION_WARN;
        int16_t temp_whole = state->temperature_dc / 10;
        snprintf(result->message, sizeof(result->message),
                 "WARN: Marginal - Cold (%dC), Predicted V: %umV",
                 temp_whole, predicted_voltage_mv);
        return;
    }

    // WARNING: High impedance indicates battery degradation
    if (state->impedance_mohm > 1500 && state->temperature_dc > TEMP_COLD_THRESH_DC ) {
        result->decision = DECISION_WARN;
        snprintf(result->message, sizeof(result->message),
                 "WARN: High impedance (%u mOhm) indicates degradation",
                 state->impedance_mohm);
        return;
    }

    // WARNING: Low SOC at any temperature
    if (result->adjusted_soc_pm < 200) {  // < 20%
        result->decision = DECISION_WARN;
        uint16_t soc_pct = result->adjusted_soc_pm / 10;
        uint16_t soc_dec = result->adjusted_soc_pm % 10;
        snprintf(result->message, sizeof(result->message),
                 "WARN: Low SOC (%u.%u%%) - Limited transmissions remaining",
                 soc_pct, soc_dec);
        return;
    }

    // GO: Conditions acceptable for transmission
    result->decision = DECISION_GO;
    uint16_t soc_pct = result->adjusted_soc_pm / 10;
    uint16_t soc_dec = result->adjusted_soc_pm % 10;
    snprintf(result->message, sizeof(result->message),
             "GO: Conditions OK - SOC: %u.%u%%, Predicted V: %umV",
             soc_pct, soc_dec, predicted_voltage_mv);
}

/**
 * FUNCTION 3: Real-Time Transmission Monitoring
 *
 * Monitors voltage during active transmission and decides whether to
 * continue or abort based on actual voltage drop vs expected behavior
 *
 * This function should be called during the transmission pulse with
 * the measured voltage under load
 *
 * @param voltage_before: Resting voltage before transmission (V)
 * @param voltage_during: Voltage measured during transmission (V)
 * @param soc: Current adjusted SOC (0.0 to 1.0)
 * @param expected_drop: Expected voltage drop from characterization (V)
 * @param result: Output structure with decision
 *
 * NOTE: The expected_drop parameter should come from your current cycling
 * data which shows voltage drop vs SOC. This is a lookup table you need
 * to populate based on your discharge characterization.
 */
void bms_monitor_transmission_int(uint16_t voltage_before_mv,
                                  uint16_t voltage_during_mv,
                                  uint16_t max_allowed_drop_mv,
                                  BmsResult_Int_t *result) {
    // Calculate actual drop
    uint16_t actual_drop_mv = (voltage_before_mv > voltage_during_mv) ?
                              (voltage_before_mv - voltage_during_mv) : 0;

    // ABORT: Critical voltage
    if (voltage_during_mv < VOLTAGE_CRITICAL_MV) {
        result->decision = DECISION_ABORT;
        snprintf(result->message, sizeof(result->message),
                 "ABORT: Critical voltage %umV - Terminate now",
                 voltage_during_mv);
        return;
    }

    // ABORT: Excessive drop
    if (actual_drop_mv > max_allowed_drop_mv) {
        result->decision = DECISION_ABORT;
        snprintf(result->message, sizeof(result->message),
                 "ABORT: Excessive drop %umV (max: %umV)",
                 actual_drop_mv, max_allowed_drop_mv);
        return;
    }

    // WARN: Approaching critical
    if (voltage_during_mv < (VOLTAGE_WARNING_MV)) {
        result->decision = DECISION_WARN;
        uint16_t margin_mv = voltage_during_mv - VOLTAGE_CRITICAL_MV;
        snprintf(result->message, sizeof(result->message),
                 "WARN: Approaching critical - %umV margin",
                 margin_mv);
        return;
    }

    // GO: Normal
    result->decision = DECISION_GO;
    snprintf(result->message, sizeof(result->message),
             "CONTINUE: TX OK - V: %umV, Drop: %umV",
             voltage_during_mv, actual_drop_mv);
}

// ============================================================================
// TEMPERATURE TREND MONITORING
// ============================================================================

// Storage for temperature history (you need to maintain this)
static TempReading_t temp_history[TEMP_HISTORY_SIZE];
static uint8_t temp_history_count = 0;
static uint8_t temp_history_index = 0;

/**
 * Add Temperature Reading to History
 *
 * Call this periodically (e.g., every 10 minutes) to build up history
 */
void bms_add_temp_reading(int16_t temperature_dc, uint32_t timestamp_ms) {
    temp_history[temp_history_index].temperature_dc = temperature_dc;
    temp_history[temp_history_index].timestamp_ms = timestamp_ms;

    temp_history_index++;
    if (temp_history_index >= TEMP_HISTORY_SIZE) {
        temp_history_index = 0;  // Wrap around (circular buffer)
    }

    if (temp_history_count < TEMP_HISTORY_SIZE) {
        temp_history_count++;
    }
}

/**
 * FUNCTION 4: Monitor Temperature Trend
 *
 * Analyzes temperature history to predict if capacity will change
 *
 * @param result: Output with trend analysis
 */
void bms_monitor_temperature_trend(TempTrendResult_t *result) {
    // Check if we have enough data
    if (temp_history_count < 10) {
        result->status = TREND_INSUFFICIENT_DATA;
        result->trend_dc_per_hour = 0;
        result->predicted_temp_1hr_dc = 0;
        result->capacity_will_decrease = false;
        result->capacity_will_increase = false;
        snprintf(result->message, sizeof(result->message),
                 "Need more data (%u/10 readings)", temp_history_count);
        return;
    }

    // Calculate average of 5 oldest readings
    int32_t sum_older = 0;
    for (uint8_t i = 0; i < 5; i++) {
        sum_older += temp_history[i].temperature_dc;
    }
    int16_t avg_older_dc = (int16_t)(sum_older / 5);

    // Calculate average of 5 most recent readings
    int32_t sum_recent = 0;
    uint8_t start_idx = (temp_history_count >= 5) ? (temp_history_count - 5) : 0;
    for (uint8_t i = start_idx; i < temp_history_count; i++) {
        sum_recent += temp_history[i].temperature_dc;
    }
    int16_t avg_recent_dc = (int16_t)(sum_recent / 5);

    // Calculate trend (change in decidegrees)
    int16_t trend_dc = avg_recent_dc - avg_older_dc;

    // Calculate time span between oldest and newest readings
    uint32_t oldest_time = temp_history[0].timestamp_ms;
    uint32_t newest_time = temp_history[temp_history_count - 1].timestamp_ms;
    uint32_t time_span_ms = newest_time - oldest_time;

    // Convert trend to per-hour rate
    // trend_dc_per_hour = trend_dc * (3600000 ms/hr) / time_span_ms
    int16_t trend_dc_per_hour;
    if (time_span_ms > 0) {
        int32_t trend_calc = ((int32_t)trend_dc * 3600000) / time_span_ms;
        trend_dc_per_hour = (int16_t)trend_calc;
    } else {
        trend_dc_per_hour = 0;
    }

    result->trend_dc_per_hour = trend_dc_per_hour;

    // Get current temperature (most recent reading)
    int16_t current_temp_dc = temp_history[temp_history_count - 1].temperature_dc;

    // Predict temperature in 1 hour
    result->predicted_temp_1hr_dc = current_temp_dc + trend_dc_per_hour;

    // Determine status and impact
    if (trend_dc < TEMP_TREND_COOLING_FAST) {
        result->status = TREND_COOLING_FAST;
        result->capacity_will_decrease = true;
        result->capacity_will_increase = false;

        int16_t trend_whole = trend_dc / 10;
        int16_t trend_dec = (trend_dc < 0) ? ((-trend_dc) % 10) : (trend_dc % 10);
        snprintf(result->message, sizeof(result->message),
                 "COOLING FAST (%d.%uC). Expect capacity reduction!",
                 trend_whole, trend_dec);
    }
    else if (trend_dc < TEMP_TREND_COOLING) {
        result->status = TREND_COOLING;
        result->capacity_will_decrease = true;
        result->capacity_will_increase = false;

        int16_t trend_whole = trend_dc / 10;
        int16_t trend_dec = (trend_dc < 0) ? ((-trend_dc) % 10) : (trend_dc % 10);
        snprintf(result->message, sizeof(result->message),
                 "Cooling (%d.%uC). Monitor capacity.",
                 trend_whole, trend_dec);
    }
    else if (trend_dc > TEMP_TREND_WARMING_FAST) {
        result->status = TREND_WARMING_FAST;
        result->capacity_will_decrease = false;
        result->capacity_will_increase = true;

        int16_t trend_whole = trend_dc / 10;
        int16_t trend_dec = trend_dc % 10;
        snprintf(result->message, sizeof(result->message),
                 "WARMING FAST (+%d.%uC). Capacity will improve!",
                 trend_whole, trend_dec);
    }
    else if (trend_dc > TEMP_TREND_WARMING) {
        result->status = TREND_WARMING;
        result->capacity_will_decrease = false;
        result->capacity_will_increase = true;

        int16_t trend_whole = trend_dc / 10;
        int16_t trend_dec = trend_dc % 10;
        snprintf(result->message, sizeof(result->message),
                 "Warming (+%d.%uC). Capacity improving.",
                 trend_whole, trend_dec);
    }
    else {
        result->status = TREND_STABLE;
        result->capacity_will_decrease = false;
        result->capacity_will_increase = false;
        snprintf(result->message, sizeof(result->message),
                 "Temperature stable.");
    }
}

/**
 * Print Temperature Trend Results
 */
void print_temp_trend_int(const TempTrendResult_t *result) {
    UART_SendString("\r\n+--- TEMPERATURE TREND -------------+\r\n");

    UART_SendString("| Status: ");
    switch(result->status) {
        case TREND_INSUFFICIENT_DATA:
            UART_SendString("NEED DATA  |\r\n");
            break;
        case TREND_COOLING_FAST:
            UART_SendString("COOL FAST  |\r\n");
            break;
        case TREND_COOLING:
            UART_SendString("COOLING    |\r\n");
            break;
        case TREND_STABLE:
            UART_SendString("STABLE     |\r\n");
            break;
        case TREND_WARMING:
            UART_SendString("WARMING    |\r\n");
            break;
        case TREND_WARMING_FAST:
            UART_SendString("WARM FAST  |\r\n");
            break;
    }

    if (result->status != TREND_INSUFFICIENT_DATA) {
        UART_SendString("| Trend:    ");
        if (result->trend_dc_per_hour >= 0) {
            UART_SendString("+");
        }
        UART_SendNumber(result->trend_dc_per_hour / 10);
        UART_SendString(".");
        uint16_t dec = (result->trend_dc_per_hour < 0) ?
                       ((-result->trend_dc_per_hour) % 10) :
                       (result->trend_dc_per_hour % 10);
        UART_SendNumber(dec);
        UART_SendString(" C/hr  |\r\n");

        UART_SendString("| Predicted: ");
        UART_SendNumber(result->predicted_temp_1hr_dc / 10);
        UART_SendString(" C (1hr) |\r\n");

        UART_SendString("| Impact:   ");
        if (result->capacity_will_decrease) {
            UART_SendString("CAP DOWN   |\r\n");
        } else if (result->capacity_will_increase) {
            UART_SendString("CAP UP     |\r\n");
        } else {
            UART_SendString("NO CHANGE  |\r\n");
        }
    }

    UART_SendString("| Info: ");
    UART_SendString(result->message);
    UART_SendString("\r\n");

    UART_SendString("+-----------------------------------+\r\n\r\n");
}

// ============================================================================
// TEMPERATURE AVERAGING (For Noisy Readings)
// ============================================================================

/**
 * Get Averaged Temperature Reading
 *
 * Takes multiple samples and averages them to reduce noise.
 * Call this instead of reading temperature directly.
 *
 * @param monitor_data: Your battery monitor data structure
 * @return: Averaged temperature in decidegrees (0.1°C)
 */
int16_t bms_get_averaged_temperature(BatteryMonitorData_t *monitor_data) {
    int32_t temp_sum = 0;
    uint8_t valid_samples = 0;

    // Take multiple samples
    for (uint8_t i = 0; i < TEMP_AVG_SAMPLES; i++) {
        // Read temperature data
        BatteryMonitor_ReadAll(monitor_data);

        // Use external sensor if available, else BQ35100
        int16_t temp_dc;
        if (monitor_data->adc_valid) {
            temp_dc = (int16_t)monitor_data->adc_temp_tenths_c;
        } else if (monitor_data->acc_present) {
            temp_dc = (int16_t)(monitor_data->acc_temp_hundredths_c / 10);
        } else {
            continue;  // Skip invalid reading
        }

        temp_sum += temp_dc;
        valid_samples++;

        // Delay between samples (except after last sample)
        if (i < TEMP_AVG_SAMPLES - 1) {
            delay_ms(TEMP_SAMPLE_DELAY);
        }
    }

    // Calculate average
    if (valid_samples > 0) {
        return (int16_t)(temp_sum / valid_samples);
    } else {
        return 0;  // No valid readings
    }
}

/* ============================================================================
 * INTEGRATION HELPER FUNCTIONS
 * ============================================================================ */

/**
 * Convert from BatteryMonitorData_t to BatteryState_Int_t
 *
 * NOTE: Requires your battery_monitor.h to be included
 */
void convert_monitor_to_bms_int(const BatteryMonitorData_t *monitor,
                                BatteryState_Int_t *state) {
    if (!monitor->acc_present) {
        state->voltage_mv = 0;
        state->temperature_dc = 200;
        state->accumulated_uah = 0;
        state->current_ma = 0;
        state->impedance_mohm = 800;
        state->timestamp_ms = 0;
        return;
    }

    // Direct copies (all in correct units!)
    state->voltage_mv = monitor->acc_voltage_mv;
    state->current_ma = monitor->acc_current_ma;
    state->accumulated_uah = monitor->acc_capacity_uah;  // DIRECT COPY - both μAh!

    // Temperature
    if (monitor->adc_valid) {
        state->temperature_dc = (int16_t)monitor->adc_temp_tenths_c;
    } else {
        state->temperature_dc = (int16_t)(monitor->acc_temp_hundredths_c / 10);
    }

    // Impedance
    if (monitor->eos_present) {
        state->impedance_mohm = monitor->eos_measured_z_mohm;
    } else {
        state->impedance_mohm = 800;
    }

    state->timestamp_ms = monitor->timestamp_ms;
}

/**
 * Print BMS Decision Results (using UART functions)
 *
 * NOTE: Requires your uart.h to be included
 * Customize this to match your preferred output format
 */
void print_bms_decision_int(const BmsResult_Int_t *result) {
    UART_SendString("\r\n+--- BMS DECISION -------------------+\r\n");

    UART_SendString("| Decision: ");
    switch(result->decision) {
        case DECISION_GO:
            UART_SendString("GO          |\r\n");
            break;
        case DECISION_WARN:
            UART_SendString("WARNING     |\r\n");
            break;
        case DECISION_BLOCK:
            UART_SendString("BLOCKED     |\r\n");
            break;
        case DECISION_ABORT:
            UART_SendString("ABORT       |\r\n");
            break;
    }

    UART_SendString("| SOC:      ");
    uint16_t soc_pct = result->adjusted_soc_pm / 10;
    uint16_t soc_dec = result->adjusted_soc_pm % 10;
    UART_SendNumber(soc_pct);
    UART_SendString(".");
    UART_SendNumber(soc_dec);
    UART_SendString("%         |\r\n");

    UART_SendString("| Capacity: ");
    uint32_t cap_mah = result->available_capacity_uah / 1000;
    UART_SendNumber(cap_mah);
    UART_SendString(" mAh      |\r\n");

    if (result->predicted_voltage_mv > 0) {
        UART_SendString("| Pred V:   ");
        UART_SendNumber(result->predicted_voltage_mv);
        UART_SendString(" mV      |\r\n");
    }

    UART_SendString("| Reason:   ");
    UART_SendString(result->message);
    UART_SendString("\r\n");

    UART_SendString("+-----------------------------------+\r\n\r\n");
}



// ============================================================================
// TEST FUNCTION
// ============================================================================
/*
void test_bms_functions_int(void) {
    BatteryMonitorData_t monitor_data;
    BatteryState_Int_t bms_state;
    BmsResult_Int_t result;

    UART_SendString("\r\n========================================\r\n");
    UART_SendString("   BMS TEST SUITE (INTEGER-ONLY)       \r\n");
    UART_SendString("========================================\r\n\r\n");

    // Read battery
    BatteryMonitor_ReadAll(&monitor_data);

    if (!monitor_data.acc_present) {
        UART_SendString("ERROR: ACC mode not detected\r\n");
        return;
    }

    convert_monitor_to_bms_int(&monitor_data, &bms_state);

    // Test 1: SOC with 4 ranges
    UART_SendString("[TEST 1] SOC Adjustment (4 Temp Ranges)\r\n\r\n");
    bms_update_soc_int(&bms_state, &result);

    UART_SendString("Accumulated: ");
    UART_SendNumber(bms_state.accumulated_uah / 1000);
    UART_SendString(" mAh\r\n");

    UART_SendString("Temperature: ");
    UART_SendNumber(bms_state.temperature_dc / 10);
    UART_SendString(" C\r\n");

    UART_SendString("Temp Range:  ");
    if (bms_state.temperature_dc < -200) {
        UART_SendString("< -20C (Very Cold)\r\n");
    } else if (bms_state.temperature_dc < 0) {
        UART_SendString("-20 to 0C (Cold)\r\n");
    } else if (bms_state.temperature_dc < 200) {
        UART_SendString("0 to 20C (Normal)\r\n");
    } else {
        UART_SendString("> 20C (Warm)\r\n");
    }

    UART_SendString("Adjusted SOC: ");
    UART_SendNumber(result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(result.adjusted_soc_pm % 10);
    UART_SendString(" %\r\n");

    UART_SendString("Available:   ");
    UART_SendNumber(result.available_capacity_uah / 1000);
    UART_SendString(" mAh\r\n\r\n");

    delay_ms(2000);

    // Test 2: Transmission decision
    UART_SendString("[TEST 2] Transmission Decision\r\n");
    bms_transmission_decision_int(&bms_state, &result);
    print_bms_decision_int(&result);

    delay_ms(2000);

    // Test 3: Voltage monitoring
    UART_SendString("[TEST 3] Voltage Drop Monitoring\r\n");
    UART_SendString("Max drop: ");
    UART_SendNumber(MAX_VOLTAGE_DROP_MV);
    UART_SendString(" mV\r\n\r\n");

    uint16_t v_before = bms_state.voltage_mv;
    uint16_t v_during = v_before - 300;  // Simulate 300mV drop

    UART_SendString("Before: ");
    UART_SendNumber(v_before);
    UART_SendString(" mV\r\n");

    UART_SendString("During: ");
    UART_SendNumber(v_during);
    UART_SendString(" mV\r\n\r\n");

    bms_monitor_transmission_int(v_before, v_during, MAX_VOLTAGE_DROP_MV, &result);
    print_bms_decision_int(&result);

    UART_SendString("========================================\r\n");
    UART_SendString("   TEST COMPLETE                        \r\n");
    UART_SendString("========================================\r\n\r\n");
}*/
