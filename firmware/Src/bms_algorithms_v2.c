/*
 * bms_algorithms_v2.c
 *
 *  Created on: Oct 16, 2025
 *      Author: rachaelguise-brown
 *
 *  Standalone BMS algorithms (v2) using SystemMonitorData_t (INA226 + TMP36).
 *
 *  - SOC: computed from monitor->accumulated_uAh and temperature-dependent capacity.
 *  - Function 2: placeholder (no impedance available).
 *  - Function 3: transmission monitoring (voltage drop checks).
 *  - Function 4: temperature trend monitoring (same two-region model).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bms_algorithms_v2.h"
#include "system_monitoring.h"
#include "uart.h"
#include "systick_timing.h"

/* ------------------------------
 * Temperature history (circular buffer)
 * ------------------------------ */
static TempReading_t temp_history[TEMP_HISTORY_SIZE];
static uint8_t temp_history_count = 0;
static uint8_t temp_history_index = 0;

/* ------------------------------------------------------------------------
 * Helper: convert SystemMonitorData_t -> BatteryState_Int_t
 *
 * Note:
 *  - SystemMonitorData_t.accumulated_uAh is positive and increases as charge
 *    is consumed. The SOC formula used in this v2 implementation uses that
 *    positive 'used' value directly.
 * ------------------------------------------------------------------------ */
void convert_system_to_bms_int(const SystemMonitorData_t *monitor,
                               BatteryState_Int_t *state)
{
    if (!monitor || !state) return;

    // Voltage from INA226
    state->voltage_mv = (uint16_t)monitor->ina226_bus_mv;

    // Temperature: monitor->adc_temp_tenths_c is in 0.1°C -> matches temperature_dc
    state->temperature_dc = (int16_t)monitor->adc_temp_tenths_c;

    // Accumulated capacity in μAh (positive == used)
    state->accumulated_uah = (int32_t)monitor->accumulated_uAh;

    // Current in mA (signed)
    state->current_ma = (int16_t)monitor->ina226_current_ma;

    // Timestamp
    state->timestamp_ms = monitor->timestamp_ms;
}

/* ------------------------------------------------------------------------
 * FUNCTION 1 (v2): Temperature-Based SOC Adjustment
 *
 * Uses monitor->accumulated_uAh (positive used μAh) and the temperature-
 * dependent full capacity constants to compute remaining capacity and SOC.
 * ------------------------------------------------------------------------ */
void bms_update_soc_v2(const SystemMonitorData_t *monitor, BmsResult_Int_t *result)
{
    if (!monitor || !result) return;

    BatteryState_Int_t state;
    convert_system_to_bms_int(monitor, &state);

    uint32_t available_uah;
    int16_t temp_dc = state.temperature_dc;

    // Choose capacity based on temperature region
    if (temp_dc < TEMP_COLD_THRESH_DC) {
        available_uah = CAPACITY_VERY_COLD_UAH;
    } else {
        available_uah = CAPACITY_WARM_UAH;
    }

    // used_uah is the amount consumed (monitor.accumulated_uAh is positive used)
    uint32_t used_uah = (state.accumulated_uah < 0) ? 0u : (uint32_t)state.accumulated_uah;

    uint32_t remaining_uah;
    if (used_uah >= available_uah) {
        remaining_uah = 0;
    } else {
        remaining_uah = available_uah - used_uah;
    }

    // SOC in per-mille (0..1000)
    uint16_t adjusted_soc_pm = 0;
    if (available_uah > 0) {
        uint64_t soc_calc = ((uint64_t)remaining_uah * 1000ULL) / (uint64_t)available_uah;
        if (soc_calc > 1000ULL) soc_calc = 1000ULL;
        adjusted_soc_pm = (uint16_t)soc_calc;
    }

    // Fill result
    result->adjusted_soc_pm = adjusted_soc_pm;
    result->available_capacity_uah = available_uah;
    result->predicted_voltage_mv = state.voltage_mv;  // default fill

    // Default decision (left to caller to set properly)
    result->decision = DECISION_WARN;

    // Build message (human readable)
    uint16_t soc_pct = adjusted_soc_pm / 10;
    uint16_t soc_dec = adjusted_soc_pm % 10;
    int16_t temp_whole = temp_dc / 10;
    uint16_t temp_dec = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    uint32_t avail_mah = available_uah / 1000U;
    const char *region = (temp_dc < TEMP_COLD_THRESH_DC) ? "COLD" : "WARM";

    snprintf(result->message, sizeof(result->message),
             "SOC: %u.%u%% | Available: %lu mAh @ %d.%u C (%s)",
             soc_pct, soc_dec, (unsigned long)avail_mah, temp_whole, temp_dec, region);
}

/* ------------------------------------------------------------------------
 * FUNCTION 2 (v2): Voltage-Based SOC Cross-Check (Cold-Region Only)
 *
 * At temperatures ≤ 0 °C, this function estimates the battery’s SOC
 *   (state-of-charge) based on voltage bands, because voltage-capacity
 *   correlation is strong in the cold region.
 * ------------------------------------------------------------------------ */
static VoltageSocState_t voltageSocState = { .last_soc_band = 4 }; // start full

void BmsAlgorithm_Function2_CheckVoltageRange(
		const BatteryState_Int_t *state,
    const BmsResult_Int_t *socResult,
    BmsVoltageCheck_t *voltageCheck)
{
    // Only active when temperature ≤ 0°C
    if (state->temperature_dc > 0)
    {
        voltageCheck->active = false;
        return;
    }

    voltageCheck->active = true;

    uint16_t voltage_mv = state->voltage_mv;
    uint8_t soc_band = voltageSocState.last_soc_band;
    uint16_t soc_upper_pm = 1000;
    uint16_t soc_lower_pm = 0;

    // Determine SOC range based on voltage
    if (voltage_mv >= SOC_100_75_VOLT_MV)
    {
        soc_band = 4;
    	voltageCheck->soc_upper_pm = 1000;  // 100%
        voltageCheck->soc_lower_pm = 750;   // 75%
    }
    else if (voltage_mv >= SOC_75_50_VOLT_MV)
    {
    	soc_band = 3;
    	voltageCheck->soc_upper_pm = 750;
        voltageCheck->soc_lower_pm = 500;
    }
    else if (voltage_mv >= SOC_50_25_VOLT_MV)
    {
    	soc_band = 2;
    	voltageCheck->soc_upper_pm = 500;
        voltageCheck->soc_lower_pm = 250;
    }
    else
    {
    	soc_band = 1;
    	voltageCheck->soc_upper_pm = 250;
        voltageCheck->soc_lower_pm = 0;
    }

    // Enforce "no upward capacity" rule
    if (soc_band < voltageSocState.last_soc_band)
    {
        voltageSocState.last_soc_band = soc_band;
    }
    else
    {
        // Lock to previous band (cannot increase)
        soc_band = voltageSocState.last_soc_band;

        switch (soc_band)
        {
            case 4: soc_upper_pm = 1000; soc_lower_pm = 750; break;
            case 3: soc_upper_pm = 750;  soc_lower_pm = 500; break;
            case 2: soc_upper_pm = 500;  soc_lower_pm = 250; break;
            case 1: soc_upper_pm = 250;  soc_lower_pm = 0;   break;
            default: break;
        }
    }

    voltageCheck->soc_lower_pm = soc_lower_pm;
    voltageCheck->soc_upper_pm = soc_upper_pm;

    // Compare with integration-based SOC (from Function 1)
    uint16_t soc_pm = socResult->adjusted_soc_pm;

    if (soc_pm <= voltageCheck->soc_upper_pm &&
        soc_pm >= voltageCheck->soc_lower_pm)
    {
        voltageCheck->in_range = true;
    }
    else
    {
        voltageCheck->in_range = false;
    }

    // Debug print (UART-safe, no floats)
    UART_SendString("[FUNC2] Voltage-Based SOC Check\r\n");
    UART_SendString("  Voltage (mV): ");
    UART_SendNumber(voltage_mv);
    UART_SendString("\r\n  Temp (0.1C): ");
    UART_SendNumber(state->temperature_dc);
    UART_SendString("\r\n  Band: ");
    UART_SendNumber(soc_band);
    UART_SendString("\r\n  Expected Range: ");
    UART_SendNumber(soc_lower_pm);
    UART_SendString(" – ");
    UART_SendNumber(soc_upper_pm);
    UART_SendString(" ‰\r\n  Integrated SOC: ");
    UART_SendNumber(soc_pm);
    UART_SendString(" ‰\r\n  Result: ");
    UART_SendString(voltageCheck->in_range ? "AGREE" : "DISAGREE");

    if (soc_band < voltageSocState.last_soc_band)
        UART_SendString(" (latched lower)");

    UART_SendString("\r\n\r\n");

}


/* ------------------------------------------------------------------------
 * FUNCTION 3 (v2): Real-Time Transmission Monitoring
 *
 * Same semantics as original: abort if critical voltage or excessive drop,
 * warn if approaching critical, go otherwise.
 * ------------------------------------------------------------------------ */
void bms_monitor_transmission_v2(uint16_t voltage_before_mv,
                                 uint16_t voltage_during_mv,
                                 uint16_t max_allowed_drop_mv,
                                 BmsResult_Int_t *result)
{
    if (!result) return;

    uint16_t actual_drop_mv = (voltage_before_mv > voltage_during_mv) ?
                              (voltage_before_mv - voltage_during_mv) : 0;

    // Critical low during transmission
    if (voltage_during_mv < VOLTAGE_CRITICAL_MV) {
        result->decision = DECISION_ABORT;
        snprintf(result->message, sizeof(result->message),
                 "ABORT: Critical voltage %umV - Terminate now",
                 voltage_during_mv);
        return;
    }

    // Excessive drop
    if (actual_drop_mv > max_allowed_drop_mv) {
        result->decision = DECISION_ABORT;
        snprintf(result->message, sizeof(result->message),
                 "ABORT: Excessive drop %umV (max: %umV)",
                 actual_drop_mv, max_allowed_drop_mv);
        return;
    }

    // Approaching critical
    if (voltage_during_mv < VOLTAGE_WARNING_MV) {
        result->decision = DECISION_WARN;
        uint16_t margin_mv = (voltage_during_mv > VOLTAGE_CRITICAL_MV) ? (voltage_during_mv - VOLTAGE_CRITICAL_MV) : 0;
        snprintf(result->message, sizeof(result->message),
                 "WARN: Approaching critical - %umV margin",
                 margin_mv);
        return;
    }

    // OK
    result->decision = DECISION_GO;
    snprintf(result->message, sizeof(result->message),
             "CONTINUE: TX OK - V: %umV, Drop: %umV",
             voltage_during_mv, actual_drop_mv);
}

/* ------------------------------------------------------------------------
 * TEMP TREND FUNCTIONS (v2) - same logic as prior implementation
 * ------------------------------------------------------------------------ */

void bms_add_temp_reading_v2(int16_t temperature_dc, uint32_t timestamp_ms)
{
    temp_history[temp_history_index].temperature_dc = temperature_dc;
    temp_history[temp_history_index].timestamp_ms = timestamp_ms;

    temp_history_index++;
    if (temp_history_index >= TEMP_HISTORY_SIZE) temp_history_index = 0;

    if (temp_history_count < TEMP_HISTORY_SIZE) temp_history_count++;
}

void bms_monitor_temperature_trend_v2(TempTrendResult_t *result)
{
    if (!result) return;

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

    // *** NEW APPROACH: Find oldest and newest by timestamp ***
    uint8_t num_valid = (temp_history_count < TEMP_HISTORY_SIZE) ? temp_history_count : TEMP_HISTORY_SIZE;

    uint8_t oldest_idx = 0;
    uint8_t newest_idx = 0;
    uint32_t oldest_time = temp_history[0].timestamp_ms;
    uint32_t newest_time = temp_history[0].timestamp_ms;

    // Search through all valid entries to find actual oldest and newest
    for (uint8_t i = 0; i < num_valid; i++) {
        if (temp_history[i].timestamp_ms < oldest_time) {
            oldest_time = temp_history[i].timestamp_ms;
            oldest_idx = i;
        }
        if (temp_history[i].timestamp_ms > newest_time) {
            newest_time = temp_history[i].timestamp_ms;
            newest_idx = i;
        }
    }

    // Average oldest 5 readings (starting from oldest_idx, going forward in time)
    int32_t sum_older = 0;
    uint8_t count_older = 0;

    // Create array of indices sorted by timestamp
    uint8_t sorted_indices[TEMP_HISTORY_SIZE];
    for (uint8_t i = 0; i < num_valid; i++) {
        sorted_indices[i] = i;
    }

    // Simple bubble sort by timestamp
    for (uint8_t i = 0; i < num_valid - 1; i++) {
        for (uint8_t j = 0; j < num_valid - i - 1; j++) {
            if (temp_history[sorted_indices[j]].timestamp_ms >
                temp_history[sorted_indices[j + 1]].timestamp_ms) {
                uint8_t temp = sorted_indices[j];
                sorted_indices[j] = sorted_indices[j + 1];
                sorted_indices[j + 1] = temp;
            }
        }
    }

    // Average oldest 5
    for (uint8_t i = 0; i < 5 && i < num_valid; i++) {
        sum_older += temp_history[sorted_indices[i]].temperature_dc;
        count_older++;
    }
    int16_t avg_older_dc = (count_older > 0) ? (int16_t)(sum_older / count_older) : 0;

    // Average newest 5
    int32_t sum_recent = 0;
    uint8_t count_recent = 0;
    uint8_t start_recent = (num_valid >= 5) ? (num_valid - 5) : 0;

    for (uint8_t i = start_recent; i < num_valid; i++) {
        sum_recent += temp_history[sorted_indices[i]].temperature_dc;
        count_recent++;
    }
    int16_t avg_recent_dc = (count_recent > 0) ? (int16_t)(sum_recent / count_recent) : 0;

    // Temperature trend
    int16_t trend_dc = avg_recent_dc - avg_older_dc;

    // Time span
    uint32_t time_span_ms = newest_time - oldest_time;

    int16_t trend_dc_per_hour = 0;
    if (time_span_ms > 0) {
        int64_t calc = ((int64_t)trend_dc * 3600000LL) / (int64_t)time_span_ms;
        trend_dc_per_hour = (int16_t)calc;
    }

    result->trend_dc_per_hour = trend_dc_per_hour;
    int16_t current_temp_dc = temp_history[temp_history_count - 1].temperature_dc;
    result->predicted_temp_1hr_dc = current_temp_dc + trend_dc_per_hour;

    bool currently_cold = (current_temp_dc < TEMP_COLD_THRESH_DC);
    bool will_be_cold = (result->predicted_temp_1hr_dc < 0);

    result->capacity_will_decrease = false;
    result->capacity_will_increase = false;

    if (trend_dc < TEMP_TREND_COOLING_FAST) {
        result->status = TREND_COOLING_FAST;
        result->capacity_will_decrease = (!currently_cold && will_be_cold);
        if (result->capacity_will_decrease) {
            snprintf(result->message, sizeof(result->message),
                     "COOLING FAST (%d.%u C). Will cross 0 C - Capacity drop expected",
                     trend_dc / 10, (uint16_t)(((-trend_dc) % 10)));
        } else {
            snprintf(result->message, sizeof(result->message),
                     "COOLING FAST (%d.%u C). Monitor temperature",
                     trend_dc / 10, (uint16_t)(((-trend_dc) % 10)));
        }
    }
    else if (trend_dc < TEMP_TREND_COOLING) {
        result->status = TREND_COOLING;
        result->capacity_will_decrease = (!currently_cold && will_be_cold);
        snprintf(result->message, sizeof(result->message),
                 "Cooling (%d.%u C). Monitor temperature",
                 trend_dc / 10, (uint16_t)(((-trend_dc) % 10)));
    }
    else if (trend_dc > TEMP_TREND_WARMING_FAST) {
        result->status = TREND_WARMING_FAST;
        result->capacity_will_increase = (currently_cold && !will_be_cold);
        snprintf(result->message, sizeof(result->message),
                 "WARMING FAST (+%d.%u C). Monitor temperature",
                 trend_dc / 10, (uint16_t)(trend_dc % 10));
    }
    else if (trend_dc > TEMP_TREND_WARMING) {
        result->status = TREND_WARMING;
        result->capacity_will_increase = (currently_cold && !will_be_cold);
        snprintf(result->message, sizeof(result->message),
                 "Warming (+%d.%u C). Monitor temperature",
                 trend_dc / 10, (uint16_t)(trend_dc % 10));
    }
    else {
        result->status = TREND_STABLE;
        snprintf(result->message, sizeof(result->message), "Temperature stable.");
    }
}

/* ------------------------------------------------------------------------
 * Print function for BMS result (v2)
 * ------------------------------------------------------------------------ */
void print_bms_decision_v2(const BmsResult_Int_t *result)
{
    if (!result) return;

    UART_SendString("\r\n+--- BMS DECISION (v2) -------------+\r\n");

    UART_SendString("| Decision: ");
    switch (result->decision) {
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
        default:
            UART_SendString("UNKNOWN     |\r\n");
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
    uint32_t cap_mah = result->available_capacity_uah / 1000U;
    UART_SendNumber(cap_mah);
    UART_SendString(" mAh      |\r\n");

    if (result->predicted_voltage_mv > 0) {
        UART_SendString("| Pred V:   ");
        UART_SendNumber(result->predicted_voltage_mv);
        UART_SendString(" mV      |\r\n");
    }

    UART_SendString("| Info:     ");
    UART_SendString(result->message);
    UART_SendString("\r\n+-----------------------------------+\r\n\r\n");
}
