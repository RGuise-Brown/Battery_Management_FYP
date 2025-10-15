/*
 * temp_threshold_test.c
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Test A: Temperature Threshold Crossing Test
 *
 * This test monitors battery as it transitions through the 0°C threshold
 * to validate capacity switching from 8.3Ah (cold) to 10.5Ah (warm) or vice versa.
 *
 * Procedure:
 * 1. Start with battery at one temperature (e.g., -25°C in freezer)
 * 2. Move battery to other temperature (e.g., room temp ~22°C)
 * 3. Monitor every 10 seconds as temperature changes
 * 4. Observe capacity switch at 0°C threshold
 * 5. Verify SOC% recalculation
 *
 * Duration: ~2-3 hours (depending on thermal mass)
 */

#include <stdint.h>
#include <stdbool.h>
#include "temp_threshold_test.h"
#include "bms_algorithms.h"
#include "battery_monitor.h"
#include "systick_timing.h"
#include "uart.h"

/* ==================== Configuration ==================== */

#define READING_INTERVAL_MS     10000      // 10 seconds between readings
#define MAX_READINGS            20          // Stop after 20 readings (~3.3 hours)
#define THRESHOLD_TEMP_DC       0           // 0.0°C threshold

/* ==================== Test State ==================== */

typedef enum {
    TEMP_TEST_IDLE,
    TEMP_TEST_RUNNING,
    TEMP_TEST_COMPLETE
} TempTestStatus_t;

typedef struct {
    TempTestStatus_t status;
    uint16_t reading_count;
    uint32_t test_start_time_ms;
    uint32_t last_reading_time_ms;
    int16_t start_temp_dc;
    bool threshold_crossed;
    bool capacity_switched;
    uint32_t capacity_before_uah;
    uint32_t capacity_after_uah;
} TempTestContext_t;

static TempTestContext_t temp_test_ctx = {0};

/* ==================== Helper Functions ==================== */

static const char* GetTempRegion(int16_t temp_dc)
{
    return (temp_dc < THRESHOLD_TEMP_DC) ? "COLD" : "WARM";
}

static void LogReading(uint16_t reading_num, BatteryMonitorData_t* monitor,
                      BmsResult_Int_t* bms_result)
{
    uint32_t elapsed_min = (GetTick_ms() - temp_test_ctx.test_start_time_ms) / 60000;

    UART_SendString("\r\n");
    UART_SendString("+============================================+\r\n");
    UART_SendString("| READING ");
    UART_SendNumber(reading_num);
    UART_SendString(" (");
    UART_SendNumber(elapsed_min);
    UART_SendString(" min elapsed)\r\n");
    UART_SendString("+============================================+\r\n");

    // Temperature
    int16_t temp_dc = monitor->adc_temp_tenths_c;
    int16_t temp_whole = temp_dc / 10;
    int16_t temp_frac_calc = (temp_dc < 0) ? (-temp_dc % 10) : (temp_dc % 10);
    uint16_t temp_frac = (uint16_t)temp_frac_calc;

    UART_SendString("| Temperature:  ");
    UART_SendSignedNumber(temp_whole);
    UART_SendString(".");
    UART_SendNumber(temp_frac);
    UART_SendString(" C  (");
    UART_SendString(GetTempRegion(temp_dc));
    UART_SendString(" region)\r\n");

    // Voltage
    UART_SendString("| Voltage:      ");
    UART_SendNumber(monitor->acc_voltage_mv);
    UART_SendString(" mV\r\n");

    // Available Capacity
    UART_SendString("| Avail Cap:    ");
    UART_SendNumber(bms_result->available_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");

    // Accumulated Capacity
    UART_SendString("| Accumulated:  ");
    UART_SendNumber(monitor->acc_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");

    // SOC
    UART_SendString("| SOC:          ");
    UART_SendNumber(bms_result->adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result->adjusted_soc_pm % 10);
    UART_SendString(" %\r\n");

    UART_SendString("+============================================+\r\n");
}

static void CheckThresholdCrossing(int16_t current_temp_dc, BmsResult_Int_t* bms_result)
{
    static int16_t previous_temp_dc = 0;
    static bool first_reading = true;

    if (first_reading) {
        previous_temp_dc = current_temp_dc;
        first_reading = false;
        return;
    }

    // Check if we crossed the threshold
    bool was_cold = (previous_temp_dc < THRESHOLD_TEMP_DC);
    bool now_cold = (current_temp_dc < THRESHOLD_TEMP_DC);

    if (was_cold != now_cold && !temp_test_ctx.threshold_crossed) {
        temp_test_ctx.threshold_crossed = true;

        UART_SendString("\r\n");
        UART_SendString("************************************************\r\n");
        UART_SendString("*   THRESHOLD CROSSED!                         *\r\n");
        UART_SendString("************************************************\r\n");

        if (was_cold && !now_cold) {
            UART_SendString("* Direction: WARMING (COLD -> WARM)           *\r\n");
            UART_SendString("* Expected:  Capacity increase                 *\r\n");
            UART_SendString("*            8.3 Ah -> 10.5 Ah                *\r\n");
        } else {
            UART_SendString("* Direction: COOLING (WARM -> COLD)           *\r\n");
            UART_SendString("* Expected:  Capacity decrease                 *\r\n");
            UART_SendString("*            10.5 Ah -> 8.3 Ah                *\r\n");
        }

        UART_SendString("************************************************\r\n\r\n");

        // Check if capacity actually switched
        if (temp_test_ctx.capacity_before_uah != bms_result->available_capacity_uah) {
            temp_test_ctx.capacity_switched = true;
            temp_test_ctx.capacity_after_uah = bms_result->available_capacity_uah;

            UART_SendString(">>> CAPACITY SWITCHED! <<<\r\n");
            UART_SendString("Before: ");
            UART_SendNumber(temp_test_ctx.capacity_before_uah / 1000);
            UART_SendString(" mAh\r\n");
            UART_SendString("After:  ");
            UART_SendNumber(temp_test_ctx.capacity_after_uah / 1000);
            UART_SendString(" mAh\r\n\r\n");
        } else {
            UART_SendString(">>> WARNING: Capacity did NOT switch! <<<\r\n\r\n");
        }
    }

    previous_temp_dc = current_temp_dc;
}

/* ==================== Public Functions ==================== */

void TempThresholdTest_Init(void)
{
    temp_test_ctx.status = TEMP_TEST_IDLE;
    temp_test_ctx.reading_count = 0;
    temp_test_ctx.test_start_time_ms = 0;
    temp_test_ctx.last_reading_time_ms = 0;
    temp_test_ctx.start_temp_dc = 0;
    temp_test_ctx.threshold_crossed = false;
    temp_test_ctx.capacity_switched = false;
    temp_test_ctx.capacity_before_uah = 0;
    temp_test_ctx.capacity_after_uah = 0;
}

void TempThresholdTest_Start(void)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("*   TEMPERATURE THRESHOLD CROSSING TEST       *\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("\r\n");

    UART_SendString("Test Objective:\r\n");
    UART_SendString("  Monitor capacity switching at 0C threshold\r\n");
    UART_SendString("\r\n");

    UART_SendString("Procedure:\r\n");
    UART_SendString("  1. Start with battery at one temperature\r\n");
    UART_SendString("     (e.g., -25C in freezer)\r\n");
    UART_SendString("  2. Move battery to other temperature\r\n");
    UART_SendString("     (e.g., room temp ~22C)\r\n");
    UART_SendString("  3. Readings every 10 minutes\r\n");
    UART_SendString("  4. Observe capacity switch at 0C\r\n");
    UART_SendString("\r\n");

    UART_SendString("Expected Results:\r\n");
    UART_SendString("  - Warming: 8.3Ah -> 10.5Ah at 0C\r\n");
    UART_SendString("  - Cooling: 10.5Ah -> 8.3Ah at 0C\r\n");
    UART_SendString("  - SOC% recalculates with new capacity\r\n");
    UART_SendString("\r\n");

    UART_SendString("Reading initial state...\r\n");

    // Read initial state
    BatteryMonitorData_t monitor_data;
    BatteryMonitor_ReadAll(&monitor_data);

    BatteryState_Int_t bms_state;
    convert_monitor_to_bms_int(&monitor_data, &bms_state);

    BmsResult_Int_t bms_result;
    bms_update_soc_int(&bms_state, &bms_result);

    temp_test_ctx.start_temp_dc = monitor_data.adc_temp_tenths_c;
    temp_test_ctx.capacity_before_uah = bms_result.available_capacity_uah;

    UART_SendString("\r\nStarting Temperature: ");
    UART_SendSignedNumber(temp_test_ctx.start_temp_dc / 10);
    UART_SendString(".");
    UART_SendNumber(abs(temp_test_ctx.start_temp_dc % 10));
    UART_SendString(" C (");
    UART_SendString(GetTempRegion(temp_test_ctx.start_temp_dc));
    UART_SendString(" region)\r\n");

    UART_SendString("Starting Capacity: ");
    UART_SendNumber(temp_test_ctx.capacity_before_uah / 1000);
    UART_SendString(" mAh\r\n\r\n");

    UART_SendString("Move battery to opposite temperature now!\r\n");
    UART_SendString("Test will begin in 10 seconds...\r\n\r\n");

    delay_ms(10000);

    temp_test_ctx.status = TEMP_TEST_RUNNING;
    temp_test_ctx.reading_count = 0;
    temp_test_ctx.test_start_time_ms = GetTick_ms();
    temp_test_ctx.last_reading_time_ms = GetTick_ms();

    UART_SendString("*** TEST STARTED ***\r\n");
}

void TempThresholdTest_Update(void)
{
    if (temp_test_ctx.status != TEMP_TEST_RUNNING) {
        return;
    }

    uint32_t current_time = GetTick_ms();

    // Check if it's time for a reading
    if ((current_time - temp_test_ctx.last_reading_time_ms) >= READING_INTERVAL_MS) {
        temp_test_ctx.last_reading_time_ms = current_time;
        temp_test_ctx.reading_count++;

        // Read battery state
        BatteryMonitorData_t monitor_data;
        BatteryMonitor_ReadAll(&monitor_data);

        BatteryState_Int_t bms_state;
        convert_monitor_to_bms_int(&monitor_data, &bms_state);

        BmsResult_Int_t bms_result;
        bms_update_soc_int(&bms_state, &bms_result);

        // Add temperature reading for trend analysis
        int16_t temp_dc = monitor_data.adc_temp_tenths_c;
        bms_add_temp_reading(temp_dc, current_time);

        // Log reading
        LogReading(temp_test_ctx.reading_count, &monitor_data, &bms_result);

        // Check for threshold crossing
        CheckThresholdCrossing(temp_dc, &bms_result);

        // Check temperature trend
        if (temp_test_ctx.reading_count >= 10) {
            TempTrendResult_t trend_result;
            bms_monitor_temperature_trend(&trend_result);

            if (trend_result.status != TREND_INSUFFICIENT_DATA) {
                UART_SendString("\r\n");
                print_temp_trend_int(&trend_result);
            }
        }

        // Check if test complete
        if (temp_test_ctx.reading_count >= MAX_READINGS) {
            TempThresholdTest_Stop("Maximum readings reached");
        }
    }
}

void TempThresholdTest_Stop(const char* reason)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*   TEMPERATURE THRESHOLD TEST COMPLETE       *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("Reason: ");
    UART_SendString(reason);
    UART_SendString("\r\n\r\n");

    UART_SendString("Test Summary:\r\n");
    UART_SendString("  Total Readings:  ");
    UART_SendNumber(temp_test_ctx.reading_count);
    UART_SendString("\r\n");

    uint32_t duration_min = (GetTick_ms() - temp_test_ctx.test_start_time_ms) / 60000;
    UART_SendString("  Duration:        ");
    UART_SendNumber(duration_min);
    UART_SendString(" minutes\r\n");

    UART_SendString("  Threshold (0C):  ");
    UART_SendString(temp_test_ctx.threshold_crossed ? "CROSSED" : "NOT CROSSED");
    UART_SendString("\r\n");

    UART_SendString("  Capacity Switch: ");
    UART_SendString(temp_test_ctx.capacity_switched ? "YES" : "NO");
    UART_SendString("\r\n");

    if (temp_test_ctx.capacity_switched) {
        UART_SendString("\r\n  Capacity Before: ");
        UART_SendNumber(temp_test_ctx.capacity_before_uah / 1000);
        UART_SendString(" mAh\r\n");

        UART_SendString("  Capacity After:  ");
        UART_SendNumber(temp_test_ctx.capacity_after_uah / 1000);
        UART_SendString(" mAh\r\n");

        UART_SendString("\r\n  >>> TEST PASSED <<<\r\n");
    } else if (temp_test_ctx.threshold_crossed) {
        UART_SendString("\r\n  >>> TEST FAILED: Threshold crossed but capacity did not switch <<<\r\n");
    } else {
        UART_SendString("\r\n  >>> TEST INCOMPLETE: Threshold not crossed <<<\r\n");
        UART_SendString("      (Temperature may not have changed enough)\r\n");
    }

    UART_SendString("\r\n************************************************\r\n\r\n");

    temp_test_ctx.status = TEMP_TEST_COMPLETE;
}

bool TempThresholdTest_IsRunning(void)
{
    return (temp_test_ctx.status == TEMP_TEST_RUNNING);
}

uint16_t TempThresholdTest_GetReadingCount(void)
{
    return temp_test_ctx.reading_count;
}
