/*
 * discharge_cycle_test.c
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Automated discharge cycle test for BMS validation
 *
 * Test Profile (1 hour cycle, repeats until battery depleted):
 * - 0-50 min:  1000mA discharge (transmission simulation)
 * - 50-55 min: 200mA discharge (low power mode)
 * - 55-60 min: 400mA discharge (medium power mode)
 *
 * NOTE: DC electronic load must be programmed externally with same sequence.
 *       Start load sequence and this code simultaneously.
 *
 * Expected duration: ~16 hours per test
 */

#include <stdint.h>
#include <stdbool.h>
#include "bms_algorithms.h"
#include "battery_monitor.h"
#include "uart.h"
#include "systick_timing.h"

/* ==================== Test Configuration ==================== */

// Timing intervals
#define VOLTAGE_READING_INTERVAL_MS     30000       // Read voltage/current every 30 seconds
#define TEMP_READING_INTERVAL_MS        60000      // Read temperature every 1 minute (for trend)
#define CYCLE_DURATION_MS               3600000     // 60 minutes per cycle

// Cycle phase timing (when BMS checks occur)
#define PHASE_HIGH_START_MS             0           // Start of high current phase
#define PHASE_HIGH_END_MS               3000000     // End of high current (50 min)
#define PHASE_LOW_START_MS              3000000     // Start of low current (50 min)
#define PHASE_LOW_END_MS                3300000     // End of low current (55 min)
#define PHASE_MEDIUM_START_MS           3300000     // Start of medium current (55 min)
#define PHASE_MEDIUM_END_MS             3600000     // End of medium current (60 min)

// Voltage drop measurement timing
#define VOLTAGE_DROP_CHECK_TIME_MS      10000       // Check voltage drop 10s after cycle starts
#define VOLTAGE_DROP_MAX_ALLOWED_MV     1000         // Maximum acceptable drop

// Safety limits
#define VOLTAGE_CUTOFF_MV               2000        // Stop test if voltage drops below this
#define MAX_CYCLES                      20          // Maximum cycles (safety limit)

/**
 * Delay function using busy wait
 * NOTE: This blocks! Only use for short delays
 */


/* ==================== Test State ==================== */

typedef enum {
    TEST_IDLE,
    TEST_RUNNING,
    TEST_COMPLETE,
    TEST_ABORTED
} TestStatus_t;

typedef struct {
    TestStatus_t status;
    uint16_t cycle_number;
    uint32_t test_start_time_ms;
    uint32_t last_voltage_reading_ms;
    uint32_t last_temp_reading_ms;
    uint16_t voltage_before_cycle_mv;
    bool voltage_drop_checked;
    bool transmission_decision_checked;
} TestContext_t;

static TestContext_t test_ctx = {0};

/* ==================== Data Logging - Human Readable Format ==================== */

/**
 * Print test header
 */
static void PrintTestHeader(void)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("*     BMS DISCHARGE CYCLE TEST START          *\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("\r\n");

    UART_SendString("Test Configuration:\r\n");
    UART_SendString("  Phase 1 (0-50 min):  1000 mA (High)\r\n");
    UART_SendString("  Phase 2 (50-55 min): 200 mA (Low)\r\n");
    UART_SendString("  Phase 3 (55-60 min): 400 mA (Medium)\r\n");
    UART_SendString("  Cycle Duration:      60 minutes\r\n");
    UART_SendString("  Voltage Cutoff:      2000 mV\r\n");
    UART_SendString("  Max Cycles:          ");
    UART_SendNumber(MAX_CYCLES);
    UART_SendString("\r\n\r\n");

    UART_SendString("IMPORTANT:\r\n");
    UART_SendString("1. Program DC load with same sequence\r\n");
    UART_SendString("2. Start DC load sequence\r\n");
    UART_SendString("3. Press any key to start logging...\r\n\r\n");
}

/**
 * Log cycle start information
 */
static void LogCycleStart(uint16_t cycle, BatteryMonitorData_t* monitor)
{
    UART_SendString("\r\n");
    UART_SendString("========================================\r\n");
    UART_SendString("  CYCLE ");
    UART_SendNumber(cycle);
    UART_SendString(" START\r\n");
    UART_SendString("========================================\r\n");

    uint32_t elapsed_total_min = ((GetTick_ms() - test_ctx.test_start_time_ms) / 60000);
    UART_SendString("Test Time:   ");
    UART_SendNumber(elapsed_total_min);
    UART_SendString(" minutes\r\n");

    UART_SendString("Voltage:     ");
    UART_SendNumber(monitor->acc_voltage_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("Temperature: ");
    int16_t temp = monitor->acc_temp_hundredths_c / 10;
    UART_SendSignedNumber(temp / 10);
    UART_SendString(".");
    UART_SendNumber(abs(temp % 10));
    UART_SendString(" C\r\n");

    UART_SendString("Accumulated: ");
    UART_SendSignedNumber(monitor->acc_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");

    UART_SendString("Current:     ");
    UART_SendSignedNumber(monitor->acc_current_ma);
    UART_SendString(" mA\r\n");

    UART_SendString("\r\n");
}

/**
 * Log periodic reading (every minute)
 */
static void LogReading(BatteryMonitorData_t* monitor, BmsResult_Int_t* bms_result)
{
    uint32_t elapsed_ms = GetTick_ms() - test_ctx.test_start_time_ms;
    uint32_t elapsed_min = elapsed_ms / 60000;
    uint32_t elapsed_sec = (elapsed_ms % 60000) / 1000;

    // Time stamp
    UART_SendString("[");
    if (elapsed_min < 100) UART_SendString(" ");
    if (elapsed_min < 10) UART_SendString(" ");
    UART_SendNumber(elapsed_min);
    UART_SendString(":");
    if (elapsed_sec < 10) UART_SendString("0");
    UART_SendNumber(elapsed_sec);
    UART_SendString("] ");

    // Voltage
    UART_SendString("V:");
    UART_SendNumber(monitor->acc_voltage_mv);
    UART_SendString("mV ");

    // Current
    UART_SendString("I:");
    UART_SendSignedNumber(monitor->acc_current_ma);
    UART_SendString("mA ");

    // Temperature
    UART_SendString("T:");
    int16_t temp = monitor->acc_temp_hundredths_c / 10;
    UART_SendSignedNumber(temp / 10);
    UART_SendString(".");
    UART_SendNumber(abs(temp % 10));
    UART_SendString("C ");

    // SOC
    UART_SendString("SOC:");
    UART_SendNumber(bms_result->adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result->adjusted_soc_pm % 10);
    UART_SendString("% ");

    // Accumulated capacity
    UART_SendString("Cap:");
    UART_SendSignedNumber(monitor->acc_capacity_uah / 1000);
    UART_SendString("mAh");

    UART_SendString("\r\n");
}

/**
 * Log voltage drop measurement
 */
static void LogVoltageDrop(uint16_t v_before, uint16_t v_during,
                          uint16_t actual_drop, BmsResult_Int_t* result)
{
    UART_SendString("\r\n");
    UART_SendString("+-----------------------------------+\r\n");
    UART_SendString("| VOLTAGE DROP MEASUREMENT          |\r\n");
    UART_SendString("+-----------------------------------+\r\n");
    UART_SendString("| Before Load: ");
    UART_SendNumber(v_before);
    UART_SendString(" mV            |\r\n");

    UART_SendString("| During Load: ");
    UART_SendNumber(v_during);
    UART_SendString(" mV            |\r\n");

    UART_SendString("| Drop:        ");
    UART_SendNumber(actual_drop);
    UART_SendString(" mV            |\r\n");

    UART_SendString("| Max Allowed: ");
    UART_SendNumber(VOLTAGE_DROP_MAX_ALLOWED_MV);
    UART_SendString(" mV            |\r\n");

    UART_SendString("| Decision:    ");
    switch(result->decision) {
        case DECISION_GO:    UART_SendString("CONTINUE     |\r\n"); break;
        case DECISION_WARN:  UART_SendString("WARNING      |\r\n"); break;
        case DECISION_ABORT: UART_SendString("ABORT        |\r\n"); break;
        default:             UART_SendString("UNKNOWN      |\r\n"); break;
    }

    UART_SendString("+-----------------------------------+\r\n");
    UART_SendString("| ");
    UART_SendString(result->message);
    UART_SendString("\r\n");
    UART_SendString("+-----------------------------------+\r\n\r\n");
}

/* ==================== Test Functions ==================== */

/**
 * Initialize test
 */
void DischargeTest_Init(void)
{
    test_ctx.status = TEST_IDLE;
    test_ctx.cycle_number = 0;
    test_ctx.test_start_time_ms = 0;
    test_ctx.last_voltage_reading_ms = 0;
    test_ctx.last_temp_reading_ms = 0;
    test_ctx.voltage_before_cycle_mv = 0;
    test_ctx.voltage_drop_checked = false;
    test_ctx.transmission_decision_checked = false;
}

/**
 * Start the discharge test
 */
void DischargeTest_Start(void)
{
    PrintTestHeader();

    // Wait for user to press key (optional - can remove)
    // UART_ReceiveChar();

    UART_SendString("*** TEST STARTED ***\r\n\r\n");

    // Initialize test state
    test_ctx.status = TEST_RUNNING;
    test_ctx.cycle_number = 1;
    test_ctx.test_start_time_ms = GetTick_ms();
    test_ctx.last_voltage_reading_ms = GetTick_ms();
    test_ctx.last_temp_reading_ms = GetTick_ms();
    test_ctx.voltage_drop_checked = false;
    test_ctx.transmission_decision_checked = false;

    // Read initial state
    BatteryMonitorData_t monitor_data;
    BatteryMonitor_ReadAll(&monitor_data);

    LogCycleStart(test_ctx.cycle_number, &monitor_data);

    // Store voltage before load applied
    test_ctx.voltage_before_cycle_mv = monitor_data.acc_voltage_mv;


    UART_SendString("\r\n=== INITIAL BATTERY STATE ===\r\n");
    UART_SendString("Accumulated Capacity (raw): ");
    UART_SendSignedNumber(monitor_data.acc_capacity_uah);
    UART_SendString(" uAh = ");
    UART_SendSignedNumber(monitor_data.acc_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");

    UART_SendString("Temperature: ");
    int16_t temp = monitor_data.acc_temp_hundredths_c / 10;
    UART_SendSignedNumber(temp / 10);
    UART_SendString(".");
    UART_SendNumber(abs(temp % 10));
    UART_SendString(" C\r\n");

    BatteryState_Int_t bms_state;
    convert_monitor_to_bms_int(&monitor_data, &bms_state);

    BmsResult_Int_t bms_result;
    bms_update_soc_int(&bms_state, &bms_result);

    UART_SendString("Available Capacity: ");
    UART_SendNumber(bms_result.available_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");

    UART_SendString("SOC: ");
    UART_SendNumber(bms_result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result.adjusted_soc_pm % 10);
    UART_SendString(" %\r\n\r\n");

    // Check if accumulated capacity is too high
    if (monitor_data.acc_capacity_uah > bms_result.available_capacity_uah) {
        UART_SendString("*** PROBLEM DETECTED ***\r\n");
        UART_SendString("Accumulated capacity is GREATER than available!\r\n");
        UART_SendString("This means BMS thinks battery is already depleted.\r\n\r\n");

        UART_SendString("Options:\r\n");
        UART_SendString("1. Reset accumulated capacity to 0\r\n");
        UART_SendString("2. Use a different battery\r\n");
        UART_SendString("3. Check if gauge was running during previous tests\r\n\r\n");
    }

}

/**
 * Stop the discharge test
 */
void DischargeTest_Stop(const char* reason)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*          TEST STOPPED                        *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("Reason: ");
    UART_SendString(reason);
    UART_SendString("\r\n");
    UART_SendString("Total Cycles Completed: ");
    UART_SendNumber(test_ctx.cycle_number - 1);
    UART_SendString("\r\n");

    uint32_t total_time_min = (GetTick_ms() - test_ctx.test_start_time_ms) / 60000;
    UART_SendString("Total Test Time: ");
    UART_SendNumber(total_time_min);
    UART_SendString(" minutes (");
    UART_SendNumber(total_time_min / 60);
    UART_SendString(" hours)\r\n");

    UART_SendString("************************************************\r\n\r\n");

    test_ctx.status = TEST_COMPLETE;
}

/**
 * Main test update - call this in your main loop
 * This should be called frequently (e.g., every 100ms or faster)
 */
void DischargeTest_Update(void)
{
    if (test_ctx.status != TEST_RUNNING) {
        return;
    }

    // Get current time
    uint32_t current_time = GetTick_ms();

    // Calculate time within current cycle
    uint32_t time_in_test = current_time - test_ctx.test_start_time_ms;
    uint32_t time_in_cycle = time_in_test % CYCLE_DURATION_MS;
    uint32_t expected_cycle = (time_in_test / CYCLE_DURATION_MS) + 1;

    // Read battery data
    BatteryMonitorData_t monitor_data;
    BatteryMonitor_ReadAll(&monitor_data);

    // Convert to BMS state
    BatteryState_Int_t bms_state;
    convert_monitor_to_bms_int(&monitor_data, &bms_state);

    BmsResult_Int_t bms_result;
    bms_update_soc_int(&bms_state, &bms_result);

    // Safety check - voltage cutoff
    if (monitor_data.acc_voltage_mv < VOLTAGE_CUTOFF_MV) {
        DischargeTest_Stop("Voltage cutoff reached - battery depleted");
        return;
    }

    // Safety check - max cycles
    if (expected_cycle > MAX_CYCLES) {
        DischargeTest_Stop("Maximum cycles reached");
        return;
    }

    // Check if new cycle started
    if (expected_cycle > test_ctx.cycle_number) {
        test_ctx.cycle_number = expected_cycle;
        test_ctx.voltage_drop_checked = false;
        test_ctx.transmission_decision_checked = false;

        LogCycleStart(test_ctx.cycle_number, &monitor_data);

        // Store voltage before this cycle's load
        test_ctx.voltage_before_cycle_mv = monitor_data.acc_voltage_mv;
    }

    // Transmission decision check (at start of each cycle, once)
    if (!test_ctx.transmission_decision_checked && time_in_cycle < 5000) {
        test_ctx.transmission_decision_checked = true;

        bms_transmission_decision_int(&bms_state, &bms_result);

        UART_SendString("\r\n");
        print_bms_decision_int(&bms_result);

        // Check if transmission blocked
        if (bms_result.decision == DECISION_BLOCK) {
            DischargeTest_Stop("Transmission BLOCKED - unsafe to continue");
            return;
        }
    }

    // Voltage drop check (10 seconds after cycle start, once per cycle)
    if (!test_ctx.voltage_drop_checked &&
        time_in_cycle >= VOLTAGE_DROP_CHECK_TIME_MS &&
        time_in_cycle < (VOLTAGE_DROP_CHECK_TIME_MS + 5000)) {

        test_ctx.voltage_drop_checked = true;

        // Measure voltage during high current load
        uint16_t voltage_during = monitor_data.acc_voltage_mv;
        uint16_t actual_drop = (test_ctx.voltage_before_cycle_mv > voltage_during) ?
                               (test_ctx.voltage_before_cycle_mv - voltage_during) : 0;

        // Monitor transmission
        BmsResult_Int_t drop_result;
        bms_monitor_transmission_int(test_ctx.voltage_before_cycle_mv,
                                    voltage_during,
                                    VOLTAGE_DROP_MAX_ALLOWED_MV,
                                    &drop_result);

        LogVoltageDrop(test_ctx.voltage_before_cycle_mv, voltage_during,
                      actual_drop, &drop_result);

        // Check if transmission should abort
        if (drop_result.decision == DECISION_ABORT) {
            DischargeTest_Stop("Transmission ABORTED - excessive voltage drop");
            return;
        }
    }

    // Periodic voltage/current reading (every minute)
    if ((current_time - test_ctx.last_voltage_reading_ms) >= VOLTAGE_READING_INTERVAL_MS) {
        test_ctx.last_voltage_reading_ms = current_time;

        LogReading(&monitor_data, &bms_result);
    }

    // Temperature trend monitoring (every 10 minutes)
    if ((current_time - test_ctx.last_temp_reading_ms) >= TEMP_READING_INTERVAL_MS) {
        test_ctx.last_temp_reading_ms = current_time;

        // Add temperature reading for trend analysis
        int16_t temp_dc = monitor_data.acc_temp_hundredths_c / 10;
        bms_add_temp_reading(temp_dc, current_time);

        // Check temperature trend
        TempTrendResult_t temp_trend;
        bms_monitor_temperature_trend(&temp_trend);

        if (temp_trend.status != TREND_INSUFFICIENT_DATA) {
            UART_SendString("\r\n");
            print_temp_trend_int(&temp_trend);
        }
    }
}

/**
 * Request test abort (can call from interrupt or button)
 */
void DischargeTest_RequestAbort(void)
{
    if (test_ctx.status == TEST_RUNNING) {
        DischargeTest_Stop("User abort requested");
    }
}

/**
 * Check if test is running
 */
bool DischargeTest_IsRunning(void)
{
    return (test_ctx.status == TEST_RUNNING);
}

/**
 * Get current cycle number
 */
uint16_t DischargeTest_GetCycleNumber(void)
{
    return test_ctx.cycle_number;
}
