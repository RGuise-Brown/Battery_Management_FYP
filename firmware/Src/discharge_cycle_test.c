/*
 * discharge_cycle_test.c
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Automated discharge cycle test for BMS validation (v2 - INA226 + TMP36)
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
#include "discharge_cycle_test.h"
#include "bms_algorithms_v2.h"          // *** CHANGED: v2 algorithms
#include "system_monitoring.h"          // *** CHANGED: SystemMonitorData_t
#include "uart.h"
#include "systick_timing.h"

/* ==================== Test Configuration ==================== */

// Timing intervals
#define VOLTAGE_READING_INTERVAL_MS     60000       // Read voltage/current every 1 minute (was 30s)
#define TEMP_READING_INTERVAL_MS        600000      // Read temperature every 10 minutes (was 1 min)
#define CYCLE_DURATION_MS               3600000     // 60 minutes per cycle
#define SOC_VOLTAGE_CHECK_INTERVAL_MS   900000      // Check SOC vs Voltage every 15 minutes

// Cycle phase timing (when BMS checks occur)
#define PHASE_HIGH_START_MS             0           // Start of high current phase
#define PHASE_HIGH_END_MS               3000000     // End of high current (50 min)
#define PHASE_LOW_START_MS              3000000     // Start of low current (50 min)
#define PHASE_LOW_END_MS                3300000     // End of low current (55 min)
#define PHASE_MEDIUM_START_MS           3300000     // Start of medium current (55 min)
#define PHASE_MEDIUM_END_MS             3600000     // End of medium current (60 min)

// Voltage drop measurement timing
#define VOLTAGE_DROP_CHECK_TIME_MS      10000       // Check voltage drop 10s after cycle starts
#define VOLTAGE_DROP_MAX_ALLOWED_MV     2000        // Maximum acceptable drop

// Safety limits
#define VOLTAGE_CUTOFF_MV               100        // Stop test if voltage drops below this
#define TEMP_THRESHOLD_DC               0           // 0°C threshold for capacity switching
#define MAX_CYCLES                      25          // Maximum cycles (safety limit)*/

/* 10 min version*/
// Timing intervals (scaled for 10-min cycle)
/*#define VOLTAGE_READING_INTERVAL_MS     10000       // Read voltage/current every 10 seconds (was 1 min)
#define TEMP_READING_INTERVAL_MS        90000       // Read temperature every 1 minute (was 10 min)
#define CYCLE_DURATION_MS               600000      // 10 minutes per cycle (was 60 min)
#define SOC_VOLTAGE_CHECK_INTERVAL_MS   150000      // Check SOC vs Voltage every 2.5 minutes (was 15 min)

// Cycle phase timing (when BMS checks occur) - scaled proportionally
#define PHASE_HIGH_START_MS             0           // Start of high current phase
#define PHASE_HIGH_END_MS               500000      // End of high current (8.33 min = 50/60 * 10)
#define PHASE_LOW_START_MS              500000      // Start of low current (8.33 min)
#define PHASE_LOW_END_MS                550000      // End of low current (9.17 min = 55/60 * 10)
#define PHASE_MEDIUM_START_MS           550000      // Start of medium current (9.17 min)
#define PHASE_MEDIUM_END_MS             600000      // End of medium current (10 min)

// Voltage drop measurement timing
#define VOLTAGE_DROP_CHECK_TIME_MS      10000       // Check voltage drop 10s after cycle starts (same)
#define VOLTAGE_DROP_MAX_ALLOWED_MV     2000        // Maximum acceptable drop (same)

// Safety limits
#define VOLTAGE_CUTOFF_MV               100        // Stop test if voltage drops below this
#define TEMP_THRESHOLD_DC               0           // 0°C threshold for capacity switching
#define MAX_CYCLES                      20          // Maximum cycles (safety limit)*/


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
    uint32_t last_soc_check_ms;             // *** ADDED
    uint32_t last_temp_add_ms;
    uint16_t voltage_before_cycle_mv;
    uint16_t voltage_previous_reading_mv;
    bool voltage_drop_checked;
    int16_t last_temp_region;           // *** ADDED: track temperature region changes
    uint32_t capacity_at_start_uah;     // *** ADDED: track initial capacity
} TestContext_t;

static TestContext_t test_ctx = {0};

/* ==================== Data Logging - Compact Format ==================== */

/**
 * Print test header
 */
static void PrintTestHeader(void)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*   BMS DISCHARGE CYCLE TEST (v2)             *\r\n");
    UART_SendString("*   Using INA226 + TMP36                      *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("\r\n");

    UART_SendString("Test Configuration:\r\n");
    UART_SendString("  Phase 1 (0-50 min):  1000 mA (High)\r\n");
    UART_SendString("  Phase 2 (50-55 min): 200 mA (Low)\r\n");
    UART_SendString("  Phase 3 (55-60 min): 400 mA (Medium)\r\n");
    UART_SendString("  Cycle Duration:      60 minutes\r\n");
    UART_SendString("  Voltage Cutoff:      ");
    UART_SendNumber(VOLTAGE_CUTOFF_MV);
    UART_SendString(" mV\r\n");
    UART_SendString("  Max Cycles:          ");
    UART_SendNumber(MAX_CYCLES);
    UART_SendString("\r\n\r\n");

    UART_SendString("IMPORTANT:\r\n");
    UART_SendString("1. Program DC load with same sequence\r\n");
    UART_SendString("2. Start DC load sequence\r\n");
    UART_SendString("3. Test will log data every minute\r\n\r\n");
}

/**
 * Log cycle start information (detailed)
 */
static void LogCycleStart(uint16_t cycle, const SystemMonitorData_t* monitor)
{
    UART_SendString("\r\n");
    UART_SendString("====== CYCLE ");
    UART_SendNumber(cycle);
    UART_SendString(" START ======\r\n");

    uint32_t elapsed_total_min = ((GetTick_ms() - test_ctx.test_start_time_ms) / 60000);
    UART_SendString("Test Time: ");
    UART_SendNumber(elapsed_total_min);
    UART_SendString(" min | V: ");
    UART_SendNumber(monitor->ina226_bus_mv);
    UART_SendString(" mV | I: ");
    UART_SendSignedNumber(monitor->ina226_current_ma);
    UART_SendString(" mA | T: ");
    int16_t temp_dc = monitor->adc_temp_tenths_c;
    UART_SendSignedNumber(temp_dc / 10);
    UART_SendString(".");
    uint16_t temp_frac = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    UART_SendNumber(temp_frac);
    UART_SendString(" C\r\n");

    BmsResult_Int_t bms_result;
    bms_update_soc_v2(monitor, &bms_result);

    UART_SendString("SOC: ");
    UART_SendNumber(bms_result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result.adjusted_soc_pm % 10);
    UART_SendString(" % | Used: ");
    UART_SendNumber(monitor->accumulated_uAh / 1000);
    UART_SendString(" mAh | Avail: ");
    UART_SendNumber(bms_result.available_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");
}

/**
 * Log periodic reading (COMPACT - one line per minute)
 */
static void LogReading(const SystemMonitorData_t* monitor, const BmsResult_Int_t* bms_result)
{
    uint32_t elapsed_ms = GetTick_ms() - test_ctx.test_start_time_ms;
    uint32_t elapsed_min = elapsed_ms / 60000;

    // Compact one-line format: [TIME] V:xxxx I:xxx T:xx.x SOC:xx.x% Cap:xxxx
    UART_SendString("[");
    if (elapsed_min < 100) UART_SendString(" ");
    if (elapsed_min < 10) UART_SendString(" ");
    UART_SendNumber(elapsed_min);
    UART_SendString("m] V:");
    UART_SendNumber(monitor->ina226_bus_mv);
    UART_SendString(" I:");
    UART_SendSignedNumber(monitor->ina226_current_ma);
    UART_SendString(" T:");
    int16_t temp_dc = monitor->adc_temp_tenths_c;
    UART_SendSignedNumber(temp_dc / 10);
    UART_SendString(".");
    uint16_t temp_frac = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    UART_SendNumber(temp_frac);
    UART_SendString(" SOC:");
    UART_SendNumber(bms_result->adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result->adjusted_soc_pm % 10);
    UART_SendString("% Used:");
    UART_SendNumber(monitor->accumulated_uAh / 1000);
    UART_SendString("mAh\r\n");
}

/**
 * Log voltage drop measurement (DETAILED - important event)
 */
static void LogVoltageDrop(uint16_t v_before, uint16_t v_during,
                          uint16_t actual_drop, const BmsResult_Int_t* result)
{
    UART_SendString("\r\n");
    UART_SendString("+---- VOLTAGE DROP CHECK ----+\r\n");
    UART_SendString("| Before: ");
    UART_SendNumber(v_before);
    UART_SendString(" mV\r\n");
    UART_SendString("| During: ");
    UART_SendNumber(v_during);
    UART_SendString(" mV\r\n");
    UART_SendString("| Drop:   ");
    UART_SendNumber(actual_drop);
    UART_SendString(" mV (max: ");
    UART_SendNumber(VOLTAGE_DROP_MAX_ALLOWED_MV);
    UART_SendString(")\r\n");
    UART_SendString("| Status: ");
    switch(result->decision) {
        case DECISION_GO:    UART_SendString("OK - CONTINUE\r\n"); break;
        case DECISION_WARN:  UART_SendString("WARNING\r\n"); break;
        case DECISION_ABORT: UART_SendString("ABORT!\r\n"); break;
        default:             UART_SendString("UNKNOWN\r\n"); break;
    }
    UART_SendString("| ");
    UART_SendString(result->message);
    UART_SendString("\r\n+----------------------------+\r\n\r\n");
}

/**
 * Log temperature threshold crossing (DETAILED - important event)
 */
static void LogTempThresholdCrossing(int16_t old_temp_dc, int16_t new_temp_dc,
                                     uint32_t old_capacity_uah, uint32_t new_capacity_uah)
{
    UART_SendString("\r\n");
    UART_SendString("!!! TEMPERATURE THRESHOLD CROSSED !!!\r\n");
    UART_SendString("+---- CAPACITY SWITCH ----+\r\n");
    UART_SendString("| Old Temp:  ");
    UART_SendSignedNumber(old_temp_dc / 10);
    UART_SendString(".");
    uint16_t old_frac = (old_temp_dc < 0) ? ((-old_temp_dc) % 10) : (old_temp_dc % 10);
    UART_SendNumber(old_frac);
    UART_SendString(" C\r\n");
    UART_SendString("| New Temp:  ");
    UART_SendSignedNumber(new_temp_dc / 10);
    UART_SendString(".");
    uint16_t new_frac = (new_temp_dc < 0) ? ((-new_temp_dc) % 10) : (new_temp_dc % 10);
    UART_SendNumber(new_frac);
    UART_SendString(" C\r\n");
    UART_SendString("| Old Cap:   ");
    UART_SendNumber(old_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");
    UART_SendString("| New Cap:   ");
    UART_SendNumber(new_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");

    if (new_temp_dc >= 0 && old_temp_dc < 0) {
        UART_SendString("| Direction: WARMING (COLD -> WARM)\r\n");
        UART_SendString("| Expected:  8.3Ah -> 10.5Ah\r\n");
    } else if (new_temp_dc < 0 && old_temp_dc >= 0) {
        UART_SendString("| Direction: COOLING (WARM -> COLD)\r\n");
        UART_SendString("| Expected:  10.5Ah -> 8.3Ah\r\n");
    }
    UART_SendString("+------------------------+\r\n\r\n");
}

/**
 * Log voltage-based SOC check (DETAILED - important validation event)
 */
static void LogSocVoltageCheck(const BmsVoltageCheck_t* check,
                               uint16_t voltage_mv,
                               int16_t temp_dc,
                               uint16_t integrated_soc_pm)
{
    UART_SendString("\r\n");
    UART_SendString("+---- SOC VOLTAGE CROSS-CHECK ----+\r\n");

    if (!check->active) {
        UART_SendString("| Status: INACTIVE (Temp > 0C)    |\r\n");
        UART_SendString("+---------------------------------+\r\n\r\n");
        return;
    }

    UART_SendString("| Voltage:      ");
    UART_SendNumber(voltage_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("| Temperature:  ");
    UART_SendSignedNumber(temp_dc / 10);
    UART_SendString(".");
    uint16_t temp_frac = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    UART_SendNumber(temp_frac);
    UART_SendString(" C (COLD)\r\n");

    UART_SendString("| Expected SOC: ");
    UART_SendNumber(check->soc_lower_pm / 10);
    UART_SendString(".");
    UART_SendNumber(check->soc_lower_pm % 10);
    UART_SendString("% - ");
    UART_SendNumber(check->soc_upper_pm / 10);
    UART_SendString(".");
    UART_SendNumber(check->soc_upper_pm % 10);
    UART_SendString("%\r\n");

    UART_SendString("| Integ. SOC:   ");
    UART_SendNumber(integrated_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(integrated_soc_pm % 10);
    UART_SendString("%\r\n");

    UART_SendString("| Agreement:    ");
    if (check->in_range) {
        UART_SendString("YES - OK\r\n");
    } else {
        UART_SendString("NO - MISMATCH!\r\n");
        UART_SendString("| >>> WARNING: SOC/Voltage disagree <<<\r\n");
    }

    UART_SendString("+---------------------------------+\r\n\r\n");
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
    test_ctx.last_soc_check_ms = 0;
    test_ctx.last_temp_add_ms = 0;
    test_ctx.voltage_before_cycle_mv = 0;
    test_ctx.voltage_drop_checked = false;
    test_ctx.last_temp_region = 0;
    test_ctx.capacity_at_start_uah = 0;
}

/**
 * Start the discharge test
 */
void DischargeTest_Start(void)
{
    PrintTestHeader();

    UART_SendString("*** TEST STARTING ***\r\n\r\n");

    // Initialize test state
    test_ctx.status = TEST_RUNNING;
    test_ctx.cycle_number = 1;
    test_ctx.test_start_time_ms = GetTick_ms();
    test_ctx.last_voltage_reading_ms = GetTick_ms();
    test_ctx.last_temp_reading_ms = GetTick_ms();
    test_ctx.last_soc_check_ms = GetTick_ms();
    test_ctx.last_temp_add_ms = GetTick_ms();
    test_ctx.voltage_drop_checked = false;

    // *** CHANGED: Read SystemMonitorData_t ***
    SystemMonitorData_t monitor_data;
    SystemMonitor_ReadAll(&monitor_data);

    // *** CHANGED: Use v2 BMS function ***
    BmsResult_Int_t bms_result;
    bms_update_soc_v2(&monitor_data, &bms_result);

    // Store initial state
    test_ctx.voltage_before_cycle_mv = monitor_data.ina226_bus_mv;
    test_ctx.last_temp_region = (monitor_data.adc_temp_tenths_c < TEMP_THRESHOLD_DC) ? -1 : 1;
    test_ctx.capacity_at_start_uah = bms_result.available_capacity_uah;

    UART_SendString("=== INITIAL BATTERY STATE ===\r\n");
    UART_SendString("Voltage:     ");
    UART_SendNumber(monitor_data.ina226_bus_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("Current:     ");
    UART_SendSignedNumber(monitor_data.ina226_current_ma);
    UART_SendString(" mA\r\n");

    UART_SendString("Temperature: ");
    int16_t temp_dc = monitor_data.adc_temp_tenths_c;
    UART_SendSignedNumber(temp_dc / 10);
    UART_SendString(".");
    uint16_t temp_frac = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    UART_SendNumber(temp_frac);
    UART_SendString(" C (");
    UART_SendString((temp_dc < 0) ? "COLD" : "WARM");
    UART_SendString(" region)\r\n");

    UART_SendString("Used Cap:    ");
    UART_SendNumber(monitor_data.accumulated_uAh / 1000);
    UART_SendString(" mAh\r\n");

    UART_SendString("Avail Cap:   ");
    UART_SendNumber(bms_result.available_capacity_uah / 1000);
    UART_SendString(" mAh\r\n");

    UART_SendString("SOC:         ");
    UART_SendNumber(bms_result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result.adjusted_soc_pm % 10);
    UART_SendString(" %\r\n\r\n");

    // Warning if starting capacity usage is high
    if (monitor_data.accumulated_uAh > (bms_result.available_capacity_uah / 2)) {
        UART_SendString("*** WARNING: Battery already partially discharged ***\r\n");
        UART_SendString("Consider resetting coulomb counter for full test\r\n\r\n");
    }

    LogCycleStart(test_ctx.cycle_number, &monitor_data);
}

/**
 * Stop the discharge test
 */
void DischargeTest_Stop(const char* reason)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*   TEST STOPPED                              *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("Reason: ");
    UART_SendString(reason);
    UART_SendString("\r\n");
    UART_SendString("Cycles Completed: ");
    UART_SendNumber(test_ctx.cycle_number - 1);
    UART_SendString("\r\n");

    uint32_t total_time_min = (GetTick_ms() - test_ctx.test_start_time_ms) / 60000;
    UART_SendString("Total Time: ");
    UART_SendNumber(total_time_min);
    UART_SendString(" min (");
    UART_SendNumber(total_time_min / 60);
    UART_SendString(" hrs)\r\n");

    // Final battery state
    SystemMonitorData_t monitor_data;
    SystemMonitor_ReadAll(&monitor_data);

    BmsResult_Int_t bms_result;
    bms_update_soc_v2(&monitor_data, &bms_result);

    UART_SendString("\r\n=== FINAL BATTERY STATE ===\r\n");
    UART_SendString("Voltage:   ");
    UART_SendNumber(monitor_data.ina226_bus_mv);
    UART_SendString(" mV\r\n");
    UART_SendString("SOC:       ");
    UART_SendNumber(bms_result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result.adjusted_soc_pm % 10);
    UART_SendString(" %\r\n");
    UART_SendString("Used Cap:  ");
    UART_SendNumber(monitor_data.accumulated_uAh / 1000);
    UART_SendString(" mAh\r\n");

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

    // *** STORE PREVIOUS VOLTAGE BEFORE READING NEW DATA ***
    // This captures the voltage from the last update (before new cycle starts)

    // *** CHANGED: Read SystemMonitorData_t ***
    SystemMonitorData_t monitor_data;
    SystemMonitor_ReadAll(&monitor_data);


    // *** CHANGED: Use v2 BMS function ***
    BmsResult_Int_t bms_result;
    bms_update_soc_v2(&monitor_data, &bms_result);

    // Safety check - voltage cutoff
    if (monitor_data.ina226_bus_mv < VOLTAGE_CUTOFF_MV) {
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

        LogCycleStart(test_ctx.cycle_number, &monitor_data);

        // Store voltage before this cycle's load
        test_ctx.voltage_before_cycle_mv = monitor_data.ina226_bus_mv;
    }

    // Voltage drop check (10 seconds after cycle start, once per cycle)
    if (!test_ctx.voltage_drop_checked &&
        time_in_cycle >= VOLTAGE_DROP_CHECK_TIME_MS &&
        time_in_cycle < (VOLTAGE_DROP_CHECK_TIME_MS + 5000)) {

        test_ctx.voltage_drop_checked = true;

        // Measure voltage during high current load
        uint16_t voltage_during = monitor_data.ina226_bus_mv;
        uint16_t actual_drop = (test_ctx.voltage_before_cycle_mv > voltage_during) ?
                               (test_ctx.voltage_before_cycle_mv - voltage_during) : 0;

        // *** CHANGED: Use v2 transmission monitoring ***
        BmsResult_Int_t drop_result;
        bms_monitor_transmission_v2(test_ctx.voltage_before_cycle_mv,
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

    // SOC Voltage Cross-Check (every 15 minutes) - Function 2
    if ((current_time - test_ctx.last_soc_check_ms) >= SOC_VOLTAGE_CHECK_INTERVAL_MS) {
        test_ctx.last_soc_check_ms = current_time;

        // Convert to BatteryState_Int_t for Function 2
        BatteryState_Int_t bms_state;
        convert_system_to_bms_int(&monitor_data, &bms_state);

        // Perform voltage-based SOC check
        BmsVoltageCheck_t voltage_check = {0};
        BmsAlgorithm_Function2_CheckVoltageRange(&bms_state, &bms_result, &voltage_check);

        // Log results (detailed format)
        LogSocVoltageCheck(&voltage_check,
                          monitor_data.ina226_bus_mv,
                          monitor_data.adc_temp_tenths_c,
                          bms_result.adjusted_soc_pm);

        // Optional: Stop test if severe mismatch detected
        if (voltage_check.active && !voltage_check.in_range) {
            // Calculate how far off the SOC is
            uint16_t soc_mid = (voltage_check.soc_lower_pm + voltage_check.soc_upper_pm) / 2;
            int32_t soc_error = (int32_t)bms_result.adjusted_soc_pm - (int32_t)soc_mid;

            // If error is > 50% (500 per-mille), something is very wrong
            if (soc_error > 500 || soc_error < -500) {
                UART_SendString("*** CRITICAL: SOC mismatch > 50% ***\r\n");
                UART_SendString("Coulomb counter may be inaccurate!\r\n\r\n");
                // Optionally stop the test:
                // DischargeTest_Stop("Critical SOC/Voltage mismatch detected");
                // return;
            }
        }
    }

    // Check for temperature threshold crossing (DETAILED EVENT)
    int16_t current_temp_dc = monitor_data.adc_temp_tenths_c;
    int16_t current_region = (current_temp_dc < TEMP_THRESHOLD_DC) ? -1 : 1;

    if (current_region != test_ctx.last_temp_region) {
        // Temperature crossed threshold!
        uint32_t old_capacity = test_ctx.capacity_at_start_uah;
        uint32_t new_capacity = bms_result.available_capacity_uah;

        LogTempThresholdCrossing(
            (test_ctx.last_temp_region < 0) ? -250 : 200,  // Representative temp
            current_temp_dc,
            old_capacity,
            new_capacity
        );

        test_ctx.last_temp_region = current_region;
        test_ctx.capacity_at_start_uah = new_capacity;
    }

    if ((current_time - test_ctx.last_temp_add_ms) >= 10000) {  // *** USE test_ctx
        test_ctx.last_temp_add_ms = current_time;                // *** USE test_ctx
        bms_add_temp_reading_v2(monitor_data.adc_temp_tenths_c, current_time);
    }

    // Periodic voltage/current reading (COMPACT - every 1 minute)
    if ((current_time - test_ctx.last_voltage_reading_ms) >= VOLTAGE_READING_INTERVAL_MS) {
        test_ctx.last_voltage_reading_ms = current_time;

        // *** STORE VOLTAGE BEFORE LOGGING (this is the "before" value for next cycle) ***
        test_ctx.voltage_previous_reading_mv = monitor_data.ina226_bus_mv;

        LogReading(&monitor_data, &bms_result);
    }

    // Temperature trend monitoring (every 10 minutes)
    if ((current_time - test_ctx.last_temp_reading_ms) >= TEMP_READING_INTERVAL_MS) {
        test_ctx.last_temp_reading_ms = current_time;

        // *** CHANGED: Use v2 temp functions ***

        // Check temperature trend (DETAILED EVENT if significant)
        TempTrendResult_t temp_trend;
        bms_monitor_temperature_trend_v2(&temp_trend);

        if (temp_trend.status != TREND_INSUFFICIENT_DATA) {
            // Only log if trend is significant
            //if (temp_trend.status == TREND_COOLING_FAST ||
              //  temp_trend.status == TREND_WARMING_FAST ||
              //  temp_trend.capacity_will_decrease ||
              //  temp_trend.capacity_will_increase) {

                UART_SendString("\r\n+---- TEMPERATURE TREND ----+\r\n");
                UART_SendString("| ");
                UART_SendString(temp_trend.message);
                UART_SendString("    ");
                UART_SendString("| Rate: ");
                UART_SendSignedNumber(temp_trend.trend_dc_per_hour / 10);
                UART_SendString(".");
                uint16_t trend_frac = (temp_trend.trend_dc_per_hour < 0) ?
                                      ((-temp_trend.trend_dc_per_hour) % 10) :
                                      (temp_trend.trend_dc_per_hour % 10);
                UART_SendNumber(trend_frac);
                UART_SendString(" C/hr    ");
                UART_SendString("| Pred 1hr: ");
                UART_SendSignedNumber(temp_trend.predicted_temp_1hr_dc / 10);
                UART_SendString(".");
                uint16_t pred_frac = (temp_trend.predicted_temp_1hr_dc < 0) ?
                                     ((-temp_trend.predicted_temp_1hr_dc) % 10) :
                                     (temp_trend.predicted_temp_1hr_dc % 10);
                UART_SendNumber(pred_frac);
                UART_SendString(" C\r\n");
                UART_SendString("+---------------------------+\r\n\r\n");
            //}
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

/*

## Summary of Changes

### **V2 System Updates:**
1. Changed to `system_monitoring.h` and `bms_algorithms_v2.h`
2. `BatteryMonitorData_t` → `SystemMonitorData_t`
3. All field names updated for INA226
4. Function calls updated to v2 versions

### **Logging Style Changes:**

**COMPACT (Normal readings every minute):**
```
[  5m] V:3420 I:-1020 T:22.3 SOC:87.5% Used:1250mAh
[  6m] V:3418 I:-1015 T:22.4 SOC:86.8% Used:1270mAh
```

**DETAILED (Important events only):**
- Cycle start: Multi-line with full details
- Voltage drop checks: Formatted box with decision
- Temperature threshold crossings: Alert box with before/after
- Temperature trends: Only if significant (fast changes or capacity impact)

### **Key Features:**
1. **Minimal noise**: One line per minute for normal operation
2. **Event highlighting**: Important events get detailed multi-line output
3. **Temperature tracking**: Automatically detects and logs threshold crossings
4. **Trend filtering**: Only logs significant temperature trends
5. **16-hour friendly**: Won't flood your logs with unnecessary data

### **Example Output Pattern:**
```
====== CYCLE 1 START ======
Test Time: 0 min | V: 3650 mV | I: -50 mA | T: 22.5 C
SOC: 98.2 % | Used: 150 mAh | Avail: 10500 mAh

+---- VOLTAGE DROP CHECK ----+
| Before: 3650 mV
| During: 2850 mV
| Drop:   800 mV (max: 1000)
| Status: OK - CONTINUE
+----------------------------+

[  1m] V:3420 I:-1020 T:22.3 SOC:87.5% Used:1250mAh
[  2m] V:3418 I:-1015 T:22.4 SOC:86.8% Used:1270mAh
...*/
