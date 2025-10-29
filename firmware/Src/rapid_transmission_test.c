/*
 * rapid_transmission_test.c
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Test B: Rapid Multi-Transmission Test (v2 - INA226 + TMP36)
 *
 * This test validates voltage behavior when multiple transmissions occur
 * in quick succession (e.g., emergency beacon mode, retransmission).
 *
 * Procedure:
 * 1. Start at ~50% SOC (mid-discharge)
 * 2. Apply load for 10 seconds (transmission 1)
 * 3. Wait 2 minutes for voltage recovery
 * 4. Apply load for 10 seconds (transmission 2)
 * 5. Wait 2 minutes
 * 6. Repeat for transmission 3
 * 7. Monitor voltage recovery after each transmission
 *
 * Duration: ~10-15 minutes
 */

#include <stdint.h>
#include <stdbool.h>
#include "rapid_transmission_test.h"
#include "bms_algorithms_v2.h"          // *** CHANGED: v2 algorithms
#include "system_monitoring.h"          // *** CHANGED: SystemMonitorData_t
#include "systick_timing.h"
#include "uart.h"

/* ==================== Configuration ==================== */

#define NUM_TRANSMISSIONS           3
#define TRANSMISSION_DURATION_MS    10000       // 10 seconds
#define RECOVERY_TIME_MS            60000      // 1 minute
#define VOLTAGE_DROP_MAX_MV         1000        // 1000mV max drop (for monitoring)
#define TX_CURRENT_MA               800        // 8000mA (0.8A) pulse

/* ==================== Test State ==================== */

typedef enum {
    RAPID_TEST_IDLE,
    RAPID_TEST_STARTING,
    RAPID_TEST_PRE_TX_MEASURE,          // *** CHANGED: just measure, no decision
    RAPID_TEST_TX_ACTIVE,
    RAPID_TEST_TX_MONITORING,
    RAPID_TEST_RECOVERY,
    RAPID_TEST_COMPLETE,
    RAPID_TEST_ABORTED
} RapidTestState_t;

typedef struct {
    RapidTestState_t state;
    uint8_t current_tx_number;
    uint32_t state_start_time_ms;
    uint16_t voltage_before_tx_mv;
    uint16_t voltage_during_tx_mv;
    uint16_t voltage_after_tx_mv;
    int16_t current_during_tx_ma;
    bool tx_completed[NUM_TRANSMISSIONS];
    uint16_t voltage_drop[NUM_TRANSMISSIONS];
} RapidTestContext_t;

static RapidTestContext_t rapid_test_ctx = {0};

/* ==================== Helper Functions ==================== */

static void LogTransmissionStart(uint8_t tx_num, const SystemMonitorData_t* monitor)
{
    UART_SendString("\r\n");
    UART_SendString("================================================\r\n");
    UART_SendString("  TRANSMISSION ");
    UART_SendNumber(tx_num);
    UART_SendString(" START\r\n");
    UART_SendString("================================================\r\n");

    UART_SendString("Voltage (resting): ");
    UART_SendNumber(monitor->ina226_bus_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("Current:           ");
    UART_SendSignedNumber(monitor->ina226_current_ma);
    UART_SendString(" mA\r\n");

    UART_SendString("Temperature:       ");
    int16_t temp_dc = monitor->adc_temp_tenths_c;
    UART_SendSignedNumber(temp_dc / 10);
    UART_SendString(".");
    uint16_t temp_frac = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    UART_SendNumber(temp_frac);
    UART_SendString(" C\r\n");

    BmsResult_Int_t bms_result;
    bms_update_soc_v2(monitor, &bms_result);

    UART_SendString("SOC:               ");
    UART_SendNumber(bms_result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result.adjusted_soc_pm % 10);
    UART_SendString(" %\r\n");

    UART_SendString("================================================\r\n\r\n");
}

static void LogTransmissionResult(uint8_t tx_num)
{
    UART_SendString("\r\n");
    UART_SendString("+-------------------------------------------+\r\n");
    UART_SendString("| TRANSMISSION ");
    UART_SendNumber(tx_num);
    UART_SendString(" RESULT\r\n");
    UART_SendString("+-------------------------------------------+\r\n");

    UART_SendString("| Completed:    ");
    UART_SendString(rapid_test_ctx.tx_completed[tx_num - 1] ? "YES" : "NO");
    UART_SendString("\r\n");

    UART_SendString("| Before:       ");
    UART_SendNumber(rapid_test_ctx.voltage_before_tx_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("| During:       ");
    UART_SendNumber(rapid_test_ctx.voltage_during_tx_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("| After:        ");
    UART_SendNumber(rapid_test_ctx.voltage_after_tx_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("| Drop:         ");
    UART_SendNumber(rapid_test_ctx.voltage_drop[tx_num - 1]);
    UART_SendString(" mV\r\n");

    UART_SendString("| Current:      ");
    UART_SendSignedNumber(rapid_test_ctx.current_during_tx_ma);
    UART_SendString(" mA\r\n");

    uint16_t recovery = (rapid_test_ctx.voltage_after_tx_mv > rapid_test_ctx.voltage_during_tx_mv) ?
                        (rapid_test_ctx.voltage_after_tx_mv - rapid_test_ctx.voltage_during_tx_mv) : 0;
    UART_SendString("| Recovery:     ");
    UART_SendNumber(recovery);
    UART_SendString(" mV\r\n");

    UART_SendString("+-------------------------------------------+\r\n\r\n");
}

/* ==================== Public Functions ==================== */

void RapidTransmissionTest_Init(void)
{
    rapid_test_ctx.state = RAPID_TEST_IDLE;
    rapid_test_ctx.current_tx_number = 0;
    rapid_test_ctx.state_start_time_ms = 0;
    rapid_test_ctx.voltage_before_tx_mv = 0;
    rapid_test_ctx.voltage_during_tx_mv = 0;
    rapid_test_ctx.voltage_after_tx_mv = 0;
    rapid_test_ctx.current_during_tx_ma = 0;

    for (uint8_t i = 0; i < NUM_TRANSMISSIONS; i++) {
        rapid_test_ctx.tx_completed[i] = false;
        rapid_test_ctx.voltage_drop[i] = 0;
    }
}

void RapidTransmissionTest_Start(void)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("*   RAPID MULTI-TRANSMISSION TEST (v2)        *\r\n");
    UART_SendString("*   Using INA226 + TMP36                      *\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("\r\n");

    UART_SendString("Test Objective:\r\n");
    UART_SendString("  Monitor voltage behavior during successive\r\n");
    UART_SendString("  transmissions with short recovery periods\r\n");
    UART_SendString("\r\n");

    UART_SendString("Test Configuration:\r\n");
    UART_SendString("  - Number of TXs:    ");
    UART_SendNumber(NUM_TRANSMISSIONS);
    UART_SendString("\r\n");
    UART_SendString("  - TX Duration:      ");
    UART_SendNumber(TRANSMISSION_DURATION_MS / 1000);
    UART_SendString(" seconds\r\n");
    UART_SendString("  - Recovery Time:    ");
    UART_SendNumber(RECOVERY_TIME_MS / 1000);
    UART_SendString(" seconds\r\n");
    UART_SendString("  - Target Current:   ");
    UART_SendNumber(TX_CURRENT_MA);
    UART_SendString(" mA\r\n");
    UART_SendString("\r\n");

    UART_SendString("Requirements:\r\n");
    UART_SendString("  - Battery should be partially discharged\r\n");
    UART_SendString("  - DC load ready for manual control\r\n");
    UART_SendString("\r\n");

    // Check battery state
    UART_SendString("Checking battery state...\r\n");

    SystemMonitorData_t monitor_data;
    SystemMonitor_ReadAll(&monitor_data);

    UART_SendString("\r\nCurrent Battery State:\r\n");
    UART_SendString("  Voltage:     ");
    UART_SendNumber(monitor_data.ina226_bus_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("  Current:     ");
    UART_SendSignedNumber(monitor_data.ina226_current_ma);
    UART_SendString(" mA\r\n");

    UART_SendString("  Temperature: ");
    int16_t temp_dc = monitor_data.adc_temp_tenths_c;
    UART_SendSignedNumber(temp_dc / 10);
    UART_SendString(".");
    uint16_t temp_frac = (temp_dc < 0) ? ((-temp_dc) % 10) : (temp_dc % 10);
    UART_SendNumber(temp_frac);
    UART_SendString(" C\r\n");

    BmsResult_Int_t bms_result;
    bms_update_soc_v2(&monitor_data, &bms_result);

    UART_SendString("  SOC (BMS):   ");
    UART_SendNumber(bms_result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result.adjusted_soc_pm % 10);
    UART_SendString(" %\r\n");

    UART_SendString("  Used Cap:    ");
    UART_SendNumber(monitor_data.accumulated_uAh / 1000);
    UART_SendString(" mAh\r\n\r\n");

    // Give voltage-based assessment (informational only)
    UART_SendString("Voltage Assessment: ");
    if (monitor_data.ina226_bus_mv > 3400) {
        UART_SendString("High (>3.4V)\r\n");
    } else if (monitor_data.ina226_bus_mv > 3200) {
        UART_SendString("Medium (3.2-3.4V)\r\n");
    } else if (monitor_data.ina226_bus_mv > 3000) {
        UART_SendString("Low (3.0-3.2V)\r\n");
    } else {
        UART_SendString("Very Low (<3.0V)\r\n");
    }

    UART_SendString("\r\n");

    UART_SendString("Starting test in 5 seconds...\r\n");
    UART_SendString("Prepare to manually control DC load.\r\n\r\n");

    delay_ms(5000);

    rapid_test_ctx.state = RAPID_TEST_STARTING;
    rapid_test_ctx.current_tx_number = 1;
    rapid_test_ctx.state_start_time_ms = GetTick_ms();

    UART_SendString("*** TEST STARTED ***\r\n");
}

void RapidTransmissionTest_Update(void)
{
    if (rapid_test_ctx.state == RAPID_TEST_IDLE ||
        rapid_test_ctx.state == RAPID_TEST_COMPLETE ||
        rapid_test_ctx.state == RAPID_TEST_ABORTED) {
        return;
    }

    uint32_t current_time = GetTick_ms();
    uint32_t elapsed_in_state = current_time - rapid_test_ctx.state_start_time_ms;

    SystemMonitorData_t monitor_data;
    SystemMonitor_ReadAll(&monitor_data);

    BmsResult_Int_t bms_result;

    switch (rapid_test_ctx.state) {

        case RAPID_TEST_STARTING:
        {
            if (elapsed_in_state >= 5000) {
                rapid_test_ctx.state = RAPID_TEST_PRE_TX_MEASURE;
                rapid_test_ctx.state_start_time_ms = current_time;
            }
            break;
        }

        case RAPID_TEST_PRE_TX_MEASURE:  // *** CHANGED: just measure, no decision
        {
            LogTransmissionStart(rapid_test_ctx.current_tx_number, &monitor_data);

            // Store voltage before transmission
            rapid_test_ctx.voltage_before_tx_mv = monitor_data.ina226_bus_mv;

            UART_SendString("\r\n");
            UART_SendString(">>> APPLY ");
            UART_SendNumber(TX_CURRENT_MA);
            UART_SendString("mA LOAD NOW <<<\r\n");
            UART_SendString("Hold for ");
            UART_SendNumber(TRANSMISSION_DURATION_MS / 1000);
            UART_SendString(" seconds...\r\n\r\n");

            rapid_test_ctx.state = RAPID_TEST_TX_ACTIVE;
            rapid_test_ctx.state_start_time_ms = current_time;

            break;
        }

        case RAPID_TEST_TX_ACTIVE:
        {
            if (elapsed_in_state >= TRANSMISSION_DURATION_MS) {
                UART_SendString("\r\n>>> REMOVE LOAD NOW <<<\r\n\r\n");

                rapid_test_ctx.tx_completed[rapid_test_ctx.current_tx_number - 1] = true;
                rapid_test_ctx.state = RAPID_TEST_RECOVERY;
                rapid_test_ctx.state_start_time_ms = current_time;
            }
            // Sample voltage during transmission (after 2 seconds to let it stabilize)
            else if (elapsed_in_state >= 2000 && elapsed_in_state < 2500) {
                rapid_test_ctx.voltage_during_tx_mv = monitor_data.ina226_bus_mv;
                rapid_test_ctx.current_during_tx_ma = monitor_data.ina226_current_ma;

                uint16_t drop = (rapid_test_ctx.voltage_before_tx_mv > rapid_test_ctx.voltage_during_tx_mv) ?
                                (rapid_test_ctx.voltage_before_tx_mv - rapid_test_ctx.voltage_during_tx_mv) : 0;

                rapid_test_ctx.voltage_drop[rapid_test_ctx.current_tx_number - 1] = drop;

                // Use v2 transmission monitoring function for logging
                bms_monitor_transmission_v2(rapid_test_ctx.voltage_before_tx_mv,
                                           rapid_test_ctx.voltage_during_tx_mv,
                                           VOLTAGE_DROP_MAX_MV,
                                           &bms_result);

                UART_SendString("\r\n--- REAL-TIME TX MONITORING ---\r\n");
                UART_SendString("Voltage During: ");
                UART_SendNumber(rapid_test_ctx.voltage_during_tx_mv);
                UART_SendString(" mV\r\n");
                UART_SendString("Current During: ");
                UART_SendSignedNumber(rapid_test_ctx.current_during_tx_ma);
                UART_SendString(" mA\r\n");
                UART_SendString("Voltage Drop:   ");
                UART_SendNumber(drop);
                UART_SendString(" mV\r\n");
                UART_SendString("Status:         ");
                UART_SendString(bms_result.message);
                UART_SendString("\r\n\r\n");

                rapid_test_ctx.state = RAPID_TEST_TX_MONITORING;
            }

            break;
        }

        case RAPID_TEST_TX_MONITORING:
        {
            if (elapsed_in_state >= TRANSMISSION_DURATION_MS) {
                UART_SendString("\r\n>>> REMOVE LOAD NOW <<<\r\n\r\n");

                rapid_test_ctx.tx_completed[rapid_test_ctx.current_tx_number - 1] = true;
                rapid_test_ctx.state = RAPID_TEST_RECOVERY;
                rapid_test_ctx.state_start_time_ms = current_time;
            }

            break;
        }

        case RAPID_TEST_RECOVERY:
        {
            // Sample voltage near end of recovery period
            if (elapsed_in_state >= RECOVERY_TIME_MS - 1000 &&
                elapsed_in_state < RECOVERY_TIME_MS) {

                rapid_test_ctx.voltage_after_tx_mv = monitor_data.ina226_bus_mv;

                LogTransmissionResult(rapid_test_ctx.current_tx_number);
            }

            if (elapsed_in_state >= RECOVERY_TIME_MS) {

                if (rapid_test_ctx.current_tx_number < NUM_TRANSMISSIONS) {
                    rapid_test_ctx.current_tx_number++;
                    rapid_test_ctx.state = RAPID_TEST_PRE_TX_MEASURE;
                    rapid_test_ctx.state_start_time_ms = current_time;

                    UART_SendString("Recovery complete. Preparing next transmission...\r\n");
                } else {
                    RapidTransmissionTest_Stop("All transmissions completed successfully");
                }
            } else {
                // Countdown every 30 seconds
                uint32_t remaining_sec = (RECOVERY_TIME_MS - elapsed_in_state) / 1000;
                static uint32_t last_countdown = 0;

                if (remaining_sec != last_countdown && remaining_sec % 30 == 0) {
                    last_countdown = remaining_sec;
                    UART_SendString("Recovery: ");
                    UART_SendNumber(remaining_sec);
                    UART_SendString(" seconds remaining...\r\n");
                }
            }

            break;
        }

        default:
            break;
    }
}

void RapidTransmissionTest_Stop(const char* reason)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*   RAPID MULTI-TRANSMISSION TEST COMPLETE    *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("Reason: ");
    UART_SendString(reason);
    UART_SendString("\r\n\r\n");

    UART_SendString("Test Summary:\r\n");
    UART_SendString("=============\r\n\r\n");

    for (uint8_t i = 0; i < NUM_TRANSMISSIONS; i++) {
        UART_SendString("Transmission ");
        UART_SendNumber(i + 1);
        UART_SendString(":\r\n");

        UART_SendString("  Completed: ");
        UART_SendString(rapid_test_ctx.tx_completed[i] ? "YES" : "NO");
        UART_SendString("\r\n");

        if (rapid_test_ctx.tx_completed[i]) {
            UART_SendString("  V Drop:    ");
            UART_SendNumber(rapid_test_ctx.voltage_drop[i]);
            UART_SendString(" mV\r\n");
        }

        UART_SendString("\r\n");
    }

    bool all_completed = true;
    for (uint8_t i = 0; i < NUM_TRANSMISSIONS; i++) {
        if (!rapid_test_ctx.tx_completed[i]) {
            all_completed = false;
            break;
        }
    }

    if (all_completed) {
        UART_SendString(">>> TEST COMPLETED <<<\r\n");
        UART_SendString("All transmissions completed\r\n");
        UART_SendString("Voltage recovery data collected\r\n");
    } else {
        UART_SendString(">>> TEST INCOMPLETE <<<\r\n");
        UART_SendString("Not all transmissions completed\r\n");
    }

    UART_SendString("\r\n************************************************\r\n\r\n");

    rapid_test_ctx.state = RAPID_TEST_COMPLETE;
}

bool RapidTransmissionTest_IsRunning(void)
{
    return (rapid_test_ctx.state != RAPID_TEST_IDLE &&
            rapid_test_ctx.state != RAPID_TEST_COMPLETE &&
            rapid_test_ctx.state != RAPID_TEST_ABORTED);
}

uint8_t RapidTransmissionTest_GetCurrentTxNumber(void)
{
    return rapid_test_ctx.current_tx_number;
}

void RapidTransmissionTest_RequestAbort(void)
{
    if (RapidTransmissionTest_IsRunning()) {
        RapidTransmissionTest_Stop("User abort requested");
        rapid_test_ctx.state = RAPID_TEST_ABORTED;
    }
}
