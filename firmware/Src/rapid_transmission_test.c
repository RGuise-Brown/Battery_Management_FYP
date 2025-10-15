/*
 * rapid_transmission_test.c
 *
 *  Created on: Oct 13, 2025
 *      Author: rachaelguise-brown
 *
 * Test B: Rapid Multi-Transmission Test
 *
 * This test validates BMS behavior when multiple transmissions occur
 * in quick succession (e.g., emergency beacon mode, retransmission).
 *
 * Procedure:
 * 1. Start at ~50% SOC (mid-discharge)
 * 2. Apply 500mA load for 10 seconds (transmission 1)
 * 3. Wait 2 minutes for voltage recovery
 * 4. Check if BMS still approves transmission
 * 5. Apply 1A load for 10 seconds (transmission 2)
 * 6. Wait 2 minutes
 * 7. Repeat for transmission 3
 * 8. Monitor voltage recovery and BMS decisions
 *
 * Duration: ~10-15 minutes
 */

#include <stdint.h>
#include <stdbool.h>
#include "rapid_transmission_test.h"
#include "bms_algorithms.h"
#include "battery_monitor.h"
#include "systick_timing.h"
#include "uart.h"

/* ==================== Configuration ==================== */

#define NUM_TRANSMISSIONS           3
#define TRANSMISSION_DURATION_MS    10000
#define RECOVERY_TIME_MS            60000
#define VOLTAGE_DROP_MAX_MV         2500 // VERY LARGE FOR TESTING
#define TX_CURRENT_MA               5000

/* ==================== Test State ==================== */

typedef enum {
    RAPID_TEST_IDLE,
    RAPID_TEST_STARTING,
    RAPID_TEST_PRE_TX_CHECK,
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
    bool tx_approved[NUM_TRANSMISSIONS];
    bool tx_completed[NUM_TRANSMISSIONS];
    uint16_t voltage_drop[NUM_TRANSMISSIONS];
} RapidTestContext_t;

static RapidTestContext_t rapid_test_ctx = {0};

/* ==================== Helper Functions ==================== */

static void LogTransmissionStart(uint8_t tx_num, BatteryMonitorData_t* monitor)
{
    UART_SendString("\r\n");
    UART_SendString("================================================\r\n");
    UART_SendString("  TRANSMISSION ");
    UART_SendNumber(tx_num);
    UART_SendString(" START\r\n");
    UART_SendString("================================================\r\n");

    UART_SendString("Voltage (resting): ");
    UART_SendNumber(monitor->acc_voltage_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("Temperature:       ");
    int16_t temp = monitor->acc_temp_hundredths_c / 10;
    UART_SendSignedNumber(temp / 10);
    UART_SendString(".");
    UART_SendNumber(abs(temp % 10));
    UART_SendString(" C\r\n");

    BatteryState_Int_t bms_state;
    convert_monitor_to_bms_int(monitor, &bms_state);
    BmsResult_Int_t bms_result;
    bms_update_soc_int(&bms_state, &bms_result);

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

    UART_SendString("| Approved:     ");
    UART_SendString(rapid_test_ctx.tx_approved[tx_num - 1] ? "YES" : "NO");
    UART_SendString("\r\n");

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

    for (uint8_t i = 0; i < NUM_TRANSMISSIONS; i++) {
        rapid_test_ctx.tx_approved[i] = false;
        rapid_test_ctx.tx_completed[i] = false;
        rapid_test_ctx.voltage_drop[i] = 0;
    }
}

void RapidTransmissionTest_Start(void)
{
    UART_SendString("\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("*   RAPID MULTI-TRANSMISSION TEST             *\r\n");
    UART_SendString("*                                              *\r\n");
    UART_SendString("************************************************\r\n");
    UART_SendString("\r\n");

    UART_SendString("Test Objective:\r\n");
    UART_SendString("  Validate BMS behavior during successive\r\n");
    UART_SendString("  transmissions with short recovery periods\r\n");
    UART_SendString("\r\n");

    UART_SendString("Test Configuration:\r\n");
    UART_SendString("  - Number of TXs:    ");
    UART_SendNumber(NUM_TRANSMISSIONS);
    UART_SendString("\r\n");
    UART_SendString("  - TX Duration:      10 seconds\r\n");
    UART_SendString("  - Recovery Time:    2 minutes\r\n");
    UART_SendString("  - TX Current:       1000 mA\r\n");
    UART_SendString("  - Max Voltage Drop: ");
    UART_SendNumber(VOLTAGE_DROP_MAX_MV);
    UART_SendString(" mV\r\n");
    UART_SendString("\r\n");

    UART_SendString("Requirements:\r\n");
    UART_SendString("  - Battery voltage > 3.0V\r\n");
    UART_SendString("  - Enough capacity for 3 x 10-second pulses\r\n");
    UART_SendString("  - DC load ready for manual control\r\n");
    UART_SendString("\r\n");

    // Check battery state
    UART_SendString("Checking battery state...\r\n");

    BatteryMonitorData_t monitor_data;
    BatteryMonitor_ReadAll(&monitor_data);

    if (!monitor_data.acc_present) {
        UART_SendString("\r\n*** ERROR: Battery monitor not responding ***\r\n");
        UART_SendString("Check connections and try again.\r\n");
        UART_SendString("Test aborted.\r\n\r\n");
        return;
    }

    UART_SendString("\r\nCurrent Battery State:\r\n");
    UART_SendString("  Voltage:     ");
    UART_SendNumber(monitor_data.acc_voltage_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("  Temperature: ");
    int16_t temp = monitor_data.acc_temp_hundredths_c / 10;
    UART_SendSignedNumber(temp / 10);
    UART_SendString(".");
    UART_SendNumber(abs(temp % 10));
    UART_SendString(" C\r\n");

    // Calculate and show SOC (if accumulated capacity is being tracked)
    BatteryState_Int_t bms_state;
    convert_monitor_to_bms_int(&monitor_data, &bms_state);
    BmsResult_Int_t bms_result;
    bms_update_soc_int(&bms_state, &bms_result);

    UART_SendString("  SOC (BMS):   ");
    UART_SendNumber(bms_result.adjusted_soc_pm / 10);
    UART_SendString(".");
    UART_SendNumber(bms_result.adjusted_soc_pm % 10);
    UART_SendString(" %\r\n\r\n");

    // Check if voltage is sufficient
    if (monitor_data.acc_voltage_mv < 3000) {
        UART_SendString("*** ERROR: Battery voltage too low ***\r\n");
        UART_SendString("Current voltage: ");
        UART_SendNumber(monitor_data.acc_voltage_mv);
        UART_SendString(" mV\r\n");
        UART_SendString("Required:        > 3000 mV\r\n\r\n");
        UART_SendString("Battery may be too depleted for this test.\r\n");
        UART_SendString("Charge battery or use a less depleted cell.\r\n");
        UART_SendString("Test aborted.\r\n\r\n");
        return;
    }

    // Give voltage-based assessment
    UART_SendString("Voltage Assessment: ");
    if (monitor_data.acc_voltage_mv > 3400) {
        UART_SendString("Excellent (High SOC)\r\n");
    } else if (monitor_data.acc_voltage_mv > 3200) {
        UART_SendString("Good (Medium SOC)\r\n");
    } else if (monitor_data.acc_voltage_mv > 3000) {
        UART_SendString("Acceptable (Lower SOC)\r\n");
    }

    UART_SendString("Battery is suitable for test.\r\n\r\n");

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

    BatteryMonitorData_t monitor_data;
    BatteryMonitor_ReadAll(&monitor_data);

    BatteryState_Int_t bms_state;
    convert_monitor_to_bms_int(&monitor_data, &bms_state);

    BmsResult_Int_t bms_result;

    switch (rapid_test_ctx.state) {

        case RAPID_TEST_STARTING:
        {
            if (elapsed_in_state >= 5000) {
                rapid_test_ctx.state = RAPID_TEST_PRE_TX_CHECK;
                rapid_test_ctx.state_start_time_ms = current_time;
            }
            break;
        }

        case RAPID_TEST_PRE_TX_CHECK:
        {
            LogTransmissionStart(rapid_test_ctx.current_tx_number, &monitor_data);

            rapid_test_ctx.voltage_before_tx_mv = monitor_data.acc_voltage_mv;

            bms_transmission_decision_int(&bms_state, &bms_result);

            UART_SendString("\r\n--- PRE-TRANSMISSION BMS CHECK ---\r\n");
            print_bms_decision_int(&bms_result);

            if (bms_result.decision == DECISION_BLOCK) {
                rapid_test_ctx.tx_approved[rapid_test_ctx.current_tx_number - 1] = false;

                UART_SendString("\r\n!!! TRANSMISSION BLOCKED !!!\r\n");
                UART_SendString("Test cannot continue.\r\n");

                RapidTransmissionTest_Stop("Transmission blocked by BMS");
                return;
            }

            rapid_test_ctx.tx_approved[rapid_test_ctx.current_tx_number - 1] = true;

            if (bms_result.decision == DECISION_WARN) {
                UART_SendString("\r\n*** WARNING: Proceeding with caution ***\r\n\r\n");
            }

            UART_SendString("\r\n");
            UART_SendString(">>> APPLY 500mA LOAD NOW <<<\r\n");
            UART_SendString("Hold for 20 seconds...\r\n\r\n");

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
            else if (elapsed_in_state >= 2000 && elapsed_in_state < 2500) {
                rapid_test_ctx.voltage_during_tx_mv = monitor_data.acc_voltage_mv;

                uint16_t drop = (rapid_test_ctx.voltage_before_tx_mv > rapid_test_ctx.voltage_during_tx_mv) ?
                                (rapid_test_ctx.voltage_before_tx_mv - rapid_test_ctx.voltage_during_tx_mv) : 0;

                rapid_test_ctx.voltage_drop[rapid_test_ctx.current_tx_number - 1] = drop;

                bms_monitor_transmission_int(rapid_test_ctx.voltage_before_tx_mv,
                                            rapid_test_ctx.voltage_during_tx_mv,
                                            VOLTAGE_DROP_MAX_MV,
                                            &bms_result);

                UART_SendString("\r\n--- REAL-TIME TX MONITORING ---\r\n");
                UART_SendString("Voltage During: ");
                UART_SendNumber(rapid_test_ctx.voltage_during_tx_mv);
                UART_SendString(" mV\r\n");
                UART_SendString("Voltage Drop:   ");
                UART_SendNumber(drop);
                UART_SendString(" mV\r\n");
                UART_SendString("Decision:       ");

                switch(bms_result.decision) {
                    case DECISION_GO:    UART_SendString("CONTINUE\r\n"); break;
                    case DECISION_WARN:  UART_SendString("WARNING\r\n"); break;
                    case DECISION_ABORT: UART_SendString("ABORT\r\n"); break;
                    default:             UART_SendString("UNKNOWN\r\n"); break;
                }

                UART_SendString("Message:        ");
                UART_SendString(bms_result.message);
                UART_SendString("\r\n\r\n");

                if (bms_result.decision == DECISION_ABORT) {
                    UART_SendString("!!! ABORTING TRANSMISSION !!!\r\n");
                    UART_SendString(">>> REMOVE LOAD IMMEDIATELY <<<\r\n\r\n");

                    RapidTransmissionTest_Stop("Transmission aborted - excessive voltage drop");
                    return;
                }

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
            if (elapsed_in_state >= RECOVERY_TIME_MS - 1000 &&
                elapsed_in_state < RECOVERY_TIME_MS) {

                rapid_test_ctx.voltage_after_tx_mv = monitor_data.acc_voltage_mv;

                LogTransmissionResult(rapid_test_ctx.current_tx_number);
            }

            if (elapsed_in_state >= RECOVERY_TIME_MS) {

                if (rapid_test_ctx.current_tx_number < NUM_TRANSMISSIONS) {
                    rapid_test_ctx.current_tx_number++;
                    rapid_test_ctx.state = RAPID_TEST_PRE_TX_CHECK;
                    rapid_test_ctx.state_start_time_ms = current_time;

                    UART_SendString("Recovery complete. Preparing next transmission...\r\n");
                } else {
                    RapidTransmissionTest_Stop("All transmissions completed successfully");
                }
            } else {
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

        UART_SendString("  Approved:  ");
        UART_SendString(rapid_test_ctx.tx_approved[i] ? "YES" : "NO");
        UART_SendString("\r\n");

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

    bool all_passed = true;
    for (uint8_t i = 0; i < NUM_TRANSMISSIONS; i++) {
        if (!rapid_test_ctx.tx_approved[i] || !rapid_test_ctx.tx_completed[i]) {
            all_passed = false;
            break;
        }
    }

    if (all_passed) {
        UART_SendString(">>> TEST PASSED <<<\r\n");
        UART_SendString("All transmissions approved and completed\r\n");
        UART_SendString("BMS handled rapid successive transmissions correctly\r\n");
    } else {
        UART_SendString(">>> TEST FAILED OR INCOMPLETE <<<\r\n");
        UART_SendString("Not all transmissions completed successfully\r\n");
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
