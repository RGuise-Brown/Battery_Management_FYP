/*
 * battery_monitor_reduced.c
 *
 *  Created on: Oct 15, 2025
 *      Author: rachaelguise-brown
 *      Description: Unified battery monitoring implementation
 */

#include "battery_monitor_reduced.h"
#include "bq35100_i2c_acc.h"
#include "adc_temp.h"
#include "temp_sensor_enable.h"
#include "uart.h"

/* Private variables */
static uint32_t sys_tick_ms = 0;

/* ==================== Initialization ==================== */

void BatteryMonitor_2_Init(void)
{
    UART_SendString("\r\n");
    UART_SendString("========================================\r\n");
    UART_SendString("   Battery Monitoring System Init      \r\n");
    UART_SendString("========================================\r\n");

    // Initialize ACC mode gauge (I2C2)
    UART_SendString("\n[1/2] Initializing ACC mode (I2C2)...\r\n");
    BQ35100_I2C2_Init();

    // Initialize temperature sensor and ADC
    UART_SendString("\n[2/2] Initializing temperature sensor...\r\n");
    TempSensor_Shutdown_Init();
    TempSensor_Enable();
    ADC_Init();
    TempSensor_Disable();

    UART_SendString("\n========================================\r\n");
    UART_SendString("   Initialization Complete!            \r\n");
    UART_SendString("========================================\r\n\r\n");
}

/* ==================== Data Acquisition ==================== */

void BatteryMonitor_2_ReadACC(BatteryMonitorData_t *data)
{
    if (!BQ35100_I2C2_IsPresent())
    {
        data->acc_present = false;
        return;
    }

    data->acc_present = true;
    data->acc_voltage_mv = BQ35100_I2C2_ReadVoltage();
    data->acc_current_ma = BQ35100_I2C2_ReadCurrent();
    data->acc_temp_hundredths_c = BQ35100_I2C2_ReadTemperature_Int();
    data->acc_battery_status = BQ35100_I2C2_ReadBatteryStatus();
    data->acc_battery_alert = BQ35100_I2C2_ReadBatteryAlert();
}

void BatteryMonitor_2_ReadADC(BatteryMonitorData_t *data)
{
    TempSensor_Enable();

    // wait to stabalise
    for (volatile int i = 0; i < 1000; i++);

	data->adc_raw = ADC_Read_Raw();

    if (data->adc_raw > 0)
    {
        data->adc_valid = true;
        data->adc_voltage_mv = ADC_ConvertTo_mV(data->adc_raw);
        data->adc_temp_tenths_c = ADC_ConvertTo_TempC_Tenths(data->adc_raw);
    }
    else
    {
        data->adc_valid = false;
    }

    TempSensor_Disable();
}

void BatteryMonitor_2_ReadAll(BatteryMonitorData_t *data)
{
    // Clear structure
    for (uint8_t *p = (uint8_t*)data; p < (uint8_t*)data + sizeof(BatteryMonitorData_t); p++)
    {
        *p = 0;
    }

    // Read all subsystems
    BatteryMonitor_2_ReadACC(data);
    BatteryMonitor_2_ReadADC(data);

    // Set timestamp
    data->timestamp_ms = sys_tick_ms++;
}

/* ==================== Display Functions ==================== */

void BatteryMonitor_2_PrintSummary(const BatteryMonitorData_t *data)
{
    UART_SendString("\r\n");
    UART_SendString("========================================\r\n");
    UART_SendString("   BATTERY MONITORING SUMMARY          \r\n");
    UART_SendString("========================================\r\n\r\n");

    /* ACC Mode Summary */
    if (data->acc_present)
    {
        UART_SendString("+--- ACC MODE (I2C2) ----------------+\r\n");
        UART_SendString("| Voltage:    ");
        UART_SendNumber(data->acc_voltage_mv);
        UART_SendString(" mV\r\n");

        UART_SendString("| Current:    ");
        if (data->acc_current_ma < 0)
        {
            UART_SendString("-");
            UART_SendNumber(-data->acc_current_ma);
        }
        else
        {
            UART_SendNumber(data->acc_current_ma);
        }
        UART_SendString(" mA\r\n");

        UART_SendString("| Temp (BQ):  ");
        int32_t whole = data->acc_temp_hundredths_c / 100;
        int32_t decimal = data->acc_temp_hundredths_c % 100;
        if (decimal < 0) decimal = -decimal;
        UART_SendNumber(whole);
        UART_SendString(".");
        if (decimal < 10) UART_SendString("0");
        UART_SendNumber(decimal);
        UART_SendString(" C\r\n");
        UART_SendString("+-----------------------------------+\r\n\r\n");
    }
    else
    {
        UART_SendString("+--- ACC MODE (I2C2) ----------------+\r\n");
        UART_SendString("| X NOT DETECTED                     |\r\n");
        UART_SendString("+-----------------------------------+\r\n\r\n");
    }


    /* Temperature Sensor Summary */
    if (data->adc_valid)
    {
        UART_SendString("+--- EXTERNAL TEMP (TMP36) ----------+\r\n");
        UART_SendString("| Voltage:    ");
        UART_SendNumber(data->adc_voltage_mv);
        UART_SendString(" mV\r\n");

        UART_SendString("| Temp:       ");
        int32_t whole = data->adc_temp_tenths_c / 10;
        int32_t decimal = data->adc_temp_tenths_c % 10;
        if (decimal < 0) decimal = -decimal;
        UART_SendNumber(whole);
        UART_SendString(".");
        UART_SendNumber(decimal);
        UART_SendString(" C\r\n");
        UART_SendString("+-----------------------------------+\r\n\r\n");
    }
    else
    {
        UART_SendString("+--- EXTERNAL TEMP (TMP36) ----------+\r\n");
        UART_SendString("| X INVALID READING                  |\r\n");
        UART_SendString("+-----------------------------------+\r\n\r\n");
    }
}

void BatteryMonitor_2_PrintDetailed(const BatteryMonitorData_t *data)
{
    UART_SendString("\r\n");
    UART_SendString("========================================\r\n");
    UART_SendString("   DETAILED BATTERY MONITORING          \r\n");
    UART_SendString("========================================\r\n\r\n");

    /* ACC Mode Detailed */
    if (data->acc_present)
    {
        UART_SendString("--- ACC MODE (I2C2) --------------------\r\n");
        UART_SendString("Voltage:             ");
        UART_SendNumber(data->acc_voltage_mv);
        UART_SendString(" mV\r\n");

        UART_SendString("Current:             ");
        if (data->acc_current_ma < 0)
        {
            UART_SendString("-");
            UART_SendNumber(-data->acc_current_ma);
        }
        else
        {
            UART_SendNumber(data->acc_current_ma);
        }
        UART_SendString(" mA\r\n");

        UART_SendString("Temperature:         ");
        int32_t whole = data->acc_temp_hundredths_c / 100;
        int32_t decimal = data->acc_temp_hundredths_c % 100;
        if (decimal < 0) decimal = -decimal;
        UART_SendNumber(whole);
        UART_SendString(".");
        if (decimal < 10) UART_SendString("0");
        UART_SendNumber(decimal);
        UART_SendString(" C\r\n");

        UART_SendString("Battery Status:      0x");
        UART_SendHex(data->acc_battery_status);
        UART_SendString("\r\n");

        UART_SendString("Battery Alert:       0x");
        UART_SendHex(data->acc_battery_alert);
        UART_SendString("\r\n\r\n");
    }
    else
    {
        UART_SendString("--- ACC MODE (I2C2) --------------------\r\n");
        UART_SendString("X DEVICE NOT DETECTED\r\n\r\n");
    }


    /* Temperature Sensor Detailed */
    if (data->adc_valid)
    {
        UART_SendString("--- TEMPERATURE SENSOR (TMP36) ---------\r\n");
        UART_SendString("ADC Raw Value:       ");
        UART_SendNumber(data->adc_raw);
        UART_SendString(" / 4095\r\n");

        UART_SendString("ADC Voltage:         ");
        UART_SendNumber(data->adc_voltage_mv);
        UART_SendString(" mV\r\n");

        UART_SendString("Temperature:         ");
        int32_t whole = data->adc_temp_tenths_c / 10;
        int32_t decimal = data->adc_temp_tenths_c % 10;
        if (decimal < 0) decimal = -decimal;
        UART_SendNumber(whole);
        UART_SendString(".");
        UART_SendNumber(decimal);
        UART_SendString(" C\r\n\r\n");
    }
    else
    {
        UART_SendString("--- TEMPERATURE SENSOR (TMP36) ---------\r\n");
        UART_SendString("X INVALID READING\r\n\r\n");
    }

    UART_SendString("========================================\r\n\r\n");
}

void BatteryMonitor_2_PrintTemperatures(const BatteryMonitorData_t *data)
{
    UART_SendString("\r\n=== TEMPERATURE COMPARISON ===\r\n\r\n");

    if (data->acc_present)
    {
        UART_SendString("ACC BQ35100 Temp:    ");
        int32_t whole = data->acc_temp_hundredths_c / 100;
        int32_t decimal = data->acc_temp_hundredths_c % 100;
        if (decimal < 0) decimal = -decimal;
        UART_SendNumber(whole);
        UART_SendString(".");
        if (decimal < 10) UART_SendString("0");
        UART_SendNumber(decimal);
        UART_SendString(" C\r\n");
    }

    if (data->adc_valid)
    {
        UART_SendString("TMP36 Sensor Temp:   ");
        int32_t whole = data->adc_temp_tenths_c / 10;
        int32_t decimal = data->adc_temp_tenths_c % 10;
        if (decimal < 0) decimal = -decimal;
        UART_SendNumber(whole);
        UART_SendString(".");
        UART_SendNumber(decimal);
        UART_SendString(" C\r\n");
    }

    UART_SendString("\r\n");
}

/* ==================== Utility Functions ==================== */

bool BatteryMonitor_2_CheckDevices(void)
{
    bool all_present = true;

    UART_SendString("\r\n=== DEVICE PRESENCE CHECK ===\r\n");

    UART_SendString("ACC Mode (I2C2):     ");
    if (BQ35100_I2C2_IsPresent())
    {
        UART_SendString("Detected\r\n");
    }
    else
    {
        UART_SendString("Not found\r\n");
        all_present = false;
    }

    UART_SendString("TMP36 Sensor:        ");
    uint16_t adc_val = ADC_Read_Raw();
    if (adc_val > 0 && adc_val < 4095)
    {
        UART_SendString("Valid reading\r\n");
    }
    else
    {
        UART_SendString("Invalid reading\r\n");
        all_present = false;
    }

    UART_SendString("\r\n");
    return all_present;
}

