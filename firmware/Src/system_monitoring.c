/*
 * system_monitoring.c
 *
 *  Created on: Oct 16, 2025
 *      Author: rachaelguise-brown
 *
 *  Description:
 *  Simple system monitoring: temperature + INA226 readings
 */

#include "system_monitoring.h"
#include "uart.h"
#include "ina226_i2c1.h"
#include "adc_temp.h"
#include "temp_sensor_enable.h"
#include "systick_timing.h"

/* ==================== Static Integration Variables ==================== */
static int64_t accumulated_uAh = 0;
static uint32_t last_integration_time_ms = 0;

/* ==================== Local Helper ==================== */
static void integrate_current_sample(int32_t current_mA, uint32_t dt_ms)
{
    // Convert mA × ms to µAh
    int64_t delta_uAh = ((int64_t)current_mA * (int64_t)dt_ms) / 3600;
    accumulated_uAh += delta_uAh;
}

/* ==================== Initialization ==================== */
void SystemMonitor_Init(void)
{
    UART_SendString("\r\n========================================\r\n");
    UART_SendString("      System Monitoring Init            \r\n");
    UART_SendString("========================================\r\n");

    UART_SendString("[1/2] Initializing Temperature Sensor...\r\n");
    TempSensor_Shutdown_Init();
    TempSensor_Enable();
    ADC_Init();
    TempSensor_Disable();

    UART_SendString("[2/2] Initializing INA226 (I2C1)...\r\n");
    INA226_I2C1_Init();

    UART_SendString("Initialization complete.\r\n\r\n");
}

/* ==================== Data Acquisition ==================== */

void SystemMonitor_ReadTemp(SystemMonitorData_t *data)
{
    TempSensor_Enable();

    // Small delay to stabilize sensor
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

void SystemMonitor_ReadINA226(SystemMonitorData_t *data)
{
    int32_t bus_mV, shunt_mV, current_mA;
    uint32_t power_mW;

    bool ok = INA226_ReadBusVoltage(&bus_mV);
    if (!ok)
    {
        data->ina226_present = false;
        UART_SendString("INA226 not responding\r\n");
        return;
    }

    data->ina226_present = true;

    INA226_ReadShuntVoltage(&shunt_mV);
    INA226_ReadCurrent_mA(&current_mA);
    INA226_ReadPower_mW(&power_mW);

    data->ina226_bus_mv   = bus_mV;
    data->ina226_shunt_mv = shunt_mV;
    data->ina226_current_ma = current_mA;
    data->ina226_power_mw   = power_mW;

    /* Integration */
    uint32_t now_ms = GetTick_ms();
    uint32_t dt_ms = now_ms - last_integration_time_ms;

    if (last_integration_time_ms != 0)
    {
        integrate_current_sample(current_mA, dt_ms);
    }

    last_integration_time_ms = now_ms;
    data->accumulated_uAh = accumulated_uAh;
}

/* High-level read function */
void SystemMonitor_ReadAll(SystemMonitorData_t *data)
{
    for (uint8_t *p = (uint8_t *)data; p < (uint8_t *)data + sizeof(SystemMonitorData_t); p++)
        *p = 0;

    SystemMonitor_ReadINA226(data);
    SystemMonitor_ReadTemp(data);
    data->timestamp_ms = GetTick_ms();
}

/* ==================== Display ==================== */
void SystemMonitor_PrintSummary(const SystemMonitorData_t *data)
{
    UART_SendString("\r\n========================================\r\n");
    UART_SendString("          SYSTEM MONITOR SUMMARY        \r\n");
    UART_SendString("========================================\r\n");

    if (data->ina226_present)
    {
        UART_SendString("+--- INA226 (I2C1) -------------------+\r\n");
        UART_SendString("| Bus Voltage:  ");
        UART_SendNumber(data->ina226_bus_mv);
        UART_SendString(" mV\r\n");

        UART_SendString("| Shunt Volt:   ");
        UART_SendNumber(data->ina226_shunt_mv);
        UART_SendString(" mV\r\n");

        UART_SendString("| Current:      ");
        UART_SendNumber(data->ina226_current_ma);
        UART_SendString(" mA\r\n");

        UART_SendString("| Power:        ");
        UART_SendNumber(data->ina226_power_mw);
        UART_SendString(" mW\r\n");

        UART_SendString("| Capacity:     ");
        UART_SendNumber((int32_t)(data->accumulated_uAh / 1000));
        UART_SendString(" mAh (");
        UART_SendNumber(data->accumulated_uAh);
        UART_SendString(" uAh)\r\n");
        UART_SendString("+------------------------------------+\r\n\r\n");
    }
    else
    {
        UART_SendString("+--- INA226 (I2C1) -------------------+\r\n");
        UART_SendString("| X NOT DETECTED                     |\r\n");
        UART_SendString("+------------------------------------+\r\n\r\n");
    }

    if (data->adc_valid)
    {
        UART_SendString("+--- TEMPERATURE (TMP36) ------------+\r\n");
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
        UART_SendString("+------------------------------------+\r\n");
    }
    else
    {
        UART_SendString("+--- TEMPERATURE (TMP36) ------------+\r\n");
        UART_SendString("| X INVALID READING                  |\r\n");
        UART_SendString("+------------------------------------+\r\n");
    }
}

/* ==================== Utility ==================== */
void SystemMonitor_ResetCapacity(void)
{
    accumulated_uAh = 0;
    last_integration_time_ms = 0;
}
