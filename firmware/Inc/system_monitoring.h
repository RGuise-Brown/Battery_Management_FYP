/*
 * system_monitoring.h
 *
 *  Created on: Oct 16, 2025
 *      Author: rachaelguise-brown
 *
 *  Description:
 *  Minimal system monitoring — reads temperature (ADC)
 *  and power/current from the INA226 sensor.
 */

#ifndef SYSTEM_MONITORING_H_
#define SYSTEM_MONITORING_H_

#include <stdint.h>
#include <stdbool.h>

/* ==================== Data Structure ==================== */
typedef struct
{
    /* Temperature Sensor Data */
    uint16_t adc_raw;               // Raw ADC value
    uint32_t adc_voltage_mv;        // Converted ADC voltage
    int32_t  adc_temp_tenths_c;     // Temperature in 0.1°C
    bool     adc_valid;             // Valid ADC reading

    /* INA226 Data */
    bool     ina226_present;        // INA226 detected
    int32_t  ina226_bus_mv;         // Bus voltage (mV)
    int32_t  ina226_shunt_mv;       // Shunt voltage (mV)
    int32_t  ina226_current_ma;     // Current (mA)
    uint32_t ina226_power_mw;       // Power (mW)

    /* Computed data */
    int64_t  accumulated_uAh;       // Integrated capacity (µAh)
    uint32_t timestamp_ms;          // Time of last reading (ms)

} SystemMonitorData_t;

/* ==================== Function Prototypes ==================== */

/* Initialization */
void SystemMonitor_Init(void);

/* Data Acquisition */
void SystemMonitor_ReadAll(SystemMonitorData_t *data);
void SystemMonitor_ReadTemp(SystemMonitorData_t *data);
void SystemMonitor_ReadINA226(SystemMonitorData_t *data);

/* Display */
void SystemMonitor_PrintSummary(const SystemMonitorData_t *data);

/* Utility */
void SystemMonitor_ResetCapacity(void);

#endif /* SYSTEM_MONITORING_H_ */
