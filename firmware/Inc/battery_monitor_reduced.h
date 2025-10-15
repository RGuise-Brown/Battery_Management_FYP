/*
 * battery_monitor_reduced.h
 *
 *  Created on: Oct 15, 2025
 *      Author: rachaelguise-brown
 */

#ifndef BATTERY_MONITOR_REDUCED_H_
#define BATTERY_MONITOR_REDUCED_H_

#include <stdint.h>
#include <stdbool.h>

/* Combined battery data structure */
typedef struct {
    /* ACC Mode Data (I2C2) */
    uint16_t acc_voltage_mv;           // Voltage in mV
    int16_t acc_current_ma;            // Current in mA (signed)
    int32_t acc_temp_hundredths_c;     // Temperature in 0.01°C
    uint8_t acc_battery_status;        // Battery status register
    uint8_t acc_battery_alert;         // Battery alert register
    bool acc_present;                  // Device detected

    /* External Temperature Sensor Data (ADC) */
    uint16_t adc_raw;                  // Raw ADC value
    uint32_t adc_voltage_mv;           // ADC voltage in mV
    int32_t adc_temp_tenths_c;         // Temperature in 0.1°C
    bool adc_valid;                    // ADC reading valid

    /* System Status */
    uint32_t timestamp_ms;             // Timestamp of reading

} BatteryMonitorData_t;

/* Initialization */
void BatteryMonitor_2_Init(void);

/* Data Acquisition */
void BatteryMonitor_2_ReadAll(BatteryMonitorData_t *data);
void BatteryMonitor_2_ReadACC(BatteryMonitorData_t *data);
void BatteryMonitor_2_ReadADC(BatteryMonitorData_t *data);

/* Display Functions */
void BatteryMonitor_2_PrintSummary(const BatteryMonitorData_t *data);
void BatteryMonitor_2_PrintDetailed(const BatteryMonitorData_t *data);
void BatteryMonitor_2_PrintACCOnly(const BatteryMonitorData_t *data);
void BatteryMonitor_2_PrintTemperatures(const BatteryMonitorData_t *data);

/* Utility Functions */
bool BatteryMonitor_2_CheckDevices(void);

#endif /* BATTERY_MONITOR_REDUCED_H_ */
