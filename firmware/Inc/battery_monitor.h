/*
 * battery_monitor.h
 *
 *  Created on: Oct 10, 2025
 *      Author: rachaelguise-brown
 *      Description: Unified battery monitoring interface combining:
 *                   - BQ35100 ACC mode (I2C2)
 *                   - BQ35100 EOS mode (I2C1)
 *                   - TMP36 temperature sensor (ADC PA3)
 */

#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

/* Combined battery data structure */
typedef struct {
    /* ACC Mode Data (I2C2) */
    uint16_t acc_voltage_mv;           // Voltage in mV
    int16_t acc_current_ma;            // Current in mA (signed)
    int32_t acc_temp_hundredths_c;     // Temperature in 0.01°C
    int32_t acc_capacity_uah;          // Accumulated capacity in μAh
    uint8_t acc_battery_status;        // Battery status register
    uint8_t acc_battery_alert;         // Battery alert register
    bool acc_present;                  // Device detected

    /* EOS Mode Data (I2C1) */
    uint16_t eos_voltage_mv;           // Voltage in mV
    int16_t eos_current_ma;            // Current in mA
    int32_t eos_temp_hundredths_c;     // Temperature in 0.01°C
    uint16_t eos_soh_percent;          // State of Health %
    uint16_t eos_scaled_r_mohm;        // Scaled resistance in mΩ
    uint16_t eos_measured_z_mohm;      // Measured impedance in mΩ
    uint16_t eos_pulse_count;          // EOS pulse count
    uint32_t eos_short_trend;          // Short trend average
    uint32_t eos_long_trend;           // Long trend average
    bool eos_flag_set;                 // EOS flag status
    bool eos_present;                  // Device detected

    /* External Temperature Sensor Data (ADC) */
    uint16_t adc_raw;                  // Raw ADC value
    uint32_t adc_voltage_mv;           // ADC voltage in mV
    int32_t adc_temp_tenths_c;         // Temperature in 0.1°C
    bool adc_valid;                    // ADC reading valid

    /* System Status */
    uint32_t timestamp_ms;             // Timestamp of reading

} BatteryMonitorData_t;

/* Initialization */
void BatteryMonitor_Init(void);

/* Data Acquisition */
void BatteryMonitor_ReadAll(BatteryMonitorData_t *data);
void BatteryMonitor_ReadACC(BatteryMonitorData_t *data);
void BatteryMonitor_ReadEOS(BatteryMonitorData_t *data);
void BatteryMonitor_ReadADC(BatteryMonitorData_t *data);

/* Display Functions */
void BatteryMonitor_PrintSummary(const BatteryMonitorData_t *data);
void BatteryMonitor_PrintDetailed(const BatteryMonitorData_t *data);
void BatteryMonitor_PrintACCOnly(const BatteryMonitorData_t *data);
void BatteryMonitor_PrintEOSOnly(const BatteryMonitorData_t *data);
void BatteryMonitor_PrintTemperatures(const BatteryMonitorData_t *data);

/* Utility Functions */
bool BatteryMonitor_CheckDevices(void);
void BatteryMonitor_PerformEOSMeasurement(void);


#endif /* BATTERY_MONITOR_H_ */
