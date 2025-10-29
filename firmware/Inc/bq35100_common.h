/*
 * bq35100_common.h
 *
 *  Created on: Oct 2, 2025
 *      Author: rachaelguise-brown
 *      Description: Common definitions for BQ35100 Fuel Gauge
 *  	This file contains all register addresses, command definitions,
 *  	and utility functions shared between different I2C implementations
 */

#ifndef BQ35100_COMMON_H_
#define BQ35100_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

/* ==================== STM32L4 Common Register Definitions ==================== */
/* For GPIOF which is used for the Gauge Enable pins for both*/

/* Base addresses */
#define RCC_BASE            0x40021000UL
#define GPIOF_BASE          0x48001400UL

/* Register access macro */
#define REG32(base, offset) (*(volatile uint32_t*)((base) + (offset)))

/* RCC Register definitions */
#define RCC_AHB2ENR         REG32(RCC_BASE, 0x4C)

/* RCC Enable Bits */
#define RCC_AHB2ENR_GPIOFEN     (1 << 5)    // GPIOF clock enable

/* GPIOF Register definitions (shared by both ACC and EOS) */
#define GPIOF_MODER         REG32(GPIOF_BASE, 0x00)
#define GPIOF_OTYPER        REG32(GPIOF_BASE, 0x04)
#define GPIOF_OSPEEDR       REG32(GPIOF_BASE, 0x08)
#define GPIOF_PUPDR         REG32(GPIOF_BASE, 0x0C)
#define GPIOF_IDR           REG32(GPIOF_BASE, 0x10)
#define GPIOF_ODR           REG32(GPIOF_BASE, 0x14)
#define GPIOF_BSRR          REG32(GPIOF_BASE, 0x18)  // Bit Set/Reset Register
#define GPIOF_AFRL          REG32(GPIOF_BASE, 0x20)  // Alternate Function Low
#define GPIOF_AFRH          REG32(GPIOF_BASE, 0x24)  // Alternate Function High

/* GPIO alternate function for I2C2 */
#define GPIO_AF4            4

/* ==================== BQ35100 Device Definitions ==================== */

/* BQ35100 I2C Address */
#define BQ35100_I2C_ADDRESS             0x55  // 7-bit address

/* ==================== BQ35100 Register Addresses ==================== */

/* Standard Commands (from TRM Chapter 11) */
#define BQ35100_CONTROL_CMD             0x00  // Control() command - 2 bytes
#define BQ35100_ACCUM_CAPACITY          0x02  // AccumulatedCapacity() - 4 bytes
#define BQ35100_TEMPERATURE             0x06  // Temperature() - 2 bytes
#define BQ35100_VOLTAGE                 0x08  // Voltage() - 2 bytes
#define BQ35100_BATTERY_STATUS          0x0A  // BatteryStatus() - 1 byte
#define BQ35100_BATTERY_ALERT           0x0B  // BatteryAlert() - 1 byte
#define BQ35100_CURRENT                 0x0C  // Current() - 2 bytes (signed)
#define BQ35100_SCALED_R                0x16  // ScaledR() - 2 bytes (EOS mode)
#define BQ35100_MEASURED_Z              0x22  // MeasuredZ() - 2 bytes (EOS mode)
#define BQ35100_INTERNAL_TEMP           0x28  // InternalTemperature() - 2 bytes
#define BQ35100_STATE_OF_HEALTH         0x2E  // StateOfHealth() - 2 bytes

/* Data Flash Access Registers */
#define BQ35100_MAC_CONTROL             0x3E  // ManufacturerAccessControl - 2 bytes
#define BQ35100_MAC_DATA                0x40  // MACData - 32 bytes available
#define BQ35100_MAC_DATA_SUM            0x60  // MACDataSum - checksum
#define BQ35100_MAC_DATA_LEN            0x61  // MACDataLen - data length

/* ==================== Control Subcommands ==================== */

#define BQ35100_CTRL_CONTROL_STATUS     0x0000  // Reports status of key features
#define BQ35100_CTRL_DEVICE_TYPE        0x0001  // Device type (0x0100 for BQ35100)
#define BQ35100_CTRL_FW_VERSION         0x0002  // Firmware version
#define BQ35100_CTRL_HW_VERSION         0x0003  // Hardware version
#define BQ35100_CTRL_CHEM_ID            0x0006  // Chemical ID
#define BQ35100_CTRL_BOARD_OFFSET       0x0009  // Board offset calibration
#define BQ35100_CTRL_CC_OFFSET          0x000A  // CC offset calibration
#define BQ35100_CTRL_CC_OFFSET_SAVE     0x000B  // Save CC offset
#define BQ35100_CTRL_GAUGE_START        0x0011  // Start gauging/measurements
#define BQ35100_CTRL_GAUGE_STOP         0x0012  // Stop gauging
#define BQ35100_CTRL_SEALED             0x0020  // Enter sealed mode
#define BQ35100_CTRL_CAL_ENABLE         0x002D  // Calibration enable
#define BQ35100_CTRL_LT_ENABLE          0x002E  // Lifetime data enable
#define BQ35100_CTRL_RESET              0x0041  // Reset device
#define BQ35100_CTRL_EXIT_CAL           0x0080  // Exit calibration
#define BQ35100_CTRL_ENTER_CAL          0x0081  // Enter calibration
#define BQ35100_CTRL_NEW_BATTERY        0xA613  // New battery (reset EOS)
#define BQ35100_CTRL_ACC_RESET 			0x0042 	// Reset ACC mode

/* ==================== Data Flash Addresses ==================== */

/* Operation Config A register (Data Flash 0x41B1) */
#define BQ35100_OP_CONFIG_A_ADDR        0x41B1

/* Operation Config A Bit Definitions */
#define BQ35100_OP_CONFIG_TEMPS         (1 << 7)  // Bit 7: External temp sensor
#define BQ35100_OP_CONFIG_EXTVCELL      (1 << 6)  // Bit 6: External voltage
#define BQ35100_OP_CONFIG_WRTEMP        (1 << 5)  // Bit 5: Write temp enable
#define BQ35100_OP_CONFIG_LF_EN         (1 << 4)  // Bit 4: Lifetime data
#define BQ35100_OP_CONFIG_GNDSEL        (1 << 2)  // Bit 2: GND selection

/* Gauging Mode Selection (Bits 1:0) */
#define BQ35100_OP_CONFIG_GMSEL_ACC     0x00  // 00 = Accumulator mode
#define BQ35100_OP_CONFIG_GMSEL_SOH     0x01  // 01 = SOH mode (LiMnO2)
#define BQ35100_OP_CONFIG_GMSEL_EOS     0x02  // 10 = EOS mode (LiSOCl2)
#define BQ35100_OP_CONFIG_GMSEL_MASK    0x03  // Mask for mode bits

/* ==================== Battery Status Bits ==================== */

#define BQ35100_BSTATUS_DSG             (1 << 0)  // Discharge detected
#define BQ35100_BSTATUS_ALERT           (1 << 2)  // Alert active

/* ==================== Battery Alert Flags ==================== */

#define BQ35100_ALERT_INITCOMP          (1 << 0)  // Initialization complete
#define BQ35100_ALERT_G_DONE            (1 << 1)  // Gauge calculations done
#define BQ35100_ALERT_EOS               (1 << 3)  // End of service (EOS mode)
#define BQ35100_ALERT_SOH_LOW           (1 << 4)  // State of health low
#define BQ35100_ALERT_TEMPHIGH          (1 << 5)  // Temperature too high
#define BQ35100_ALERT_TEMPLOW           (1 << 6)  // Temperature too low
#define BQ35100_ALERT_BATLOW            (1 << 7)  // Battery voltage low

/* ==================== Temperature Conversion ==================== */

#define BQ35100_TEMP_KELVIN_OFFSET      2731.5f   // 0°C = 273.15K, in 0.1K = 2731.5
#define BQ35100_TEMP_KELVIN_OFFSET_INT  27315     // For integer math (hundredths)

/* ==================== Common Utility Functions ==================== */

/**
 * @brief  Convert raw temperature to Celsius (integer version)
 * @param  raw_temp: Raw temperature in 0.1°K units
 * @retval Temperature in hundredths of °C (e.g., 2534 = 25.34°C)
 */
static inline int32_t BQ35100_ConvertToCelsius_Int(uint16_t raw_temp)
{
    // BQ35100 returns temperature in 0.1°K units
    // Convert to hundredths of °C for precision without float
    // Formula: °C = (raw * 0.1) - 273.15
    // In hundredths: (°C * 100) = (raw * 10) - 27315
    return ((int32_t)raw_temp * 10) - BQ35100_TEMP_KELVIN_OFFSET_INT;
}

/**
 * @brief  Convert raw voltage to millivolts
 * @param  raw_voltage: Raw voltage reading
 * @retval Voltage in mV
 */
static inline uint16_t BQ35100_ConvertToMillivolts(uint16_t raw_voltage)
{
    // BQ35100 voltage is already in mV
    return raw_voltage;
}

/**
 * @brief  Convert raw current to milliamps
 * @param  raw_current: Raw current reading (signed)
 * @retval Current in mA
 */
static inline int16_t BQ35100_ConvertToMilliamps(int16_t raw_current)
{
    // BQ35100 current is already in mA
    return raw_current;
}


#endif /* BQ35100_COMMON_H_ */
