/*
 * bq35100_i2c_eos.h
 *
 *  Created on: Oct 2, 2025
 *      Author: rachaelguise-brown
 *      Description: BQ35100 Fuel Gauge driver using I2C2 in End-Of-Service (EOS) Mode
 *  	I2C1 Pins: PB0 = SDA, PB8 = SCL
 *  	Mode: End-Of_Service for for Li/SOCl2 battery
 */

#ifndef BQ35100_I2C_EOS_H_
#define BQ35100_I2C_EOS_H_

#include "bq35100_common.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== STM32L4 I2C1 Register Definitions ==================== */

// Base addresses (reusing from TMP117 implementation)
#define RCC_BASE            0x40021000UL
#define GPIOB_BASE          0x48000400UL
#define I2C1_BASE           0x40005400UL

// Register access macro
#define REG32(base, offset) (*(volatile uint32_t*)((base) + (offset)))

// RCC Register definitions
#define RCC_AHB2ENR         REG32(RCC_BASE, 0x4C)
#define RCC_APB1ENR1        REG32(RCC_BASE, 0x58)
#define RCC_CCIPR           REG32(RCC_BASE, 0x88)

// GPIO Register definitions for GPIOB
#define GPIOB_MODER         REG32(GPIOB_BASE, 0x00)
#define GPIOB_OTYPER        REG32(GPIOB_BASE, 0x04)
#define GPIOB_OSPEEDR       REG32(GPIOB_BASE, 0x08)
#define GPIOB_PUPDR         REG32(GPIOB_BASE, 0x0C)
#define GPIOB_AFRH          REG32(GPIOB_BASE, 0x24)

// I2C Register definitions
#define I2C1_CR1            REG32(I2C1_BASE, 0x00)
#define I2C1_CR2            REG32(I2C1_BASE, 0x04)
#define I2C1_TIMINGR        REG32(I2C1_BASE, 0x10)
#define I2C1_ISR            REG32(I2C1_BASE, 0x18)
#define I2C1_ICR            REG32(I2C1_BASE, 0x1C)
#define I2C1_TXDR           REG32(I2C1_BASE, 0x28)
#define I2C1_RXDR           REG32(I2C1_BASE, 0x24)

// GPIO alternate function for I2C1
#define GPIO_AF4            4

// I2C bit definitions
#define I2C_CR1_PE          (1UL << 0)
#define I2C_ISR_BUSY        (1UL << 15)
#define I2C_ISR_TXE         (1UL << 0)
#define I2C_ISR_TC          (1UL << 6)
#define I2C_ISR_NACKF       (1UL << 4)
#define I2C_ISR_TXIS        (1UL << 1)
#define I2C_ISR_STOPF       (1UL << 5)
#define I2C_ISR_RXNE        (1UL << 2)

// I2C CR2 bit definitions
#define I2C_CR2_START       (1UL << 13)
#define I2C_CR2_STOP        (1UL << 14)
#define I2C_CR2_AUTOEND     (1UL << 25)
#define I2C_CR2_RD_WRN      (1UL << 10)

// I2C ICR bit definitions
#define I2C_ICR_NACKCF      (1UL << 4)
#define I2C_ICR_STOPCF      (1UL << 5)

// Register addresses (add to existing defines)
#define BQ35100_BATTERY_STATUS      0x0A

// Data Flash addresses for EOS monitoring
#define BQ35100_OP_CONFIG_A_ADDR    0x41B1
#define BQ35100_EOS_PULSE_COUNT     0x419E  // 2 bytes
#define BQ35100_SHORT_TREND_AVG     0x41A6  // 4 bytes
#define BQ35100_LONG_TREND_AVG      0x41AA  // 4 bytes


/* ==================== Function Prototypes ==================== */

// Initialization and Configuration
void BQ35100_I2C1_Init(void);
bool BQ35100_I2C1_IsPresent(void);
bool BQ35100_I2C1_ScanAddress(uint8_t address);

// Configuration Functions
void BQ35100_I2C1_ConfigureEOSMode(void);

// Sealing/Unsealing Functions
bool BQ35100_I2C1_Unseal(void);
bool BQ35100_I2C1_IsSealed(void);

// Gauge Enable Pin Control (PF5)
void BQ35100_EOS_GaugePin_Enable(void);
void BQ35100_EOS_GaugePin_Disable(void);

// Low-level I2C functions
bool BQ35100_I2C1_WriteCommand(uint8_t cmd, uint16_t data);
uint16_t BQ35100_I2C1_ReadWord(uint8_t cmd);

// Control Functions
uint16_t BQ35100_I2C1_ReadControlStatus(void);
uint16_t BQ35100_I2C1_ReadDeviceType(void);
void BQ35100_I2C1_GaugeStart(void);
void BQ35100_I2C1_GaugeStop(void);

// Temperature Reading Functions
uint16_t BQ35100_I2C1_ReadRawTemp(void);
int32_t BQ35100_I2C1_ReadTemperature_Int(void);

// Voltage and Current Reading Functions
uint16_t BQ35100_I2C1_ReadVoltage(void);
int16_t BQ35100_I2C1_ReadCurrent(void);

// EOS Mode Specific Functions
uint16_t BQ35100_I2C1_ReadScaledR(void);        // Battery resistance
uint16_t BQ35100_I2C1_ReadMeasuredZ(void);      // Battery impedance
uint16_t BQ35100_I2C1_ReadStateOfHealth(void);  // State of health percentage
uint8_t BQ35100_I2C1_ReadBatteryAlert(void);    // Alert flags
bool BQ35100_I2C1_IsEOSDetected(void);          // Check if EOS flag set
bool BQ35100_I2C1_IsGaugeDone(void);            // Check if calculations complete
void BQ35100_NewBattery(void);             // Reset for new battery installation


// Configuration Functions
bool BQ35100_I2C1_EnableExternalThermistor(void);
uint8_t BQ35100_I2C1_ReadOperationConfigA(void);
bool BQ35100_I2C1_WriteOperationConfigA(uint8_t config);
bool BQ35100_I2C1_WriteDataFlash(uint16_t address, uint8_t* data, uint8_t length);
uint8_t BQ35100_I2C1_ReadDataFlashByte(uint16_t address);
uint16_t BQ35100_I2C1_ReadInternalTemp(void);  // Read internal temp for comparison


// Temperature Compensation Functions
int16_t BQ35100_I2C1_ReadRTableScale(void);
uint16_t BQ35100_I2C1_ReadChemID(void);
void BQ35100_I2C1_CheckTemperatureCompensation(void);


// Test and Diagnostic Functions
void BQ35100_I2C1_Test(void);
void BQ35100_I2C1_ScanDevice(void);
void BQ35100_I2C1_DiagnosticTest(void);  // Compare internal vs external temp
void BQ35100_I2C1_HardwareTest(void);    // Check hardware connections
void BQ35100_I2C1_EOSTest(void);         // Complete EOS mode test
void BQ35100_I2C1_PrintEOSStatus(void);  // Print all EOS-related values

#endif /* BQ35100_I2C_EOS_H_ */
