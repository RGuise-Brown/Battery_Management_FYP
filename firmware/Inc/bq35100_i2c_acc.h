/*
 * bq35100_i2c_acc.h
 *
 *  Created on: Oct 2, 2025
 *      Author: rachaelguise-brown
 *      Description: BQ35100 Fuel Gauge driver using I2C2 in Accumulator Mode
 *  	I2C2 Pins: PF0 = SDA, PF1 = SCL, PF2 = SMBA
 *  	Mode: ACCUMULATOR for capacity tracking
 */

#ifndef BQ35100_I2C_ACC_H_
#define BQ35100_I2C_ACC_H_

#include "bq35100_common.h"

/* ==================== STM32L4 I2C2 Register Definitions ==================== */

/* Base addresses */
#define RCC_BASE            0x40021000UL
#define GPIOF_BASE          0x48001400UL
#define I2C2_BASE           0x40005800UL

/* Register access macro */
#define REG32(base, offset) (*(volatile uint32_t*)((base) + (offset)))

/* RCC Register definitions */
#define RCC_AHB2ENR         REG32(RCC_BASE, 0x4C)
#define RCC_APB1ENR1        REG32(RCC_BASE, 0x58)
#define RCC_CCIPR           REG32(RCC_BASE, 0x88)

/* RCC Enable Bits */
#define RCC_APB1ENR1_I2C2EN     (1 << 22)   // I2C2 clock enable

/* GPIO Register definitions for GPIOF */
// In common file as used for the Gauge Enable Pin

/* I2C2 Register definitions */
#define I2C2_CR1            REG32(I2C2_BASE, 0x00)
#define I2C2_CR2            REG32(I2C2_BASE, 0x04)
#define I2C2_TIMINGR        REG32(I2C2_BASE, 0x10)
#define I2C2_ISR            REG32(I2C2_BASE, 0x18)
#define I2C2_ICR            REG32(I2C2_BASE, 0x1C)
#define I2C2_TXDR           REG32(I2C2_BASE, 0x28)
#define I2C2_RXDR           REG32(I2C2_BASE, 0x24)

/* GPIO alternate function for I2C2 */
#define GPIO_AF4            4

/* I2C bit definitions */
#define I2C_CR1_PE          (1UL << 0)
#define I2C_ISR_BUSY        (1UL << 15)
#define I2C_ISR_TXE         (1UL << 0)
#define I2C_ISR_TC          (1UL << 6)
#define I2C_ISR_NACKF       (1UL << 4)
#define I2C_ISR_TXIS        (1UL << 1)
#define I2C_ISR_STOPF       (1UL << 5)
#define I2C_ISR_RXNE        (1UL << 2)

/* I2C CR2 bit definitions */
#define I2C_CR2_START       (1UL << 13)
#define I2C_CR2_STOP        (1UL << 14)
#define I2C_CR2_AUTOEND     (1UL << 25)
#define I2C_CR2_RD_WRN      (1UL << 10)

/* I2C ICR bit definitions */
#define I2C_ICR_NACKCF      (1UL << 4)
#define I2C_ICR_STOPCF      (1UL << 5)

/* I2C Timeout */
#define I2C2_TIMEOUT        10000

/* ==================== Function Prototypes ==================== */

/* Initialization and Configuration */
void BQ35100_I2C2_Init(void);
bool BQ35100_I2C2_IsPresent(void);
bool BQ35100_I2C2_ScanAddress(uint8_t address);
void BQ35100_I2C2_ResetAccumulatedCapacity(void);

// Gauge Enable Pin Control (PF3)
void BQ35100_ACC_GaugePin_Enable(void);
void BQ35100_ACC_GaugePin_Disable(void);

/* Low-level I2C functions */
bool BQ35100_I2C2_WriteCommand(uint8_t cmd, uint16_t data);
uint16_t BQ35100_I2C2_ReadWord(uint8_t cmd);
uint8_t BQ35100_I2C2_ReadByte(uint8_t cmd);
uint32_t BQ35100_I2C2_ReadDWord(uint8_t cmd);

/* Control Functions */
uint16_t BQ35100_I2C2_ReadControlStatus(void);
uint16_t BQ35100_I2C2_ReadDeviceType(void);
void BQ35100_I2C2_GaugeStart(void);
void BQ35100_I2C2_GaugeStop(void);
void BQ35100_I2C2_Reset(void);
void BQ35100_I2C2_Reset_ACC(void);
void BQ35100_I2C2_Seal(void);

/* Temperature Reading Functions */
uint16_t BQ35100_I2C2_ReadRawTemp(void);
int32_t BQ35100_I2C2_ReadTemperature_Int(void);
float BQ35100_I2C2_ReadTemperature_Float(void);
uint16_t BQ35100_I2C2_ReadInternalTemp(void);

/* Voltage and Current Reading Functions */
uint16_t BQ35100_I2C2_ReadVoltage(void);
int16_t BQ35100_I2C2_ReadCurrent(void);

/* Accumulator Mode Specific Functions */
int32_t BQ35100_I2C2_ReadAccumulatedCapacity(void);  // Accumulated capacity in µAh
uint8_t BQ35100_I2C2_ReadBatteryStatus(void);
uint8_t BQ35100_I2C2_ReadBatteryAlert(void);

/* Data Flash Functions */
uint8_t BQ35100_I2C2_ReadDataFlashByte(uint16_t address);
uint8_t BQ35100_I2C2_ReadOperationConfigA(void);
bool BQ35100_I2C2_WriteOperationConfigA(uint8_t config);

/* Unseal Functions */
bool BQ35100_I2C2_Unseal(void);
bool BQ35100_I2C2_IsSealed(void);

void BQ35100_I2C2_ConfigureACCMode(void);

/* Test and Diagnostic Functions */
void BQ35100_I2C2_Test(void);
void BQ35100_I2C2_ScanDevice(void);
void BQ35100_I2C2_DiagnosticTest(void);
void BQ35100_I2C2_PrintACCStatus(void);

#endif /* BQ35100_I2C_ACC_H_ */
