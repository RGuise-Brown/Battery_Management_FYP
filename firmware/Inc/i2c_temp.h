/*
 * i2c_temp.h
 *
 *  Created on: Sep 29, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: I2C implementation for STM32L4
 * 		Using I2C1 peripheral
 * 		Pins on board:
 * 		SCL = PB8
 * 		SDA = PB9
 */

#ifndef I2C_TEMP_H_
#define I2C_TEMP_H_

#include <stdint.h>
#include <stdbool.h>

// Base addresses
#define RCC_BASE                    0x40021000UL
#define GPIOB_BASE                  0x48000400UL
#define I2C1_BASE                   0x40005400UL

// Register access macro
#define REG32(base, offset)         (*(volatile uint32_t*)((base) + (offset)))

// RCC Register definitions
#define RCC_AHB2ENR                 REG32(RCC_BASE, 0x4C)
#define RCC_APB1ENR1                REG32(RCC_BASE, 0x58)
#define RCC_CCIPR                   REG32(RCC_BASE, 0x88)

// GPIO Register definitions for GPIOB
#define GPIOB_MODER                 REG32(GPIOB_BASE, 0x00)
#define GPIOB_OTYPER                REG32(GPIOB_BASE, 0x04)
#define GPIOB_OSPEEDR               REG32(GPIOB_BASE, 0x08)
#define GPIOB_PUPDR                 REG32(GPIOB_BASE, 0x0C)
#define GPIOB_AFRH                  REG32(GPIOB_BASE, 0x24)

// I2C Register definitions
#define I2C1_CR1                    REG32(I2C1_BASE, 0x00)
#define I2C1_CR2                    REG32(I2C1_BASE, 0x04)
#define I2C1_TIMINGR                REG32(I2C1_BASE, 0x10)
#define I2C1_ISR                    REG32(I2C1_BASE, 0x18)
#define I2C1_ICR                    REG32(I2C1_BASE, 0x1C)
#define I2C1_TXDR                   REG32(I2C1_BASE, 0x28)
#define I2C1_RXDR                   REG32(I2C1_BASE, 0x24)

// GPIO alternate function for I2C1
#define GPIO_AF4                    4

// I2C bit definitions
#define I2C_CR1_PE                  (1UL << 0)
#define I2C_ISR_BUSY                (1UL << 15)
#define I2C_ISR_TXE                 (1UL << 0)
#define I2C_ISR_TC                  (1UL << 6)
#define I2C_ISR_NACKF               (1UL << 4)
#define I2C_ISR_TXIS   				(1UL << 1)
#define I2C_ISR_STOPF  				(1UL << 5)
#define I2C_ISR_RXNE 				(1UL << 2)

// I2C CR2 bit definitions
#define I2C_CR2_START               (1UL << 13)
#define I2C_CR2_STOP                (1UL << 14)
#define I2C_CR2_AUTOEND             (1UL << 25)
#define I2C_CR2_RD_WRN 				(1UL << 10)

// I2C ICR bit definitions
#define I2C_ICR_NACKCF              (1UL << 4)
#define I2C_ICR_STOPCF              (1UL << 5)

// I2C timing value for 100kHz @ 16MHz system clock
#define I2C_TIMING_100KHZ           0x00303D5BUL

// TMP117 specific definitions
#define TMP117_I2C_ADDRESS          0x48
#define TMP117_TEMP_REGISTER        0x00
#define TMP117_CONFIG_REGISTER      0x01
#define TMP117_HIGH_LIMIT_REGISTER	0x02
#define TMP117_LOW_LIMIT_REGISTER 	0x03
#define TMP117_DEVICE_ID_REGISTER 	0x0F

// TMP117 expected device ID
#define TMP117_DEVICE_ID 0x0117

// FUNCTION PROTOTYPES

// Initialisation, Configuration and Testing
void I2C_Init(void);
bool I2C_IsReady(void);
void I2C_SendByte(uint8_t data);
bool I2C_ScanAddress(uint8_t address);
void I2C_ScanTMP117(void);

//TMP117 Functions
bool TMP117_IsPresent(void);
uint16_t TMP117_ReadRawTemp(void);
int32_t TMP117_ConvertToTempC(uint16_t raw_temp);
void TMP117_Test(void);
uint16_t TMP117_ReadDeviceID(void);   //Device ID read function

// I2C Communication Functions
bool I2C_WriteRegister(uint8_t device_addr, uint8_t reg_addr, uint16_t data);
uint16_t I2C_ReadRegister(uint8_t device_addr, uint8_t reg_addr);

// Finding I2C busses and Checking Configuration
void I2C_ScanBus(void);
void I2C_TestBusLines(void);    // MISSING: Function declaration
void I2C_VerifyGPIOConfig(void);


#endif /* I2C_TEMP_H_ */
