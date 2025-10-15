/*
 * ina226_i2c1.h
 *
 *  Created on: Oct 15, 2025
 *      Author: rachaelguise-brown
 */

#ifndef INA226_I2C1_H_
#define INA226_I2C1_H_

#include "bq35100_common.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== STM32L4 I2C1 Register Definitions ==================== */

// Base addresses
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

/* INA226 I2C address (A1=A0=GND) */
#define INA226_I2C_ADDR_7BIT   0x40U

/* INA226 Register addresses */
#define INA226_REG_CONFIG      0x00
#define INA226_REG_SHUNTV      0x01
#define INA226_REG_BUSV        0x02
#define INA226_REG_POWER       0x03
#define INA226_REG_CURRENT     0x04
#define INA226_REG_CALIB       0x05
#define INA226_REG_MASKEN      0x06
#define INA226_REG_ALRTLIMIT   0x07

/* Chosen scaling constants (integers only) */
// don't touch
#define INA226_SHUNT_LSB_UV        25U/10U /* 2.5 uV = 25/10 uV - use integer math via (reg*5)/2 for uV */

// For 0.1 Ohm
//#define INA226_CURRENT_LSB_UA      30    /* 30 microamp per current-bit */
//#define INA226_POWER_LSB_UW        (25 * INA226_CURRENT_LSB_UA) /* 25 * current_lsb -> micro-watts per power-bit (750 uW) */

/* Precomputed calibration register for Rshunt = 0.15 ohm and Current_LSB = 30 uA:
   CAL = trunc(0.00512 / (Current_LSB * Rshunt))
   = 1137 -> 0x0471
*/
//#define INA226_CALIB_VALUE     0x0471U

/* For 75 Ω shunt test */
#define INA226_CURRENT_LSB_UA      1    /* 1 microamp per current-bit */
#define INA226_POWER_LSB_UW        25   /* 25 microwatt per power-bit (25 * Current_LSB_uA) */
#define INA226_CALIB_VALUE         0x08E3U  /* decimal 68 */ // 0x0044U

/* Configuration chosen for ~1 second updates:
   AVG = 128 (AVG bits = 100)
   VBUSCT = 110 (4.156 ms)
   VSHCT = 110  (4.156 ms)
   MODE = 111 (shunt + bus, continuous)
   => Configuration register = 0x09B7
*/
#define INA226_CONFIG_VALUE    0x09B7U

/* Public API */
void INA226_I2C1_Init(void);
bool INA226_ConfigAndCal(void);

/* Read once (single atomic read of outputs). Results returned in integer units:
   - bus_mV  : millivolts (0..)
   - shunt_mV: millivolts (signed) (small values likely 0)
   - current_mA: milliamp (signed)
   - power_mW: milliwatt (unsigned)
   Returns true if read successful.
*/
bool INA226_ReadOnce(int32_t *bus_mV, int32_t *shunt_mV, int32_t *current_mA, uint32_t *power_mW);

/* Continuous read: blocking loop that prints measurements via UART at ~1 Hz.
   Call from a dedicated task or from main if desired. Returns if stop_requested becomes true.
*/
void INA226_ReadContinuous(volatile bool *stop_requested);

#endif /* INA226_I2C1_H_ */
