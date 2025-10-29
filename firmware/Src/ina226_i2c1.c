/*
 * ina226_i2c1.c
 *
 *  Created on: Oct 16, 2025
 *      Author: rachaelguise-brown
 */


#include "ina226_i2c1.h"
#include "uart.h"

#include <stdint.h>
#include <stddef.h>


/* ==================== Private Helper Functions ==================== */

static void GPIOB_EnableClock(void)
{
    RCC_AHB2ENR |= (1U << 1); // Enable GPIOB clock
}

static void I2C1_EnableClock(void)
{
    RCC_APB1ENR1 |= (1U << 21); // Enable I2C1 clock

    // Configure I2C1 clock source to PCLK1
    RCC_CCIPR &= ~(3U << 12);
    RCC_CCIPR |= (0U << 12);
}

static void GPIO_ConfigureI2C1Pins(void)
{
    // Configure PB8 (SCL) as alternate function
    GPIOB_MODER &= ~(3U << 16);
    GPIOB_MODER |= (2U << 16);

    // Configure PB9 (SDA) as alternate function
    GPIOB_MODER &= ~(3U << 18);
    GPIOB_MODER |= (2U << 18);

    // Set both pins to open drain
    GPIOB_OTYPER |= (1U << 8);
    GPIOB_OTYPER |= (1U << 9);

    // Set pull-up resistors
    GPIOB_PUPDR &= ~(3U << 16);
    GPIOB_PUPDR |= (1U << 16);
    GPIOB_PUPDR &= ~(3U << 18);
    GPIOB_PUPDR |= (1U << 18);

    // Set high speed
    GPIOB_OSPEEDR |= (3U << 16);
    GPIOB_OSPEEDR |= (3U << 18);

    // Set PB8 to AF4 (I2C1_SCL)
    GPIOB_AFRH &= ~(0xF << 0);
    GPIOB_AFRH |= (GPIO_AF4 << 0);

    // Set PB9 to AF4 (I2C1_SDA)
    GPIOB_AFRH &= ~(0xF << 4);
    GPIOB_AFRH |= (GPIO_AF4 << 4);
}

static void I2C1_Configure(void)
{
    // Disable I2C1 before configuration
    I2C1_CR1 &= ~I2C_CR1_PE;

    // Wait for PE=0 to be confirmed
    uint32_t timeout = 10000;
    while ((I2C1_CR1 & I2C_CR1_PE) && timeout--);

    if (timeout == 0)
    {
        return;
    }

    // Configure timing register for 100kHz @ 16MHz PCLK
    I2C1_TIMINGR = 0x00303D5B;

    // Configure filters
    uint32_t cr1_config = 0;
    I2C1_CR1 = cr1_config;

    // Enable I2C1
    I2C1_CR1 |= I2C_CR1_PE;

    // Wait for enable to take effect
    for (volatile int i = 0; i < 1000; i++);
}

void I2C1_Init(void)
{
    // Enable clocks
    GPIOB_EnableClock();
    I2C1_EnableClock();

    // Configure GPIO pins
    GPIO_ConfigureI2C1Pins();

    // Configure I2C
    I2C1_Configure();
}

/* Small helper timeouts */
#define I2C_TIMEOUT_LOOPS   100000U

/* Helper: write 1 register address + 2 data bytes (write word) */
static bool i2c1_write_word(uint8_t dev_addr_7b, uint8_t reg_addr, uint16_t value)
{
    uint32_t timeout;
    uint8_t msb = (uint8_t)((value >> 8) & 0xFF);
    uint8_t lsb = (uint8_t)(value & 0xFF);

    /* Configure CR2 for a write of 3 bytes: reg pointer + MSB + LSB
       Many STM32 examples set CR2 like: CR2 = (dev_addr<<1) | (nbytes<<16) | AUTOEND | START
       We'll follow that pattern (dev_addr is 7-bit).
    */
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF; /* clear flags */

    /* Set SADD (address), NBYTES = 3, AUTOEND, START, write direction:
       SADD placed as (addr << 1) (R/W bit is LSB). */
    I2C1_CR2 = ((uint32_t)dev_addr_7b << 1) | (3U << 16) | I2C_CR2_AUTOEND | I2C_CR2_START;

    /* Wait for TXIS then send register pointer and data bytes. */
    timeout = I2C_TIMEOUT_LOOPS;
    /* first byte: register pointer */
    while (!(I2C1_ISR & I2C_ISR_TXIS))
    {
        if (--timeout == 0) { UART_SendString("I2C write timeout (TXIS ptr)\r\n"); return false; }
        if (I2C1_ISR & I2C_ISR_NACKF) { I2C1_ICR = I2C_ICR_NACKCF; UART_SendString("I2C NACK on addr (write ptr)\r\n"); return false; }
    }
    I2C1_TXDR = reg_addr;

    /* second byte: MSB */
    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_TXIS))
    {
        if (--timeout == 0) { UART_SendString("I2C write timeout (TXIS msb)\r\n"); return false; }
        if (I2C1_ISR & I2C_ISR_NACKF) { I2C1_ICR = I2C_ICR_NACKCF; UART_SendString("I2C NACK on msb\r\n"); return false; }
    }
    I2C1_TXDR = msb;

    /* third byte: LSB */
    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_TXIS))
    {
        if (--timeout == 0) { UART_SendString("I2C write timeout (TXIS lsb)\r\n"); return false; }
        if (I2C1_ISR & I2C_ISR_NACKF) { I2C1_ICR = I2C_ICR_NACKCF; UART_SendString("I2C NACK on lsb\r\n"); return false; }
    }
    I2C1_TXDR = lsb;

    /* wait for STOP or TC/AUTOEND to complete */
    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_STOPF))
    {
        if (--timeout == 0) { UART_SendString("I2C write timeout (STOP)\r\n"); return false; }
    }

    /* Clear STOP flag */
    I2C1_ICR = I2C_ICR_STOPCF;
    return true;
}

/* Helper: read 2 bytes from register (read word). We do a register-pointer write (1 byte), then read 2 bytes. */
static bool i2c1_read_word(uint8_t dev_addr_7b, uint8_t reg_addr, uint16_t *out_word)
{
    uint32_t timeout;
    uint8_t hi = 0, lo = 0;

    /* First: write pointer (1 byte) */
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    I2C1_CR2 = ((uint32_t)dev_addr_7b << 1) | (1U << 16) | I2C_CR2_AUTOEND | I2C_CR2_START;
    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_TXIS))
    {
        if (--timeout == 0) { UART_SendString("I2C rw timeout (TXIS ptr)\r\n"); return false; }
        if (I2C1_ISR & I2C_ISR_NACKF) { I2C1_ICR = I2C_ICR_NACKCF; UART_SendString("I2C NACK on addr (w ptr)\r\n"); return false; }
    }
    I2C1_TXDR = reg_addr;

    /* Wait for STOP of the pointer write sequence */
    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_STOPF))
    {
        if (--timeout == 0) { UART_SendString("I2C rw timeout (STOP ptr)\r\n"); return false; }
    }
    I2C1_ICR = I2C_ICR_STOPCF;

    /* Now: perform the read of 2 bytes using RD_WRN and NBYTES=2 */
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    I2C1_CR2 = ((uint32_t)dev_addr_7b << 1) | (2U << 16) | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START;

    /* Wait for RXNE twice to pull two bytes */
    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_RXNE))
    {
        if (--timeout == 0) { UART_SendString("I2C read timeout (RXNE 1)\r\n"); return false; }
        if (I2C1_ISR & I2C_ISR_NACKF) { I2C1_ICR = I2C_ICR_NACKCF; UART_SendString("I2C NACK on read\r\n"); return false; }
    }
    hi = (uint8_t)I2C1_RXDR;

    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_RXNE))
    {
        if (--timeout == 0) { UART_SendString("I2C read timeout (RXNE 2)\r\n"); return false; }
    }
    lo = (uint8_t)I2C1_RXDR;

    /* wait for STOP */
    timeout = I2C_TIMEOUT_LOOPS;
    while (!(I2C1_ISR & I2C_ISR_STOPF))
    {
        if (--timeout == 0) { UART_SendString("I2C read timeout (STOP)\r\n"); return false; }
    }
    I2C1_ICR = I2C_ICR_STOPCF;

    *out_word = ((uint16_t)hi << 8) | (uint16_t)lo;
    return true;
}

/* ==================== Initialisation and Calibration  ==================== */

/* Public init: assumes user already enabled clocks and configured GPIOs & I2C timing as in your posted code.
   We'll program INA226 registers here.
*/
void INA226_I2C1_Init(void)
{
    UART_SendString("INA226 init start\r\n");

    I2C1_Init();

    /* I2C peripheral already configured by your BQ35100_I2C1_Init call; we only configure INA226 now. */
    if (!INA226_ConfigAndCal())
    {
        UART_SendString("INA226 config failed\r\n");
    }
    else
    {
        UART_SendString("INA226 configured OK\r\n");
    }
}

bool INA226_IsPresent(void)
{
    uint16_t raw;
    bool ok = i2c1_read_word(INA226_I2C_ADDR_7BIT, INA226_REG_CONFIG, &raw);
    if (!ok)
    {
        UART_SendString("INA226 not detected on I2C1\r\n");
        return false;
    }

    UART_SendString("INA226 detected on I2C1\r\n");
    return true;
}

/* Write configuration and calibration values to INA226 */
bool INA226_ConfigAndCal(void)
{
    bool ok;

    /* Write configuration register */
    ok = i2c1_write_word(INA226_I2C_ADDR_7BIT, INA226_REG_CONFIG, (uint16_t)INA226_CONFIG_VALUE);
    if (!ok)
    {
        UART_SendString("Failed to write config\r\n");
        return false;
    }
    UART_SendString("Config written: ");
    UART_SendHex(INA226_CONFIG_VALUE);
    UART_SendString("\r\n");

    /* Write calibration register */
    ok = i2c1_write_word(INA226_I2C_ADDR_7BIT, INA226_REG_CALIB, (uint16_t)INA226_CALIB_VALUE);
    if (!ok)
    {
        UART_SendString("Failed to write calib\r\n");
        return false;
    }
    UART_SendString("Calib written: ");
    UART_SendHex(INA226_CALIB_VALUE);
    UART_SendString("\r\n");

    /* Optionally clear mask/enable and alert limit (we leave them at zero) */
    i2c1_write_word(INA226_I2C_ADDR_7BIT, INA226_REG_MASKEN, 0x0000);
    i2c1_write_word(INA226_I2C_ADDR_7BIT, INA226_REG_ALRTLIMIT, 0x0000);

    return true;
}

/* ==================== Read Value Functions  ==================== */

/* Read once and convert to integer units (mV, mA, mW). No floating point.
   Returns true on success.
*/
bool INA226_ReadOnce(int32_t *bus_mV, int32_t *shunt_mV, int32_t *current_mA, uint32_t *power_mW)
{
    if (!INA226_ReadBusVoltage(bus_mV))   return false;
    if (!INA226_ReadShuntVoltage(shunt_mV)) return false;
    if (!INA226_ReadCurrent_mA(current_mA)) return false;
    if (!INA226_ReadPower_mW(power_mW))   return false;

    /*UART_SendString("Bus: ");   UART_SendNumber(*bus_mV);
    UART_SendString(" mV, Shunt: "); UART_SendNumber(*shunt_mV);
    UART_SendString(" mV, I: "); UART_SendNumber(*current_mA);
    UART_SendString(" mA, P: "); UART_SendNumber(*power_mW);
    UART_SendString(" mW\r\n");*/
    return true;
}


/* Continuous read loop: prints values via UART every ~1 second until *stop_requested becomes true.
   This is blocking; call from a task or background context.
*/
void INA226_ReadContinuous(volatile bool *stop_requested)
{
    UART_SendString("INA226 continuous read start\r\n");

    while (stop_requested == NULL || !(*stop_requested))
    {
        int32_t bus_mV;
        int32_t shunt_mV;
        int32_t current_mA;
        uint32_t power_mW;

        if (INA226_ReadOnce(&bus_mV, &shunt_mV, &current_mA, &power_mW))
        {
            /* Print nicely */
            UART_SendString("Bus: ");
            UART_SendNumber(bus_mV); UART_SendString(" mV, ");
            UART_SendString("Shunt: ");
            UART_SendNumber(shunt_mV); UART_SendString(" mV, ");
            UART_SendString("I: ");
            UART_SendNumber(current_mA); UART_SendString(" mA, ");
            UART_SendString("P: ");
            UART_SendNumber(power_mW); UART_SendString(" mW\r\n");
        }
        else
        {
            UART_SendString("INA226 read failed in continuous loop\r\n");
        }

        /* Blocking delay ~1 second. We choose a simple software delay to avoid HAL. */
        for (volatile uint32_t i = 0; i < 4000000U; ++i)
        {
            if (stop_requested && (*stop_requested)) break;
        }
    }

    UART_SendString("INA226 continuous read stopped\r\n");
}

void INA226_ReadPrint(void)
{
    int32_t bus_mV;
    int32_t shunt_mV;
    int32_t current_mA;
    uint32_t power_mW;

    if (INA226_ReadOnce(&bus_mV, &shunt_mV, &current_mA, &power_mW))
    {
        /* Print nicely */
        UART_SendString("Bus: ");
        UART_SendNumber(bus_mV); UART_SendString(" mV, ");
        UART_SendString("Shunt: ");
        UART_SendNumber(shunt_mV); UART_SendString(" mV, ");
        UART_SendString("I: ");
        UART_SendNumber(current_mA); UART_SendString(" mA, ");
        UART_SendString("P: ");
        UART_SendNumber(power_mW); UART_SendString(" mW\r\n");
    }
    else
    {
        UART_SendString("INA226 read failed in continuous loop\r\n");
    }

}

void INA226_Compare_Measured_vs_Calculated(void)
{
    int32_t bus_mV, shunt_mV;
    int32_t current_calc_mA, current_meas_mA;
    uint32_t power_calc_mW, power_meas_mW;

    UART_SendString("\r\n=== INA226 Comparison: Measured vs Calculated ===\r\n");

    /* Read bus and shunt voltage first */
    if (!INA226_ReadBusVoltage(&bus_mV))
    {
        UART_SendString("Failed to read bus voltage\r\n");
        return;
    }
    if (!INA226_ReadShuntVoltage(&shunt_mV))
    {
        UART_SendString("Failed to read shunt voltage\r\n");
        return;
    }

    /* --- Calculate current and power manually --- */
    /* I_calc = Vshunt_mV / Rshunt_mOhm */
    current_calc_mA = (shunt_mV * 1000) / INA226_RSHUNT_mOHM;   /* mA */
    power_calc_mW   = (bus_mV * current_calc_mA) / 1000;        /* mW */

    /* --- Read measured (register) values --- */
    if (!INA226_ReadCurrent_mA(&current_meas_mA))
    {
        UART_SendString("Failed to read measured current\r\n");
        return;
    }

    if (!INA226_ReadPower_mW(&power_meas_mW))
    {
        UART_SendString("Failed to read measured power\r\n");
        return;
    }

    /* --- Print comparison --- */
    UART_SendString("Bus Voltage: ");
    UART_SendNumber(bus_mV); UART_SendString(" mV\r\n");

    UART_SendString("Shunt Voltage: ");
    UART_SendNumber(shunt_mV); UART_SendString(" mV\r\n");

    UART_SendString("\r\n-- Current Comparison --\r\n");
    UART_SendString("Calculated: ");
    UART_SendNumber(current_calc_mA);
    UART_SendString(" mA\r\n");
    UART_SendString("Measured (INA226): ");
    UART_SendNumber(current_meas_mA);
    UART_SendString(" mA\r\n");

    UART_SendString("\r\n-- Power Comparison --\r\n");
    UART_SendString("Calculated: ");
    UART_SendNumber(power_calc_mW);
    UART_SendString(" mW\r\n");
    UART_SendString("Measured (INA226): ");
    UART_SendNumber(power_meas_mW);
    UART_SendString(" mW\r\n");

    UART_SendString("============================================\r\n\r\n");
}


/* ========== Individual read functions ========== */

/* Read Bus Voltage in millivolts */
bool INA226_ReadBusVoltage(int32_t *bus_mV)
{
    uint16_t raw;
    if (!i2c1_read_word(INA226_I2C_ADDR_7BIT, INA226_REG_BUSV, &raw))
        return false;

    /* LSB = 1.25 mV = 5/4 mV */
    *bus_mV = ((int32_t)raw * 5) / 4;
    return true;
}

/* Read Shunt Voltage in millivolts */
bool INA226_ReadShuntVoltage(int32_t *shunt_mV)
{
    uint16_t raw;
    if (!i2c1_read_word(INA226_I2C_ADDR_7BIT, INA226_REG_SHUNTV, &raw))
        return false;

    int32_t shunt_uV = ((int16_t)raw * 5) / 2;    /* 2.5 µV/bit */
    *shunt_mV = shunt_uV / 1000;                  /* to mV */
    return true;
}

/* Read Current from register in microamps */
bool INA226_ReadCurrent_mA(int32_t *current_mA)
{
    uint16_t raw;
    if (!i2c1_read_word(INA226_I2C_ADDR_7BIT, INA226_REG_CURRENT, &raw))
        return false;

    int32_t raw_signed = (int16_t)raw;
    /* convert raw × Current_LSB (µA/bit) → mA */
    int32_t current_uA = raw_signed * INA226_CURRENT_LSB_UA;
    *current_mA = (current_uA + 500) / 1000;     /* rounded mA */
    return true;
}

/* Read Current (computed from shunt voltage) in microamps */
bool INA226_ReadCurrentCalc_mA(int32_t *current_mA)
{
    int32_t shunt_mV;
    if (!INA226_ReadShuntVoltage(&shunt_mV))
        return false;

    /* Convert to µA: I = Vshunt / Rshunt */
    /* Vshunt_mV → mA = (Vshunt_mV) / Rshunt_mOHM */
    *current_mA = (shunt_mV) / INA226_RSHUNT_mOHM;
    return true;
}

/* Read Power from Register in microwatts */
bool INA226_ReadPower_mW(uint32_t *power_mW)
{
    uint16_t raw;
    if (!i2c1_read_word(INA226_I2C_ADDR_7BIT, INA226_REG_POWER, &raw))
        return false;

    uint32_t power_uW = (uint32_t)raw * INA226_POWER_LSB_UW;
    *power_mW = (power_uW + 500) / 1000;
    return true;
}

/* Read Power (computed from Bus × Current) in microwatts */
bool INA226_ReadPowerCalc_mW(uint32_t *power_mW)
{
    int32_t bus_mV, current_mA;
    if (!INA226_ReadBusVoltage(&bus_mV))   return false;
    if (!INA226_ReadCurrentCalc_mA(&current_mA)) return false;

    int64_t tmp = (int64_t)bus_mV * (int64_t)current_mA; /* µW */
    *power_mW = (uint32_t)(tmp / 1000);                  /* mW */
    return true;
}

