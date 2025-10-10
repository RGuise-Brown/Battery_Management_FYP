/*
 * bq35100_i2c_acc.c
 *
 *  Created on: Oct 2, 2025
 *      Author: rachaelguise-brown
 *      Description: BQ35100 Fuel Gauge implementation using I2C2 in Accumulator Mode
 *  	I2C2 Pins: PF0 = SDA, PF1 = SCL, PF2 = SMBA
 *  	CN9 pins 21, 19, 17
 *  	PF3 = Gauge Enable (CN8 pin 14)
 *
 */

#include "bq35100_i2c_acc.h"
#include "uart.h"

/* ==================== Private Helper Functions ==================== */

static void GPIOF_EnableClock(void)
{
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
}

static void I2C2_EnableClock(void)
{
    RCC_APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

    // Configure I2C2 clock source to PCLK1
    RCC_CCIPR &= ~(3U << 14);  // Clear I2C2SEL bits
    RCC_CCIPR |= (0U << 14);   // Select PCLK1
}

static void GPIO_ConfigureI2C2Pins(void)
{
    // Configure PF0 (SDA) as alternate function
    GPIOF_MODER &= ~(3U << 0);
    GPIOF_MODER |= (2U << 0);

    // Configure PF1 (SCL) as alternate function
    GPIOF_MODER &= ~(3U << 2);
    GPIOF_MODER |= (2U << 2);

    // Configure PF2 (SMBA) as alternate function
    GPIOF_MODER &= ~(3U << 4);
    GPIOF_MODER |= (2U << 4);

    // Set all pins to open drain
    GPIOF_OTYPER |= (1U << 0);  // PF0
    GPIOF_OTYPER |= (1U << 1);  // PF1
    GPIOF_OTYPER |= (1U << 2);  // PF2

    // Set pull-up resistors (or no pull if external pull-ups)
    GPIOF_PUPDR &= ~(3U << 0);
    GPIOF_PUPDR |= (1U << 0);   // PF0 pull-up
    GPIOF_PUPDR &= ~(3U << 2);
    GPIOF_PUPDR |= (1U << 2);   // PF1 pull-up
    GPIOF_PUPDR &= ~(3U << 4);
    GPIOF_PUPDR |= (1U << 4);   // PF2 pull-up

    // Set high speed
    GPIOF_OSPEEDR |= (3U << 0);   // PF0
    GPIOF_OSPEEDR |= (3U << 2);   // PF1
    GPIOF_OSPEEDR |= (3U << 4);   // PF2

    // Set alternate function to AF4 (I2C2)
    GPIOF_AFRL &= ~(0xF << 0);
    GPIOF_AFRL |= (GPIO_AF4 << 0);  // PF0

    GPIOF_AFRL &= ~(0xF << 4);
    GPIOF_AFRL |= (GPIO_AF4 << 4);  // PF1

    GPIOF_AFRL &= ~(0xF << 8);
    GPIOF_AFRL |= (GPIO_AF4 << 8);  // PF2
}

static void I2C2_Configure(void)
{
    // Disable I2C2 before configuration
    I2C2_CR1 &= ~I2C_CR1_PE;

    // Wait for PE=0 to be confirmed
    uint32_t timeout = I2C2_TIMEOUT;
    while ((I2C2_CR1 & I2C_CR1_PE) && timeout--);

    if (timeout == 0)
    {
        return;
    }

    // Configure timing register for 100kHz @ 16MHz PCLK
    I2C2_TIMINGR = 0x00303D5B;

    // Configure filters (clear all, use defaults)
    I2C2_CR1 = 0;

    // Enable I2C2
    I2C2_CR1 |= I2C_CR1_PE;

    // Wait for enable to take effect
    for (volatile int i = 0; i < 1000; i++);
}

/* ==================== Gauge Enable Pin Control (PF3) ==================== */

static void BQ35100_ACC_GaugeEnable_Init(void)
{
    // GPIOF clock is already enabled by GPIO_ConfigureI2C2Pins()
    // But we'll ensure it's on
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOFEN;

    // Wait for clock to stabilize
    for (volatile int i = 0; i < 1000; i++);

    // Configure PF3 as output
    // Clear mode bits for PF3 (bits 7:6)
    GPIOF_MODER &= ~(3U << (3 * 2));
    // Set as output (01)
    GPIOF_MODER |= (1U << (3 * 2));

    // Configure as push-pull output
    GPIOF_OTYPER &= ~(1U << 3);

    // Set medium speed
    GPIOF_OSPEEDR &= ~(3U << (3 * 2));
    GPIOF_OSPEEDR |= (1U << (3 * 2));

    // No pull-up/pull-down
    GPIOF_PUPDR &= ~(3U << (3 * 2));

    // Start with gauge disabled (GE pin LOW)
    BQ35100_ACC_GaugePin_Disable();

    UART_SendString("PF3 configured for BQ35100 ACC Gauge Enable\r\n");
}

void BQ35100_ACC_GaugePin_Enable(void)
{
    // Set PF3 high using BSRR register (atomic set)
    GPIOF_BSRR = (1U << 3);

    // Give device time to power up
    for (volatile int i = 0; i < 20000; i++); // ~1ms at 16MHz

    UART_SendString("BQ35100 ACC Gauge Enable pin HIGH\r\n");
}

void BQ35100_ACC_GaugePin_Disable(void)
{
    // Clear PF3 using BSRR register (atomic clear)
    GPIOF_BSRR = (1U << (3 + 16));

    UART_SendString("BQ35100 ACC Gauge Enable pin LOW\r\n");
}

/* ==================== Initialization Functions ==================== */

void BQ35100_I2C2_Init(void)
{
    UART_SendString("Initializing BQ35100 on I2C2 (Accumulator Mode)...\r\n");

    // Enable clocks
    GPIOF_EnableClock();
    I2C2_EnableClock();

    // Configure GPIO pins
    GPIO_ConfigureI2C2Pins();

    // Initialize and enable the Gauge Enable pin (PF3)
    BQ35100_ACC_GaugeEnable_Init();
    BQ35100_ACC_GaugePin_Enable();  // Power up the gauge

    // Configure I2C
    I2C2_Configure();

    UART_SendString("BQ35100 I2C2 initialization complete\r\n");

    // Wait a bit for device to be ready
    for(volatile int i = 0; i < 100000; i++);

    // Start the gauge (this enables measurements)
    UART_SendString("Starting gauge in Accumulator mode...\r\n");
    BQ35100_I2C2_GaugeStart();

    // Wait for gauge to start
    for(volatile int i = 0; i < 100000; i++);
}

bool BQ35100_I2C2_ScanAddress(uint8_t address)
{
    // Wait for bus to be free
    uint32_t timeout = I2C2_TIMEOUT;
    while ((I2C2_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    // Clear any previous flags
    I2C2_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Configure transfer: 7-bit address, 1 byte, auto-end
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)address << 1);
    I2C2_CR2 |= (1 << 16);
    I2C2_CR2 |= I2C_CR2_AUTOEND;

    // Start the transfer
    I2C2_CR2 |= I2C_CR2_START;

    // Wait for TXIS or NACK
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) && timeout--);
    if (timeout == 0) return false;

    if (I2C2_ISR & I2C_ISR_NACKF)
    {
        I2C2_ICR = I2C_ICR_NACKCF;
        return false;
    }

    // Write dummy byte
    I2C2_TXDR = 0x00;

    // Wait for transfer complete
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & (I2C_ISR_TC | I2C_ISR_NACKF)) && timeout--);

    if (timeout == 0 || (I2C2_ISR & I2C_ISR_NACKF))
    {
        I2C2_ICR = I2C_ICR_NACKCF;
        return false;
    }

    return true;
}

bool BQ35100_I2C2_IsPresent(void)
{
    return BQ35100_I2C2_ScanAddress(BQ35100_I2C_ADDRESS);
}

void BQ35100_I2C2_ScanDevice(void)
{
    UART_SendString("=== BQ35100 I2C2 Device Scan ===\r\n");
    UART_SendString("Testing address 0x55... ");

    if (BQ35100_I2C2_ScanAddress(BQ35100_I2C_ADDRESS))
    {
        UART_SendString("BQ35100 FOUND on I2C2!\r\n");
    }
    else
    {
        UART_SendString("No response\r\n");
        UART_SendString("Check connections and power\r\n");
    }

    UART_SendString("\r\n");
}

/* ==================== Low-Level I2C Functions ==================== */

uint16_t BQ35100_I2C2_ReadWord(uint8_t cmd)
{
    uint16_t result = 0;
    uint32_t timeout;

    // Wait for bus to be free
    timeout = I2C2_TIMEOUT;
    while ((I2C2_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0;

    // Clear any previous flags
    I2C2_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Step 1: Write command, NO STOP
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= (1 << 16); // 1 byte to write
    //I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    // Wait for TXIS
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0;

    // Send command
    I2C2_TXDR = cmd;

    // Wait for transfer complete
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0;

    // Step 2: Read 2 bytes from device - WITH REPEATED START
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= I2C_CR2_RD_WRN; // Read mode
    I2C2_CR2 |= (2 << 16); // 2 bytes to read
    I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    // Read first byte (LSB - BQ35100 uses little-endian)
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0;
    uint8_t lsb = I2C2_RXDR;

    // Read second byte (MSB)
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0;
    uint8_t msb = I2C2_RXDR;

    // Combine LSB and MSB (little-endian)
    result = (msb << 8) | lsb;

    return result;
}

uint8_t BQ35100_I2C2_ReadByte(uint8_t cmd)
{
    uint32_t timeout;

    // Wait for bus to be free
    timeout = I2C2_TIMEOUT;
    while ((I2C2_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0xFF;

    // Clear flags
    I2C2_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Write command to point to register
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= (1 << 16); // 1 byte to write
    I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;

    I2C2_TXDR = cmd;

    // Wait for transfer complete
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0xFF;

    // Read 1 byte
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= I2C_CR2_RD_WRN;
    I2C2_CR2 |= (1 << 16); // 1 byte to read
    I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0xFF;

    uint8_t data = I2C2_RXDR;

    return data;
}

uint32_t BQ35100_I2C2_ReadDWord(uint8_t cmd) // Double Word - 4 bytes
{
    uint32_t result = 0;
    uint32_t timeout;

    // Wait for bus to be free
    timeout = I2C2_TIMEOUT;
    while ((I2C2_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0;

    // Clear any previous flags
    I2C2_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Step 1: Write command byte - NO STOP
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= (1 << 16);
    //I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
     if (timeout == 0) return 0;

     I2C2_TXDR = cmd;

     timeout = I2C2_TIMEOUT;
     while (!(I2C2_ISR & I2C_ISR_TC) && timeout--);
     if (timeout == 0) return 0;

     // Step 2: Read 4 bytes (little-endian) - WITH REPEATED START
     I2C2_CR2 = 0;
     I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
     I2C2_CR2 |= I2C_CR2_RD_WRN;
     I2C2_CR2 |= (4 << 16); // 4 bytes to read
     I2C2_CR2 |= I2C_CR2_AUTOEND;
     I2C2_CR2 |= I2C_CR2_START;

     // Read bytes in little-endian order
     for (int i = 0; i < 4; i++)
     {
         timeout = I2C2_TIMEOUT;
         while (!(I2C2_ISR & I2C_ISR_RXNE) && timeout--);
         if (timeout == 0) return 0;

         uint8_t byte = I2C2_RXDR;
         result |= ((uint32_t)byte << (i * 8));
     }

     return result;
 }

bool BQ35100_I2C2_WriteCommand(uint8_t cmd, uint16_t data)
{
    uint32_t timeout;

    // Wait for bus to be free
    timeout = I2C2_TIMEOUT;
    while ((I2C2_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    // Clear any previous flags
    I2C2_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Write command + 2 data bytes (3 bytes total)
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= (3 << 16); // 3 bytes to write
    I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    // Wait for TXIS and send command
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C2_TXDR = cmd;


    // Send LSB
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C2_TXDR = (uint8_t)(data & 0xFF);

    // Send MSB
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C2_TXDR = (uint8_t)((data >> 8) & 0xFF);

    // Wait for transfer complete
    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return false;

    return true;
}

/* ==================== Control Functions ==================== */

uint16_t BQ35100_I2C2_ReadControlStatus(void)
{
    // Write control command first
    if (!BQ35100_I2C2_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_CONTROL_STATUS))
    {
        return 0;
    }

    // Small delay for device to process
    for (volatile int i = 0; i < 5000; i++);

    // Read result from control registers
    return BQ35100_I2C2_ReadWord(BQ35100_CONTROL_CMD);
}

uint16_t BQ35100_I2C2_ReadDeviceType(void)
{
    // Write control command
    if (!BQ35100_I2C2_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_DEVICE_TYPE))
    {
        return 0;
    }

    // Small delay
    for (volatile int i = 0; i < 5000; i++);

    // Read result
    return BQ35100_I2C2_ReadWord(BQ35100_CONTROL_CMD);
}

void BQ35100_I2C2_GaugeStart(void)
{
    BQ35100_I2C2_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_GAUGE_START);
    UART_SendString("BQ35100 I2C2: Gauge started\r\n");
}

void BQ35100_I2C2_GaugeStop(void)
{
    BQ35100_I2C2_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_GAUGE_STOP);
    UART_SendString("BQ35100 I2C2: Gauge stopped\r\n");
}

void BQ35100_I2C2_Reset(void)
{
    BQ35100_I2C2_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_RESET);
    UART_SendString("BQ35100 I2C2: Device reset\r\n");

    // Wait for reset to complete
    for (volatile int i = 0; i < 500000; i++);
}

/* ==================== Temperature Functions ==================== */

uint16_t BQ35100_I2C2_ReadRawTemp(void)
{
    return BQ35100_I2C2_ReadWord(BQ35100_TEMPERATURE);
}

int32_t BQ35100_I2C2_ReadTemperature_Int(void)
{
    uint16_t raw_temp = BQ35100_I2C2_ReadRawTemp();
    return BQ35100_ConvertToCelsius_Int(raw_temp);
}

uint16_t BQ35100_I2C2_ReadInternalTemp(void)
{
    return BQ35100_I2C2_ReadWord(BQ35100_INTERNAL_TEMP);
}

/* ==================== Voltage and Current Functions ==================== */

uint16_t BQ35100_I2C2_ReadVoltage(void)
{
    return BQ35100_I2C2_ReadWord(BQ35100_VOLTAGE);
}

int16_t BQ35100_I2C2_ReadCurrent(void)
{
    return (int16_t)BQ35100_I2C2_ReadWord(BQ35100_CURRENT);
}

/* ==================== Accumulator Mode Functions ==================== */

int32_t BQ35100_I2C2_ReadAccumulatedCapacity(void)
{
    // AccumulatedCapacity is a 4-byte value (32-bit) in µAh
    uint32_t raw = BQ35100_I2C2_ReadDWord(BQ35100_ACCUM_CAPACITY);
    return (int32_t)raw;
}

uint8_t BQ35100_I2C2_ReadBatteryStatus(void)
{
    return BQ35100_I2C2_ReadByte(BQ35100_BATTERY_STATUS);
}

uint8_t BQ35100_I2C2_ReadBatteryAlert(void)
{
    return BQ35100_I2C2_ReadByte(BQ35100_BATTERY_ALERT);
}

/* ==================== Data Flash Functions ==================== */

uint8_t BQ35100_I2C2_ReadDataFlashByte(uint16_t address)
{
    uint32_t timeout;

    // Step 1: Write data flash address to ManufacturerAccessControl
    timeout = I2C2_TIMEOUT;
    while ((I2C2_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0xFF;

    I2C2_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Write: Command (0x3E) + Address LSB + Address MSB
    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= (3 << 16);
    I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C2_TXDR = BQ35100_MAC_CONTROL;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C2_TXDR = (uint8_t)(address & 0xFF);

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C2_TXDR = (uint8_t)((address >> 8) & 0xFF);

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0xFF;

    // Delay for device to fetch data
    for (volatile int i = 0; i < 10000; i++);

    // Step 2: Read from MACData
    I2C2_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= (1 << 16);
    I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C2_TXDR = BQ35100_MAC_DATA;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0xFF;

    I2C2_CR2 = 0;
    I2C2_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C2_CR2 |= I2C_CR2_RD_WRN;
    I2C2_CR2 |= (1 << 16);
    I2C2_CR2 |= I2C_CR2_AUTOEND;
    I2C2_CR2 |= I2C_CR2_START;

    timeout = I2C2_TIMEOUT;
    while (!(I2C2_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0xFF;

    uint8_t data = I2C2_RXDR;

    return data;
}

uint8_t BQ35100_I2C2_ReadOperationConfigA(void)
{
    return BQ35100_I2C2_ReadDataFlashByte(BQ35100_OP_CONFIG_A_ADDR);
}

bool BQ35100_I2C2_WriteOperationConfigA(uint8_t config)
{
    // Writing to data flash requires unsealing and proper checksum
    // This is a simplified version - full implementation would need
    // unseal, write, checksum calculation, and seal
    // See BQ35100 TRM Chapter 10 for complete procedure
    UART_SendString("Warning: Data flash write not fully implemented\r\n");
    return false;
}

/* ==================== Test Functions ==================== */

void BQ35100_I2C2_Test(void) // External Temp, current, voltage, accumulated capacity
{
    UART_SendString("=== BQ35100 I2C2 Accumulator Mode Test ===\r\n");

    if (!BQ35100_I2C2_IsPresent())
    {
        UART_SendString("ERROR: BQ35100 not detected on I2C2\r\n");
        return;
    }

    UART_SendString("BQ35100 detected on I2C2!\r\n");

    // Read device type
    uint16_t device_type = BQ35100_I2C2_ReadDeviceType();
    UART_SendString("Device Type: 0x");
    UART_SendHex(device_type);
    UART_SendString(" (should be 0x0100)\r\n");

    // Read temperatures
    int32_t ext_temp = BQ35100_I2C2_ReadTemperature_Int();
    int32_t int_temp = BQ35100_ConvertToCelsius_Int(BQ35100_I2C2_ReadInternalTemp());

    UART_SendString("External Temp: ");
    int32_t whole = ext_temp / 100;
    int32_t decimal = ext_temp % 100;
    if (decimal < 0) decimal = -decimal;
    UART_SendNumber(whole);
    UART_SendString(".");
    if (decimal < 10) UART_SendString("0");
    UART_SendNumber(decimal);
    UART_SendString(" C\r\n");

    // Read voltage
    uint16_t voltage = BQ35100_I2C2_ReadVoltage();
    UART_SendString("Voltage: ");
    UART_SendNumber(voltage);
    UART_SendString(" mV\r\n");

    // Read current
    int16_t current = BQ35100_I2C2_ReadCurrent();
    UART_SendString("Current: ");
    UART_SendNumber(current);
    UART_SendString(" mA\r\n");


    // Read accumulated capacity
    uint32_t capacity = BQ35100_I2C2_ReadAccumulatedCapacity();
    UART_SendString("Accumulated Capacity: ");
    UART_SendNumber(capacity);
    UART_SendString(" uAh\r\n");

    UART_SendString("\r\n");
}

void BQ35100_I2C2_DiagnosticTest(void) //  External and internal temp
{
    UART_SendString("\r\n=== BQ35100 I2C2 Diagnostic Test ===\r\n");

    uint16_t temp_reading = BQ35100_I2C2_ReadRawTemp();
    uint16_t internal_temp = BQ35100_I2C2_ReadInternalTemp();

    UART_SendString("Temperature() reading: ");
    UART_SendNumber(temp_reading);
    UART_SendString(" (0.1K) = ");

    int32_t temp_c = BQ35100_ConvertToCelsius_Int(temp_reading);
    int32_t whole = temp_c / 100;
    int32_t decimal = temp_c % 100;
    if (decimal < 0) decimal = -decimal;
    UART_SendNumber(whole);
    UART_SendString(".");
    if (decimal < 10) UART_SendString("0");
    UART_SendNumber(decimal);
    UART_SendString(" C\r\n");

    UART_SendString("InternalTemperature() reading: ");
    UART_SendNumber(internal_temp);
    UART_SendString(" (0.1K) = ");

    int32_t int_temp_c = BQ35100_ConvertToCelsius_Int(internal_temp);
    whole = int_temp_c / 100;
    decimal = int_temp_c % 100;
    if (decimal < 0) decimal = -decimal;
    UART_SendNumber(whole);
    UART_SendString(".");
    if (decimal < 10) UART_SendString("0");
    UART_SendNumber(decimal);
    UART_SendString(" C\r\n");

    if (temp_reading == internal_temp)
    {
        UART_SendString("\r\n*** WARNING: Both readings are IDENTICAL! ***\r\n");
        UART_SendString("This means you're reading the INTERNAL sensor, not the thermistor!\r\n");
        UART_SendString("Check:\r\n");
        UART_SendString("  1. Operation Config A bit 7 (TEMPS) must be set to 1\r\n");
        UART_SendString("  2. Thermistor must be connected between REG25 and TS pins\r\n");
        UART_SendString("  3. Device may need configuration via TI Battery Management Studio\r\n");
    }
    else
    {
        UART_SendString("\r\nReadings are different - external thermistor may be working!\r\n");
        int32_t diff = temp_reading - internal_temp;
        if (diff < 0) diff = -diff;
        UART_SendString("Difference: ");
        UART_SendNumber(diff);
         UART_SendString(" (0.1K) = ");
         UART_SendNumber((diff * 10) / 100);
         UART_SendString(".");
         UART_SendNumber((diff * 10) % 100);
         UART_SendString(" C\r\n");
     }

     UART_SendString("\r\nTry heating the thermistor with your finger.\r\n");
     UART_SendString("The Temperature() reading should increase if external thermistor is working.\r\n");
     UART_SendString("\r\n");
 }

void BQ35100_I2C2_PrintACCStatus(void)
{
    UART_SendString("\r\n=== BQ35100 I2C2 Accumulator Status ===\r\n");

    uint32_t capacity = BQ35100_I2C2_ReadAccumulatedCapacity();
    uint16_t voltage = BQ35100_I2C2_ReadVoltage();
    int16_t current = BQ35100_I2C2_ReadCurrent();
    int32_t temp = BQ35100_I2C2_ReadTemperature_Int();
    uint8_t status = BQ35100_I2C2_ReadBatteryStatus();
    uint8_t alert = BQ35100_I2C2_ReadBatteryAlert();

    UART_SendString("Accumulated Capacity: ");
    UART_SendNumber(capacity);
    UART_SendString(" uAh\r\n");

    UART_SendString("Voltage: ");
    UART_SendNumber(voltage);
    UART_SendString(" mV\r\n");

    UART_SendString("Current: ");
    if (current < 0)
    {
        UART_SendString("-");
        current = -current;
    }
    UART_SendNumber(current);
    UART_SendString(" mA\r\n");

    UART_SendString("Temperature: ");
    int32_t whole = temp / 100;
    int32_t decimal = temp % 100;
    if (temp < 0 && whole == 0)
    {
        UART_SendString("-");
    }
    if (decimal < 0) decimal = -decimal;
    UART_SendNumber(whole);
    UART_SendString(".");
    if (decimal < 10) UART_SendString("0");
    UART_SendNumber(decimal);
    UART_SendString(" C\r\n");

    UART_SendString("Status: 0x");
    UART_SendHex(status);
    if (status & BQ35100_BSTATUS_DSG)
        UART_SendString(" [DISCHARGE]");
    if (status & BQ35100_BSTATUS_ALERT)
        UART_SendString(" [ALERT]");
    UART_SendString("\r\n");

    UART_SendString("Alert: 0x");
    UART_SendHex(alert);
    if (alert & BQ35100_ALERT_INITCOMP)
        UART_SendString(" [INIT_COMP]");
    if (alert & BQ35100_ALERT_G_DONE)
        UART_SendString(" [GAUGE_DONE]");
    if (alert & BQ35100_ALERT_BATLOW)
        UART_SendString(" [BAT_LOW]");
    if (alert & BQ35100_ALERT_TEMPLOW)
        UART_SendString(" [TEMP_LOW]");
    if (alert & BQ35100_ALERT_TEMPHIGH)
        UART_SendString(" [TEMP_HIGH]");
    UART_SendString("\r\n");

    // Read and display Operation Config A
    uint8_t op_config = BQ35100_I2C2_ReadOperationConfigA();
    UART_SendString("Operation Config A: 0x");
    UART_SendHex(op_config);
    UART_SendString(" [");

    if (op_config & BQ35100_OP_CONFIG_TEMPS)
        UART_SendString("EXT_TEMP ");
    else
        UART_SendString("INT_TEMP ");
    uint8_t mode = op_config & BQ35100_OP_CONFIG_GMSEL_MASK;
     if (mode == BQ35100_OP_CONFIG_GMSEL_ACC)
         UART_SendString("ACC_MODE");
     else if (mode == BQ35100_OP_CONFIG_GMSEL_SOH)
         UART_SendString("SOH_MODE");
     else if (mode == BQ35100_OP_CONFIG_GMSEL_EOS)
         UART_SendString("EOS_MODE");
     else
         UART_SendString("UNKNOWN_MODE");

     UART_SendString("]\r\n\r\n");
 }
