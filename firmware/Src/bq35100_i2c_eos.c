/*
 * bq35100_i2c_eos.c
 *
 *  Created on: Oct 2, 2025
 *      Author: rachaelguise-brown
 *      Description: BQ35100 Fuel Gauge implementation using I2C2 in End-Of-Service Mode
 *  	I2C2 Pins: PB9 = SDA, PB8 = SCL
 */

#include "bq35100_i2c_eos.h"
#include "uart.h"

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

/* ==================== Gauge Enable Pin Control (PF5) ==================== */

static void BQ35100_EOS_GaugeEnable_Init(void)
{
    // GPIOF clock should already be enabled by I2C configuration
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOFEN;

    // Wait for clock to stabilize
    for (volatile int i = 0; i < 1000; i++);

    // Configure PF5 as output
    // Clear mode bits for PF5 (bits 11:10)
    GPIOF_MODER &= ~(3U << (5 * 2));
    // Set as output (01)
    GPIOF_MODER |= (1U << (5 * 2));

    // Configure as push-pull output
    GPIOF_OTYPER &= ~(1U << 5);

    // Set medium speed
    GPIOF_OSPEEDR &= ~(3U << (5 * 2));
    GPIOF_OSPEEDR |= (1U << (5 * 2));

    // No pull-up/pull-down
    GPIOF_PUPDR &= ~(3U << (5 * 2));

    // Start with gauge disabled (GE pin LOW)
    BQ35100_EOS_GaugePin_Disable();

    UART_SendString("PF5 configured for BQ35100 EOS Gauge Enable\r\n");
}

void BQ35100_EOS_GaugePin_Enable(void)
{
    // Set PF5 high using BSRR register (atomic set)
    GPIOF_BSRR = (1U << 5);

    // Give device time to power up
    for (volatile int i = 0; i < 20000; i++); // ~1ms at 16MHz

    UART_SendString("BQ35100 EOS Gauge Enable pin HIGH\r\n");
}

void BQ35100_EOS_GaugePin_Disable(void)
{
    // Clear PF5 using BSRR register (atomic clear)
    GPIOF_BSRR = (1U << (5 + 16));

    UART_SendString("BQ35100 EOS Gauge Enable pin LOW\r\n");
}

/* ==================== Initialization Functions ==================== */

void BQ35100_I2C1_Init(void)
{
    UART_SendString("Initializing BQ35100...\r\n");

    // Enable clocks
    GPIOB_EnableClock();
    I2C1_EnableClock();

    // Configure GPIO pins
    GPIO_ConfigureI2C1Pins();

    // Initialize and enable the Gauge Enable pin (PF5)
    BQ35100_EOS_GaugeEnable_Init();
    BQ35100_EOS_GaugePin_Enable();  // Power up the gauge

    // Configure I2C
    I2C1_Configure();

    UART_SendString("BQ35100 I2C initialization complete\r\n");

    BQ35100_I2C1_CheckConfiguration();

    // Wait a bit for device to be ready
    for(volatile int i = 0; i < 100000; i++);

    // Start the gauge (this enables measurements)
    UART_SendString("Starting gauge...\r\n");
    BQ35100_I2C1_GaugeStart();

    // Wait for gauge to start
    for(volatile int i = 0; i < 100000; i++);
}

bool BQ35100_I2C1_ScanAddress(uint8_t address)
{
    // Wait for bus to be free
    uint32_t timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    // Clear any previous flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Configure transfer: 7-bit address, 1 byte, auto-end
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)address << 1);
    I2C1_CR2 |= (1 << 16);
    I2C1_CR2 |= I2C_CR2_AUTOEND;

    // Start the transfer
    I2C1_CR2 |= I2C_CR2_START;

    // Wait for TXIS or NACK
    timeout = 10000;
    while (!(I2C1_ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) && timeout--);
    if (timeout == 0) return false;

    if (I2C1_ISR & I2C_ISR_NACKF)
    {
        I2C1_ICR = I2C_ICR_NACKCF;
        return false;
    }

    // Write dummy byte
    I2C1_TXDR = 0x00;

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & (I2C_ISR_TC | I2C_ISR_NACKF)) && timeout--);

    if (timeout == 0 || (I2C1_ISR & I2C_ISR_NACKF))
    {
        I2C1_ICR = I2C_ICR_NACKCF;
        return false;
    }

    return true;
}

bool BQ35100_I2C1_IsPresent(void)
{
    return BQ35100_I2C1_ScanAddress(BQ35100_I2C_ADDRESS);
}

void BQ35100_I2C1_ScanDevice(void)
{
    UART_SendString("=== BQ35100 Device Scan ===\r\n");
    UART_SendString("Testing address 0x55... ");

    if (BQ35100_I2C1_ScanAddress(BQ35100_I2C_ADDRESS))
    {
        UART_SendString("BQ35100 FOUND!\r\n");
    }
    else
    {
        UART_SendString("No response\r\n");
        UART_SendString("Check connections and power\r\n");
    }

    UART_SendString("\r\n");
}

void BQ35100_I2C1_CheckConfiguration(void)
{
    UART_SendString("\n=== EOS MODE CONFIGURATION CHECK ===\r\n");

    // Read Operation Config A
    uint8_t config = BQ35100_I2C1_ReadOperationConfigA();
    UART_SendString("Operation Config A: 0x");
    UART_SendHex(config);
    UART_SendString("\r\n");

    // Check GMSEL bits [1:0]
    uint8_t mode = config & 0x03;
    UART_SendString("GMSEL bits [1:0]: ");
    UART_SendNumber(mode);
    UART_SendString(" = ");
    switch(mode) {
        case 0x00:
            UART_SendString("ACCUMULATOR mode\r\n");
            UART_SendString("*** ERROR: Should be 0x02 for EOS mode! ***\r\n");
            break;
        case 0x01:
            UART_SendString("STATE-OF-HEALTH mode\r\n");
            UART_SendString("*** ERROR: Should be 0x02 for EOS mode! ***\r\n");
            break;
        case 0x02:
            UART_SendString("END-OF-SERVICE mode [CORRECT]\r\n");
            break;
        case 0x03:
            UART_SendString("INVALID\r\n");
            break;
    }

    // Check if sealed
    bool sealed = BQ35100_I2C1_IsSealed();
    UART_SendString("Device sealed: ");
    UART_SendString(sealed ? "YES\r\n" : "NO\r\n");

    UART_SendString("\n");
}

/* ==================== Low-Level I2C Functions ==================== */

uint16_t BQ35100_I2C1_ReadWord(uint8_t cmd)
{
    uint16_t result = 0;
    uint32_t timeout;

    // Wait for bus to be free
    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0;

    // Clear any previous flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Step 1: Write command byte
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (1 << 16); // 1 byte to write
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Wait for TXIS
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0;

    // Send command
    I2C1_TXDR = cmd;

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0;

    // Step 2: Read 2 bytes from device
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= I2C_CR2_RD_WRN; // Read mode
    I2C1_CR2 |= (2 << 16); // 2 bytes to read
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Read first byte (LSB - BQ35100 uses little-endian)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0;

    uint8_t lsb = I2C1_RXDR;

    // Read second byte (MSB)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0;

    uint8_t msb = I2C1_RXDR;

    // Combine LSB and MSB (little-endian)
    result = (msb << 8) | lsb;

    return result;
}

bool BQ35100_I2C1_WriteCommand(uint8_t cmd, uint16_t data)
{
    uint32_t timeout;

    // Wait for bus to be free
    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    // Clear any previous flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Write command + 2 data bytes (3 bytes total)
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (3 << 16); // 3 bytes to write
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Wait for TXIS and send command
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = cmd;

    // Send LSB
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = (uint8_t)(data & 0xFF);

    // Send MSB
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = (uint8_t)((data >> 8) & 0xFF);

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return false;

    return true;
}

/* ==================== Control Functions ==================== */

uint16_t BQ35100_I2C1_ReadControlStatus(void)
{
    // Write control command first
    if (!BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_CONTROL_STATUS))
    {
        return 0;
    }

    // Small delay for device to process
    for (volatile int i = 0; i < 5000; i++);

    // Read result from control registers
    return BQ35100_I2C1_ReadWord(BQ35100_CONTROL_CMD);
}

uint16_t BQ35100_I2C1_ReadDeviceType(void)
{
    // Write control command
    if (!BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_DEVICE_TYPE))
    {
        return 0;
    }

    // Small delay
    for (volatile int i = 0; i < 5000; i++);

    // Read result
    return BQ35100_I2C1_ReadWord(BQ35100_CONTROL_CMD);
}

void BQ35100_I2C1_GaugeStart(void)
{
    BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_GAUGE_START);
    UART_SendString("BQ35100: Gauge started\r\n");
}

void BQ35100_I2C1_GaugeStop(void)
{
    BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_GAUGE_STOP);
    UART_SendString("BQ35100: Gauge stopped\r\n");
}

/* ==================== Voltage, Current, Impedance ==================== */
uint16_t BQ35100_I2C1_ReadVoltage(void)
{
    return BQ35100_I2C1_ReadWord(BQ35100_VOLTAGE);
}

int16_t BQ35100_I2C1_ReadCurrent(void)
{
    // Current is a SIGNED 16-bit value (can be positive or negative)
    // Positive = discharge, Negative = charge (shouldn't happen with primary battery)
    return (int16_t)BQ35100_I2C1_ReadWord(BQ35100_CURRENT);
}

uint16_t BQ35100_I2C1_ReadScaledR(void)
{
    // Scaled resistance in milliohms (mΩ)
    // Only updated in EOS mode
    return BQ35100_I2C1_ReadWord(BQ35100_SCALED_R);
}

uint16_t BQ35100_I2C1_ReadMeasuredZ(void)
{
    // Measured impedance in milliohms (mΩ)
    // Only updated in EOS mode
    return BQ35100_I2C1_ReadWord(BQ35100_MEASURED_Z);
}

/* ==================== EOS MODE Functions ==================== */

uint16_t BQ35100_I2C1_ReadStateOfHealth(void)
{
    // State of Health as percentage (0-100%)
    return BQ35100_I2C1_ReadWord(BQ35100_STATE_OF_HEALTH);
}

uint8_t BQ35100_I2C1_ReadBatteryAlert(void)
{
    // Read BatteryAlert register
    // Returns flags for various alert conditions
    // Reading this register also clears the ALERT pin

    uint32_t timeout;

    // Wait for bus to be free
    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0xFF;

    // Clear flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Write command to point to BatteryAlert register
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (1 << 16); // 1 byte to write
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;

    I2C1_TXDR = BQ35100_BATTERY_ALERT;

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0xFF;

    // Read 1 byte
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= I2C_CR2_RD_WRN;
    I2C1_CR2 |= (1 << 16); // 1 byte to read
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0xFF;

    uint8_t alert = I2C1_RXDR;

    return alert;
}

bool BQ35100_I2C1_IsEOSDetected(void)
{
    uint8_t alerts = BQ35100_I2C1_ReadBatteryAlert();
    return (alerts & BQ35100_ALERT_EOS) != 0;
}

bool BQ35100_I2C1_IsGaugeDone(void)
{
    uint8_t alerts = BQ35100_I2C1_ReadBatteryAlert();
    return (alerts & BQ35100_ALERT_G_DONE) != 0;
}

void BQ35100_I2C1_NewBattery(void)
{
    // Send NEW_BATTERY command to reset EOS tracking for fresh battery
    BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, BQ35100_CTRL_NEW_BATTERY);
    UART_SendString("BQ35100: New battery command sent - EOS tracking reset\r\n");
}

/* ==================== Data Flash Functions ==================== */

uint8_t BQ35100_I2C1_ReadDataFlashByte(uint16_t address)
{
    /*
     * Reads a byte from BQ35100 Data Flash
     *
     * Protocol (from TRM Chapter 12):
     * 1. Write the data flash address to ManufacturerAccessControl (0x3E/0x3F)
     * 2. Read the data from MACData (0x40-0x5F)
     * 3. Verify checksum (optional but recommended)
     *
     * The BQ35100 returns 32 bytes of data starting at the requested address
     */

    uint32_t timeout;

    // Step 1: Write data flash address to ManufacturerAccessControl
    // Address is written in LITTLE ENDIAN format (LSB first, then MSB)

    // Wait for bus to be free
    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0xFF;

    // Clear any previous flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Write: Command (0x3E) + Address LSB + Address MSB (3 bytes total)
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (3 << 16); // 3 bytes to write
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Send command byte (ManufacturerAccessControl register address)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C1_TXDR = BQ35100_MAC_CONTROL;

    // Send address LSB (low byte of data flash address)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C1_TXDR = (uint8_t)(address & 0xFF);

    // Send address MSB (high byte of data flash address)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C1_TXDR = (uint8_t)((address >> 8) & 0xFF);

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0xFF;

    // Small delay for device to process request and fetch data from flash
    for (volatile int i = 0; i < 10000; i++);

    // Step 2: Read the data from MACData (0x40)
    // The BQ35100 returns 32 bytes of data starting from the requested address
    // We only need the first byte (offset 0 from the address we requested)

    // Clear flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Write command to set read pointer to MACData
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (1 << 16); // 1 byte to write (register address)
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0xFF;
    I2C1_TXDR = BQ35100_MAC_DATA; // Point to MACData register

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0xFF;

    // Now read 1 byte from MACData
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= I2C_CR2_RD_WRN; // Read mode
    I2C1_CR2 |= (1 << 16); // 1 byte to read
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Read the byte
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0xFF;

    uint8_t data = I2C1_RXDR;

    return data;
}

uint8_t BQ35100_I2C1_ReadOperationConfigA(void)
{
    /*
     * Reads Operation Config A register from data flash
     *
     * Address: 0x41B1
     * Bit 7 (TEMPS): 0 = Internal temp sensor, 1 = External thermistor
     * Bit 6 (EXTVCELL): External voltage cell measurement
     * Bit 5 (WRTEMP): Write temperature enable
     * Bit 4 (LF_EN): Lifetime data enable
     * Bit 3: Reserved
     * Bit 2 (GNDSEL): Ground select for ADC
     * Bit 1-0 (GMSEL): Gauging mode select
     *
     * Default: 0x80 (binary: 10000000) - External temp enabled
     */

    return BQ35100_I2C1_ReadDataFlashByte(BQ35100_OP_CONFIG_A_ADDR);
}

/* ==================== Temperature Functions ==================== */

uint16_t BQ35100_I2C1_ReadRawTemp(void)
{
    return BQ35100_I2C1_ReadWord(BQ35100_TEMPERATURE);
}

int32_t BQ35100_I2C1_ConvertToCelsius_Int(uint16_t raw_temp)
{
    // BQ35100 returns temperature in 0.1°K units
    // Convert to hundredths of °C for better precision without float
    // Formula: °C = (raw * 0.1) - 273.15
    // In hundredths: (°C * 100) = (raw * 10) - 27315

    int32_t temp_hundredths_c = ((int32_t)raw_temp * 10) - 27315;

    return temp_hundredths_c;  // Returns temperature in 0.01°C units
                                // e.g., 2534 = 25.34°C, -550 = -5.50°C
}

int32_t BQ35100_I2C1_ReadTemperature_Int(void)
{
    uint16_t raw_temp = BQ35100_I2C1_ReadRawTemp();
    return BQ35100_ConvertToCelsius_Int(raw_temp);
}

uint16_t BQ35100_I2C1_ReadInternalTemp(void)
{
    // Internal temperature is at command 0x28
    return BQ35100_I2C1_ReadWord(BQ35100_INTERNAL_TEMP);
}

/* ==================== EOS Mode Monitoring Functions ==================== */

uint16_t BQ35100_I2C1_ReadEOSPulseCount(void) // Number of GAUGE_START/GAUGE_STOP cycles completed - default threshold = 20 pulses
{
    uint8_t lsb = BQ35100_I2C1_ReadDataFlashByte(0x419E);
    uint8_t msb = BQ35100_I2C1_ReadDataFlashByte(0x419F);
    return (msb << 8) | lsb;
}

uint32_t BQ35100_I2C1_ReadShortTrendAverage(void) // fast-moving average used for EOS detection.
{
    uint8_t byte0 = BQ35100_I2C1_ReadDataFlashByte(0x41A6);
    uint8_t byte1 = BQ35100_I2C1_ReadDataFlashByte(0x41A7);
    uint8_t byte2 = BQ35100_I2C1_ReadDataFlashByte(0x41A8);
    uint8_t byte3 = BQ35100_I2C1_ReadDataFlashByte(0x41A9);

    return ((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) |
           ((uint32_t)byte1 << 8) | byte0;
}

uint32_t BQ35100_I2C1_ReadLongTrendAverage(void) // slow-moving average used for EOS detection baseline.
{
    uint8_t byte0 = BQ35100_I2C1_ReadDataFlashByte(0x41AA);
    uint8_t byte1 = BQ35100_I2C1_ReadDataFlashByte(0x41AB);
    uint8_t byte2 = BQ35100_I2C1_ReadDataFlashByte(0x41AC);
    uint8_t byte3 = BQ35100_I2C1_ReadDataFlashByte(0x41AD);

    return ((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) |
           ((uint32_t)byte1 << 8) | byte0;
}


uint8_t BQ35100_I2C1_ReadGaugingMode(void)
{
    uint8_t config = BQ35100_I2C1_ReadOperationConfigA();
    return config & 0x03;  // Extract bits 1:0

    /*
     *  Returns:
 *   00 = ACCUMULATOR mode
 *   01 = STATE-OF-HEALTH mode (Li-MnO2)
 *   10 = END-OF-SERVICE mode (Li-SOCl2)
 *   11 = Invalid
     */
}

bool BQ35100_I2C1_IsGaugeActive(void) // true if gauge is running (between GAUGE_START and GAUGE_STOP)
{
    uint16_t status = BQ35100_I2C1_ReadControlStatus();
    return (status & (1 << 7)) != 0;  // GA bit (bit 7)
}

uint16_t BQ35100_I2C1_ReadBatteryStatus(void)
{
    return BQ35100_I2C1_ReadWord(BQ35100_BATTERY_STATUS);
    /*
    * Bit 3: EOS flag (set when end-of-service detected, cannot be cleared)
    * Bit 4: SOH_LOW flag
    */
}

bool BQ35100_I2C1_IsEOSFlagSet(void)
{
    uint16_t status = BQ35100_I2C1_ReadBatteryStatus();
    return (status & (1 << 3)) != 0;  // EOS bit
}

/* ==================== Sealing/Unsealing Functions ==================== */

bool BQ35100_I2C1_Unseal(void)
{
    /*
     * Unseals the device to allow data flash writes
     * Default unseal keys stored in flash: 0x0414, 0x3672
     * But must be sent BYTE-SWAPPED: 0x1404, 0x7236
     */

    UART_SendString("Unsealing device...\r\n");

    // Send first unseal key BYTE-SWAPPED (0x0414 stored, send 0x1404)
    if (!BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, 0x0414))
    {
        UART_SendString("  ERROR: Failed to send first unseal key\r\n");
        return false;
    }

    // Small delay
    for (volatile int i = 0; i < 10000; i++);

    // Send second unseal key BYTE-SWAPPED (0x3672 stored, send 0x7236)
    if (!BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, 0x3672))
    {
        UART_SendString("  ERROR: Failed to send second unseal key\r\n");
        return false;
    }

    // Wait for unseal to complete
    for (volatile int i = 0; i < 50000; i++);

    // Verify unsealed by reading control status
    uint16_t status = BQ35100_I2C1_ReadControlStatus();

    UART_SendString("Control Status after unseal: 0x");
    UART_SendHex(status);
    UART_SendString("\r\n");

    // Check SEC[1:0] bits (bits 14:13)
    // 11 = Sealed, 10 = Unsealed, 01 = Full Access
    uint8_t sec_bits = (status >> 13) & 0x03;

    UART_SendString("Security bits [SEC1:0]: ");
    UART_SendNumber(sec_bits);
    UART_SendString(" (");
    switch(sec_bits)
    {
        case 3:
            UART_SendString("SEALED");
            break;
        case 2:
            UART_SendString("UNSEALED");
            break;
        case 1:
            UART_SendString("FULL ACCESS");
            break;
        default:
            UART_SendString("RESERVED");
            break;
    }
    UART_SendString(")\r\n");

    if (sec_bits == 3)  // Still sealed
    {
        UART_SendString("  ERROR: Device still sealed!\r\n");
        return false;
    }

    UART_SendString("[OK] Device unsealed successfully!\r\n");
    return true;
}

bool BQ35100_I2C1_IsSealed(void)
{
    uint16_t status = BQ35100_I2C1_ReadControlStatus();
    uint8_t sec_bits = (status >> 13) & 0x03;
    return (sec_bits == 3);  // SEC[1:0] = 11 means sealed
}
/* ==================== Configuration Functions ==================== */

bool BQ35100_I2C1_WriteDataFlashByte(uint16_t address, uint8_t value)
{
    /*
     * Writes a byte to BQ35100 Data Flash using correct protocol from SLUA790
     */

    uint32_t timeout;

    UART_SendString("  Writing 0x");
    UART_SendHex(value);
    UART_SendString(" to address 0x");
    UART_SendHex(address);
    UART_SendString("...\r\n");

    uint8_t addr_lsb = (uint8_t)(address & 0xFF);
    uint8_t addr_msb = (uint8_t)((address >> 8) & 0xFF);

    // Step 1: Write address to register 0x3E (ManufacturerAccessControl)
    // Sequence: 0x3E, addr_lsb, addr_msb

    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0)
    {
        UART_SendString("  ERROR: Bus busy\r\n");
        return false;
    }

    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (3 << 16); // 3 bytes
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Send 0x3E
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = 0x3E;

    // Send address LSB
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = addr_lsb;

    // Send address MSB
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = addr_msb;

    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return false;

    for (volatile int i = 0; i < 5000; i++);

    // Step 2: Write data to register 0x40 (MACData)

    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (2 << 16); // 2 bytes
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Send 0x40
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = 0x40;

    // Send data value
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = value;

    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return false;

    for (volatile int i = 0; i < 5000; i++);

    // Step 3: Calculate checksum (CORRECT METHOD from SLUA790)
    // Checksum = inverse of last byte of (addr_msb + addr_lsb + value)
    uint16_t sum = addr_msb + addr_lsb + value;
    uint8_t checksum = ~((uint8_t)(sum & 0xFF));

    UART_SendString("  Address LSB: 0x");
    UART_SendHex(addr_lsb);
    UART_SendString(", MSB: 0x");
    UART_SendHex(addr_msb);
    UART_SendString("\r\n");
    UART_SendString("  Sum: 0x");
    UART_SendHex(sum);
    UART_SendString(", Checksum: 0x");
    UART_SendHex(checksum);
    UART_SendString("\r\n");

    // Write checksum to register 0x60

    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (2 << 16);
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Send 0x60
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = 0x60;

    // Send checksum
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = checksum;

    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return false;

    for (volatile int i = 0; i < 5000; i++);

    // Step 4: Write length to register 0x61
    // Length = 4 + number of data bytes = 4 + 1 = 5

    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)BQ35100_I2C_ADDRESS << 1);
    I2C1_CR2 |= (2 << 16);
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Send 0x61
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = 0x61;

    // Send length (5)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return false;
    I2C1_TXDR = 0x05;

    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return false;

    // Wait for flash write
    for (volatile int i = 0; i < 200000; i++);

    UART_SendString("  Write sequence complete\r\n");

    return true;
}

void BQ35100_I2C1_ConfigureEOSMode(void)
{
    UART_SendString("\r\n=== Configuring BQ35100 for EOS Mode ===\r\n");

    // Check if device is sealed
    if (BQ35100_I2C1_IsSealed())
    {
        UART_SendString("Device is SEALED. Unsealing...\r\n");
        if (!BQ35100_I2C1_Unseal())
        {
            UART_SendString("[ERROR] Failed to unseal device!\r\n");
            UART_SendString("Cannot write to data flash while sealed.\r\n\r\n");
            return;
        }
    }
    else
    {
        UART_SendString("Device is already UNSEALED.\r\n");
    }

    // Read current configuration
    uint8_t current_config = BQ35100_I2C1_ReadOperationConfigA();
    UART_SendString("Current Operation Config A: 0x");
    UART_SendHex(current_config);
    UART_SendString("\r\n");

    // Check if already configured correctly
    if ((current_config & 0x03) == 0x02)
    {
        UART_SendString("Already configured for EOS mode!\r\n\r\n");
        return;
    }

    // Set bits 1:0 to 10 (EOS mode) while preserving other bits
    uint8_t new_config = (current_config & 0xFC) | 0x02;

    UART_SendString("New Operation Config A: 0x");
    UART_SendHex(new_config);
    UART_SendString("\r\n");

    // Write to data flash (using corrected function with proper checksum)
    UART_SendString("Writing to data flash...\r\n");
    if (BQ35100_I2C1_WriteDataFlashByte(BQ35100_OP_CONFIG_A_ADDR, new_config))
    {
        UART_SendString("[OK] Configuration written successfully!\r\n");

        // Wait for write to complete
        for (volatile int i = 0; i < 500000; i++);

        UART_SendString("Configuration saved to flash.\r\n");
    }
    else
    {
        UART_SendString("[ERROR] Failed to write configuration!\r\n");
    }

    UART_SendString("\r\n");
}

/* ==================== Comprehensive EOS Monitoring Test ==================== */

void BQ35100_I2C1_EOSMonitoringTest(void)
{
    UART_SendString("\r\n=== BQ35100 EOS Mode Monitoring Test ===\r\n\r\n");

    // 1. Check if device is present
    if (!BQ35100_I2C1_IsPresent())
    {
        UART_SendString("ERROR: BQ35100 not detected!\r\n");
        return;
    }
    UART_SendString("[OK] BQ35100 detected\r\n");

    // 2. Read and verify gauging mode
    uint8_t mode = BQ35100_I2C1_ReadGaugingMode();
    UART_SendString("\nGauging Mode: ");
    switch(mode)
    {
        case 0:
            UART_SendString("ACCUMULATOR (00)\r\n");
            UART_SendString("*** WARNING: Not in EOS mode! ***\r\n");
            break;
        case 1:
            UART_SendString("STATE-OF-HEALTH (01)\r\n");
            UART_SendString("*** WARNING: This is for Li-MnO2, not Li-SOCl2! ***\r\n");
            break;
        case 2:
            UART_SendString("END-OF-SERVICE (10) - CORRECT for Li-SOCl2\r\n");
            break;
        case 3:
            UART_SendString("INVALID (11)\r\n");
            break;
    }

    // 3. Check if gauge is active
    bool active = BQ35100_I2C1_IsGaugeActive();
    UART_SendString("Gauge Status: ");
    if (active)
    {
        UART_SendString("ACTIVE (collecting data)\r\n");
    }
    else
    {
        UART_SendString("IDLE (not collecting data)\r\n");
        UART_SendString("*** Call GAUGE_START() to begin measurements ***\r\n");
    }

    // 4. Read current measurements
    UART_SendString("\n--- Current Measurements ---\r\n");

    uint16_t voltage = BQ35100_I2C1_ReadVoltage();
    UART_SendString("Voltage: ");
    UART_SendNumber(voltage);
    UART_SendString(" mV\r\n");

    int16_t current = BQ35100_I2C1_ReadCurrent();
    UART_SendString("Current: ");
    if (current < 0)
    {
        UART_SendString("-");
        current = -current;
    }
    UART_SendNumber(current);
    UART_SendString(" mA\r\n");

    uint16_t scaledR = BQ35100_I2C1_ReadScaledR();
    UART_SendString("Scaled R: ");
    UART_SendNumber(scaledR);
    UART_SendString(" mOhm\r\n");

    uint16_t impedance = BQ35100_I2C1_ReadMeasuredZ();
    UART_SendString("Measured Z: ");
    UART_SendNumber(impedance);
    UART_SendString(" mOhm\r\n");

    // 5. Read EOS tracking data
    UART_SendString("\n--- EOS Tracking Data ---\r\n");

    uint16_t pulseCount = BQ35100_I2C1_ReadEOSPulseCount();
    UART_SendString("EOS Pulse Count: ");
    UART_SendNumber(pulseCount);
    UART_SendString(" / 120 (threshold)\r\n");

    if (pulseCount < 120)
    {
        UART_SendString("  -> Need ");
        UART_SendNumber(120 - pulseCount);
        UART_SendString(" more pulses before EOS detection active\r\n");
    }

    uint32_t shortTrend = BQ35100_I2C1_ReadShortTrendAverage();
    UART_SendString("Short Trend Average: ");
    UART_SendNumber(shortTrend);
    UART_SendString("\r\n");

    uint32_t longTrend = BQ35100_I2C1_ReadLongTrendAverage();
    UART_SendString("Long Trend Average: ");
    UART_SendNumber(longTrend);
    UART_SendString("\r\n");

    // Calculate trend ratio if both are non-zero
    if (longTrend > 0 && shortTrend > 0)
    {
        // Calculate percentage difference
        // (shortTrend - longTrend) / longTrend * 100
        int32_t diff = (int32_t)shortTrend - (int32_t)longTrend;
        int32_t percent = (diff * 100) / (int32_t)longTrend;

        UART_SendString("Trend Difference: ");
        if (percent < 0)
        {
            UART_SendString("-");
            percent = -percent;
        }
        UART_SendNumber(percent);
        UART_SendString("% (EOS triggers at +20%)\r\n");

        if (percent >= 20)
        {
            UART_SendString("*** SHORT TREND IS 20%+ HIGHER - EOS SHOULD BE FLAGGED! ***\r\n");
        }
    }
    else
    {
        UART_SendString("  -> Trends not yet initialized (need more measurements)\r\n");
    }

    // 6. Check EOS flags
    UART_SendString("\n--- EOS Detection Status ---\r\n");

    uint16_t battStatus = BQ35100_I2C1_ReadBatteryStatus();
    bool eosFlag = BQ35100_I2C1_IsEOSFlagSet();

    UART_SendString("Battery Status Register: 0x");
    UART_SendHex(battStatus);
    UART_SendString("\r\n");

    UART_SendString("EOS Flag: ");
    if (eosFlag)
    {
        UART_SendString("SET - END-OF-SERVICE DETECTED!\r\n");
    }
    else
    {
        UART_SendString("Not set\r\n");
    }

    uint8_t battAlert = BQ35100_I2C1_ReadBatteryAlert();
    UART_SendString("Battery Alert Register: 0x");
    UART_SendHex(battAlert);
    UART_SendString("\r\n");

    // 7. State of Health
    uint16_t soh = BQ35100_I2C1_ReadStateOfHealth();
    UART_SendString("State of Health: ");
    UART_SendNumber(soh);
    UART_SendString("%\r\n");

    // 8. Temperature
    uint16_t temp = BQ35100_I2C1_ReadRawTemp();
    int32_t temp_c = BQ35100_I2C1_ConvertToCelsius_Int(temp);
    UART_SendString("Temperature: ");
    UART_SendNumber(temp_c / 100);
    UART_SendString(".");
    int32_t decimal = temp_c % 100;
    if (decimal < 0) decimal = -decimal;
    if (decimal < 10) UART_SendString("0");
    UART_SendNumber(decimal);
    UART_SendString(" C\r\n");

    UART_SendString("\n=== End of EOS Monitoring Test ===\r\n\r\n");
}

/**
 * @brief Perform a single EOS measurement cycle
 *
 * This simulates one periodic measurement in your application.
 * Call this function periodically (e.g., every 24 hours) to build
 * up impedance trend data.
 */
void BQ35100_I2C1_PerformEOSMeasurement(void) // Simulates one periodic measurement
{
    UART_SendString("\r\n--- Starting EOS Measurement Cycle ---\r\n");

    // Send GAUGE_START
    BQ35100_I2C1_GaugeStart();

    // Wait for measurement to complete
    // In real application, this should be during your load pulse
    for(volatile uint32_t i = 0; i < 16000000; i++); // ~1 second at 16MHz

    // Send GAUGE_STOP
    BQ35100_I2C1_GaugeStop();

    // Wait for G_DONE flag
    uint32_t timeout = 500;
    while (timeout-- && !BQ35100_I2C1_IsGaugeDone())
    {
        for(volatile uint32_t i = 0; i < 100000; i++); // ~6ms delay
    }

    if (BQ35100_I2C1_IsGaugeDone())
    {
        UART_SendString("[OK] Measurement complete (G_DONE set)\r\n");
    }
    else
    {
        UART_SendString("[WARNING] G_DONE not set - may need longer delay\r\n");
    }

    // Read and display updated values
    uint16_t impedance = BQ35100_I2C1_ReadMeasuredZ();
    uint16_t pulseCount = BQ35100_I2C1_ReadEOSPulseCount();

    UART_SendString("Impedance: ");
    UART_SendNumber(impedance);
    //UART_SendString(" mOhm \r\n");
    UART_SendString(" mOhm, Pulse Count: ");
    UART_SendNumber(pulseCount);
    UART_SendString("\r\n");

    UART_SendString("--- Measurement Cycle Complete ---\r\n\r\n");
}

/* ==================== R Table Scale Functions ==================== */

int16_t BQ35100_I2C1_ReadRTableScale(void)
{
    /*
     * Reads the R Table Scale from data flash
     * Address: 0x4257 (2 bytes, signed integer)
     * Default: -1 (temperature compensation disabled)
     * If > 0: Temperature compensation is enabled with that scaling factor
     */
    uint8_t lsb = BQ35100_I2C1_ReadDataFlashByte(0x4257);
    uint8_t msb = BQ35100_I2C1_ReadDataFlashByte(0x4258);

    // Combine as signed 16-bit integer
    return (int16_t)((msb << 8) | lsb);
}

void BQ35100_I2C1_CheckTemperatureCompensation(void)
{
    UART_SendString("\r\n=== Temperature Compensation Check ===\r\n");

    // Read CHEM ID
    uint16_t chem_id = BQ35100_I2C1_ReadChemID();
    UART_SendString("CHEM ID: 0x");
    UART_SendHex(chem_id);
    UART_SendString("\r\n");

    // Read R Table Scale
    int16_t r_table_scale = BQ35100_I2C1_ReadRTableScale();
    UART_SendString("R Table Scale: ");

    if (r_table_scale == -1)
    {
        UART_SendString("-1 (DISABLED)\r\n");
        UART_SendString("\r\nTemperature compensation is NOT active.\r\n");
        UART_SendString("MeasuredZ = ScaledR (no temperature correction)\r\n\r\n");
        UART_SendString("Why?\r\n");
        UART_SendString("  - R Table Scale = -1 means not calibrated\r\n");
        UART_SendString("  - Ra tables exist but aren't being used\r\n");
        UART_SendString("  - BQ Studio calibration required to enable\r\n\r\n");
        UART_SendString("Impact:\r\n");
        UART_SendString("  - Impedance readings won't compensate for temperature\r\n");
        UART_SendString("  - Cold temps will show falsely high impedance\r\n");
        UART_SendString("  - May trigger premature EOS warnings in freezing conditions\r\n");
    }
    else
    {
        UART_SendNumber(r_table_scale);
        UART_SendString(" (ENABLED)\r\n");
        UART_SendString("\r\nTemperature compensation IS active!\r\n");
        UART_SendString("ScaledR will be temperature-corrected.\r\n");
    }

    UART_SendString("\r\n=== Check Complete ===\r\n\r\n");
}

uint16_t BQ35100_I2C1_ReadChemID(void)
{
    /*
     * Reads the CHEM_ID (chemical identifier)
     * Shows which chemistry profile is loaded
     */

    // Write control command to read CHEM_ID (command 0x0006)
    if (!BQ35100_I2C1_WriteCommand(BQ35100_CONTROL_CMD, 0x0006))
    {
        return 0;
    }

    // Small delay for device to process
    for (volatile int i = 0; i < 5000; i++);

    // Read result from control registers
    return BQ35100_I2C1_ReadWord(BQ35100_CONTROL_CMD);
}

/* ==================== Test Functions ==================== */

void BQ35100_I2C1_DiagnosticTest(void)
{
    UART_SendString("\r\n=== BQ35100 Diagnostic Test ===\r\n");

    // Read both temperature sources
    uint16_t temp_reading = BQ35100_I2C1_ReadRawTemp();
    uint16_t internal_temp = BQ35100_I2C1_ReadInternalTemp();

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

    // Compare the two
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
    }

    // Try to heat/cool thermistor and check if reading changes
    UART_SendString("\r\nTry heating the thermistor with your finger.\r\n");
    UART_SendString("The Temperature() reading should increase if external thermistor is working.\r\n");
    UART_SendString("\r\n");
}

void BQ35100_I2C1_Test(void) // need to change float read to int
{
    UART_SendString("=== BQ35100 Temperature Test ===\r\n");

    // Check if BQ35100 is present
    if (!BQ35100_I2C1_IsPresent())
    {
        UART_SendString("ERROR: BQ35100 not detected at address 0x55\r\n");
        return;
    }

    UART_SendString("BQ35100 detected successfully!\r\n");

    // Read device type
    uint16_t device_type = BQ35100_I2C1_ReadDeviceType();
    UART_SendString("Device Type: 0x");
    UART_SendHex(device_type);
    UART_SendString(" (should be 0x0100 for BQ35100)\r\n");

    // Read raw temperature
    uint16_t raw_temp = BQ35100_I2C1_ReadRawTemp();
    UART_SendString("Raw temperature value: ");
    UART_SendNumber(raw_temp);
    UART_SendString(" (0.1°K units)\r\n");

    // Convert to Celsius
    float temp_c = BQ35100_ConvertToCelsius_Int(raw_temp);

    // Display temperature
    int temp_whole = (int)temp_c;
    int temp_decimal = (int)((temp_c - temp_whole) * 100);
    if (temp_decimal < 0) temp_decimal = -temp_decimal;

    UART_SendString("Temperature: ");
    if (temp_c < 0 && temp_whole == 0)
    {
        UART_SendString("-");
    }
    UART_SendNumber(temp_whole);
    UART_SendString(".");
    if (temp_decimal < 10) UART_SendString("0");
    UART_SendNumber(temp_decimal);
    UART_SendString(" °C\r\n");

    // Read voltage
    uint16_t voltage = BQ35100_I2C1_ReadVoltage();
    UART_SendString("Battery Voltage: ");
    UART_SendNumber(voltage);
    UART_SendString(" mV\r\n");

    UART_SendString("\r\n");
}

void BQ35100_I2C1_HardwareTest(void)
{
    UART_SendString("\r\n=== BQ35100 Hardware Connection Test ===\r\n");

    UART_SendString("\r\nReading temperatures over 10 seconds...\r\n");
    UART_SendString("Please HEAT the thermistor with your finger NOW!\r\n\r\n");

    uint16_t initial_temp = BQ35100_I2C1_ReadRawTemp();
    uint16_t initial_int = BQ35100_I2C1_ReadInternalTemp();

    UART_SendString("Initial readings:\r\n");
    UART_SendString("  Temperature():     ");
    UART_SendNumber(initial_temp);
    UART_SendString(" raw\r\n");
    UART_SendString("  InternalTemp():    ");
    UART_SendNumber(initial_int);
    UART_SendString(" raw\r\n\r\n");

    UART_SendString("Monitoring for 10 seconds...\r\n");

    uint16_t max_temp = initial_temp;
    uint16_t min_temp = initial_temp;
    uint16_t max_int = initial_int;
    uint16_t min_int = initial_int;

    for (int i = 0; i < 10; i++)
    {
        // Delay 1 second
        for(volatile uint32_t j = 0; j < 1000000; j++);

        uint16_t temp = BQ35100_I2C1_ReadRawTemp();
        uint16_t int_temp = BQ35100_I2C1_ReadInternalTemp();

        if (temp > max_temp) max_temp = temp;
        if (temp < min_temp) min_temp = temp;
        if (int_temp > max_int) max_int = int_temp;
        if (int_temp < min_int) min_int = int_temp;

        UART_SendString("  [");
        UART_SendNumber(i + 1);
        UART_SendString("s] Temp: ");
        UART_SendNumber(temp);
        UART_SendString(", Internal: ");
        UART_SendNumber(int_temp);
        UART_SendString("\r\n");
    }

    UART_SendString("\r\nResults:\r\n");
    UART_SendString("Temperature() range: ");
    UART_SendNumber(min_temp);
    UART_SendString(" to ");
    UART_SendNumber(max_temp);
    UART_SendString(" (change: ");
    UART_SendNumber(max_temp - min_temp);
    UART_SendString(")\r\n");

    UART_SendString("InternalTemp() range: ");
    UART_SendNumber(min_int);
    UART_SendString(" to ");
    UART_SendNumber(max_int);
    UART_SendString(" (change: ");
    UART_SendNumber(max_int - min_int);
    UART_SendString(")\r\n\r\n");

    // Analyze results
    uint16_t temp_change = max_temp - min_temp;
    uint16_t int_change = max_int - min_int;

    UART_SendString("Analysis:\r\n");

    if (temp_change < 3)
    {
        UART_SendString("*** PROBLEM: Temperature() didn't change! ***\r\n");
        UART_SendString("Possible causes:\r\n");
        UART_SendString("  1. Thermistor not connected to REG25 and TS pins\r\n");
        UART_SendString("  2. Open circuit in thermistor\r\n");
        UART_SendString("  3. REG25 not providing bias voltage\r\n");
        UART_SendString("  4. Gauge not in ACTIVE mode\r\n");
        UART_SendString("\r\nCheck with multimeter:\r\n");
        UART_SendString("  - Resistance across thermistor: should be ~10k ohm\r\n");
        UART_SendString("  - Voltage on REG25 pin: should be ~2.5V\r\n");
        UART_SendString("  - Voltage on TS pin: should change when heating thermistor\r\n");
    }
    else if (temp_change < 10)
    {
        UART_SendString("Temperature() changed slightly (");
        UART_SendNumber(temp_change);
        UART_SendString(" units = ");
        int32_t change_c = ((int32_t)temp_change * 10) / 100;
        UART_SendNumber(change_c);
        UART_SendString(".");
        UART_SendNumber(((int32_t)temp_change * 10) % 100);
        UART_SendString("C)\r\n");
        UART_SendString("This is a weak response. Check:\r\n");
        UART_SendString("  1. Is thermistor making good thermal contact?\r\n");
        UART_SendString("  2. Heat it more aggressively (hold finger firmly)\r\n");
    }
    else
    {
        UART_SendString("Temperature() changed significantly! (");
        UART_SendNumber(temp_change);
        UART_SendString(" units = ");
        int32_t change_c = ((int32_t)temp_change * 10) / 100;
        UART_SendNumber(change_c);
        UART_SendString(".");
        UART_SendNumber(((int32_t)temp_change * 10) % 100);
        UART_SendString("C)\r\n");
        UART_SendString("*** External thermistor is working correctly! ***\r\n");
    }

    UART_SendString("\r\n");
}


