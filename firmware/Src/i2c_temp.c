/*
 * i2c_temp.c
 *
 *  Created on: Sep 29, 2025
 *      Author: rachaelguise-brown
 *
 * 		Description: I2C implementation for STM32L4
 * 		Using I2C1 peripheral
 * 		Pins on board:
 * 		SCL = PB8
 * 		SDA = PB9
 */


#include "i2c_temp.h"
#include "clock_config.h"
#include "uart.h"

// Private helper functions

static void GPIOB_EnableClock(void)
{
    RCC_AHB2ENR |= (1U << 1); // Enable GPIOB clock
}

static void I2C1_EnableClock(void)
{
    RCC_APB1ENR1 |= (1U << 21); // Enable I2C1 clock

    // Configure I2C1 clock source to PCLK1 (from reference manual)
    // Clear I2C1SEL bits first
    RCC_CCIPR &= ~(3U << 12);
    // Set I2C1SEL to PCLK1 (00)
    RCC_CCIPR |= (0U << 12);
}

static void GPIO_ConfigureI2C1Pins(void)
{
    // Configure PB8 (SCL) as alternate function
    GPIOB_MODER &= ~(3U << 16); // Clear PB8 mode bits (bits 16-17)
    GPIOB_MODER |= (2U << 16);  // Set PB8 as alternate function

    // Configure PB9 (SDA) as alternate function
    GPIOB_MODER &= ~(3U << 18); // Clear PB9 mode bits (bits 18-19)
    GPIOB_MODER |= (2U << 18);  // Set PB9 as alternate function

    // Set both pins to open drain (required for I2C)
    GPIOB_OTYPER |= (1U << 8);  // Set PB8 to open drain
    GPIOB_OTYPER |= (1U << 9);  // Set PB9 to open drain

    // Set pull-up resistors
    GPIOB_PUPDR &= ~(3U << 16); // Clear PB8 pull settings
    GPIOB_PUPDR |= (1U << 16);  // Set PB8 pull-up
    GPIOB_PUPDR &= ~(3U << 18); // Clear PB9 pull settings
    GPIOB_PUPDR |= (1U << 18);  // Set PB9 pull-up

    // Set high speed
    GPIOB_OSPEEDR |= (3U << 16); // Set PB8 to high speed
    GPIOB_OSPEEDR |= (3U << 18); // Set PB9 to high speed

    // Set PB8 to AF4 (I2C1_SCL) - PB8 uses AFRH register (bit 0-3)
    GPIOB_AFRH &= ~(0xF << 0);      // Clear AF bits for PB8
    GPIOB_AFRH |= (GPIO_AF4 << 0);  // Set AF4 for PB8

    // Set PB9 to AF4 (I2C1_SDA) - PB9 uses AFRH register (bit 4-7)
    GPIOB_AFRH &= ~(0xF << 4);      // Clear AF bits for PB9
    GPIOB_AFRH |= (GPIO_AF4 << 4);  // Set AF4 for PB9
}

static void I2C1_Configure(void)
{
    // Disable I2C1 before configuration
    I2C1_CR1 &= ~I2C_CR1_PE;

    // Wait for PE=0 to be confirmed (from reference manual)
    uint32_t timeout = 10000;
    while ((I2C1_CR1 & I2C_CR1_PE) && timeout--);

    if (timeout == 0)
    {
        // Could add error handling here
        return;
    }

    // Configure timing register (must be done when PE=0)
    // For 16MHz PCLK and 100kHz I2C (from reference manual Table 343)
    // PRESC=1, SCLDEL=4, SDADEL=2, SCLH=0xF, SCLL=0x13
    I2C1_TIMINGR = 0x00303D5B; // This matches the manual's 16MHz/100kHz example

    // Configure filters (must be done when PE=0)
    uint32_t cr1_config = 0;
    // Keep analog filter enabled (ANFOFF=0)
    // Set digital filter to 0 (DNF=0000)
    I2C1_CR1 = cr1_config;

    // Enable I2C1
    I2C1_CR1 |= I2C_CR1_PE;

    // Wait a bit for enable to take effect
    for (volatile int i = 0; i < 1000; i++);
}

// Public functions

void I2C_Init(void)
{
    // Enable clocks
    GPIOB_EnableClock();
    I2C1_EnableClock();

    // Configure GPIO pins
    GPIO_ConfigureI2C1Pins();

    // Configure I2C
    I2C1_Configure();
}

bool I2C_IsReady(void)
{
    // Check if I2C is enabled and not busy
    return ((I2C1_CR1 & I2C_CR1_PE) != 0) && ((I2C1_ISR & I2C_ISR_BUSY) == 0);
}

void I2C_SendByte(uint8_t data)
{
    // Wait until transmit register is empty
    while (!(I2C1_ISR & I2C_ISR_TXE));

    // Send the byte
    I2C1_TXDR = data;
}

bool I2C_ScanAddress(uint8_t address)
{
    // Wait for bus to be free
    uint32_t timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return false;

    // Clear any previous flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Configure transfer: 7-bit address, 0 bytes, auto-end
    I2C1_CR2 = 0; // Clear CR2
    I2C1_CR2 |= ((uint32_t)address << 1); // Set 7-bit address (shifted left)
    I2C1_CR2 |= (1 << 16); // 1 bytes to transfer (NBYTES)
    I2C1_CR2 |= I2C_CR2_AUTOEND; // Auto-end after transfer

    // Start the transfer
    I2C1_CR2 |= I2C_CR2_START;

    // Wait for TXIS (transmit ready) or NACK
    timeout = 10000;
    while (!(I2C1_ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)) && timeout--);
    if (timeout == 0) return false;

    if (I2C1_ISR & I2C_ISR_NACKF)
    {
        I2C1_ICR = I2C_ICR_NACKCF; // Clear NACK
        return false;
    }

    // Write dummy byte
    I2C1_TXDR = 0x00;

    // Wait for transfer complete or NACK
    timeout = 10000;
    while (!(I2C1_ISR & (I2C_ISR_TC | I2C_ISR_NACKF)) && timeout--);

    if (timeout == 0)
    {
        return false;
    }

    // Check if we got a NACK (device not present)
    if (I2C1_ISR & I2C_ISR_NACKF)
    {
        I2C1_ICR = I2C_ICR_NACKCF; // Clear NACK flag
        return false;
    }

    // Device responded (ACK received)
    return true;
}

void I2C_ScanBus(void)
{

    UART_SendString("Starting I2C bus scan...\r\n");

    uint8_t devices_found = 0;

    // Scan addresses 0x08 to 0x77 (valid 7-bit I2C addresses)
    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        if (I2C_ScanAddress(addr))
        {
            UART_SendString("Device found at address: 0x");
            // Convert to hex string manually
            uint8_t high_nibble = (addr >> 4) & 0x0F;
            uint8_t low_nibble = addr & 0x0F;

            // Send high nibble
            if (high_nibble < 10)
            {
                UART_SendChar('0' + high_nibble);
            } else
            {
                UART_SendChar('A' + (high_nibble - 10));
            }

            // Send low nibble
            if (low_nibble < 10)
            {
                UART_SendChar('0' + low_nibble);
            } else
            {
                UART_SendChar('A' + (low_nibble - 10));
            }

            UART_SendString(" (decimal: ");
            UART_SendNumber(addr);
            UART_SendString(")\r\n");
            devices_found++;
        }

        // Small delay between scans
        for (volatile int i = 0; i < 1000; i++);
    }

    UART_SendString("Scan complete. Found ");
    UART_SendNumber(devices_found);
    UART_SendString(" device(s)\r\n\r\n");
}

void I2C_ScanTMP117(void)
{

    UART_SendString("=== TMP117 Specific Scan ===\r\n");
    UART_SendString("Checking all possible TMP117 addresses...\r\n");

    // TMP117 possible addresses based on ADD0 pin configuration
    uint8_t tmp117_addresses[] = {0x48, 0x49, 0x4A, 0x4B};
    bool found = false;

    for (int i = 0; i < 4; i++)
    {
        uint8_t addr = tmp117_addresses[i];
        UART_SendString("Testing address 0x");

        // Send hex address
        uint8_t high_nibble = (addr >> 4) & 0x0F;
        uint8_t low_nibble = addr & 0x0F;

        if (high_nibble < 10)
        {
            UART_SendChar('0' + high_nibble);
        } else
        {
            UART_SendChar('A' + (high_nibble - 10));
        }

        if (low_nibble < 10)
        {
            UART_SendChar('0' + low_nibble);
        } else
        {
            UART_SendChar('A' + (low_nibble - 10));
        }

        UART_SendString("... ");

        if (I2C_ScanAddress(addr)) // NO SHIFTING
        {
            UART_SendString("TMP117 FOUND!\r\n");
            found = true;
        } else
        {
            UART_SendString("No response\r\n");
        }

        // Small delay
        for (volatile int j = 0; j < 5000; j++);
    }

    if (!found)
    {
        UART_SendString("\r\nTMP117 NOT DETECTED - Check:\r\n");
    }

    UART_SendString("\r\n");
}

/*
 * @brief
 * TMP117 specific functions including:
 *  _IsPresent()
 *  _ReadRawTemp()
 *  _ReadTemp()
 *  _ConvertToTempC()
 *  _Test()
 */



bool TMP117_IsPresent(void)
{
    return I2C_ScanAddress(TMP117_I2C_ADDRESS);
}

uint16_t TMP117_ReadRawTemp(void)
{

    //UART_SendString("Temp read start...\r\n");

    // Add a small delay before starting
    for(volatile int i = 0; i < 100000; i++);

    // Read 2 bytes from TMP117 temperature register (0x00)
    uint16_t temp_raw = 0;
    uint32_t timeout;

    //UART_SendString("Checking bus status...\r\n");
    //UART_SendString("I2C1_ISR = 0x");
    //UART_SendNumber(I2C1_ISR);
    //UART_SendString("\r\n");

    // Wait for bus to be free
    //UART_SendString("Waiting for bus free...\r\n");
    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) {
        //UART_SendString("ERROR: Bus busy timeout\r\n");
    	return 0;
    }
    //UART_SendString("Bus is free\r\n");

    // Clear any previous flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    //UART_SendString("Flags cleared\r\n");

    // Step 1: Write register address
    //UART_SendString("Setting up write transaction...\r\n");
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)TMP117_I2C_ADDRESS << 1); // Device address
    I2C1_CR2 |= (1 << 16); // 1 byte to write (register address)
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    //UART_SendString("Starting write...\r\n");

    // Wait for TXIS and send register address
    //UART_SendString("Waiting for TXIS...\r\n");
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0;

    I2C1_TXDR = TMP117_TEMP_REGISTER; // Register 0x00 (temperature)

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) {
        //UART_SendString("ERROR: TXIS timeout\r\n");
        //UART_SendString("I2C1_ISR = 0x");
        //UART_SendNumber(I2C1_ISR);
        //UART_SendString("\r\n");
    	return 0;
    }

    //UART_SendString("TXIS ready\r\n");

    I2C1_TXDR = TMP117_TEMP_REGISTER;
    //UART_SendString("Register address sent\r\n");

    // Wait for transfer complete
    //UART_SendString("Waiting for TC...\r\n");
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) {
        //UART_SendString("ERROR: TC timeout\r\n");
        return 0;
    }
    //UART_SendString("Write complete\r\n");

    // Step 2: Read 2 bytes from device
    //UART_SendString("Setting up read transaction...\r\n");
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)TMP117_I2C_ADDRESS << 1);
    I2C1_CR2 |= I2C_CR2_RD_WRN; // Read mode
    I2C1_CR2 |= (2 << 16); // 2 bytes to read
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    //UART_SendString("Starting read...\r\n");

    // Read first byte (MSB)
    //UART_SendString("Waiting for first RXNE...\r\n");
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) {
        //UART_SendString("ERROR: First RXNE timeout\r\n");
        return 0;
    }

    uint8_t msb = I2C1_RXDR;
    //UART_SendString("MSB read: 0x");
    //UART_SendNumber(msb);
    //UART_SendString("\r\n");

    // Read second byte (LSB)
    //UART_SendString("Waiting for second RXNE...\r\n");
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) {
        //UART_SendString("ERROR: Second RXNE timeout\r\n");
        return 0;
    }

    uint8_t lsb = I2C1_RXDR;
    //UART_SendString("LSB read: 0x");
    //UART_SendNumber(lsb);
    //UART_SendString("\r\n");

    // Combine MSB and LSB
    temp_raw = (msb << 8) | lsb;
    //UART_SendString("Temperature read complete!\r\n");

    return temp_raw;
}

int32_t TMP117_ConvertToTempC(uint16_t raw_temp)
{

    // TMP117 temperature calculation:
    // Temperature = (raw_value * 7.8125) / 1000 degrees Celsius
    // Resolution is 0.0078125°C per LSB

    int16_t signed_temp = (int16_t)raw_temp; // interpret as signed 16-bit

    // Scale into hundredths of a °C
    int32_t temp_c_hundredths = ((int32_t)signed_temp * 78125L) / 1000L;

    return temp_c_hundredths; // e.g. 2534 = 25.34 °C, -5500 = -55.00 °C
}

void TMP117_Test(void)
{
    UART_SendString("=== TMP117 Temperature Reading Test ===\r\n");

    // Check if TMP117 is present
    if (!TMP117_IsPresent()) {
        UART_SendString("ERROR: TMP117 not detected at address 0x48\r\n");
        return;
    }

    UART_SendString("TMP117 detected successfully!\r\n");

    // Read raw temperature
    uint16_t raw_temp = TMP117_ReadRawTemp();
    UART_SendString("Raw temperature value: 0x");
    UART_SendNumber(raw_temp);
    UART_SendString(" (decimal: ");
    UART_SendNumber(raw_temp);
    UART_SendString(")\r\n");

    // Read temperature in Celsius
    float temp_c = TMP117_ConvertToTempC(raw_temp);

    // Convert float to integer parts for display (since we don't have printf)
    int temp_whole = (int)temp_c;
    int temp_decimal = (int)((temp_c - temp_whole) * 100);
    if (temp_decimal < 0) temp_decimal = -temp_decimal; // Handle negative temperatures

    UART_SendString("Temperature: ");
    if (temp_c < 0 && temp_whole == 0) {
        UART_SendString("-"); // Handle -0.xx case
    }
    UART_SendNumber(temp_whole);
    UART_SendString(".");
    if (temp_decimal < 10) UART_SendString("0"); // Leading zero for decimals < 10
    UART_SendNumber(temp_decimal);
    UART_SendString(" C\r\n\r\n");
}

/*
 * @brief Read TMP117 Device ID register to verify device
 * @return Device ID value (should be 0x0117 for TMP117)
 */
uint16_t TMP117_ReadDeviceID(void)
{
    uint16_t device_id = 0;
    uint32_t timeout;

    // Wait for bus to be free
    timeout = 10000;
    while ((I2C1_ISR & I2C_ISR_BUSY) && timeout--);
    if (timeout == 0) return 0;

    // Clear any previous flags
    I2C1_ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // Step 1: Write register address (0x0F for Device ID)
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)TMP117_I2C_ADDRESS << 1); // Device address
    I2C1_CR2 |= (1 << 16); // 1 byte to write (register address)
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Wait for TXIS and send register address
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TXIS) && timeout--);
    if (timeout == 0) return 0;

    I2C1_TXDR = TMP117_DEVICE_ID_REGISTER; // Register 0x0F (Device ID)

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);
    if (timeout == 0) return 0;

    // Step 2: Read 2 bytes from device
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)TMP117_I2C_ADDRESS << 1); // Device address
    I2C1_CR2 |= I2C_CR2_RD_WRN; // Read mode
    I2C1_CR2 |= (2 << 16); // 2 bytes to read
    I2C1_CR2 |= I2C_CR2_AUTOEND;
    I2C1_CR2 |= I2C_CR2_START;

    // Read first byte (MSB)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0;

    uint8_t msb = I2C1_RXDR;

    // Read second byte (LSB)
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_RXNE) && timeout--);
    if (timeout == 0) return 0;

    uint8_t lsb = I2C1_RXDR;

    // Wait for transfer complete
    timeout = 10000;
    while (!(I2C1_ISR & I2C_ISR_TC) && timeout--);

    // Combine MSB and LSB
    device_id = (msb << 8) | lsb;

    return device_id;
}

/*
 * @brief
 * Testing I2C Bus Lines, checking GPIO Configuration and printing out results
 * _TestBusLines()
 * _VerifyGPIOConfig()
 *
 */

void I2C_TestBusLines(void)
{

    UART_SendString("=== I2C Bus Line Test ===\r\n");

    // First, temporarily disable I2C to read GPIO states
    I2C1_CR1 &= ~I2C_CR1_PE;

    // Configure PB8 and PB9 as inputs temporarily to read line states
    uint32_t saved_moder = GPIOB_MODER;
    GPIOB_MODER &= ~(0xF << 16); // Clear PB8 and PB9 to input mode

    // Read input data register
    uint32_t gpio_idr = *(volatile uint32_t*)(GPIOB_BASE + 0x10); // IDR offset

    UART_SendString("GPIO Line States (as inputs):\r\n");
    UART_SendString("PB8 (SCL): ");
    if (gpio_idr & (1 << 8)) {
        UART_SendString("HIGH (pulled up) - GOOD\r\n");
    } else
    {
        UART_SendString("LOW - BAD (should be pulled high)\r\n");
    }

    UART_SendString("PB9 (SDA): ");
    if (gpio_idr & (1 << 9)) {
        UART_SendString("HIGH (pulled up) - GOOD\r\n");
    } else
    {
        UART_SendString("LOW - BAD (should be pulled high)\r\n");
    }

    // Restore GPIO configuration
    GPIOB_MODER = saved_moder;

    // Re-enable I2C
    I2C1_CR1 |= I2C_CR1_PE;

    UART_SendString("\r\nDiagnostic suggestions:\r\n");
    UART_SendString("- Both lines should read HIGH when no communication\r\n");
    UART_SendString("- If LOW: check pull-up resistors or short circuits\r\n");
    UART_SendString("- Verify TMP117 is powered and not holding lines low\r\n");
    UART_SendString("\r\n");
}

void I2C_VerifyGPIOConfig(void)
{

    UART_SendString("=== GPIO Configuration Verification ===\r\n");

    // Check clock enables
    UART_SendString("Clock Status:\r\n");
    UART_SendString("GPIOB Clock: ");
    if (RCC_AHB2ENR & (1U << 1)) {
        UART_SendString("ENABLED\r\n");
    } else
    {
        UART_SendString("DISABLED - ERROR!\r\n");
    }

    UART_SendString("I2C1 Clock: ");
    if (RCC_APB1ENR1 & (1U << 21))
    {
        UART_SendString("ENABLED\r\n");
    } else
    {
        UART_SendString("DISABLED - ERROR!\r\n");
    }

    // Detailed GPIO register analysis
    uint32_t moder = GPIOB_MODER;
    uint32_t otyper = GPIOB_OTYPER;
    uint32_t pupdr = GPIOB_PUPDR;
    uint32_t afrh = GPIOB_AFRH;

    UART_SendString("\r\nGPIOB Register Values:\r\n");
    UART_SendString("MODER = 0x");
    UART_SendNumber(moder);
    UART_SendString("\r\n");

    // Check PB8 configuration (bits 16-17 in MODER)
    uint32_t pb8_mode = (moder >> 16) & 0x3;
    UART_SendString("PB8 Mode: ");
    switch (pb8_mode) {
        case 0: UART_SendString("INPUT"); break;
        case 1: UART_SendString("OUTPUT"); break;
        case 2: UART_SendString("ALTERNATE FUNCTION"); break;
        case 3: UART_SendString("ANALOG"); break;
    }
    if (pb8_mode == 2)
    {
        UART_SendString(" - CORRECT\r\n");
    } else
    {
        UART_SendString(" - ERROR! Should be ALTERNATE FUNCTION\r\n");
    }

    // Check PB9 configuration (bits 18-19 in MODER)
    uint32_t pb9_mode = (moder >> 18) & 0x3;
    UART_SendString("PB9 Mode: ");
    switch (pb9_mode)
    {
        case 0: UART_SendString("INPUT"); break;
        case 1: UART_SendString("OUTPUT"); break;
        case 2: UART_SendString("ALTERNATE FUNCTION"); break;
        case 3: UART_SendString("ANALOG"); break;
    }
    if (pb9_mode == 2)
    {
        UART_SendString(" - CORRECT\r\n");
    } else
    {
        UART_SendString(" - ERROR! Should be ALTERNATE FUNCTION\r\n");
    }

    // Check output type (open drain required for I2C)
    UART_SendString("\r\nOutput Type:\r\n");
    UART_SendString("PB8 Output Type: ");
    if (otyper & (1U << 8))
    {
        UART_SendString("OPEN DRAIN - CORRECT\r\n");
    } else
    {
        UART_SendString("PUSH-PULL - ERROR! Should be OPEN DRAIN\r\n");
    }

    UART_SendString("PB9 Output Type: ");
    if (otyper & (1U << 9))
    {
        UART_SendString("OPEN DRAIN - CORRECT\r\n");
    } else
    {
        UART_SendString("PUSH-PULL - ERROR! Should be OPEN DRAIN\r\n");
    }

    // Check pull-up/pull-down
    UART_SendString("\r\nPull resistors:\r\n");
    uint32_t pb8_pupd = (pupdr >> 16) & 0x3;
    uint32_t pb9_pupd = (pupdr >> 18) & 0x3;

    UART_SendString("PB8 Pull: ");
    switch (pb8_pupd)
    {
        case 0: UART_SendString("NONE"); break;
        case 1: UART_SendString("PULL-UP - CORRECT"); break;
        case 2: UART_SendString("PULL-DOWN"); break;
        case 3: UART_SendString("RESERVED"); break;
    }
    UART_SendString("\r\n");

    UART_SendString("PB9 Pull: ");
    switch (pb9_pupd)
    {
        case 0: UART_SendString("NONE"); break;
        case 1: UART_SendString("PULL-UP - CORRECT"); break;
        case 2: UART_SendString("PULL-DOWN"); break;
        case 3: UART_SendString("RESERVED"); break;
    }
    UART_SendString("\r\n");

    // Check alternate function
    UART_SendString("\r\nAlternate Function:\r\n");
    UART_SendString("AFRH = 0x");
    UART_SendNumber(afrh);
    UART_SendString("\r\n");

    uint32_t pb8_af = (afrh >> 0) & 0xF;
    uint32_t pb9_af = (afrh >> 4) & 0xF;

    UART_SendString("PB8 AF: ");
    UART_SendNumber(pb8_af);
    if (pb8_af == 4)
    {
        UART_SendString(" (AF4 - I2C1) - CORRECT\r\n");
    } else
    {
        UART_SendString(" - ERROR! Should be 4 (AF4 for I2C1)\r\n");
    }

    UART_SendString("PB9 AF: ");
    UART_SendNumber(pb9_af);
    if (pb9_af == 4)
    {
        UART_SendString(" (AF4 - I2C1) - CORRECT\r\n");
    } else
    {
        UART_SendString(" - ERROR! Should be 4 (AF4 for I2C1)\r\n");
    }

    // Check I2C registers
    UART_SendString("\r\nI2C1 Configuration:\r\n");
    UART_SendString("CR1 = 0x");
    UART_SendNumber(I2C1_CR1);
    UART_SendString(" (PE bit: ");
    if (I2C1_CR1 & I2C_CR1_PE)
    {
        UART_SendString("ENABLED)\r\n");
    } else
    {
        UART_SendString("DISABLED)\r\n");
    }

    UART_SendString("ISR = 0x");
    UART_SendNumber(I2C1_ISR);
    UART_SendString(" (BUSY: ");
    if (I2C1_ISR & I2C_ISR_BUSY)
    {
        UART_SendString("YES, ");
    } else
    {
        UART_SendString("NO, ");
    }
    UART_SendString("TXE: ");
    if (I2C1_ISR & I2C_ISR_TXE)
    {
        UART_SendString("YES)\r\n");
    } else
    {
        UART_SendString("NO)\r\n");
    }

    UART_SendString("\r\n");
}
