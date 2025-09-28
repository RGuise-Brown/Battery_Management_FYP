/*
 * main.c
 *
 *  Created on: Sep 20, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: Working implementation with modular functions
 *
 */

#include <stdint.h>
#include <stdbool.h>

// Include created files
#include "led.h"
#include "uart.h"
#include "clock_config.h"
#include "adc_temp.h"
#include "temp_sensor_enable.h"
#include "i2c.h"


// Function prototypes
void delay (volatile uint32_t count);


int main(void)
{
	uint32_t counter = 0;

	// Step 1: Set clock speed (choose one)
	//Clock_Init(CLOCK_4MHZ);    // HSI 4MHz - default
	Clock_Init(CLOCK_16MHZ);   // MSI 16MHz - 4x faster

	delay(10000);

	// Step 2: Initialize peripherals
	LED2_Config();
	UART_Init();
	//ADC_Init();
	//TempSensor_Shutdown_Init(); // Only needed for one of the ADCs

	LED2_On();
	delay(10000000);

    // Send startup message
    UART_SendString("=== TMP117 Temperature Sensor Test ===\r\n");
    delay(10000);

    // Step 1: Initialize I2C
    UART_SendString("Initialising I2C...\r\n");
    I2C_Init();
    delay(10000);

    // Check if TMP117 is present
        UART_SendString("Checking for TMP117...\r\n");
        if (TMP117_IsPresent()) {
            UART_SendString("TMP117 found!\r\n");

            // Read device ID to verify
            uint16_t device_id = TMP117_ReadDeviceID();
            UART_SendString("Device ID: ");
            UART_SendHex(device_id);
            if (device_id == TMP117_DEVICE_ID) {
                UART_SendString(" - CORRECT!\r\n");
            } else {
                UART_SendString(" - Unexpected (expected 0x0117)\r\n");
            }
        } else {
            UART_SendString("ERROR: TMP117 not found!\r\n");
        }

        UART_SendString("Starting temperature readings...\r\n\r\n");

    while (1)
    {
        counter++;

        if (counter % 10 == 0) // Every 10 seconds
        {
            UART_SendString("=== Temperature Reading - Iteration ");
            UART_SendNumber(counter);
            UART_SendString(" ===\r\n");

            if (TMP117_IsPresent()) {
                // Read raw temperature
                uint16_t raw_temp = TMP117_ReadRawTemp();

                UART_SendString("Back from temp read, raw value: 0x");
                UART_SendHex(raw_temp);
                UART_SendString("\r\n");

                // Read temperature in Celsius
                UART_SendString("Starting conversion...\r\n");
                int32_t temp_c_hundredths = TMP117_ConvertToTempC(raw_temp);
                UART_SendString("Conversion complete\r\n");

                int temp_whole = temp_c_hundredths / 10000;   // whole degrees
                int temp_decimal = temp_c_hundredths % 100; // decimal part

                UART_SendString("Temperature: ");
                UART_SendNumber(temp_whole);
                UART_SendChar('.');
                if (temp_decimal < 10) UART_SendChar('0'); // leading zero for e.g. .05
                UART_SendNumber(temp_decimal);
                UART_SendString(" C\r\n");

                // Toggle LED to show activity
                LED2_Toggle();
            } else {
                UART_SendString("ERROR: TMP117 not responding!\r\n");
                // Try to reinitialize I2C
                UART_SendString("Attempting to reinitialize I2C...\r\n");
                I2C_Init();
            }

            UART_SendString("\r\n");
        }
        else if (counter % 5 == 0) // Every 5 seconds
        {
            // Quick status check
            UART_SendString("Status check - iteration ");
            UART_SendNumber(counter);
            if (I2C_IsReady()) {
                UART_SendString(": I2C READY");
            } else {
                UART_SendString(": I2C NOT READY");
            }

            if (TMP117_IsPresent()) {
                UART_SendString(", TMP117 PRESENT\r\n");
            } else {
                UART_SendString(", TMP117 NOT PRESENT\r\n");
            }
        }

        delay(1000000); // 1 second delay
    }
}

// FUNCTIONS
void delay(volatile uint32_t count)
{
    while (count--) {
        asm("nop"); // do nothing (1 CPU cycle)
    }
}
