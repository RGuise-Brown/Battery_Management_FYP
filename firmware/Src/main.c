/*
 * main.c
 *
 *  Created on: Sep 20, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: Working implementation with modular functions for unified temperature sensing with three selectable methods
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

// Temperature sensing method selection
typedef enum {
	TMP117_I2C = 1, 			// TMP117 via I2C
	TMP36_WITH_ENABLE = 2,		// TMP36 with shutdown pin control
	TMP36_BASIC = 3				// TMP36 basic (no shutdown control)
}temp_method_t;

// Change this variable to select temperature method
#define SELECTED_TEMP_METHOD 	TMP36_BASIC

// Function prototypes
void delay (volatile uint32_t count);
void init_selected_temp_method(temp_method_t method);
void read_display_temp(temp_method_t method, uint32_t counter);
void flash_led_for_method(temp_method_t method);


int main(void)
{
	uint32_t counter = 0;

	//Set clock speed (choose one)
	//Clock_Init(CLOCK_4MHZ);    // HSI 4MHz - default
	Clock_Init(CLOCK_16MHZ);   // MSI 16MHz - 4x faster

	delay(10000);

	// Initialize peripherals
	LED2_Config();
	UART_Init();

	// Initialize the selected temperature sensing method
	init_selected_temp_method(SELECTED_TEMP_METHOD);

	LED2_On();
	delay(10000000);

    // Send startup message
    UART_SendString("=== Temperature Sensor Test ===\r\n");
    UART_SendString("Method: ");

    switch(SELECTED_TEMP_METHOD)
    {
        case TMP117_I2C:
            UART_SendString("TMP117 via I2C (LED: Fast flash)\r\n");
            break;
        case TMP36_WITH_ENABLE:
            UART_SendString("TMP36 with shutdown control (LED: Medium flash)\r\n");
            break;
        case TMP36_BASIC:
            UART_SendString("TMP36 basic (LED: Slow flash)\r\n");
            break;
        default:
            UART_SendString("Unknown method!\r\n");
            break;
    }

    UART_SendString("Clock: ");
    UART_SendNumber(GetClockFreq() / 1000000);
    UART_SendString(" MHz\r\n");
    UART_SendString("LPUART @ ");
    UART_SendNumber(BAUD_RATE);
    UART_SendString(" baud\r\n");
    UART_SendString("-----------------------------\r\n\r\n");

    delay(100000);

    // Flash LED pattern to indicate selected method at startup
    UART_SendString("Indicating selected method with LED pattern...\r\n");
    for(int i = 0; i < 3; i++) {
        flash_led_for_method(SELECTED_TEMP_METHOD);
        delay(2000000); // 2 second pause between pattern repetitions
    }

    while (1)
    {
        counter++;

        if (counter % 10 == 0) // Every 10 seconds
        {
            UART_SendString("=== Temperature Reading - Iteration ");
            UART_SendNumber(counter);
            UART_SendString(" ===\r\n");

            read_display_temp(SELECTED_TEMP_METHOD, counter);

            UART_SendString("\r\n");
        }
        else if (counter % 5 == 0) // Every 5 seconds - status check
        {
            UART_SendString("Status check - iteration ");
            UART_SendNumber(counter);

            switch(SELECTED_TEMP_METHOD) {
                case TMP117_I2C:
                    if (I2C_IsReady())
                    {
                        UART_SendString(": TMP117 I2C METHOD ACTIVE");} else {
                        UART_SendString(", I2C NOT READY");
                    }

                    if (TMP117_IsPresent())
                    {
                        UART_SendString(", TMP117 PRESENT\r\n");
                    } else {
                        UART_SendString(", TMP117 NOT PRESENT\r\n");
                    }
                    break;

                case TMP36_WITH_ENABLE:
                    UART_SendString(": TMP36 WITH ENABLE METHOD ACTIVE\r\n");
                    break;

                case TMP36_BASIC:
                    UART_SendString(": TMP36 BASIC METHOD ACTIVE\r\n");
                    break;

                default:
                    UART_SendString(": UNKNOWN METHOD\r\n");
                    break;
            }
        }

        delay(1000000); // 1 second delay
    }
}

// Initialize the selected temperature sensing method
void init_selected_temp_method(temp_method_t method)
{
    switch(method) {
        case TMP117_I2C:
            // Initialize I2C
            UART_SendString("Initializing I2C...\r\n");
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
            break;

        case TMP36_WITH_ENABLE:
            // Initialize ADC and temperature sensor with shutdown control
            ADC_Init();
            TempSensor_Shutdown_Init();
            UART_SendString("TMP36 with shutdown control initialized\r\n");
            break;

        case TMP36_BASIC:
            // Initialize ADC only
            ADC_Init();
            UART_SendString("TMP36 basic initialized\r\n");
            break;

        default:
            UART_SendString("ERROR: Unknown temperature method!\r\n");
            break;
    }
}

// Read and display temperature based on selected method
void read_display_temp(temp_method_t method, uint32_t counter)
{
    switch(method) {
        case TMP117_I2C:
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

            } else {
                UART_SendString("ERROR: TMP117 not responding!\r\n");
                // Try to reinitialize I2C
                UART_SendString("Attempting to reinitialize I2C...\r\n");
                I2C_Init();
            }
            break;

        case TMP36_WITH_ENABLE:
            // Enable temperature sensor before reading
            TempSensor_Enable();
            delay(10000); // Allow sensor to stabilize

            // Get ADC readings
            uint16_t adc_raw = ADC_Read_Raw();
            uint32_t voltage_mv = ADC_ConvertTo_mV_Precise(adc_raw);

            // Get temperature with better precision
            int32_t temp_tenths = ADC_ConvertTo_TempC_Tenths(adc_raw);

            UART_SendString("Raw ADC: ");
            UART_SendNumber(adc_raw);
            UART_SendString(" (0-4095)\r\n");

            UART_SendString("Voltage: ");
            UART_SendNumber(voltage_mv);
            UART_SendString(" mV\r\n");

            // Display temperature with 1 decimal place
            if (temp_tenths >= 0) {
                int temp_whole = temp_tenths / 10;
                int temp_decimal = temp_tenths % 10;
                UART_SendString("Temperature: ");
                UART_SendNumber(temp_whole);
                UART_SendChar('.');
                UART_SendNumber(temp_decimal);
                UART_SendString(" C\r\n");
            } else {
                int temp_whole = (-temp_tenths) / 10;
                int temp_decimal = (-temp_tenths) % 10;
                UART_SendString("Temperature: -");
                UART_SendNumber(temp_whole);
                UART_SendChar('.');
                UART_SendNumber(temp_decimal);
                UART_SendString(" C\r\n");
            }

            // Disable sensor to save power
            TempSensor_Disable();
            break;

        case TMP36_BASIC:
            // Get ADC readings directly
            uint16_t adc_raw_basic = ADC_Read_Raw();
            uint32_t voltage_mv_basic = ADC_ConvertTo_mV(adc_raw_basic);

            // Get temperature with better precision
            int32_t temp_tenths_basic = ADC_ConvertTo_TempC_Tenths(adc_raw_basic);

            UART_SendString("Raw ADC: ");
            UART_SendNumber(adc_raw_basic);
            UART_SendString(" (0-4095)\r\n");

            UART_SendString("Voltage: ");
            UART_SendNumber(voltage_mv_basic);
            UART_SendString(" mV\r\n");

            // Display temperature with 1 decimal place
            if (temp_tenths_basic >= 0) {
                int temp_whole_basic = temp_tenths_basic / 10;
                int temp_decimal_basic = temp_tenths_basic % 10;
                UART_SendString("Temperature: ");
                UART_SendNumber(temp_whole_basic);
                UART_SendChar('.');
                UART_SendNumber(temp_decimal_basic);
                UART_SendString(" C\r\n");
            } else {
                int temp_whole_basic = (-temp_tenths_basic) / 10;
                int temp_decimal_basic = (-temp_tenths_basic) % 10;
                UART_SendString("Temperature: -");
                UART_SendNumber(temp_whole_basic);
                UART_SendChar('.');
                UART_SendNumber(temp_decimal_basic);
                UART_SendString(" C\r\n");
            }
            break;

        default:
            UART_SendString("ERROR: Unknown temperature method in reading function!\r\n");
            break;
    }
}

// Flash LED with different patterns for each temperature method
void flash_led_for_method(temp_method_t method)
{
    switch(method) {
        case TMP117_I2C:
            // Fast flash pattern: 3 quick flashes
            for(int i = 0; i < 3; i++) {
                LED2_On();
                delay(150000);   // 150ms on
                LED2_Off();
                delay(150000);   // 150ms off
            }
            break;

        case TMP36_WITH_ENABLE:
            // Medium flash pattern: 2 medium flashes
            for(int i = 0; i < 2; i++) {
                LED2_On();
                delay(400000);   // 400ms on
                LED2_Off();
                delay(400000);   // 400ms off
            }
            break;

        case TMP36_BASIC:
            // Slow flash pattern: 1 long flash
            LED2_On();
            delay(1000000);      // 1000ms on
            LED2_Off();
            delay(500000);       // 500ms off
            break;

        default:
            // Error pattern: rapid flashing
            for(int i = 0; i < 5; i++) {
                LED2_On();
                delay(100000);   // 100ms on
                LED2_Off();
                delay(100000);   // 100ms off
            }
            break;
    }
}

// FUNCTIONS
void delay(volatile uint32_t count)
{
    while (count--) {
        asm("nop"); // do nothing (1 CPU cycle)
    }
}
