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
#include "led.h"
#include "uart.h"
#include "clock_config.h"
#include "adc_temp.h"

// Function prototypes
void delay (volatile uint32_t count);

int main(void)
{
	uint32_t counter = 0;

	// Step 1: Set clock speed (choose one)
	//Clock_Init(CLOCK_4MHZ);    // HSI 4MHz - default
	Clock_Init(CLOCK_16MHZ);   // MSI 16MHz - 4x faster

	// Step 2: Initialize peripherals
	LED2_Config();
	UART_Init();
	ADC_Init();

	// Step 3: Send startup message
	UART_SendString("STM32L4 NUCLEO-L4R5ZI-P Working!\r\n");
	UART_SendString("LPUART1 on PG7/PG8 with VDDIO2 enabled\r\n");
	UART_SendString("Clock: ");
	UART_SendNumber(GetClockFreq() / 1000000);
	UART_SendString(" MHz\r\n");
	UART_SendString("LPUART @ ");
	UART_SendNumber(BAUD_RATE);
    UART_SendString(" baud\r\n");

	UART_SendString("=== ADC Voltage Monitor ===\r\n");
    UART_SendString("PA3 (A0) Voltage Reading\r\n");
	UART_SendString("Range: 0-3.3V, Resolution: 12-bit\r\n");
    UART_SendString("-----------------------------\r\n\r\n");

	LED2_On();

    while (1)
    {
        LED2_Toggle(); //OFF

        // Get ADC readings
        uint16_t adc_raw = ADC_Read_Raw();

        LED2_Toggle(); // ON
        delay(10000);

        uint32_t voltage_mv = ADC_ConvertTo_mV(adc_raw);

        LED2_Toggle(); //OFF

        UART_SendString("Reading #");
        UART_SendNumber(counter++);
        UART_SendString(":\r\n");

        UART_SendString("  Raw: ");
        UART_SendNumber(adc_raw);
        UART_SendString(" (0-4095)\r\n");

        UART_SendString("  Voltage: ");
        UART_SendNumber(voltage_mv);
        UART_SendString(" mV\r\n\r\n");

        UART_SendString("\r\n");
        delay(500000);  // delay second between readings

        LED2_Toggle(); //ON
    }
}

// FUNCTIONS
void delay(volatile uint32_t count)
{
    while (count--) {
        asm("nop"); // do nothing (1 CPU cycle)
    }
}
