/*
 * uart.c
 *
 *  Created on: Sep 20, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: UART/ USART implementation for STM32L4
 *      Virtual COM port uses LPUART
 *      Pins on board:
 *      TX = PG7
 *      RX = PG8
 *      GPIOG requires VDDIO2 power domain enabled
 */

#include "uart.h"
#include <stdint.h>
#include <clock_config.h>


// Private helper functions

static void Enable_VDDIO2(void)
{
	// Enable PWR peripheral clock
	RCC_APB1ENR1 |= (1U << 28); // Enable PWR clock

	// ENable VDDIO2 power domain for GPIOG
	PWR_CR2 |= (1U << 9); // Enable IOSV bit (VDDIO2 independent I/Os supply valid)
}

static void GPIOG_EnableClock(void)
{
	RCC_AHB2ENR |= (1U << 6); // Enable GPIOG clock
}

static void LPUART1_EnableClock(void)
{
	RCC_APB1ENR2 |= (1U << 0); // Enable LPUART1 clock
}

static void GPIO_ConfigureLPUART1Pins(void)
{
    // Configure PG7 (TX) as alternate function
    GPIOG_MODER &= ~(3U << 14);  // Clear PG7 mode bits (bits 14-15)
    GPIOG_MODER |= (2U << 14);   // Set PG7 as alternate function

    // Configure PG8 (RX) as alternate function
    GPIOG_MODER &= ~(3U << 16);  // Clear PG8 mode bits (bits 16-17)
    GPIOG_MODER |= (2U << 16);   // Set PG8 as alternate function

    // Set PG7 to AF8 (LPUART1_TX) - PG7 uses AFRL register
    GPIOG_AFRL &= ~(0xF << 28);  // Clear AF bits for PG7 (bits 28-31)
    GPIOG_AFRL |= (GPIO_AF8 << 28); // Set AF8 for PG7

    // Set PG8 to AF8 (LPUART1_RX) - PG8 uses AFRH register
    GPIOG_AFRH &= ~(0xF << 0);   // Clear AF bits for PG8 (bits 0-3)
    GPIOG_AFRH |= (GPIO_AF8 << 0); // Set AF8 for PG8
}

static void LPUART1_Configure(void)
{
    // Configure LPUART1 for specified baud rate
    uint32_t clock_freq = GetClockFreq();
    // LPUART BRR = (256 * clock_freq) / baud_rate
    uint32_t brr_value = (256 * clock_freq) / BAUD_RATE;
    LPUART1_BRR = brr_value;

    // Enable transmitter, receiver, and LPUART1
    LPUART1_CR1 = (1U << 3) | (1U << 2) | (1U << 0);  //  TE - transmitter | RE - receiver | UE - usart
}

// Public functions

void UART_Init(void)
{
	// Critical: Enable VDDIO2 power domain first
	Enable_VDDIO2();

	// Enable clocks
	GPIOG_EnableClock();
	LPUART1_EnableClock();

	// Configure GPIO Pins
	GPIO_ConfigureLPUART1Pins();

	// Configure USART
	LPUART1_Configure();
}

void UART_SendChar(char ch)
{
	// Wait until transmit data register is empty
	while (! (LPUART1_ISR & (1U << 7))); // WAIT for TXE flag

	// Send character
	LPUART1_TDR = ch;
}

void UART_SendString(const char* str)
{
	while (*str)
	{
		UART_SendChar(*str++);
	}
}

void UART_SendNumber(uint32_t num)
{
	char buffer[12]; // Enough for 32-bit number
	int i = 0;

	// Handle zero case
	if (num == 0)
	{
		UART_SendChar('0');
		return;
	}

	// Convert number to string (reverse order)
	while (num > 0)
	{
		buffer[i++] = '0' + (num % 10);
		num/= 10;
	}

	// Send digits in correct order
	while (i > 0)
	{
		UART_SendChar(buffer [--i]);
	}
}
