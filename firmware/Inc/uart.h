/*
 * uart.h
 *
 *  Created on: Sep 20, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: UART/ USART functions for STM32L4 communication with PC
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

// Register and Pin Definitions

// Base addresses
#define RCC_BASE        0x40021000U
#define PWR_BASE		0x40007000U
#define GPIOG_BASE      0x48001800U // GPIO G
#define LPUART1_BASE    0x40008000U // LPUART1 connected to virtual COM


// RCC registers
#define RCC_AHB2ENR 	(*(volatile uint32_t*)(RCC_BASE + 0x4C))
#define RCC_APB1ENR1 	(*(volatile uint32_t*)(RCC_BASE + 0x58))
#define RCC_APB1ENR2 	(*(volatile uint32_t*)(RCC_BASE + 0x5C))

// PWR registers
#define PWR_CR2 		(*(volatile uint32_t*)(PWR_BASE + 0x04))

// GPIOG registers
#define GPIOG_MODER 	(*(volatile uint32_t*)(GPIOG_BASE + 0x00))
#define GPIOG_AFRL 		(*(volatile uint32_t*)(GPIOG_BASE + 0X20)) //alternate function
#define GPIOG_AFRH 		(*(volatile uint32_t*)(GPIOG_BASE + 0x24))


// LPUART1 registers
#define LPUART1_CR1     (*(volatile uint32_t*)(LPUART1_BASE + 0x00)) //control register
#define LPUART1_BRR     (*(volatile uint32_t*)(LPUART1_BASE + 0x0C)) // baud rate register
#define LPUART1_ISR     (*(volatile uint32_t*)(LPUART1_BASE + 0x1C)) // interrupt and status register
#define LPUART1_TDR     (*(volatile uint32_t*)(LPUART1_BASE + 0x28)) // transmit data register

// Pin definitions - LPUART uses PG7 (TX) and PG8 (RX)
#define LPUART1_TX_PIN  7
#define LPUART1_RX_PIN  8

// USART2 alternate function is AF7
#define GPIO_AF8        0x08    // LPUART1 uses AF8

// System clock frequency (assuming 4MHz HSI)
#define SYSCLK_FREQ 	4000000UL
#define BAUD_RATE 		9600

// Function prototypes
void UART_Init(void);
void UART_SendChar(char ch);
void UART_SendString(const char* str);
void UART_SendNumber(uint32_t num);
void UART_SendSignedNumber(int32_t num); // supports negatives
void UART_SendHex(uint32_t num);

#endif /* UART_H_ */
