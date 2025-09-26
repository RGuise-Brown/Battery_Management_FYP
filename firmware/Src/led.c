/*
 * led.c
 *
 *  Created on: Sep 20, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: To get the blue LED (LED2) to flash
 */

#include "led.h"
#include <stdint.h>

// Base addresses
#define RCC_BASE        0x40021000U
#define GPIOB_BASE      0x48000400U

// Registers
#define RCC_AHB2ENR     (*(volatile uint32_t*)(RCC_BASE + 0x4C))
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))
#define GPIOB_BSRR      (*(volatile uint32_t*)(GPIOB_BASE + 0x18))

// LED2 is PB7
#define LED_PIN 7

// Private helper
static void GPIOB_EnableClock(void)
{
    RCC_AHB2ENR |= (1U << 1);
}

void LED2_Config(void)
{
    GPIOB_EnableClock();
    GPIOB_MODER &= ~(3U << (LED_PIN * 2));
    GPIOB_MODER |=  (1U << (LED_PIN * 2));
}

void LED2_On(void)
{
	GPIOB_BSRR = (1U << LED_PIN);
}

void LED2_Off(void)
{
	GPIOB_BSRR = (1U << (LED_PIN + 16));
}

void LED2_Toggle(void)
{
	GPIOB_ODR ^= (1U << LED_PIN);
}
