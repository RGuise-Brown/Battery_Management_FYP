/*
 * temp_sensor_enable.c
 *
 *  Created on: Sep 26, 2025
 *      Author: rachaelguise-brown
 *
 *      Implementation to enable Shutdown pin on Temp Sensor
 */

#include "temp_sensor_enable.h"
#include "uart.h"
#include "adc_temp.h"

/**
 * @brief Initialize PG0 as output for temperature sensor shutdown control
 * on C9 - pin 29 (bottom left)
 */
void TempSensor_Shutdown_Init(void)
{
    UART_SendString("Initializing PG0 for temperature sensor shutdown control...\r\n");

    // Enable GPIOG clock
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOG_EN;

    // Wait for clock to stabilize
    for (volatile int i = 0; i < 1000; i++);

    // Configure PG0 as output
    // Clear mode bits for PG0 (bits 1:0)
    TEMP_GPIOG_MODER &= ~(3U << (TEMP_SHUTDOWN_PIN * 2));
    // Set as output (01)
    TEMP_GPIOG_MODER |= (1U << (TEMP_SHUTDOWN_PIN * 2));

    // Configure as push-pull output (default, but explicit)
    TEMP_GPIOG_OTYPER &= ~(1U << TEMP_SHUTDOWN_PIN);

    // Set medium speed
    TEMP_GPIOG_OSPEEDR &= ~(3U << (TEMP_SHUTDOWN_PIN * 2));
    TEMP_GPIOG_OSPEEDR |= (1U << (TEMP_SHUTDOWN_PIN * 2));

    // No pull-up/pull-down
    TEMP_GPIOG_PUPDR &= ~(3U << (TEMP_SHUTDOWN_PIN * 2));

    // Start with sensor disabled (shutdown pin LOW)
    TempSensor_Disable();

    UART_SendString("PG0 configured as output for temperature sensor shutdown\r\n");
}

/**
 * @brief Enable temperature sensor (set PG0 HIGH)
 */
void TempSensor_Enable(void)
{
    // Set PG0 high using BSRR register (atomic set)
    TEMP_GPIOG_BSRR = (1U << TEMP_SHUTDOWN_PIN);

    // Give sensor time to stabilize after enabling
    for (volatile int i = 0; i < 20000; i++);  // ~1ms at 16MHz

    // Optional debug message (comment out for less verbose output)
    //UART_SendString("Temperature sensor enabled\r\n");
}

/**
 * @brief Disable temperature sensor (set PG0 LOW)
 */
void TempSensor_Disable(void)
{
    // Clear PG0 using BSRR register (atomic clear)
    TEMP_GPIOG_BSRR = (1U << (TEMP_SHUTDOWN_PIN + 16));

    // Optional debug message (comment out for less verbose output)
    // UART_SendString("Temperature sensor disabled\r\n");
}
