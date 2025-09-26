/*
 * temp_sensor_enable.h
 *
 *  Created on: Sep 26, 2025
 *      Author: rachaelguise-brown
 */

#ifndef TEMP_SENSOR_ENABLE_H_
#define TEMP_SENSOR_ENABLE_H_

#include <stdint.h>

// PG0 shutdown pin definitions
#define TEMP_SHUTDOWN_GPIO_BASE  0x48001800U  // GPIOG base address
#define TEMP_SHUTDOWN_PIN        0            // PG0

// GPIO registers for PG0
#define TEMP_GPIOG_MODER  (*(volatile uint32_t*)(TEMP_SHUTDOWN_GPIO_BASE + 0x00))
#define TEMP_GPIOG_OTYPER (*(volatile uint32_t*)(TEMP_SHUTDOWN_GPIO_BASE + 0x04))
#define TEMP_GPIOG_OSPEEDR (*(volatile uint32_t*)(TEMP_SHUTDOWN_GPIO_BASE + 0x08))
#define TEMP_GPIOG_PUPDR  (*(volatile uint32_t*)(TEMP_SHUTDOWN_GPIO_BASE + 0x0C))
#define TEMP_GPIOG_BSRR   (*(volatile uint32_t*)(TEMP_SHUTDOWN_GPIO_BASE + 0x18))

// RCC register for GPIOG clock (already defined in your code, but for completeness)
#define RCC_AHB2ENR_GPIOG_EN (1U << 6)  // GPIOG clock enable bit

// Power to GPIOG already enabled from UART

// Function prototypes
void TempSensor_Shutdown_Init(void);
void TempSensor_Enable(void);
void TempSensor_Disable(void);


#endif /* TEMP_SENSOR_ENABLE_H_ */
