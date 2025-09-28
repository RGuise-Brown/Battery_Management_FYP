/*
 * adc_temp.h
 *
 *  Created on: Sep 22, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: using ADC to get temperatures
 *      no HAL libraries used
 */

#ifndef ADC_TEMP_H_
#define ADC_TEMP_H_

#include <stdint.h>

// Base addresses
#define RCC_BASE 			0x40021000U
#define ADC1_BASE 			0x50040000U
#define GPIOA_BASE 			0x48000000U

// RCC registers
#define RCC_AHB2ENR 		(*(volatile uint32_t*)(RCC_BASE + 0x4C))
#define RCC_AHB2RSTR 		(*(volatile uint32_t*)(RCC_BASE + 0x2C))
#define RCC_CR 				(*(volatile uint32_t*)(RCC_BASE + 0x00))
#define RCC_CFGR 			(*(volatile uint32_t*)(RCC_BASE + 0x08))
#define RCC_CCIPR 			(*(volatile uint32_t*)(RCC_BASE + 0x88))

// ADC1 registers
#define ADC1_ISR 			(*(volatile uint32_t*)(ADC1_BASE + 0x00))
#define ADC1_CR 			(*(volatile uint32_t*)(ADC1_BASE + 0x08))
#define ADC1_CFGR 			(*(volatile uint32_t*)(ADC1_BASE + 0x0C))
#define ADC1_SMPR1 			(*(volatile uint32_t*)(ADC1_BASE + 0x14))
#define ADC1_SQR1 			(*(volatile uint32_t*)(ADC1_BASE + 0x30))
#define ADC1_DR 			(*(volatile uint32_t*)(ADC1_BASE + 0x40))

// ADC common registers (shared)
#define ADC_CCR   (*(volatile uint32_t*)(ADC1_BASE + 0x308))

// GPIOA registers
#define GPIOA_MODER 		(*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_PUPDR 		(*(volatile uint32_t*)(GPIOA_BASE + 0x0C))

// Constants
#define ADC_TIMEOUT 		50000
#define ADC_REF_VOLTAGE 	3300.0f  // 3.3V in mV
#define ADC_MAX_VALUE 		4095.0f    // 12-bit resolution

// Global system clock
extern uint32_t SystemCoreClock;

// Function prototypes
void ADC_Init(void);
uint16_t ADC_Read_Raw(void);
float ADC_ConvertTo_Voltage(uint16_t raw);
uint32_t ADC_ConvertTo_mV(uint16_t raw);
uint32_t ADC_ConvertTo_mV_Precise(uint16_t raw);
int32_t ADC_ConvertTo_TempC_Hundredths(uint16_t raw);
int32_t ADC_ConvertTo_TempC_Tenths(uint16_t raw);


#endif /* ADC_TEMP_H_ */
