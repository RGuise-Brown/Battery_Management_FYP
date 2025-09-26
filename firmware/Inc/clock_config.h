/*
 * clock_config.h
 *
 *  Created on: Sep 20, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: Clock configuration and management - kept SIMPLE
 */

#ifndef CLOCK_CONFIG_H_
#define CLOCK_CONFIG_H_

#include <stdint.h>

// Clock configuration options
typedef enum
{
	CLOCK_4MHZ = 0, 	// Default after reset
	CLOCK_16MHZ, 		// Faster MSI
}Clock_Config_t;

// Function prototypes
void Clock_Init(Clock_Config_t config);
uint32_t GetClockFreq(void);

// Helper macros for common frequencies
#define FREQ_4MHZ 	4000000UL
#define FREQ_16MHZ 	16000000UL

// Base addresses
#define RCC_BASE 0x40021000U

// RCC registers
#define RCC_CR       (*(volatile uint32_t*)(RCC_BASE + 0x00))
#define RCC_ICSCR    (*(volatile uint32_t*)(RCC_BASE + 0x04))
#define RCC_CFGR     (*(volatile uint32_t*)(RCC_BASE + 0x08))
#define RCC_CCIPR    (*(volatile uint32_t*)(RCC_BASE + 0x88))

// RCC_CR bit definitions
#define RCC_CR_HSION_Pos    (8U)
#define RCC_CR_HSION        (1UL << RCC_CR_HSION_Pos)
#define RCC_CR_HSIRDY_Pos   (10U)
#define RCC_CR_HSIRDY       (1UL << RCC_CR_HSIRDY_Pos)
#define RCC_CR_MSION_Pos    (0U)
#define RCC_CR_MSION        (1UL << RCC_CR_MSION_Pos)
#define RCC_CR_MSIRDY_Pos   (1U)
#define RCC_CR_MSIRDY       (1UL << RCC_CR_MSIRDY_Pos)
#define RCC_CR_MSIRANGE_Pos (4U)
#define RCC_CR_MSIRANGE     (0xFUL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRGSEL_Pos (3U)
#define RCC_CR_MSIRGSEL     (1UL << RCC_CR_MSIRGSEL_Pos)

// RCC_CFGR bit definitions
#define RCC_CFGR_SW_Pos     (0U)
#define RCC_CFGR_SW         (0x3UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_MSI     (0x0UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSI16   (0x1UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SWS_Pos    (2U)
#define RCC_CFGR_SWS        (0x3UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_MSI    (0x0UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_HSI16  (0x1UL << RCC_CFGR_SWS_Pos)

// RCC_CCIPR bit definitions (for ADC clock source)
#define RCC_CCIPR_ADCSEL_Pos (28U)
#define RCC_CCIPR_ADCSEL     (0x3UL << RCC_CCIPR_ADCSEL_Pos)
#define RCC_CCIPR_ADCSEL_NONE (0x0UL << RCC_CCIPR_ADCSEL_Pos)
#define RCC_CCIPR_ADCSEL_SYSCLK (0x1UL << RCC_CCIPR_ADCSEL_Pos)


#endif /* CLOCK_CONFIG_H_ */
