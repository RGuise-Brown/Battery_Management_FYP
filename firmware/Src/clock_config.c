/*
 * clock_config.c
 *
 *  Created on: Sep 20, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: Clock configuration implementation - kept simple
 *
 */

#include "clock_config.h"

// Global variable to track current system clock frequency
static uint32_t g_ClockFreq = FREQ_4MHZ; // Default MSI 4MHz

void Clock_Init(Clock_Config_t config)
{
    switch(config)
    {
        case CLOCK_4MHZ:
            // Default MSI 4MHz - ensure it's properly configured
            // Enable MSI
            RCC_CR |= RCC_CR_MSION;
            // Wait for MSI ready
            while (!(RCC_CR & RCC_CR_MSIRDY));

            // Set MSI range to 4MHz (range 6)
            RCC_CR &= ~RCC_CR_MSIRANGE;
            RCC_CR |= (6UL << RCC_CR_MSIRANGE_Pos);
            RCC_CR |= RCC_CR_MSIRGSEL;

            // Switch system clock to MSI
            RCC_CFGR &= ~RCC_CFGR_SW;
            RCC_CFGR |= RCC_CFGR_SW_MSI;

            // Wait for system clock switch
            while ((RCC_CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);

            // Configure ADC clock to use system clock
            RCC_CCIPR &= ~RCC_CCIPR_ADCSEL;
            RCC_CCIPR |= RCC_CCIPR_ADCSEL_SYSCLK;

            g_ClockFreq = 4000000;
            break;

        case CLOCK_16MHZ:
            // Enable HSI16
            RCC_CR |= RCC_CR_HSION;

            // Wait for HSI16 ready
            while (!(RCC_CR & RCC_CR_HSIRDY));

            // Switch system clock to HSI16
            RCC_CFGR &= ~RCC_CFGR_SW;
            RCC_CFGR |= RCC_CFGR_SW_HSI16;

            // Wait for system clock switch
            while ((RCC_CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI16);

            // Configure ADC clock to use system clock
            RCC_CCIPR &= ~RCC_CCIPR_ADCSEL;
            RCC_CCIPR |= RCC_CCIPR_ADCSEL_SYSCLK;

            g_ClockFreq = 16000000;
            break;
    }
}

uint32_t GetClockFreq(void)
{
    return g_ClockFreq;
}

// New function to get ADC clock frequency
uint32_t GetADCClockFreq(void)
{
    // Since we're using system clock for ADC, return system clock frequency
    return g_ClockFreq;
}



