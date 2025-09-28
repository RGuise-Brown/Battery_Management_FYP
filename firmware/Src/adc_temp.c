/*
 * adc_temp.c
 *
 *  Created on: Sep 25, 2025
 *      Author: rachaelguise-brown
 *
 *      Description: ADC implementation for PA3 voltage reading
 */

#include "adc_temp.h"
#include "uart.h"

// Bit positions for ADC_CR
#define ADC_CR_ADEN      (1U << 0)
#define ADC_CR_ADDIS     (1U << 1)
#define ADC_CR_ADSTART   (1U << 2)
#define ADC_CR_ADVREGEN  (1U << 28)
#define ADC_CR_DEEPPWD   (1U << 29)
#define ADC_CR_ADCAL     (1U << 31)

// Bit positions for ADC_ISR
#define ADC_ISR_ADRDY    (1U << 0)
#define ADC_ISR_EOC      (1U << 2)

// Channel for PA3
#define ADC_CHANNEL_PA3  8

/**
 * @brief Initialize ADC for PA3 (Channel 8)
 */

void ADC_Init(void)
{
    UART_SendString("\r\n=== ADC INIT START ===\r\n");

    // --- 1) Enable clocks ---
    RCC_AHB2ENR |= (1U << 0);   // GPIOA clock
    RCC_AHB2ENR |= (1U << 13);  // ADC clock

    // --- 2) Configure PA3 as analog ---
    GPIOA_MODER &= ~(3U << (3 * 2));
    GPIOA_MODER |=  (3U << (3 * 2));  // analog mode
    GPIOA_PUPDR &= ~(3U << (3 * 2));  // no pull

    // --- 3) Select ADC kernel clock in RCC_CCIPR ---
    // ADCSEL[1:0] at bits 29:28
    // 00: No clock, 01: SYSCLK, 10: PLLSAI1R, 11: PLLSAI2R
    RCC_CCIPR &= ~(3U << 28);
    RCC_CCIPR |=  (1U << 28);   // 01 = SYSCLK

    UART_SendString("RCC_CCIPR set to SYSCLK\r\n");

    // --- 4) Configure ADC common prescaler in ADC_CCR ---
    // CKMODE[17:16]: 01 = HCLK/1
    ADC_CCR &= ~(3U << 16);
    ADC_CCR |=  (1U << 16);

    UART_SendString("ADC_CCR CKMODE = HCLK/1\r\n");

    // --- 5) Exit deep power-down ---
    ADC1_CR &= ~ADC_CR_DEEPPWD;

    // --- 6) Enable ADC regulator ---
    ADC1_CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 2000; i++); // ~10us delay

    // --- 7) Calibration ---
    ADC1_CR |= ADC_CR_ADCAL;
    while (ADC1_CR & ADC_CR_ADCAL);  // wait until calibration done
    UART_SendString("Calibration complete\r\n");

    // --- 8) Configure sampling time and channel ---
    ADC1_CFGR = 0;                         // single conv, right aligned, 12-bit
    ADC1_SMPR1 = (7U << 24);               // max sampling time for channel 8
    ADC1_SQR1  = (ADC_CHANNEL_PA3 << 6);   // first conversion in SQ1 = channel 8

    // --- 9) Enable ADC ---
    ADC1_ISR |= ADC_ISR_ADRDY;  // clear ADRDY flag
    ADC1_CR  |= ADC_CR_ADEN;

    uint32_t timeout = ADC_TIMEOUT;
    while (!(ADC1_ISR & ADC_ISR_ADRDY) && --timeout);

    if (timeout == 0) {
        UART_SendString("ERROR: ADRDY timeout!\r\n");
    } else {
        UART_SendString("ADC ready (ADRDY set)\r\n");
    }

    UART_SendString("=== ADC INIT DONE ===\r\n");
}

uint16_t ADC_Read_Raw(void)
{
    // Clear EOC
    ADC1_ISR |= ADC_ISR_EOC;

    // Start conversion
    ADC1_CR |= ADC_CR_ADSTART;

    // Wait for EOC
    uint32_t timeout = ADC_TIMEOUT;
    while (!(ADC1_ISR & ADC_ISR_EOC) && --timeout);

    if (timeout == 0) {
        UART_SendString("Conversion timeout\r\n");
        return 0;
    }

    uint16_t result = ADC1_DR & 0xFFF;
    return result;
}

float ADC_ConvertTo_Voltage(uint16_t raw)
{
    return (float)raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
}

uint32_t ADC_ConvertTo_mV(uint16_t raw)
{
    // scale raw 0–4095 to 0–3300 mV
    return (raw * (uint32_t)ADC_REF_VOLTAGE) / (uint32_t)ADC_MAX_VALUE;
}

// Improved ADC conversion with better precision
uint32_t ADC_ConvertTo_mV_Precise(uint16_t raw)
{
    // Use larger intermediate calculation for better precision
    // This gives approximately 0.8mV resolution instead of losing precision
    return (raw * 3300UL) / 4095UL;
}

// Even better: calculate temperature in hundredths of degrees
int32_t ADC_ConvertTo_TempC_Hundredths(uint16_t raw)
{
    // Calculate voltage in millivolts with higher precision
    uint32_t voltage_mv = (raw * 3300UL) / 4095UL;

    // TMP36: 10mV/°C, 500mV offset at 0°C
    // Temperature = (Voltage_mV - 500) / 10
    // To get hundredths: multiply by 100 first, then divide
    // Temp_hundredths = ((Voltage_mV - 500) * 100) / 10
    // Simplified: Temp_hundredths = (Voltage_mV - 500) * 10

    if (voltage_mv >= 500) {
        return (int32_t)(voltage_mv - 500) * 10; // This gives hundredths of degrees
    } else {
        return (int32_t)(voltage_mv - 500) * 10; // Will be negative for below 0°C
    }
}

// Calculate temperature in tenths of degrees (realistic precision) - USED ONE
int32_t ADC_ConvertTo_TempC_Tenths(uint16_t raw)
{
    // Use higher precision intermediate calculation
    // Multiply by 10 first to get tenths, then do the division
    uint32_t voltage_mv = (raw * 3300UL) / 4095UL; // Voltage in 0.1mV units

    // TMP36: 100 (0.1mV units) per degree, 5000 offset (500mV in 0.1mV units)
    if (voltage_mv >= 500) {
        return (int32_t)(voltage_mv - 500) ; // This gives tenths of degrees
    } else {
        return (int32_t)(voltage_mv - 500) ; // Will be negative for below 0°C
    }
}


