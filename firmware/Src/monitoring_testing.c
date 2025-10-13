/*
 * monitoring_testing.c
 *
 *  Created on: Oct 12, 2025
 *      Author: rachaelguise-brown
 */

/*
 * BQ35100 Comprehensive Validation Test
 *
 * This test validates all measurements against known reference values
 * using a programmable DC load to create controlled test conditions.
 *
 * Prerequisites:
 * - Programmable DC load connected to battery
 * - Multimeter for reference voltage measurements
 * - Known ambient temperature (thermometer)
 * - Your battery's actual internal resistance (measured manually)
 */

#include "bq35100_i2c_acc.h"
#include "bq35100_i2c_eos.h"
#include "adc_temp.h"
#include "temp_sensor_enable.h"
#include "uart.h"


/* ==================== Test Configuration ==================== */

// Reference values - UPDATE THESE WITH YOUR ACTUAL VALUES
#define REF_BATTERY_VOLTAGE_MV      3610    // Measured with multimeter (no load)
#define REF_INTERNAL_RESISTANCE_MOHM 610   // Your measured value
#define REF_AMBIENT_TEMP_C          24.0    // Room temperature

// Test current levels (mA)
#define TEST_CURRENT_LOW_MA         250
#define TEST_CURRENT_MED_MA         100
#define TEST_CURRENT_HIGH_MA        500
#define TEST_CURRENT_MAX_MA         1000

// Tolerance thresholds
#define VOLTAGE_TOLERANCE_MV        50      // ±50mV acceptable
#define CURRENT_TOLERANCE_MA        30      // ±30mA acceptable
#define TEMP_TOLERANCE_C            2.0     // ±2°C acceptable
#define IMPEDANCE_TOLERANCE_PERCENT 20      // ±20% acceptable (uncalibrated)

/* ==================== Test Result Structure ==================== */

typedef struct {
    // Test parameters
    uint16_t load_current_ma;
    uint16_t expected_voltage_mv;
    uint16_t expected_voltage_drop_mv;

    // ACC Mode readings
    uint16_t acc_voltage_mv;
    int16_t acc_current_ma;
    int32_t acc_capacity_uah;
    int32_t acc_temp_c_x100;

    // EOS Mode readings
    uint16_t eos_voltage_mv;
    uint16_t eos_impedance_mohm;
    uint16_t eos_pulse_count;
    bool eos_gdone;

    // TMP36 readings
    int32_t tmp36_temp_c_x100;

    // Pass/Fail flags
    bool voltage_pass;
    bool current_pass;
    bool capacity_pass;
    bool temp_pass;
    bool impedance_pass;

} TestResult;

/* ==================== Helper Functions ==================== */

static void PrintTestHeader(const char* test_name)
{
    UART_SendString("\r\n");
    UART_SendString("========================================\r\n");
    UART_SendString("   ");
    UART_SendString(test_name);
    UART_SendString("\r\n");
    UART_SendString("========================================\r\n\r\n");
}

static void PrintTestResult(TestResult* result)
{
    UART_SendString("--- Test Results ---\r\n");
    UART_SendString("Load Current: ");
    UART_SendNumber(result->load_current_ma);
    UART_SendString(" mA\r\n\r\n");

    // Voltage comparison
    UART_SendString("VOLTAGE:\r\n");
    UART_SendString("  Expected:  ");
    UART_SendNumber(result->expected_voltage_mv);
    UART_SendString(" mV\r\n");
    UART_SendString("  ACC Read:  ");
    UART_SendNumber(result->acc_voltage_mv);
    UART_SendString(" mV (");
    int16_t v_diff = (int16_t)result->acc_voltage_mv - (int16_t)result->expected_voltage_mv;
    if (v_diff >= 0) UART_SendString("+");
    UART_SendSignedNumber(v_diff);
    UART_SendString(" mV)\r\n");
    UART_SendString("  EOS Read:  ");
    UART_SendNumber(result->eos_voltage_mv);
    UART_SendString(" mV\r\n");
    UART_SendString("  Status:    ");
    UART_SendString(result->voltage_pass ? "[PASS]" : "[FAIL]");
    UART_SendString("\r\n\r\n");

    // Current comparison
    UART_SendString("CURRENT:\r\n");
    UART_SendString("  Expected:  -");
    UART_SendNumber(result->load_current_ma);
    UART_SendString(" mA (discharge)\r\n");
    UART_SendString("  ACC Read:  ");
    UART_SendSignedNumber(result->acc_current_ma);
    UART_SendString(" mA\r\n");
    UART_SendString("  Status:    ");
    UART_SendString(result->current_pass ? "[PASS]" : "[FAIL]");
    UART_SendString("\r\n\r\n");

    // Capacity reading
    UART_SendString("CAPACITY (ACC Mode):\r\n");
    UART_SendString("  Reading:   ");
    UART_SendSignedNumber(result->acc_capacity_uah);
    UART_SendString(" uAh\r\n");
    UART_SendString("  Status:    ");
    UART_SendString(result->capacity_pass ? "[PASS - Accumulating]" : "[FAIL - Not accumulating]");
    UART_SendString("\r\n\r\n");

    // Temperature comparison
    UART_SendString("TEMPERATURE:\r\n");
    UART_SendString("  Expected:  ");
    UART_SendNumber((int32_t)REF_AMBIENT_TEMP_C);
    UART_SendString(".");
    UART_SendNumber((int32_t)((REF_AMBIENT_TEMP_C - (int32_t)REF_AMBIENT_TEMP_C) * 100));
    UART_SendString(" C\r\n");

    UART_SendString("  ACC BQ:    ");
    UART_SendNumber(result->acc_temp_c_x100 / 100);
    UART_SendString(".");
    int32_t dec = result->acc_temp_c_x100 % 100;
    if (dec < 10) UART_SendString("0");
    UART_SendNumber(dec);
    UART_SendString(" C\r\n");

    UART_SendString("  TMP36:     ");
    UART_SendNumber(result->tmp36_temp_c_x100 / 100);
    UART_SendString(".");
    dec = result->tmp36_temp_c_x100 % 100;
    if (dec < 10) UART_SendString("0");
    UART_SendNumber(dec);
    UART_SendString(" C\r\n");

    UART_SendString("  Status:    ");
    UART_SendString(result->temp_pass ? "[PASS]" : "[FAIL]");
    UART_SendString("\r\n\r\n");

    // Impedance comparison
    /*UART_SendString("IMPEDANCE (EOS Mode):\r\n");
    UART_SendString("  Expected:  ");
    UART_SendNumber(REF_INTERNAL_RESISTANCE_MOHM);
    UART_SendString(" mOhm\r\n");
    UART_SendString("  EOS Read:  ");
    UART_SendNumber(result->eos_impedance_mohm);
    UART_SendString(" mOhm\r\n");
    UART_SendString("  Pulse Cnt: ");
    UART_SendNumber(result->eos_pulse_count);
    UART_SendString("\r\n");
    UART_SendString("  G_DONE:    ");
    UART_SendString(result->eos_gdone ? "SET" : "NOT SET");
    UART_SendString("\r\n");
    UART_SendString("  Status:    ");
    UART_SendString(result->impedance_pass ? "[PASS]" : "[FAIL]");
    UART_SendString("\r\n");*/
}

static uint16_t CalculateExpectedVoltage(uint16_t load_current_ma)
{
    // V_load = V_oc - (I * R_int)
    // All in mV and mA, so result is in mV
    uint32_t voltage_drop = ((uint32_t)load_current_ma * REF_INTERNAL_RESISTANCE_MOHM) / 1000;

    if (voltage_drop > REF_BATTERY_VOLTAGE_MV)
        return 0;

    return REF_BATTERY_VOLTAGE_MV - voltage_drop;
}

static bool CheckTolerance(int32_t measured, int32_t expected, int32_t tolerance)
{
    int32_t diff = measured - expected;
    if (diff < 0) diff = -diff;
    return (diff <= tolerance);
}

/* ==================== Main Test Functions ==================== */

void RunVoltageAccuracyTest(TestResult* result, uint16_t load_current_ma)
{
    result->load_current_ma = load_current_ma;
    result->expected_voltage_mv = CalculateExpectedVoltage(load_current_ma);
    result->expected_voltage_drop_mv = ((uint32_t)load_current_ma * REF_INTERNAL_RESISTANCE_MOHM) / 1000;

    // Wait for voltage to stabilize
    for(volatile uint32_t i = 0; i < 10000000; i++);

    // Read ACC mode voltage
    result->acc_voltage_mv = BQ35100_I2C2_ReadVoltage();

    // Read EOS mode voltage
    result->eos_voltage_mv = BQ35100_I2C1_ReadVoltage();

    // Check if within tolerance
    result->voltage_pass = CheckTolerance(
        result->acc_voltage_mv,
        result->expected_voltage_mv,
        VOLTAGE_TOLERANCE_MV
    );
}

void RunCurrentAccuracyTest(TestResult* result, uint16_t load_current_ma)
{
    result->load_current_ma = load_current_ma;

    // Wait for current measurement to stabilize
    for(volatile uint32_t i = 0; i < 20000000; i++);

    // Read ACC mode current (negative = discharge)
    result->acc_current_ma = BQ35100_I2C2_ReadCurrent();

    // Check if within tolerance
    // Expected is negative (discharge)
    int16_t expected_current = -(int16_t)load_current_ma;
    result->current_pass = CheckTolerance(
        result->acc_current_ma,
        expected_current,
        CURRENT_TOLERANCE_MA
    );
}

void RunCapacityAccuracyTest(TestResult* result, int32_t previous_capacity)
{
    // Read current capacity
    result->acc_capacity_uah = BQ35100_I2C2_ReadAccumulatedCapacity();

    // Check if capacity is changing (accumulating)
    // For discharge, capacity should be decreasing (becoming more negative)
    if (previous_capacity != 0)
    {
        int32_t capacity_change = result->acc_capacity_uah - previous_capacity;

        // Should be negative (discharging)
        // Allow small tolerance for measurement noise
        result->capacity_pass = (capacity_change < -5); // At least 5uAh decrease
    }
    else
    {
        // First reading, just check it's a valid value
        result->capacity_pass = true;
    }
}

void RunTemperatureAccuracyTest(TestResult* result)
{
    // Read all temperature sources
    result->acc_temp_c_x100 = BQ35100_I2C2_ReadTemperature_Int();

    // Read TMP36 via ADC
    uint16_t adc_raw = ADC_Read_Raw();
    if (adc_raw > 0)
    {
        // Convert to temperature in tenths of degree C
        int32_t temp_tenths = ADC_ConvertTo_TempC_Tenths(adc_raw);
        // Convert tenths to hundredths for consistency with BQ35100 format
        result->tmp36_temp_c_x100 = temp_tenths * 10;
    }
    else
    {
        result->tmp36_temp_c_x100 = 0;  // Invalid reading
    }

    // Check if within tolerance (use TMP36 as reference)
    int32_t ref_temp_x100 = (int32_t)(REF_AMBIENT_TEMP_C * 100);
    int32_t tolerance_x100 = (int32_t)(TEMP_TOLERANCE_C * 100);

    bool acc_temp_ok = CheckTolerance(result->acc_temp_c_x100, ref_temp_x100, tolerance_x100);
    bool tmp36_ok = CheckTolerance(result->tmp36_temp_c_x100, ref_temp_x100, tolerance_x100);

    result->temp_pass = (acc_temp_ok && tmp36_ok);
}

void RunImpedanceAccuracyTest(TestResult* result)
{
    UART_SendString("\r\n*** PREPARE FOR IMPEDANCE MEASUREMENT ***\r\n");
    UART_SendString("1. Set DC load to 0mA (no load)\r\n");
    UART_SendString("2. Wait for voltage to stabilize\r\n");
    UART_SendString("Waiting 10 seconds...\r\n\r\n");

    for(volatile uint32_t i = 0; i < 16000000; i++); // 10 seconds

    // Start EOS measurement
    UART_SendString("Starting EOS measurement...\r\n");
    BQ35100_I2C1_GaugeStop(); // Make sure stopped
    for(volatile uint32_t i = 0; i < 5000000; i++);

    // Clear any old G_DONE flags
    uint8_t dummy = BQ35100_I2C1_ReadBatteryAlert();
    (void)dummy; // Suppress unused warning

    BQ35100_I2C1_GaugeStart();
    UART_SendString("GAUGE_START sent\r\n\r\n");

    UART_SendString("*** SET DC LOAD TO 500mA NOW! ***\r\n");
    UART_SendString("Waiting 5 seconds for measurement...\r\n\r\n");

    for(volatile uint32_t i = 0; i < 80000000; i++); // 5 seconds

    UART_SendString("Stopping measurement...\r\n");
    BQ35100_I2C1_GaugeStop();

    UART_SendString("(You can set load back to 0mA now)\r\n\r\n");

    // Wait for processing
    for(volatile uint32_t i = 0; i < 32000000; i++);

    // Check G_DONE
    uint32_t timeout = 500;
    result->eos_gdone = false;
    for(uint32_t i = 0; i < timeout; i++)
    {
        if (BQ35100_I2C1_IsGaugeDone())
        {
            result->eos_gdone = true;
            UART_SendString("G_DONE detected!\r\n");
            break;
        }
        for(volatile uint32_t j = 0; j < 160000; j++); // 10ms between checks
    }

    if (!result->eos_gdone)
    {
        UART_SendString("G_DONE still not set after timeout.\r\n");
    }


    // Read results
    result->eos_impedance_mohm = BQ35100_I2C1_ReadMeasuredZ();
    result->eos_pulse_count = BQ35100_I2C1_ReadEOSPulseCount();

    UART_SendString("Reading measurement results:\r\n");
    UART_SendString("  Impedance: ");
    UART_SendNumber(result->eos_impedance_mohm);
    UART_SendString(" mOhm\r\n");
    UART_SendString("  Pulse count: ");
    UART_SendNumber(result->eos_pulse_count);
    UART_SendString("\r\n\r\n");

    // Check impedance tolerance (wider tolerance for uncalibrated)
    uint32_t tolerance = (REF_INTERNAL_RESISTANCE_MOHM * IMPEDANCE_TOLERANCE_PERCENT) / 100;
    result->impedance_pass = CheckTolerance(
        result->eos_impedance_mohm,
        REF_INTERNAL_RESISTANCE_MOHM,
        tolerance
    ) && result->eos_gdone;
}

/* ==================== Master Test Sequence ==================== */

void BQ35100_ComprehensiveValidationTest(void)
{
    TestResult result;
    int32_t previous_capacity = 0;  // <-- This should define it

    PrintTestHeader("COMPREHENSIVE VALIDATION TEST");

    UART_SendString("This test validates all measurements against known values.\r\n");
    UART_SendString("You will need to control the DC load during this test.\r\n\r\n");

    UART_SendString("Reference Values:\r\n");
    UART_SendString("  Battery Voltage (OC): ");
    UART_SendNumber(REF_BATTERY_VOLTAGE_MV);
    UART_SendString(" mV\r\n");
    UART_SendString("  Internal Resistance:  ");
    UART_SendNumber(REF_INTERNAL_RESISTANCE_MOHM);
    UART_SendString(" mOhm\r\n");
    UART_SendString("  Ambient Temperature:  ");
    UART_SendNumber((int32_t)REF_AMBIENT_TEMP_C);
    UART_SendString(" C\r\n\r\n");


    UART_SendString("Test will begin in 5 seconds...\r\n");

    for(volatile uint32_t i = 0; i < 8000000; i++); // 5 second delay

    // ==================================================
    // TEST 1: No Load Voltage
    // ==================================================
    PrintTestHeader("TEST 1: No-Load Voltage");
    UART_SendString("Set DC load to 0mA (open circuit)\r\n");
    UART_SendString("Waiting 10 seconds...\r\n\r\n");
    for(volatile uint32_t i = 0; i < 16000000; i++);

    RunVoltageAccuracyTest(&result, 0);
    RunTemperatureAccuracyTest(&result);
    result.current_pass = (result.acc_current_ma >= -5 && result.acc_current_ma <= 5); // Near zero
    result.capacity_pass = true; // First reading, always pass
    previous_capacity = result.acc_capacity_uah;
    PrintTestResult(&result);

    // ==================================================
    // TEST 2: Low Load (20mA)
    // ==================================================
    PrintTestHeader("TEST 2: Low Load Voltage & Current");
    UART_SendString("Set DC load to 50mA\r\n");
    UART_SendString("Waiting 10 seconds...\r\n\r\n");
    for(volatile uint32_t i = 0; i < 10000000; i++);

    RunVoltageAccuracyTest(&result, TEST_CURRENT_LOW_MA);
    RunCurrentAccuracyTest(&result, TEST_CURRENT_LOW_MA);
    RunTemperatureAccuracyTest(&result);
    RunCapacityAccuracyTest(&result, previous_capacity);
    previous_capacity = result.acc_capacity_uah;
    PrintTestResult(&result);

    // ==================================================
    // TEST 3: Medium Load (100mA)
    // ==================================================
    PrintTestHeader("TEST 3: Medium Load Voltage & Current");
    UART_SendString("Set DC load to 100mA\r\n");
    UART_SendString("Waiting 10 seconds...\r\n\r\n");
    for(volatile uint32_t i = 0; i < 10000000; i++);

    RunVoltageAccuracyTest(&result, TEST_CURRENT_MED_MA);
    RunCurrentAccuracyTest(&result, TEST_CURRENT_MED_MA);
    RunTemperatureAccuracyTest(&result);
    RunCapacityAccuracyTest(&result, previous_capacity);
    previous_capacity = result.acc_capacity_uah;
    PrintTestResult(&result);

    // ==================================================
    // TEST 4: High Load (500mA)
    // ==================================================
    PrintTestHeader("TEST 4: High Load Voltage & Current");
    UART_SendString("Set DC load to 500mA\r\n");
    UART_SendString("Waiting 10 seconds...\r\n\r\n");
    for(volatile uint32_t i = 0; i < 10000000; i++);

    RunVoltageAccuracyTest(&result, TEST_CURRENT_HIGH_MA);
    RunCurrentAccuracyTest(&result, TEST_CURRENT_HIGH_MA);
    RunTemperatureAccuracyTest(&result);
    RunCapacityAccuracyTest(&result, previous_capacity);
    previous_capacity = result.acc_capacity_uah;
    PrintTestResult(&result);

    // ==================================================
    // TEST 5: Maximum Load (1000mA)
    // ==================================================
    PrintTestHeader("TEST 5: Maximum Load Voltage & Current");
    UART_SendString("Set DC load to 1000mA\r\n");
    UART_SendString("Waiting 10 seconds...\r\n\r\n");
    for(volatile uint32_t i = 0; i < 10000000; i++);

    RunVoltageAccuracyTest(&result, TEST_CURRENT_MAX_MA);
    RunCurrentAccuracyTest(&result, TEST_CURRENT_MAX_MA);
    RunTemperatureAccuracyTest(&result);
    RunCapacityAccuracyTest(&result, previous_capacity);
    previous_capacity = result.acc_capacity_uah;
    PrintTestResult(&result);

    // ==================================================
    // TEST 6: Impedance Measurement (EOS Mode)
    // ==================================================
    PrintTestHeader("TEST 6: EOS Mode Impedance");
    RunImpedanceAccuracyTest(&result);
    PrintTestResult(&result);

    // ==================================================
    // FINAL SUMMARY
    // ==================================================
    PrintTestHeader("VALIDATION TEST COMPLETE");
    UART_SendString("All tests completed.\r\n");
    UART_SendString("Review results above for any failures.\r\n\r\n");

    UART_SendString("Common Issues:\r\n");
    UART_SendString("- Voltage FAIL: Check sense resistor value, ADC calibration\r\n");
    UART_SendString("- Current FAIL: Verify shunt resistor value, polarity\r\n");
    UART_SendString("- Temp FAIL: Check thermistor connection, verify external sensor\r\n");
    UART_SendString("- Impedance FAIL: Needs calibration or higher pulse current\r\n");
    UART_SendString("\r\n");
}

