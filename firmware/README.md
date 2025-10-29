# Battery Monitoring System

A comprehensive battery monitoring system implementation with support for multiple temperature sensors, INA226 power monitor, and BQ35100 fuel gauge integration for real-time battery parameter tracking and analysis.

## Overview

This project implements a complete battery monitoring system designed for embedded applications. It provides real-time battery voltage, current, capacity, and temperature monitoring with support for multiple sensor types and monitoring algorithms.

## Key Features

- **Multi-Sensor Temperature Monitoring**: 
  - TMP117 digital temperature sensor
  - TMP36 (TO-92 package) analog temperature sensor
  - TMP36 with shutdown (SOT-23 package) analog temperature sensor
- **INA226 Power Monitor**: Fully implemented and calibrated for accurate current and power measurement
- **BQ35100 Fuel Gauge**: Partial implementation with working features:
  - Current, voltage, and capacity readings
  - Mode switching between EOS (End-of-Service) and ACC (Accumulator) modes
  - EOS impedance and scaled impedance measurement
  - Note: I2C calibration not yet functional
- **Real-time Monitoring**: Continuous battery voltage, current, and temperature tracking
- **Advanced Algorithms**: Multiple monitoring algorithm implementations for data analysis
- **I2C Communication**: Robust I2C interface for all sensor communication
- **Temperature Protection**: Multi-point temperature sensing with threshold monitoring
- **Discharge Cycle Testing**: Automated discharge cycle testing capabilities
- **System Monitoring**: Comprehensive system health monitoring and logging

## Project Structure

### Core Monitoring Files
Uses the individual component files to create a single file that reads all of the values.

- **bms_monitor_reduced.c/h** -  Monitoring implementation for INA226 and TMP36 (SOT-23)
- **bms_monitor.c/h** - Monitoring implementation for BQ35100 and TMP36 (SOT-23)

### Core Algorithm Files
Monitoring algorithms to track, temperature adjusted state-of-charge, voltage drop during high current transmissions, voltage throughout discharge and temperature trend
- **bms_algorithms_v2.c/h** - Version 2 of monitoring algorithms with improvements
- **bms_algorithms.c/h** - Monitoring algorithm implementation version 1, designed for BQ35100 with EOS but not fully tested without EOS calibration

### Hardware Interface

#### Battery Monitoring ICs

**INA226 Power Monitor** (Fully Implemented ✓)
- **ina226_i2c1.c/h** - Complete INA226 interface with calibration
- Accurate current, voltage, and power measurements
- Fully calibrated and operational

**BQ35100 Fuel Gauge** (Partially Implemented)
- **bq35100_i2c_acc.c/h** - BQ35100 accumulator mode interface
- **bq35100_i2c_eos.c/h** - BQ35100 end-of-service mode interface
- **bq35100_common.h** - Common definitions for BQ35100
- Working features:
  - Current, voltage, and capacity reading
  - Mode switching (EOS ↔ ACC)
  - EOS impedance and scaled impedance measurement
- Known limitations:
  - I2C calibration not functional

#### Temperature Sensors

- **TMP117** - Digital I2C temperature sensor
- **TMP36 (TO-92)** - Analog temperature sensor
- **TMP36 with Shutdown (SOT-23)** - Analog temperature sensor with power control

#### Communication Interfaces

- **i2c.c/h** - Generic I2C communication layer
- **i2c_temp.c/h** - I2C temperature sensor interface
- **adc_temp.c/h** - ADC-based temperature measurement (for analog sensors)

### Monitoring & Testing

- **battery_monitor.c/h** - Main battery monitoring module
- **battery_monitor_reduced.h** - Reduced footprint monitoring
- **system_monitoring.c/h** - Overall system health monitoring
- **discharge_cycle_test.c/h** - Automated discharge testing
- **rapid_transmission_test.c** - High-speed data transmission testing
- **monitoring_testing.c** - Monitoring system validation

### System Components

- **main.c** - Main application entry point
- **clock_config.c/h** - System clock configuration
- **uart.c/h** - UART communication interface
- **led.c/h** - LED status indicators
- **syscalls.c** - System call implementations
- **systick_timing.c/h** - System tick timing utilities

### Utilities

- **adc_temp.c/h** - ADC-based temperature measurement
- **temp_sensor_enable.c/h** - Temperature sensor control
- **temp_threshold_test.c/h** - Temperature threshold testing

## Hardware Requirements

### Microcontroller
- STM32 Microcontroller (configured via clock_config)

### Battery Monitoring ICs
- **INA226** - Power monitor (fully supported)
- **BQ35100** - Fuel gauge (partial support)

### Temperature Sensors
- **TMP117** - Digital I2C temperature sensor
- **TMP36 (TO-92)** - Analog temperature sensor
- **TMP36 (SOT-23)** - Analog temperature sensor with shutdown capability

### Peripherals
- LED indicators for status
- UART interface for debugging and data output

## Getting Started

### Prerequisites

- ARM GCC toolchain
- STM32 development environment
- Hardware setup with required sensors

### Building

```bash
# Clone the repository
git clone [your-repo-url]

# Build the project
make all
```

### Configuration

1. Configure clock settings in `clock_config.c`
2. Set up I2C bus parameters in `i2c.c`
3. Configure monitoring algorithm parameters in `bms_algorithms.h`
4. Adjust temperature thresholds in `temp_threshold_test.h`
5. Calibrate INA226 settings in `ina226_i2c1.c` (already calibrated)
6. Configure BQ35100 mode settings (EOS/ACC) as needed

**Note**: BQ35100 I2C calibration is not currently functional and may require manual calibration through other means.

## Usage

The system initializes all peripherals and begins continuous monitoring upon startup. Battery data can be accessed via:

- UART interface for real-time debugging
- LED indicators for status
- System monitoring interface for detailed metrics

## Testing

The project includes several test modules:

- **Discharge Cycle Test**: Validates battery behavior during discharge
- **Rapid Transmission Test**: Tests high-speed data communication
- **Temperature Threshold Test**: Verifies thermal protection
- **Monitoring Testing**: Validates monitoring accuracy

## Algorithm Versions

This project contains multiple monitoring algorithm implementations:

- **Original** (`bms_algorithms.c`) - Initial implementation
- **Version 2** (`bms_algorithms_v2.c`) - Refined algorithms with improved accuracy


## Implementation Status

### Fully Implemented ✓
- INA226 power monitor with calibration
- TMP117 digital temperature sensor
- TMP36 analog temperature sensors (both variants)
- Mode switching for BQ35100 (EOS/ACC)
- Basic BQ35100 reading (current, voltage, capacity)
- EOS impedance measurements

### Partially Implemented ⚠️
- BQ35100 I2C calibration (not functional)


## Acknowledgments

- Texas Instruments for BQ35100 documentation
- STMicroelectronics for STM32 support

---

**Last Updated**: October 2025
