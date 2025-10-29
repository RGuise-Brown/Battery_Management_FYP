# Battery Monitoring System for Lithium Thionyl Chloride Batteries

A battery monitoring system designed to track the health and state of charge of lithium thionyl chloride batteries operating in extreme cold conditions in the Southern Ocean Marginal Ice Zone (SO MIZ).

## Project Overview

This project was completed as a final year project (EEE4022S) for a Mechatronics Engineering degree at the University of Cape Town (UCT) in 2025.

**Author**: Rachael Guise-Brown  
**Supervisor**: Robyn Verrinder  
**Institution**: University of Cape Town

## Application

This battery monitoring system was designed for the **SHARC (Southern Hemisphere Antarctic Research Collective) Buoy**, which operates in the Southern Ocean Marginal Ice Zone. The system focuses on monitoring battery health in extreme cold conditions and adjusts the expected state of charge based on temperature changes.

## Battery Technology

Lithium thionyl chloride (Li-SOCl₂) batteries present unique monitoring challenges:
- **Primary battery** (non-rechargeable)
- **Very flat voltage discharge curve** - voltage alone is insufficient for accurate SOC estimation
- **Temperature-dependent performance** - capacity varies significantly with temperature

Due to these characteristics, the system uses **coulomb counting** and other methods to estimate state of charge, particularly important in the cold operating conditions of the Southern Ocean.

## System Features

### Hardware
- **INA226 Power Monitor**: Fully implemented and calibrated for accurate current and power measurement
- **BQ35100 Fuel Gauge**: Partial implementation but challenges with calibration and configuration without evaluation module (EVM)
- **Temperature Monitoring**: Three sensors tested
  - TMP117 digital I2C sensor
  - TMP36 (TO-92) analog sensor
  - TMP36 with shutdown (SOT-23) analog sensor (chosen sensor)
- **STM32 Nucleo-L4R5ZI**: Main microcontroller platform

### Firmware
- Real-time current, voltage, and capacity monitoring
- Temperature-compensated state of charge estimation
- Multiple algorithm implementations for data analysis
- I2C communication with all sensors
- Support for BQ35100 mode switching (EOS/ACC)
- Discharge cycle testing capabilities

## Repository Structure

```
├── firmware/          # Embedded C code for STM32
├── hardware/          # KiCad PCB designs and schematics
└── docs/              # Datasheets and reference documentation
```

## Implementation Status

### Fully Implemented ✓
- INA226 power monitor with calibration
- Temperature sensor array (TMP117, TMP36 sensors)
- BQ35100 basic functionality:
  - Current, voltage, and capacity reading
  - Mode switching (EOS ↔ ACC)
  - EOS impedance measurements (uncalibrated)
- Real-time monitoring algorithms
- Temperature-compensated SOC estimation

### Known Limitations
- BQ35100 I2C calibration is not functional through firmware

## Getting Started

### Hardware Setup
1. Review the hardware designs in `/hardware`
2. Assemble or obtain the custom PCBs:
   - BQ35100 fuel gauge board
   - Temperature sensor board
   - STM32 Nucleo Hat with INA226
3. Connect boards to STM32 Nucleo-L4R5ZI

### Firmware Setup
1. Navigate to `/firmware` directory
2. Build and flash the firmware to the STM32 Nucleo-L4R5ZI
3. Configure parameters as needed for your battery specifications

### Documentation
Refer to `/docs` for component datasheets and reference materials.

## Key Technologies

- **Microcontroller**: STM32L4R5ZI (ARM Cortex-M4)
- **Development Environment**: ARM GCC toolchain, STM32CubeIDE
- **Hardware Design**: KiCad
- **Communication Protocols**: I2C, UART
- **Programming Language**: C

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- **Robyn Verrinder** - Project supervisor
- **University of Cape Town** - Department of Electrical Engineering
- **SHARC Project** - Application context and requirements
- **Texas Instruments** - BQ35100 and INA226 reference designs

## Contact

Rachael Guise-Brown  
University of Cape Town  
2025

---

*Final Year Project (EEE4022S) - Mechatronics Engineering*  
*University of Cape Town*
