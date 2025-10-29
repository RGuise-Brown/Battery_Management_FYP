# Hardware Documentation

Hardware design files for the Battery Monitoring System, including custom PCB designs created in KiCad.

## Overview

Three main PCB designs that integrate with an STM32 Nucleo-L4R5ZI development board to create a complete battery monitoring solution.

## PCB Designs

### 1. STM32 Nucleo Hat (STM_Nucleo_Hat)
Board borrowed from ARU that contains INA226 power monitor which is the only component used in this project.

**Files**: Located in `/STM_Nucleo_Hat/` directory

---

### 2. BQ35100 Board (FYP_PMIC)
Custom board featuring the Texas Instruments BQ35100 fuel gauge for battery monitoring.

**Files**: Located in `/FYP_PMIC/` directory

---

### 3. Temperature Sensor Board (FYP_temp)
Custom board with three temperature sensors:
- TMP117 (digital I2C sensor)
- TMP36 (TO-92 analog sensor)
- TMP36 with shutdown (SOT-23 analog sensor)

**Files**: Located in `/FYP_temp/` directory

---

## Schematics

Full schematics are provided in both PDF and image formats for easy viewing:

### BQ35100 Board
- `BQ35100 Schematic.pdf` - Full schematic documentation
- `BQ35100 Schematic.png` - Quick reference image

### INA226 Board
- `INA226 Full Board Schematic.pdf` - Complete board schematic
- `INA226 Schematic.jpeg` - Quick reference image

### Temperature Sensor Board
- `Temp Board Schematic.pdf` - Full schematic documentation
- `Temp Schematic.png` - Quick reference image

## KiCad Component Files

Custom component libraries and footprints are located in the `/KiCad component files/` directory. These include:
- Custom symbols for BQ35100, INA226, and temperature sensors

## Design Tools

**Software**: KiCad 6.0 or later

## Getting Started

Open the `.kicad_pro` file in the respective board directory, or view the PDF schematics directly.

## Component Information

**Key ICs Used**:
- INA226 power monitor (on STM Nucleo Hat)
- BQ35100 fuel gauge
- TMP117 digital temperature sensor
- TMP36 analog temperature sensors (x2)

**Microcontroller**: STM32 Nucleo-L4R5ZI development board

## Notes

- INA226 is fully implemented and calibrated
- BQ35100 has partial implementation (see firmware README for details)
- All designs created in KiCad

---

For firmware documentation, see the main repository README.
