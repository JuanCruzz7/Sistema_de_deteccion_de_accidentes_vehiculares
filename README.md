# Vehicular Accident Detection System
### Sistema de deteccion de accidentes vehiculares
*Undergraduate Thesis Project · UTN San Francisco · 2025*

---

## Overview

Autonomous embedded device capable of detecting road accidents and automatically alerting emergency services with the vehicle's GPS location — even when no driver or bystander can make the call.

Designed for scenarios where:
- All occupants are incapacitated
- No witnesses are present near the accident scene

The system operates independently of the vehicle's electrical supply via an onboard battery, ensuring it remains functional even after a total power failure.

---

## System Architecture

```
[Vehicle Power]
      |
      v
[Auxiliary Power Supply + BMS + Li-ion Battery Pack]
      |
      v
[Microcontroller]
      |
      |---> [MPU6050 - Accelerometer + Gyroscope]  (I2C)
      |---> [GPS Module - GY-NEO6MV2]              (UART)
      |---> [SIM800L - GSM/GPRS Communication]     (UART)
      |---> [MQ-9 - Gas Sensor]                    (Analog)
      |---> [DS18B20 - Temperature Sensor]         (1-Wire)
      |
      v
[Accident Detection Logic]
[Impact + Orientation + Speed analysis]
      |
      v
[SMS Alert with GPS coordinates --> Emergency Contact]
```

---

## Features

- **Accident detection** - analyzes impact force, vehicle orientation, and speed via 6-DOF IMU (MPU6050)
- **GPS location** - captures geographic coordinates and sends them in the emergency alert
- **Fire detection** - monitors CO and flammable gas (MQ-9) and temperature (DS18B20) to send a follow-up alert if fire is detected
- **GSM communication** - sends and receives SMS via 2G network (SIM800L), chosen for stability and coverage in rural areas
- **Auxiliary power** - Li-ion battery pack (3x 18650, 3.7V / 5800mAh) with BMS for operation after vehicle power failure
- **Custom PCB** - double-sided 10x10 cm board designed in CircuitMaker (Altium), manufactured by JLPCB
- **3D-printed enclosure** - designed in Tinkercad and printed in PLA

---

## Hardware Components

| Module | Function | Protocol |
|---|---|---|
| MPU6050 | Accelerometer + Gyroscope (6-DOF) | I2C |
| GY-NEO6MV2 | GPS positioning | UART |
| SIM800L | GSM/GPRS communication (SMS) | UART |
| MQ-9 | CO and flammable gas detection | Analog |
| DS18B20 | Temperature sensing | 1-Wire |
| BMS + 18650 cells | Auxiliary power supply | - |

---

## Software

- **IDE:** Arduino IDE
- **Language:** C
- **Logic flow:**
  1. Continuous monitoring of IMU data for impact detection
  2. On confirmed accident: capture GPS coordinates
  3. Send SMS alert with location to pre-assigned emergency number
  4. Continue monitoring for fire/smoke and send follow-up alert if detected

---

## Results

The system successfully met all project objectives:
- Reliable accident detection with low false-positive rate
- GPS location transmitted via SMS within seconds of detection
- Continued operation after simulated vehicle power failure
- Functional prototype mounted and tested on a real vehicle

---

## Author

**Juan Cruz Armando** - Electronic Engineer, UTN San Francisco

[cruzjuanarmando@gmail.com](mailto:cruzjuanarmando@gmail.com) | [github.com/JuanCruzz7](https://github.com/JuanCruzz7)
