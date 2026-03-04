Vehicular Accident Detection System
Sistema de detección de accidentes vehiculares — Undergraduate Thesis Project · UTN San Francisco, 2025

Overview
Autonomous embedded device capable of detecting road accidents and automatically alerting emergency services with the vehicle's GPS location, even when no driver or bystander can make the call.
Designed for scenarios where:

  - All occupants are incapacitated
  - No witnesses are present near the accident scene

The system operates independently of the vehicle's electrical supply via an onboard battery, ensuring it remains functional even after a total power failure.

System Architecture
[Vehicle Power] ──► [Auxiliary BMS + Li-ion Battery Pack]
                              │
                    [STM32 / ATmega Microcontroller]
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
   [MPU6050 IMU]         [GPS Module]       [SIM800L GSM]
   [DS18B20 Temp]        [MQ-9 Gas]         [SMS Alert]
   (I2C / 1-Wire)        (UART)             (UART / GSM)
          │
   [Accident Detection Logic]
   [Impact + Orientation + Speed analysis]

Features:
  * Accident detection — analyzes impact force, vehicle orientation, and speed via 6-DOF IMU (MPU6050)
  * GPS location — captures geographic coordinates and sends them in the emergency alert (GY-NEO6MV2)
  * Fire detection — monitors CO and flammable gas concentration (MQ-9) and temperature (DS18B20) to update alert status after initial report
  * GSM communication — sends and receives SMS via 2G network (SIM800L), chosen for stability and range in low-coverage areas
  * Auxiliary power supply — Li-ion battery pack (3× 18650, 3.7V / 5800mAh) with BMS, allowing operation after vehicle electrical failure
  * Custom PCB — double-sided 10×10 cm board designed in CircuitMaker (Altium), manufactured by JLPCB and hand-assembled
  * 3D-printed enclosure — designed in Tinkercad and printed in PLA for mechanical protection


Hardware Components
ModuleFunctionProtocolMPU6050Accelerometer + Gyroscope (6-DOF)I2CGY-NEO6MV2GPS positioningUARTSIM800LGSM/GPRS communication (SMS)UARTMQ-9CO and flammable gas detectionAnalogDS18B20Temperature sensing1-WireBMS + 18650 cellsAuxiliary power supply—

Software

IDE: Arduino IDE
Language: C
Libraries: Arduino HAL abstraction layer
Logic flow:

Continuous monitoring of IMU data for impact detection
On confirmed accident: capture GPS coordinates
Send SMS alert with location to pre-assigned emergency number
Continue monitoring for fire/smoke and send follow-up alert if detected


Repository Contents
/Sistema de detección de accidentes vehiculares
    └── main firmware source code (.ino / .cpp)
README.md

Results
The system successfully met all project objectives:

Reliable accident detection with low false-positive rate
GPS location transmitted via SMS within seconds of detection
Continued operation after simulated vehicle power failure
Functional prototype mounted and tested on a real vehicle


Author
Juan Cruz Armando — Electronic Engineer, UTN San Francisco
cruzjuanarmando@gmail.com · github.com/JuanCruzz7
