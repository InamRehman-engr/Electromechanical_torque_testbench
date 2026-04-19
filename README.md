# Electromechanical_torque_testbench
STM32F401RE-based torque measurement system for servo motor testing using a DYJN104 torque sensor and HX711 load cell amplifier. Designed to measure torque generated during AC servo motor loading tests, with Fuji GYA-152BC1 servo motor and RYS152A3VVS amplifier/controller integration.


# Servo Torque Measurement System using STM32F401RE, DYJN104, and HX711

This repository contains the firmware and hardware interfacing logic for a **torque measurement system** built around the **STM32F401RE** microcontroller, a **DYJN104 torque sensor**, and the **HX711 load cell amplifier/ADC**.  

The main purpose of this project is to measure the torque produced during servo motor testing. In this setup, the torque is applied through a servo motor test bench where the motor under load is monitored using the torque sensor, while the actuation/load side is driven using a **Fuji GYA-152BC1 AC servo motor** with the **RYS152A3VVS amplifier/controller**.

---

## Overview

Servo motor testing requires accurate torque feedback to evaluate motor behavior under different operating conditions. This project provides a low-cost embedded solution for reading torque data from a sensor and processing it on an STM32 microcontroller.

The system uses:

- **STM32F401RE** as the main controller
- **DYJN104 torque sensor** for torque measurement
- **HX711** for amplification and digitization of the torque sensor signal
- **Fuji GYA-152BC1 AC servo motor** as the torque-applying/load motor
- **RYS152A3VVS amplifier/controller** for driving the AC servo motor

The measured torque data can be used for:
- servo motor characterization
- load testing
- calibration experiments
- performance validation
- embedded control and monitoring applications

---

## Hardware Used

### Microcontroller
- **STM32F401RE**
  - ARM Cortex-M4 based microcontroller
  - Used for sensor interfacing, signal processing, and data output

### Torque Measurement
- **DYJN104 Torque Sensor**
  - Used to sense the mechanical torque applied in the test setup

### Signal Conditioning / ADC
- **HX711**
  - 24-bit ADC commonly used for load cells and strain-gauge based sensors
  - Converts the low-level analog output of the torque sensor into digital data readable by STM32

### Motor / Actuation Side
- **Fuji GYA-152BC1**
  - AC servo motor used as the torque-applying/load motor in the setup

### Amplifier / Controller
- **RYS152A3VVS**
  - Servo amplifier/controller for operating the Fuji AC servo motor

---

## System Working Principle

1. The **Fuji AC servo motor** applies torque in the mechanical setup.
2. The **DYJN104 torque sensor** senses the resulting torque.
3. The sensor output is fed into the **HX711**.
4. The **HX711** amplifies and converts the signal into digital form.
5. The **STM32F401RE** reads this data, processes it, and calculates torque values.
6. The measured torque can then be displayed, logged, or transmitted for further analysis.

---

## Features

- Interface STM32F401RE with HX711
- Read raw sensor values from DYJN104 torque sensor
- Perform calibration and offset compensation
- Convert raw readings into torque values
- Suitable for servo motor torque testing applications
- Modular structure for future expansion such as LCD display, serial logging, or closed-loop analysis

---

## Repository Contents

Typical contents of this repository may include:

- `Core/Src/` – main firmware source files
- `Core/Inc/` – header files
- `Drivers/` – STM32 HAL and CMSIS drivers
- `README.md` – project documentation
- calibration or test files if included

If this repository contains CubeIDE project files, it can be opened directly in **STM32CubeIDE**.

---

## Pin Connections

The exact pin mapping depends on your STM32CubeMX / firmware configuration, but typically HX711 requires:

- **DT (Data Output)** → GPIO input pin on STM32
- **SCK (Clock)** → GPIO output pin on STM32
- **VCC** → 5V or module supply as required
- **GND** → Common ground

> Important:  
> The **HX711**, STM32, and sensor ground must be common.  
> Incorrect grounding is one of the most common reasons for unstable or meaningless readings.

---

## Software Requirements

- **STM32CubeIDE**
- **STM32 HAL drivers**
- Basic C firmware for HX711 communication
- Optional serial terminal for debugging torque values

---

## Getting Started

### 1. Clone the repository
```bash
git clone <your-repo-link>
