# Electromechanical Torque Test Bench

STM32F401RE-based **servo torque measurement and current monitoring system** for servo motor testing using a **DYJN104 torque sensor**, **HX711 load cell amplifier**, and **INA219 current sensor** over **I2C**, with **RTOS-based data acquisition and processing**.

The system is designed to measure:

- **Torque**
- **Servo current draw**
- **Torque vs current relationship**
- **Servo-under-test performance graph**

It is intended for electromechanical test bench applications where a servo motor is tested under load and its electrical and mechanical behavior are analyzed together.

---

## Project Overview

This project provides an embedded solution for **real-time torque and current measurement** during servo motor testing.

The setup uses:

- **STM32F401RE** as the main controller
- **DYJN104 torque sensor** to measure applied torque
- **HX711** to amplify and digitize the torque sensor output
- **INA219** to measure servo current through **I2C**
- **RTOS** for task-based acquisition, processing, and logging
- **Fuji GYA-152BC1 AC servo motor**
- **RYS152A3VVS amplifier/controller**

The goal is not only to read torque, but also to correlate torque with current consumption so the performance of the servo under test can be evaluated properly.

---

## Main Objectives

- Measure **torque** from the mechanical test setup
- Measure **current drawn by the servo**
- Process both signals in real time using **RTOS tasks**
- Generate **torque vs current relationship**
- Support plotting of **servo-under-test performance graphs**
- Provide a modular base for future closed-loop or automated testing

---

## Hardware Used

## 1. Microcontroller
### **STM32F401RE**
- ARM Cortex-M4 based microcontroller
- Used for:
  - HX711 interfacing
  - INA219 interfacing through I2C
  - torque calculation
  - current calculation
  - RTOS task scheduling
  - serial/debug output

---

## 2. Torque Sensor
### **DYJN104 Torque Sensor**
- Used to measure torque in the shaft/load path
- Produces a very small signal that requires amplification and digitization

---

## 3. Torque Signal Conditioning
### **HX711**
- 24-bit ADC for strain-gauge / bridge-based sensors
- Reads the output of the DYJN104 torque sensor
- Provides high-resolution digital readings to STM32

---

## 4. Current Measurement
### **INA219**
- Digital current and bus voltage monitor
- Connected to STM32 using **I2C**
- Used to measure:
  - shunt voltage
  - bus voltage
  - load current
  - power (optional)

Typical I2C address used:
- **0x40** (default INA219 address)

---

## 5. Motor / Actuation Side
### **Fuji GYA-152BC1**
- AC servo motor used in the test bench setup

---

## 6. Servo Amplifier / Controller
### **RYS152A3VVS**
- Servo amplifier/controller used to drive the Fuji AC servo motor

---

## System Working Principle

1. The **servo motor under test** operates in the mechanical setup.
2. The applied shaft torque is sensed by the **DYJN104 torque sensor**.
3. The torque sensor signal is amplified and digitized through the **HX711**.
4. The **INA219** measures the current being drawn by the servo through the power path.
5. The **STM32F401RE** reads both torque and current data.
6. Using **RTOS tasks**, the controller:
   - samples torque
   - samples current
   - filters/calibrates data
   - computes torque and current values
   - relates torque to current
   - sends data for logging or plotting
7. The collected data can be used to generate:
   - **torque vs current graph**
   - **servo performance graph under testing conditions**

---

## Features

- STM32F401RE interface with **HX711**
- STM32F401RE interface with **INA219 over I2C**
- **RTOS-based** task separation
- Torque sensor offset calibration
- Current measurement and monitoring
- Real-time torque calculation
- Real-time current calculation
- Torque-current relation analysis
- Suitable for servo characterization and load testing
- Expandable for logging, LCD, UART plotting, or PC-based analysis

---

## RTOS-Based Architecture

This project uses **RTOS** to manage sensing and processing in separate tasks.

Typical task breakdown:

### **1. HX711 Task**
- Reads raw torque sensor values
- Applies zero-offset compensation
- Converts raw value into torque units

### **2. INA219 Task**
- Reads current and bus voltage through I2C
- Converts raw register values into usable current readings

### **3. Processing Task**
- Synchronizes torque and current data
- Computes torque-current pairs
- Prepares data for graphing and analysis

### **4. Logging / Communication Task**
- Sends readings through UART / serial
- Can be used for PC logging, CSV generation, or live plotting

This structure improves code organization and makes future expansion easier.

---

## Measured Parameters

The system can provide:

- **Raw HX711 value**
- **Torque sensor zero offset**
- **Net torque sensor reading**
- **Calculated torque**
- **INA219 bus voltage**
- **INA219 shunt voltage**
- **Measured servo current**
- **Optional calculated power**
- **Torque vs current data pair**

---

## Torque vs Current Relation

A major purpose of this setup is to study how servo current changes with applied torque.

By reading both sensors together, the system can generate data such as:

- low torque → low current
- medium torque → medium current
- high torque → high current

This allows:
- validation of servo loading behavior
- efficiency observation
- calibration experiments
- performance benchmarking
- abnormal current detection under load

---

## Graphs / Test Output

Using serial logging or saved CSV data, the following plots can be generated:

### **1. Torque vs Current Graph**
Shows how much current the servo draws at different torque levels.

### **2. Torque vs Time Graph**
Shows how torque changes during the test.

### **3. Current vs Time Graph**
Shows servo current behavior during load application.

### **4. Servo Under Testing Graph**
Combined graph or test profile showing the servo behavior under load conditions.

These graphs are useful for:
- performance validation
- servo characterization
- comparison between no-load and loaded operation
- identifying abnormal behavior or saturation

---

## Repository Contents

Typical repository structure:

```text
Electromechanical_torque_testbench/
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
├── Middlewares/
│   └── FreeRTOS/
├── README.md
└── STM32CubeIDE project files
