# AIFRED: Hermano - Adaptive Intelligent Gripper with Pressure Feedback

This project implements an adaptive gripper control system with real-time pressure feedback capabilities, designed to detect different material properties and automatically adjust grip force.

## Overview

The AIFRED Hermano system uses a BLDC motor with torque control to operate a gripper mechanism, along with advanced sensing to detect pressure and angle. The control algorithm enables automatic grip detection, material softness assessment, and adaptive control.

## Features

### Automatic Grip Detection
The system automatically stops the gripper when:
- Maximum pressure threshold is reached
- Rate of pressure change falls below threshold
- Angular velocity stabilizes for a defined period
- Safety timeout is exceeded

### Adaptive Pressure Sensing
- Real-time calculation of pressure derivative (dP/dt)
- Exponential smoothing filter to reduce noise
- Dynamic pressure thresholds based on maximum pressure

### Multiple Control Modes
- **Closing Mode**: Activated by Button 1 or detected movement
- **Opening Mode**: Activated by Button 2
- **Holding Mode**: Applies minimal voltage to maintain grip

### Softness Detection
The system quantifies material hardness/softness by:
1. Measuring pressure applied over time
2. Analyzing the rate of pressure increase
3. Using the pressure-time curve's derivative to classify materials

## Hardware Components

- BLDC motor with 3PWM driver
- TLE5012 angle sensor (via SPI)
- TLx493D 3D magnetic sensor for pressure detection
- Two control buttons
- Arduino-compatible microcontroller

## Setup and Usage

### Hardware Setup
1. Connect the TLE5012 sensor to the SPI pins defined in the code
2. Connect the TLx493D magnetic sensor via I2C (Wire1)
3. Connect the BLDC motor and driver according to pin definitions
4. Connect buttons to the defined input pins with pull-up resistors

### Control Interface
- **Button 1**: Start closing the gripper
- **Button 2**: 
  - When pressed: Opens the gripper
  - When released: Re-enables closing functionality
- **Serial Commander**: Control target voltage with 'T' command

### LED Indicators
- Activity indicators show the current state of the gripper

## Configuration

Key parameters can be adjusted in the code:
- `PRESSURE_MAX`: Maximum pressure threshold
- `DP_DT_THRESHOLD`: Threshold for pressure derivative
- `SMOOTHING_FACTOR`: Filter coefficient for sensor data
- `ANGULAR_VELOCITY_THRESHOLD`: Threshold for motion detection
- `DPDT_STABLE_SAMPLES`: Number of samples to confirm stability

## Data Analysis

The system outputs diagnostic data via serial for monitoring:
- Current pressure
- Pressure derivative (dP/dt)
- Current state (closing/holding)

