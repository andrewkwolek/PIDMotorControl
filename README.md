# PID Motor Controller

## Overview
This project integrates multiple controllers and sensors to implement a precise PID motor controller, enabling both trajectory tracking and position holding. The system architecture features an inner current control loop operating
at 5 kHz and an outer position control loop running at 200 Hz. Each loop is executed via its own Interrupt Service Routine (ISR) and Timer, ensuring efficient and accurate control outputs. The result is a versatile user interface
that offers extensive options for configuring gains, adjusting position control, reading sensor data, and tracking trajectories.

## Hardware
- DC Motor
- Raspberry Pi Pico 2
- INA219 Current Sensor
- PIC32MX170F256B
- HBridge Breakout Board

https://github.com/user-attachments/assets/541f5b41-fbe0-4114-9cf3-9288c79ccc2f
