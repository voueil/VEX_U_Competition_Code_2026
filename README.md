# VEX U Competition Robot

## Overview
This repository contains the complete VEX V5 competition robot code. It includes:

- PID-based autonomous driving (distance and turning)
- Arcade-style driver control
- Intake system control
- Optical sensor integration
- Pneumatic output control

The code is designed for a robot with 4-wheel drive, multiple intake motors, and sensors for navigation and object detection.

---

## Hardware Configuration

### Brain & Controller
- **VEX V5 Brain**
- **VEX V5 Controller** (Primary)

### Sensors
- Inertial Sensor – PORT5
- Optical Sensor – PORT6
- Digital Output – 3-Wire Port A

### Drive Motors
- Left Motors: PORT2, PORT4
- Right Motors: PORT1, PORT3

### Intake Motors
- PORT7 – PORT11

---

## PID Constants

```cpp
// Drive PID
double kP_drive = 0.02, kI_drive = 0, kD_drive = 0;

// Turn PID
double kP_turn = 0.13, kI_turn = 0, kD_turn = 0;
