# Three-Axis Camera Stabilization Gimbal

A custom-built three-axis active camera stabilization platform using brushless direct drive control. This project implements Field Oriented Control (FOC) for smooth, precise motor control to maintain stable footage during movement.

## Overview

This gimbal uses real-time sensor data from an MPU6050 IMU to counteract unwanted camera movement, maintaining level footage even while walking or running. The system maintains a predefined home position (pitch: 90°, roll: 0°, yaw: -13°) and continuously corrects any deviation using the SimpleFOC library.

## Features

- Three-axis stabilization (pitch, roll, yaw)
- Smooth homing sequence on startup
- Real-time orientation correction using FOC
- Low-cost alternative to commercial gimbals ($300-800)

## Hardware Requirements

| Component                | Quantity | Description                    |
| ------------------------ | -------- | ------------------------------ |
| Teensy 4.1               | 1        | Main microcontroller           |
| 2208 Brushless DC Motors | 3        | One per axis                   |
| MPU6050 IMU              | 2        | 6-axis accelerometer/gyroscope |
| FOC Mini Driver          | 3        | Motor drivers                  |
| 7.4V LiPo Battery        | 1        | 2000mAh capacity               |
| 5V Regulator             | 1        | Power regulation               |
| 0.96" LCD                | 1        | Status display                 |

## Software Requirements

- PlatformIO or Arduino IDE
- SimpleFOC Library
- Teensy Loader

## Setup

### 1. Install Required Packages

```bash
# Install base development tools
sudo pacman -S base-devel git

# Install Arduino CLI (optional, for command-line builds)
sudo pacman -S arduino-cli

```

### 2. Install Teensy udev Rules

Teensy requires udev rules to be recognized without root privileges:

```bash
# Download and install Teensy udev rules
sudo curl -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add your user to the dialout group
sudo usermod -aG dialout $USER
sudo usermod -aG uucp $USER

```

### 3. Install Teensy Loader

```bash
# Install from AUR (using yay or paru)
yay -S teensy-loader-cli

# Or build from source
git clone https://github.com/PaulStoffregen/teensy_loader_cli.git
cd teensy_loader_cli
make
sudo cp teensy_loader_cli /usr/local/bin/
```

# Wiring Diagram

```
Teensy 4.1
├── Pin 2,3,4    → FOC Mini #1 (Roll Motor)
├── Pin 5,6,7    → FOC Mini #2 (Pitch Motor)
├── Pin 8,9,10   → FOC Mini #3 (Yaw Motor)
├── Pin 18 (SDA) → MPU6050 SDA
├── Pin 19 (SCL) → MPU6050 SCL
├── VIN          → 5V Regulator Output
└── GND          → Common Ground

7.4V LiPo → FOC Mini VIN (all three)
         → 5V Regulator Input
```

### Current Challenges

1. **PID Tuning**: Finding optimal P, I, D values requires patience. Too high causes oscillation; too low results in slow response.

2. **IMU Noise**: The MPU6050 readings are affected by magnetic interference from the BLDC motors, causing jitter in stabilization.

### Planned Improvements

- [ ] Add magnetic shielding around IMU
- [ ] Implement AS5600 magnetic encoders for motor position feedback
- [ ] Utilize second MPU6050 on gimbal body for feedforward control
- [ ] Implement complementary or Kalman filter for sensor fusion
- [ ] Add joystick control for manual camera movement

