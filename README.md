# Embedded-Real-Time-Systems-Practice

This repository contains various embedded systems projects focusing on real-time control, navigation, and sensor integration for autonomous systems. Each project demonstrates the integration of sensors, actuators, and control algorithms for specific embedded applications.

---

## Table of Contents

- [Overview](#overview)
- [Projects](#projects)
  - [LED_Switch_Sequence.ino](#led_switch_sequenceino)
  - [Distance_LED_Display.ino](#distance_led_displayino)
  - [Sharp_Distance_Sensor.ino](#sharp_distance_sensorino)
  - [Hovercraft_Control_FSM.ino](#hovercraft_control_fsmino)
  - [Hovercraft_IMU_Control.ino](#hovercraft_imu_controlino)
  - [Hovercraft_PD_Control_IMU.ino](#hovercraft_pd_control_imuino)
  - [Hovercraft_OpticalFlow_IMU.ino](#hovercraft_opticalflow_imuino)
  - [Hovercraft_PD_Control_Camera_IMU.ino](#hovercraft_pd_control_camera_imuino)
  - [Hovercraft_PD_Control_With_OpticalFlow_IMU.ino](#hovercraft_pd_control_with_opticalflow_imuino)
- [Dependencies](#dependencies)
- [Setup](#setup)
- [Hardware Requirements](#hardware-requirements)
- [Usage](#usage)
- [License](#license)

---

## Overview

This repository demonstrates embedded systems projects that integrate real-time feedback control, sensor data processing, and motor actuation for various use cases like hovercraft control, LED sequencing, and distance measurement.

Each `.ino` file represents a standalone project or module that can be executed on an Arduino-compatible microcontroller.

---

## Projects

### LED_Switch_Sequence.ino

**Description:**  
This project controls a sequence of LEDs based on a switch input. The sequence iterates through four states, illuminating different LEDs in each state.

**Key Features:**
- Simple GPIO and digital write/read functionality.
- Implements delays to control sequence timing.
- Great for learning basic digital I/O.

---

### Distance_LED_Display.ino

**Description:**  
Displays distance measured (in meters) using LEDs. The LED pattern changes based on predefined distance ranges, with higher ranges illuminating specific LEDs.

**Key Features:**
- Distance-to-LED mapping logic.
- Use of GPIO pins for both input and output.
- Demonstrates practical distance visualization using LEDs.

---

### Sharp_Distance_Sensor.ino

**Description:**  
Reads distance data from a Sharp IR distance sensor and outputs the computed distance over the Serial Monitor.

**Key Features:**
- Converts analog sensor values into real-world distance using a calibration equation.
- Filters and bounds sensor readings to handle out-of-range values.
- Outputs distance at regular intervals.

---

### Hovercraft_Control_FSM.ino

**Description:**  
A state machine-based hovercraft control system. Controls lift fan and directional motors through defined states like `START`, `ACTIVATE`, `MOVE`, and `DEACTIVATE`.

**Key Features:**
- Implements a finite state machine for structured state transitions.
- Controls PWM motors for lift and thrust.
- Provides modularity for hovercraft control.

---

### Hovercraft_IMU_Control.ino

**Description:**  
Introduces IMU-based control for the hovercraft. The project integrates real-time orientation feedback from an MPU9250 IMU to maintain hovercraft stability.

**Key Features:**
- IMU integration for yaw control.
- Basic PD control for rotational stability.
- Real-time Serial Monitor feedback for debugging.

---

### Hovercraft_PD_Control_IMU.ino

**Description:**  
Enhances hovercraft IMU control with a PD feedback loop. Adjusts the hovercraft’s yaw based on rotational feedback for precise orientation control.

**Key Features:**
- Proportional-Derivative (PD) control for yaw stabilization.
- Configurable gain parameters for tuning.
- Includes structured motor control logic.

---

### Hovercraft_OpticalFlow_IMU.ino

**Description:**  
Integrates optical flow sensors and an IMU to track hovercraft motion in x-y space. Computes translational motion using camera slip data.

**Key Features:**
- Combines optical flow data with IMU orientation.
- Computes velocity and absolute position for the hovercraft.
- Low-pass filters for smooth motion estimation.

---

### Hovercraft_PD_Control_Camera_IMU.ino

**Description:**  
Combines PD control for rotational stability with optical flow data for precise hovercraft motion. Tracks and compensates for drift while maintaining directional stability.

**Key Features:**
- Uses IMU yaw feedback for rotational control.
- Tracks translational drift using optical flow sensors.
- Modular state machine implementation for navigation.

---

### Hovercraft_PD_Control_With_OpticalFlow_IMU.ino

**Description:**  
This is the most advanced hovercraft control system in the repository. Combines PD rotational control, optical flow-based x-y tracking, and position control using IMU data.

**Key Features:**
- Real-time hovercraft position and velocity control.
- Combines IMU and optical flow data for robust navigation.
- Implements multi-stage navigation goals with precise motion tuning.

---

## Dependencies

The projects use the following Arduino libraries:
- [MPU9250](https://github.com/bolderflight/MPU9250) for IMU integration.
- [PWMServo](https://www.arduino.cc/reference/en/libraries/pwmservo/) for servo motor control.
- [OpticalFlowCamera](https://github.com/your-library-link) for optical flow sensor data.
- `PeriodicAction` for periodic task management.

Install these libraries via the Arduino Library Manager or download them from the linked repositories.

---

## Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/Keeby-Astro/Embedded-Real-Time-Systems-Practice.git
   cd Embedded-Real-Time-Systems-Practice
   ```

2. Install the necessary libraries.

3. Open the desired `.ino` file in the Arduino IDE, select your microcontroller board, and upload.

---

## Hardware Requirements

- **Microcontroller:** Arduino Mega, Teensy, or equivalent with sufficient GPIO pins.
- **Sensors:**
  - **IMU:** MPU9250 for orientation and angular velocity feedback.
  - **Optical Flow Cameras:** For x-y motion tracking.
  - **Sharp IR Distance Sensor** (for distance-related projects).
- **Actuators:**
  - Brushless motors with ESCs (Electronic Speed Controllers).
  - LEDs for display and indicators.
  - PWM-controlled servo motors.
- **Power Supply:** Ensure appropriate power for motors and sensors.

---

## Usage

1. **Load a project:** Select and upload the `.ino` file corresponding to the project you want to run.
2. **Connect peripherals:** Refer to the pin assignments in the source code and ensure sensors, motors, and other peripherals are properly connected.
3. **Monitor data:** Use the Serial Monitor in the Arduino IDE for debugging and runtime information.
4. **Tune parameters:** Adjust control gains (e.g., `Kp`, `Kd`) and thresholds to fit your hardware setup.

---

## License

This repository is licensed under the [MIT License](LICENSE). Feel free to use, modify, and distribute the code with proper attribution.
