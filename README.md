# Fuzzy Logic DC Motor Control System

A technical implementation of a Mamdani-style Fuzzy Logic Controller (FLC) designed for an Arduino-based system. This project utilizes real-time feedback from an ultrasonic sensor and a potentiometer to regulate the speed and direction of a DC motor through high-frequency PWM.

## Project Overview

The controller processes two inputs to determine the optimal motor response. It is designed to maintain system stability by evaluating linguistic variables through a 9-rule inference engine. The use of a custom PWM frequency (245.5 Hz) ensures improved torque characteristics and reduced acoustic noise from the motor windings.

## Hardware Configuration

### Components
* Microcontroller: Arduino (Uno, Nano, or compatible AVR board)
* Motor Driver: L298N or similar H-Bridge
* Sensors: HC-SR04 Ultrasonic Sensor, 10k Ohm Potentiometer
* Actuator: DC Gear Motor

### Pin Mapping

| Component | Pin | Function |
| :--- | :---: | :--- |
| POT_PIN | A5 | Potentiometer Analog Input |
| US_TRIG_PIN | 5 | Ultrasonic Trigger |
| US_ECHO_PIN | 6 | Ultrasonic Echo |
| MOTOR_PWM_PIN | 9 | PWM Speed Control |
| MOTOR_IN1_PIN | 10 | Direction Control 1 |
| MOTOR_IN2_PIN | 11 | Direction Control 2 |

## Control System Design

### 1. Universe of Discourse
The system operates within the following defined ranges:
* Potentiometer Angle: 100.0 to 170.0
* Ultrasonic Distance: 4.0 cm to 34.0 cm
* Motor Output: -35.0 to 35.0 (Directional PWM)

### 2. Fuzzy Sets and Membership Functions
The input and output variables are divided into the following linguistic terms:

**Potentiometer (Input):**
* High Angle (100 - 135)
* Balanced Angle (110 - 160)
* Low Angle (135 - 170)

**Ultrasonic Sensor (Input):**
* Low Distance (4.0 - 20.5)
* Medium Distance (10.0 - 30.0)
* High Distance (20.5 - 34.0)

**Motor Output (Output):**
* Counter-Clockwise (CCW)
* No Rotation
* Clockwise (CW)

### 3. Rule Base
The system evaluates the following inference rules:

| Rule | Potentiometer | Ultrasonic | Motor Output |
| :--- | :--- | :--- | :--- |
| 1 | High Angle | Low Distance | Clockwise |
| 2 | Balanced Angle | Low Distance | Clockwise |
| 3 | Low Angle | Low Distance | No Rotation |
| 4 | High Angle | Medium Distance | Clockwise |
| 5 | Balanced Angle | Medium Distance | No Rotation |
| 6 | Low Angle | Medium Distance | Counter-Clockwise |
| 7 | High Angle | High Distance | No Rotation |
| 8 | Balanced Angle | High Distance | Counter-Clockwise |
| 9 | Low Angle | High Distance | Counter-Clockwise |

## Technical Implementation

### PWM Optimization
The code uses the `AVR_PWM` library to override default Arduino PWM frequencies. This allows for a specific carrier frequency of 245.5 Hz, which is often more efficient for small DC motors and reduces electromagnetic interference.

### Deadband Management
A software deadband of 1.0 is implemented to prevent "motor jitter" or hunting when the fuzzy output is near zero, protecting the H-bridge and reducing power consumption.

## Installation

1. Clone this repository to your local machine.
2. Ensure the following libraries are installed in your Arduino IDE:
   * **FuzzyLibrary** (eFLL)
   * **AVR_PWM**
3. Open the `.ino` file and upload it to your board.
4. Open the Serial Monitor at 115200 baud to view system telemetry.

## License
This project is released under the MIT License.
