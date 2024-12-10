# Robert - Hell Machine

**FPV Car Project at UCU**

## Introduction

Welcome to the **Robert - Hell Machine** project repository. This Readme provides instructions and main info for using our self-driving car. The Robert - Hell Machine was developed by students of the Applied Sciences Faculty at the Ukrainian Catholic University (UCU).

## Authors 

- **Dzoban Maksym**
- **Lysyk Lev-Fedir**
- **Kotliarchuk Oksana**
- **Pakholoch Viktor**

## Project History

Robert dates back to 2019 when Mykola Morgunenko (@Myralllka) and Yuriy Pasichnyk (@Fenix) initiated the project as part of the "Principles of Computer Organization" course at UCU. Their initial prototype laid the work for next iterations, such as real-time operating systems and advanced motor control mechanisms under the mentorship of Professor Oleg Farenuk and Volodymyr Davidenko from SoftServe. In the following years, other contributors changed microcontroller to ESP32, added many new features such as LIDAR, line following. During our course, we implemented Pirst Person View and organized work on the project.

## System Architecture

The system architecture is divided into several interconnected modules:

1. **Control Unit**: Uses the ESP32-S3-Wroom1 microcontroller to process inputs from the ELRS transmitter and manage the vehicle's movement and stability.

2. **Motor Control Module**: BLD-300B BLDC motor drivers to control the speed and direction of each wheel. PWM signals from the microcontroller allow precise speed regulation.

3. **Sensor Integration**: Includes encoders (AS5600), and Hall sensors to collect data and monitor wheel rotation. This helps the vehicle avoid obstacles and measure speed accurately.

4. **Pneumatic System**: Uses a pneumatic compressor and adjustable valves to change the vehicle's suspension. This improves stability and allows the vehicle to adapt to different terrains.

5. **Communication Module**: First Person View system for live video transmission.

## Modules and Components

### Control Unit

The Control Unit uses the ESP32-S3-Wroom1 microcontroller to manage the vehicle's operations. It processes signals from the ELRS transmitter and controls the vehicle's movement and stability. The microcontroller has two cores, which allow it to handle multiple tasks at the same time. This ensures that the vehicle runs smoothly and responds quickly to commands and sensor inputs.

### Motor Control Module

The Motor Control Module uses  4 BLD-300B BLDC motor drivers to control the four hub motors of the vehicle. Each motor driver receives PWM (Pulse Width Modulation) signals from the microcontroller. These signals adjust the speed and direction of the motors based on real-time commands. Initially, the team used QS-909 JYQD motor drivers, but they were unreliable above 25V. Switching to BLD-300B drivers provided better performance and stability, even though they required more troubleshooting due to limited documentation.

### Sensor Integration

- **Encoders (AS5600)**: Attached to each of 2 wheel bases, encoders measure how much each wheel has turned. This information is crucial for tracking the vehicle's speed and position accurately.
- **Hall Sensors**: These sensors monitor the speed of each wheel by detecting changes in the magnetic field.

### Pneumatic System

The Pneumatic System uses a compressor and adjustable valves to control the vehicle's suspension. By adjusting the suspension, the vehicle remains stable and can adapt to different surfaces, improving both comfort and performance. It can be turned on by a command from RC controller.
### Communication Module

The main communication module is the FPV (First Person View) system. It includes a BetaFPV ELRS Nano RX receiver connected to a Taranis Tx12 controller, which receives control signals from the controller, allowing the operator to remotely manage the vehicle's movements. The vehicle is equipped with a 4.8 GHz camera that streams real-time video to the operator, providing a clear view of the vehicle's surroundings. The video captured by the camera is transmitted to the operator's FPV monitor using a 4.8 GHz transmitter, enabling the operator to see what the vehicle sees and improving navigation and control.

### Specifications

#### Vehicle Characteristics

- **Maximum Speed**: 5 m/s
- **Dimensions**:
  - **Length**: 1.2 m
  - **Width**: 0.4 m
  - **Height**:
    - **Before Suspension**: 0.3 m
    - **After Suspension**: 0.45 m
- **Weight**: 45 kg (including batteries and pneumatic suspension)


## Usage

To use the Robert - Hell Machine, first turn on both the 12V and 36V batteries and ensure that all indicators are functioning correctly. Use the Taranis Tx12 controller to send movement commands to the vehicle and monitor the FPV feed for real-time video from the vehicle's camera. When you want to add AI recognition or computer vision features, disable the FPV mode in code.
---
We would like to thank our mentors during this semester, Andriy Ozhovych and Oleh Ferenyuk, for their support.
For any questions, support, or further assistance, please contact the authors!
