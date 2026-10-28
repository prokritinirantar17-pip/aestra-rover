# Aestra Agricultural Rover

*An intelligent rover designed for autonomous navigation, real-time environmental sensing, and remote monitoring through an IoT dashboard.*

---

## Project Overview

The Autonomous Smart Agricultural Monitoring Rover combines autonomous movement, agricultural sensing, and IoT-based monitoring in one mobile platform. It is designed to navigate while avoiding obstacles, collect useful environmental readings, and present the information through a dashboard.

## Key Features

- **Autonomous Navigation with Obstacle Avoidance:** Moves autonomously while detecting and avoiding obstacles.
- **Smart Agricultural Sensing:** Monitors soil moisture, light intensity, and temperature.
- **IoT Dashboard Monitoring:** Provides a visual interface for viewing sensor readings and interacting with the rover.

## Team Information

**Team Name:** Aestra

### Team Members

- Nirantar Prokriti
- Ariyan Tahsin Pine

### Individual Contributions

#### Nirantar Prokriti

Prokriti primarily managed the rover's mechanical assembly, ensuring that components and their wiring were properly secured and connected. She also contributed to the rover's nature-inspired physical design, recorded and edited the rover's demonstration video, and managed the GitHub repository to organize the project's code and documentation.

#### Ariyan Tahsin Pine

Ariyan primarily managed the rover's software development and code organization. He developed the interactive dashboard, enabling users to monitor sensor readings and interact with the rover through a visual interface. He also led the rover's CAD design and helped organize the project files on GitHub.

## Project Demonstration

Youtube link: https://youtu.be/NGa8FXGdQLw?si=oLqmobG9AG4ja6r_

## Setup and Operating Instructions

1. **Power on the rover:** Switch on the robot using its physical power button.
2. **Upload the code:** Connect the ESP32 to a computer using a USB-C cable and upload the required program.
3. **Connect to Wi-Fi:** Connect the rover and the dashboard system to the required Wi-Fi network.
4. **Open the Serial Monitor:** Open the ESP32 Serial Monitor to check its output and connection status.
5. **Launch the dashboard:** Open the rover's dashboard on the computer.
6. **Disconnect USB-C:** Once the code has been uploaded and the system is running correctly, disconnect the USB-C cable. The rover remains powered by its battery system.
7. **Monitor and operate:** Use the dashboard to view sensor readings and interact with the rover's available functions, including autonomous operation.

## Motor Selection Justification

The rover uses four individual DC motors, with one motor driving each wheel, to provide the movement required for navigation. This arrangement allows all four wheels to be driven by the robot's motor system.

A servo motor is used to move and rotate the soil-moisture sensor probe into position so it can reach the ground and take soil-moisture readings. The DC motors support the rover's movement, while the servo provides controlled movement for the sensing mechanism.
