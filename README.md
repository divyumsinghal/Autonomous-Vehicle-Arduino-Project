# Autonomous-Vehicle-Project (In Progress)

A small Autonomous Vehicle controlled using an Arduino Board, using a wireless connection and an interactive GUI built using processing.

## Overview

Welcome to the Autonomous Vehicle Project! This project implements an Arduino-based autonomous vehicle capable of navigating its environment, avoiding obstacles, following lines, and displaying patterns on an LED matrix. It leverages various sensors, actuators, and the Huskylens vision sensor for intelligent object detection. The vehicle can be remotely controlled through a graphical user interface (GUI) built using the Processing programming language.

## Table of Contents

- [Project Structure](#project-structure)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Features](#features)
   - [Obstacle Avoidance](#obstacle-avoidance)
   - [Line Following](#line-following)
   - [Remote Control](#remote-control)
   - [LED Matrix Display](#led-matrix-display)
   - [Huskylens Vision Sensor](#huskylens-vision-sensor)
   - [Connection Setup and Server Communication](#connection-setup-and-server-communication)
   - [Speed Tracking using Encoders](#speed-tracking-using-encoders)

- [Troubleshooting](#troubleshooting)
- [License](#license)

## Project Structure

The project is organized into two main components:

1. __Arduino Code (`autonomous_vehicle_project.ino`):__

   - The main Arduino sketch that controls the vehicle's behavior.
   - Implements the algorithm with various functions for obstacle avoidance, motor control, speed tracking, and communication with the Huskylens sensor.
   - Integrates LED matrix display functions and checks for client connections.

2. __Processing Code (`processing_gui.pde`):__

   - A Processing sketch that serves as a graphical user interface (GUI) for remotely controlling the autonomous vehicle.
   - Utilizes the ControlP5 library for GUI elements, including buttons and sliders.
   - Sends commands to the Arduino through a network connection.

## Hardware Requirements

1. Arduino Board (e.g., Arduino Uno)
2. Infrared Sensors
3. Ultrasonic Sensor
4. Motors and Motor Driver
5. Wheel Encoders
6. LED Matrix
7. Huskylens Vision Sensor
8. WiFi Module (WiFiS3)
9. Chassis, Wheels, and Power Supply

## Software Requirements

1. Arduino IDE
2. Processing IDE
3. Necessary Libraries

## Installation

1. Clone the repository:

```bash {"id":"01HQAZKC8W1K65H8E20W87QJW6"}
git clone https://github.com/divyumsinghal/Autonomous-Vehicle-Project.git

```

2. **Arduino Code:**

   - Open the Arduino sketch (`autonomous_vehicle_project.ino`) in the Arduino IDE.
   - Install the required libraries, e.g.:
      - `HUSKYLENS` for communicating with the Huskylens sensor.
      - `Arduino_LED_Matrix` for handling the LED matrix; etc.

   - Connect the Arduino board to your computer and upload the sketch.

3. **Processing Code:**

   - Open the Processing sketch (`ProcessingGui.pde`) in the Processing IDE.
   - Run the Processing sketch to launch the GUI.

## Usage

1. Connect the hardware components as per the pin configurations in the Arduino sketch.
2. Power on the autonomous vehicle.
3. Launch the Processing GUI.
4. Enter the WiFi credentials in the Arduino sketch.
5. Control the vehicle using the GUI buttons and sliders.

## Configuration

### Arduino Configuration

1. **WiFi Credentials:**

   - Replace the placeholders in the Arduino sketch (`.ino`) with your WiFi credentials:

2. **Other Configurations:**

   - Adjust other configuration parameters in the sketch, such as pin assignments, speed limits, and obstacle detection thresholds, according to your hardware setup.

### Processing Configuration

1. **Arduino IP Address:**

- Replace the placeholder in the Processing sketch (`processing_gui.pde`) with the actual IP address of your Arduino:

```java {"id":"01HQAZKC8W1K65H8E20ZR3QTR7"}
String serverAddress = "replace-with-arduino-ip-address";
```

- You should get it printed in your Serial when the code is run.

2. **GUI Customization:**

   - Customize the GUI layout, button positions, and labels based on your preferences.

## Features

### Obstacle Avoidance

The vehicle is equipped with an ultrasonic sensor for obstacle detection, avoidance amd safe following. The algorithm implements intelligent obstacle avoidance logic, adjusting the vehicle's speed based on the proximity of obstacles. If an obstacle is too close, the vehicle stops to prevent collisions.

### Line Following

Utilizing the two IR sensors, the vehicle can follow lines marked on the ground. The `keepMovingCheckingIRSensors()` function in the algorithm processes the data and adjusts the vehicle's motion to stay on the line.

### Remote Control

The Processing GUI provides a user-friendly interface for remotely controlling the autonomous vehicle. It includes buttons for forward and backward motion, turning left and right, starting and stopping the vehicle, adjusting speed, and displaying predefined patterns on

### LED Matrix Display

The project incorporates an LED matrix that serves both functional and expressive purposes. The LED matrix is used to display various predefined patterns, such as a smiley face, heart, and a custom "W5" pattern. These patterns are loaded onto the matrix using functions like `displaySmiley()`, `displayHeart()`, and `displayW5()`. The LED matrix enhances the visual appeal of the autonomous vehicle and can be customized with additional patterns as needed.

### Huskylens Vision Sensor

The project integrates the Huskylens vision sensor, which provides advanced object detection and tracking capabilities. Using the `HUSKYLENS` library, the Arduino communicates with the Huskylens sensor via I2C. The vision sensor can recognize and track objects, including blocks and arrows, offering detailed information such as position coordinates and unique IDs. This functionality enables the autonomous vehicle to interact intelligently with its environment, responding to detected objects in real-time.

### Connection Setup and Server Communication

The autonomous vehicle is equipped with WiFi capabilities using the NINA module. The `connectionSetup()` function establishes a connection to a WiFi network, obtaining an IP address for the Arduino on the network. Additionally, the vehicle can communicate with clients over the WiFi network using the `checkServer()` function, which checks for incoming client connections and sends a "Hello Client" message to the connected client.

### Speed Tracking using Encoders

The system employs motor encoders to track the speed of each wheel. The `speedTrackingUsingEncoders()` function calculates the speeds of the left and right motors based on pulse counts from the encoders. This information is crucial for maintaining a consistent and controlled motion of the vehicle.

These features collectively contribute to the overall functionality and versatility of the autonomous vehicle, making it capable of intelligent navigation, object detection, and interactive communication. The modular design allows for easy customization and expansion of features to suit specific project requirements.

## Troubleshooting

If you encounter issues:

- Double-check hardware connections.
- Ensure correct library installations.
- Adjust configuration parameters in the Arduino sketch.

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).
