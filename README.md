# Autonomous-Vehicle-Project (In Progress)

Welcome to the world of autonomous vehicles, where innovation meets intelligence! The Autonomous Vehicle Project represents a pioneering endeavor in the realm of robotics, integrating cutting-edge technology with a dash of creativity to craft a marvel of modern engineering. Let's embark on a journey through the intricate design and ingenious functionalities of this project.

## Overview

Behold the Autonomous Vehicle Project, a symphony of Arduino brilliance orchestrated to create a vehicle that not only moves but navigates its surroundings with finesse. This project is a testament to human ingenuity, employing an array of sensors, actuators, and intelligent algorithms to enable autonomous operation. From evading obstacles to tracing lines and even captivating audiences with LED matrix displays, this vehicle is a multifaceted marvel designed to captivate and inspire.

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
   - [PID Controllers and Self-Organizing Algorithm](#pid-controllers-and-self-organizing-algorithm)

- [Troubleshooting](#troubleshooting)
- [License](#license)

## Project Structure

At the heart of this project lies a meticulously organized structure, comprising two core components:

1. **Arduino Code (`autonomous_vehicle_project.ino`):**
   - This is where the magic unfolds, with the Arduino sketch serving as the brain of the autonomous vehicle.
   - From intricate algorithms for obstacle avoidance and motor control to seamless integration with sensors and actuators, every line of code in this sketch is a testament to innovation.
   - Notably, it hosts the sophisticated PID controllers and a self-organizing algorithm that imbue the vehicle with intelligence and adaptability.

2. **Processing Code (`processing_gui.pde`):**
   - Complementing the Arduino sketch is the Processing code, a graphical user interface (GUI) that empowers users to interact with the vehicle effortlessly and control it remotely.
   - Through intuitive controls and real-time feedback, this GUI elevates the user experience, making remote control a breeze.
   - Utilizes the ControlP5 library for GUI elements, including buttons and sliders and meter library for the spedometer.
   - Sends commands to the Arduino through a network connection.

## Hardware Requirements

Fueling the brilliance of this project are the following hardware components:

1. Arduino Board
2. Infrared Sensors
3. Ultrasonic Sensor
4. Motors and Motor Driver
5. Wheel Encoders
6. LED Matrix
7. Huskylens Vision Sensor
8. WiFi Module (WiFiS3)
9. Chassis, Wheels, and Power Supply

## Software Requirements

To bring this project to life, you'll need:

1. Arduino IDE or equivalent setup on vs code/ other IDE
2. Processing IDE or equivalent setup on vs code/ other IDE
3. Necessary Libraries

## Installation

Embark on your journey with these simple steps:

1. **Clone the Repository:**
   - Begin by cloning the repository to your local machine using the provided command.

```bash
git clone https://github.com/divyumsinghal/Autonomous-Vehicle-Project.git
```

2. **Arduino Code:**
   - Open the Arduino sketch (`autonomous_vehicle_project.ino`) in the Arduino IDE.
   - Install the required libraries,  e.g.:
      - `HUSKYLENS` for communicating with the Huskylens sensor.
      - `Arduino_LED_Matrix` for handling the LED matrix; etc.
   - Connect your Arduino board and upload the sketch.

3. **Processing Code:**
   - Open the Processing sketch (`ProcessingGui.pde`) in the Processing IDE.
   - Run the sketch to launch the GUI and immerse yourself in the world of autonomous control.

## Usage

Experience the thrill of autonomy with these simple steps:

1. **Hardware Setup:**
   - Connect the hardware components according to the pin configurations specified in the Arduino sketch.
   - Power on the autonomous vehicle and prepare for adventure.

2. **Software Interaction:**
   - Launch the Processing GUI and witness the interface come to life.
   - Enter your WiFi credentials in the Arduino sketch for seamless connectivity.
   - Take control of the vehicle using the intuitive buttons and sliders provided in the GUI.

## Configuration

Tailor the project to your preferences with these configuration options:

### Arduino Configuration

1. **WiFi Credentials:**
   - Personalize the Arduino sketch by replacing placeholders with your WiFi credentials. Or if you have a board which can build its own WAP, then use that.

2. **Customization:**
   - Fine-tune various configuration parameters such as pin assignments, PID constants, speed limits, and obstacle detection thresholds to suit your specific requirements.

### Processing Configuration

1. **Arduino IP Address:**
   - Ensure seamless communication by replacing placeholders with the actual IP address of your Arduino in the Processing sketch.

2. **GUI Customization:**
   - Let your creativity shine as you customize the GUI layout, button positions, and labels to enhance user interaction.

## Features

### Obstacle Avoidance

Navigate with confidence thanks to intelligent obstacle avoidance logic that adjusts vehicle speed based on proximity to obstacles, preventing collisions and ensuring safe traversal of the environment.

### Line Following

Follow the path laid out before you with precision, utilizing infrared sensors to detect and track lines on the ground, ensuring smooth and accurate navigation.

### Remote Control

Empower users with seamless remote control capabilities through a user-friendly GUI, offering intuitive controls for forward and backward motion, turning, speed adjustment, and more.

### LED Matrix Display

Illuminate your surroundings with a dazzling LED matrix display, showcasing an array of predefined patterns ranging from smiley faces to custom designs, adding a touch of flair to your autonomous vehicle.

### Huskylens Vision Sensor

Unlock the power of advanced object detection and tracking with the Huskylens vision sensor, enabling real-time recognition and tracking of objects in the vehicle's environment.

### Connection Setup and Server Communication

Stay connected with the world around you through WiFi capabilities, establishing communication with clients over the network and ensuring seamless interaction with external devices.

### Speed Tracking using Encoders

Maintain precise control over vehicle speed with motor encoders that track wheel movement, providing crucial feedback for optimizing motion and ensuring smooth navigation.

### PID Controllers and Self-Organizing Algorithm

Marvel at the sophistication of PID controllers that fine-tune motor performance with precision, ensuring optimal operation in varying conditions. Delve into the realm of self-organizing algorithms that adapt and optimize vehicle behavior based on real-time feedback, pushing the boundaries of autonomous navigation.

## Troubleshooting

Encountered a bump in the road? Fear not! Troubleshooting tips are here to guide you through any challenges you may face, from hardware connections to library installations and configuration adjustments.

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).


Dive into the world of autonomous vehicles and let your imagination soar as you explore the endless possibilities of robotics and automation. The journey awaits!
