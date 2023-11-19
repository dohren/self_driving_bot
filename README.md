# self-driving-bot

## Introduction

This documentation provides an overview of a ROS2 package found in a specific repository designed for transforming an Ikea changing table into a self-driving robot. This innovative project takes a budget-friendly approach by utilizing an inexpensive, second-hand Ikea changing table and equipping it with essential robotic components to enable autonomous movement.


![IKEA TABLE SIMULATION](assets/images/ikea_table_simulation.png  "IKEA table simulation")
![IKEA TABLE PHYSICAL](assets/images/ikea_table_physical.png  "IKEA table physical")

## Core Components and Assembly
### 1. Ikea Changing Table:

The project’s foundation is built on a used Ikea changing table, acquired at an approximate cost of 15 Euros.

### 2. Motion Mechanism:

Two 12V DC Motors: Installed at the rear to facilitate the robot’s movement.
Two Furniture Casters: Attached at the front for easy and smooth directional shifts.
Two Roof Battens: Utilized possibly for structural reinforcement or additional mounting surfaces, although the specific application is not mentioned in the initial information provided.

### 3. Control System:

Raspberry Pi 4 (4GB): Acts as the central processing unit, managing and executing the ROS2 packages and algorithms for autonomous navigation.
ESP32: Works in conjunction with a motor shield to control and manage the operations of the 12V DC motors.

### 4. Sensing and Navigation:

ydlidar: Mounted upside down on the lower plate, it scans the robot’s environment to assist in navigation and obstacle avoidance.
Camera: Installed on the robot but not employed for navigation or control functions in the current configuration.
Motor Decoders: Attached to the 12V DC motors to gather data on the robot’s movement, assisting in precise navigation and control.

## Working Mechanism

The autonomous Ikea table robot is powered and controlled by the synergy of the Raspberry Pi and ESP32. The Raspberry Pi, loaded with ROS2 packages, processes the data received from the ydlidar and motor decoders to make real-time decisions for navigation and obstacle avoidance.

The 12V DC motors, governed by the ESP32 with a motor shield, are responsible for the robot’s movement. The ydlidar, strategically placed, continually scans the environment, feeding data back to the central processing system for informed and responsive navigational adjustments.

## Conclusion

This ROS2 package exemplifies an innovative, budget-conscious approach to autonomous robotics. By converting an Ikea changing table into a self-driving entity, this project demonstrates the potential for creating functional robotic systems without a significant financial investment while promoting recycling and upcycling practices in robotics development.