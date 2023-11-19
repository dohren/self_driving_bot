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

# Robots


## Box Robot

<img src="assets/images/box_physical.jpg "  width="60%" height="30%">

### Parts
|                 | Product                               | Seller   | Price |
| :-------------: | :-----------------------------------: | :------: | :---: |
| Motor           | 2 x DC 12 V DIY Encoder Getriebemotor | [Amazon](https://www.amazon.de/Getriebemotor-Halterung-Magnetgetriebe-Drehzahlreduzierung-Vollmetallgetriebe/dp/B07WRYQZNR/ref=asc_df_B07WRYQZNR)                                                      |  40,39 € |
| Micro Processor | ESP32 Dev Board | [Amazon](https://www.amazon.de/AZDelivery-NodeMCU-Development-Nachfolgermodell-ESP8266/dp/B074RGW2VQ)      |   9,99 € |
| Mini Computer   | Raspberry Pi 4 Model B 4GB   | [Amazon](https://www.amazon.de/Raspberry-SD-Karte-Ultimatives-Quad-Core-unterst%C3%BCtzt/dp/B082PSBBMM)    | 149,99 € |
| Lidar Sensor    | IdLidar LD19                 | [Amazon](https://www.amazon.de/InnoMaker-Extra-Small-Resistance-brushless-Navigation/dp/B09VKZ9YNT)        |  90,00 € |
| 12v DC Akku     | Eleoption DC12300 Super-Akku | [Amazon](https://www.amazon.de/3000-mAh-dc12300-Super-wiederaufladbar-St%C3%B6psel-baterias/dp/B06ZYQPK3P) |  29,99 € |
| 5v USB Akku     | Power Bank 26800mAh Akku     | [Amazon](https://www.amazon.de/Power-Bank-26800mAh-Externer-Akku/dp/B08FCLJ9N1) |  19,99 € |
| Chassis         | Box                          | [laublust](https://www.laublust.de/products/holzkiste-mit-deckel-natur-unbehandelt-fsc%C2%AE-erinnerungsbox-gross-klein-aufbewahrungskiste-aus-holz-diverse-grossen) |  9,99 € |
| Plugs             | QitinDasen 12              | [Amazon](https://www.amazon.de/QitinDasen-Verbinder-Terminalblock-M%C3%A4nnlich-Weiblich/dp/B07MF69TMX)    |   6,98 € |
| Cables            | Breadboard jumpers         | [Amazon](https://www.amazon.de/dp/B0BRM568FM/ref=sspa_dk_detail_0) |  9,99 € |
| USB Cables        | Micro USB / USB C          |                          |  10,00 € |
| Front Roller      | Micro USB / USB C          | [Amazon](https://www.amazon.de/Smart-Chassis-Motors-Encoder-Battery/dp/B01LXY7CM3)                         |  16,79 € |
| Gamepad          | Wireless Controller         | [Amazon](https://www.amazon.de/iNNEXT-Controller-USB-Empf%C3%A4nger-Nintendo-Spiele-Unterst%C3%BCtzung/dp/B07FTCWBSY)                         |  11,50 € |
| Total             |                            |                          | 365.23 € |

### Software

|                    | Software                              | 
| :-------------:    | :-----------------------------------: | 
| Raspberry OS       | [Ubuntu for Raspberry PI](https://ubuntu.com/download/raspberry-pi/thank-you?version=22.04.3&architecture=desktop-arm64+raspi) |
| Ros2               | [ROS2 Humble](https://docs.ros.org/en/humble/index.html)    |
| Navigation         | [Ros2 Nav2](https://navigation.ros.org/)                    |
| micro-ROS          | [micro-ROS](https://github.com/micro-ROS/micro_ros_arduino) |
| Visual Studio Code | [VSCode](https://code.visualstudio.com/)                    |
| Rufus              | [micro-ROS](https://rufus.ie/de/)                           |
| Arduino IDE        | [Arduino IDE](https://www.arduino.cc/en/software)          |   
| Lidar Driver       | [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)          |         




