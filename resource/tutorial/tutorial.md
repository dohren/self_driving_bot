## Introduction

Installation
* https://docs.ros.org/en/humble/index.html
* https://navigation.ros.org/getting_started/index.html
* clone repo in workspace/src folder and source ~/ros2/ros2_workspace/install/setup.bash

YdLidar konfigurieren:
* https://github.com/YDLIDAR/YDLidar-SDK
* git clone git@github.com:dohren/ydlidar_ros2_driver.git and checkout ikea-tisch-fix

Run Simulation
* Download models
* fix /ros2_workspace/src/self_driving_bot/urdf/ikea_table.urdf path in filename
* add following line to ~/.bashrc
    export GAZEBO_MODEL_PATH=~/ros2/ros2_workspace/src/self_driving_bot/models:$GAZEBO_MODEL_PATH
    source ~/.bashrc 
* ros2 launch self_driving_bot self_driving_bot_simulation_navigation.py

Gazebo not opening
* ps -aef | grep gazebo 
* kill -9 pid