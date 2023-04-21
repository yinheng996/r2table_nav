# Food Delivery Robot


This is an open-source project for a Food Delivery Robot, developed as part of the __EG2310 Fundamentals of System Design__ module, Sem 2 AY22/23, under the _Innovation and Design Programme (iDP)_ at National University of Singapore (NUS).

The project team, Studio 2 Group 3, comprises of Loh Yin Heng, Wang Bo, Jeanette Sim Yu, Hang Jin Guang, and Ananya Agarwal.

## Description
Our objective is to build a food delivery robot that seamlessly navigates through a restaurant, collects an order, and delivers it to the designated table. Our design is based on the Turtlebot 3 platform and utilizes ROS 2 Foxy as its operating system. We employ the MQTT messaging protocol for inter-system communication.

To achieve our objective, we have built a dispenser equipped with a keypad that allows for the selection of the table. Our dispenser is powered by an ESP32 microcontroller, which serves as the brain of the dispenser. This allows for seamless communication between the dispenser and the robot, and enables us to easily modify and update the dispenser's functionality.

This repository houses the code for robot navigation, MQTT communication and dispenser functions.
You may check out our project report under the `/documentations` folder for more details on our robot and its mission.

## Operating Instructions
### Installation
1. Follow the instructions here to setup ROS 2 Foxy on laptop and turtlebot. <br/>
    (You may stop once you are able to teleoperate the turtlebot from your computer.) <br/> https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. Follow the instructions here to setup MQTT X on your laptop. <br/>
    (We are using a cloud-based MQTT broker for this project (MQTT X) for better consistency.) <br/> https://mqttx.app/docs/downloading-and-installation
3. Follow the instructions here to setup Paho-MQTT on your laptop <br/>
https://pypi.org/project/paho-mqtt/
4. Refer to our `/hardware` folder for the schematics of the circuits that we use to replicate our robot system. <br/>

__Important__: Please note that our project has been developed using Ubuntu 20.04.4, ROS2 Foxy, and Python 3.6, as well as a Raspberry Pi 3B+ and a DOIT ESP32 Devkit V1 Board. As such, modifications may be necessary when using other software systems or hardware platforms.

### Setup

1. Clone our repository into your Home directory. <br/>

    ```
    git clone https://github.com/yinheng996/r2table_nav.git 
2. Shift the entire folder hardware_bringup into your turtlebot3_ws using scp, then build the workspace, for example
    
    ```
    scp -r path_to_r2auto_nav/turtlebot3_ws/src/hardware_bringup ubuntu@(ip-address-of-pi):~/turtlebot3/src
    ssh ubuntu@(ip-address-of-pi)
    cd turtlebot3_ws
    colcon build 
3. cd into colcon_ws and colcon build the workspace to setup the ros package on your laptop

    ```
    cd path_to_r2auto_nav/colcon_ws
    colcon build 
## System check
Before you start using the robot, make sure everything is running properly by using the factory_test package. Follow these instructions to check the system:

#### For your Turtlebot
##### In one terminal
    ssh ubuntu@(ip-address-of-pi)
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
##### In another terminal
    ssh ubuntu@(ip-address-of-pi)
    ros2 launch hardware_bringup hardware.launch.py
Finally, we will run the script in your laptop to check that the whole system is working
#### For your Laptop
##### In one terminal
    ros2 run auto_nav factory_test

Follow the instructions printed on your terminal, and if everything works out fine, it means the system is ready to go.
