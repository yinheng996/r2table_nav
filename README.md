# Food Delivery Robot


This is an open-source project for a Food Delivery Robot, developed as part of the __EG2310 Fundamentals of System Design__ module, Sem 2 AY22/23, under the _Innovation and Design Programme (iDP)_ at National University of Singapore (NUS).

The project team, Studio 2 Group 3, comprises of Loh Yin Heng, Wang Bo, Jeanette Sim Yu, Hang Jin Guang, and Ananya Agarwal.

## Description
This project includes building a dispenser and modifying a turtlebot 3. The intersystem communication protocol is MQTT, press the keypad on the the dispesner to select table, dispenser will dispense food to the robto and robto naviaget to assigned table is designed to build a food delivery robot that navigates through a restaurant, picks up an order, and delivers it to the assigned table. The robot is based on the Turtlebot 3 platform and uses ROS 2 Foxy as its operating system. It also utilizes MQTT messaging protocol for communication.

## Operating Instructions
### Installation
1. Follow the instructions here to setup ROS 2 Foxy on laptop and turtlebot https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. Follow the instructions here to setup MQTT on your laptop

### Setup

1. Clone our repository into your Home directory. <br/>

    ```
    git clone https://github.com/yinheng996/r2table_nav.git
```
2. Shift the entire folder hardware_bringup into your turtlebot3_ws using scp, then build the workspace, for example
    
    ```
    scp -r path_to_r2auto_nav/turtlebot3_ws/src/hardware_bringup ubuntu@(ip-address-of-pi):~/turtlebot3/src
    ssh ubuntu@(ip-address-of-pi)
    cd turtlebot3_ws
    colcon build
  
    ```
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
  
