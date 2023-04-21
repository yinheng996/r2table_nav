# Food Delivery Robot


This is an open-source project for a Food Delivery Robot, developed as part of the __EG2310 Fundamentals of System Design__ module, Sem 2 AY22/23, under the _Innovation and Design Programme (iDP)_ at National University of Singapore (NUS).

The project team, Studio 2 Group 3, comprises of Loh Yin Heng, Wang Bo, Jeanette Sim Yu, Hang Jin Guang, and Ananya Agarwal.

## Description
Our objective is to build a food delivery robot that seamlessly navigates through a restaurant, collects an order, and delivers it to the designated table. Our design is based on the Turtlebot 3 platform and utilizes ROS 2 Foxy as its operating system. We employ the MQTT messaging protocol for inter-system communication.

To achieve our objective, we have built a dispenser equipped with a keypad that allows for the selection of the table. Our dispenser is powered by an ESP32 microcontroller, which serves as the brain of the dispenser. This allows for seamless communication between the dispenser and the robot, and enables us to easily modify and update the dispenser's functionality.

This repository houses the code for robot navigation, MQTT communication and dispenser functions.
You may check out our project report under the `/documentations` folder for more details on our robot and its mission.

## File Organisation
TODO: update after finalised

## Operating Instructions
### Installation
1. Follow the instructions here to setup ROS 2 Foxy on laptop and turtlebot. <br/>
    (You may stop once you are able to teleoperate the turtlebot from your computer.) <br/> 
    https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. Follow the instructions here to setup MQTT X on your laptop. <br/>
    (We are using a cloud-based MQTT broker for this project (MQTT X) for better consistency.) <br/> 
    https://mqttx.app/docs/downloading-and-installation
3. Follow the instructions here to setup ESP32 software. <br/> 
    https://esp32io.com/tutorials/esp32-software-installization
5. Follow the instructions here to setup Arduino IDE on your laptop. <br/> 
    https://www.arduino.cc/en/software <br/>
    Install the follow libraries on your Arduino IDE:
    * PubSubClient by Nick O'Leary
    * EspMQTTClient by Patrick
    * Keypad by Mark
    * ESP32Servo by Kevin
    * ezButton by ArduinoGetStarted.com
5. Follow the instructions here to setup Paho-MQTT on your laptop. <br/>
https://pypi.org/project/paho-mqtt/
6. Refer to our `/hardware` folder for the schematics of the circuits that we use to replicate our robot system. <br/>

__Important__: Please note that our project has been developed using Ubuntu 20.04.4, ROS2 Foxy, Arduino 2.1.0 and Python 3.6, as well as a Raspberry Pi 3B+ and a DOIT ESP32 Devkit V1 Board. As such, modifications may be necessary when using other software systems or hardware platforms.

### Setup

1. On your Ubuntu Machine, clone our repository into your Home directory. Compile the workspace. <br/>

    ```
    cd colcon_ws/src/auto_nav
    git clone https://github.com/yinheng996/r2table_nav.git 
    cd colcon_ws
    colcon build
2. Copy the `/py_pubsub` folder onto the Raspberry Pi. Compile the workspace. <br/>
    
    ```
    ssh ubuntu@<RPi IP address>
    scp -r <path to r2table_nav directory>/py_pubsub ubuntu@<RPi IP address>:~/turtlebot_ws/src 
    cd turtlebot3_ws
    colcon build 
3. We will use Arduino IDE to setup the ESP32.
    1. Open the code for the ESP32 in the Arduino IDE. This will be located in the repository that you cloned in step 1. In the code, make any necessary changes or modifications.
    2. Connect the ESP32 to your computer using a USB cable.
    3. In the Arduino IDE, select the correct board and serial port. To do this, navigate to `Tools` -> `Board` and select `ESP32 Dev Module`. Then, navigate to `Tools` -> `Port` and select the correct serial port.
    4. Click on the `Upload` button in the Arduino IDE to upload the code onto the ESP32.
    5. Once the upload is complete, disconnect the ESP32 from your computer and power it using an external power source. The code should now be running on the ESP32.

## System check
Before you start using the robot, make sure everything is running properly by using the factory_test package. Follow these instructions to check the system:

#### For your Turtlebot
##### In one terminal
    ssh ubuntu@<RPi IP address>
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
##### In another terminal
    ssh ubuntu@<RPi IP address>
    ros2 launch hardware_bringup hardware.launch.py
Finally, we will run the script in your laptop to check that the whole system is working
#### For your Laptop
##### In one terminal
    ros2 run auto_nav factory_test

Follow the instructions printed on your terminal, and if everything works out fine, it means the system is ready to go.

## Calibration and Configuration
TODO: after tidying code

## Running the Code
A total of 4 terminals would be required to run the code, in addition to the MQTT X GUI.
##### In Terminal 1: To bring up TurtleBot3
    ssh ubuntu@<RPi IP address>
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
##### In Terminal 2: To host Bot Limit Switch Publisher
    ssh ubuntu@<RPi IP address>
    cd <path to r2table_nav directory>/py_pubsub_robot
    python3 limit_switch.py 
##### In Terminal 3: To host Bot LDR Publisher
    ssh ubuntu@<RPi IP address>
    cd <path to r2table_nav directory>/py_pubsub_robot
    python3 IR.py
##### In Terminal 4: To run navigation code
    cd <path to r2table_nav directory>/table_nav/table_nav
    python3 r2tcheckpt_nav.py
#### In MQTT X:
