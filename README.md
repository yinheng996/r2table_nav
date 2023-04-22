# Food Delivery Robot


This is an open-source project for a Food Delivery Robot, developed as part of the __EG2310 Fundamentals of System Design__ module, Sem 2 AY22/23, under the _Innovation and Design Programme (iDP)_ at National University of Singapore (NUS).

The project team, Studio 2 Group 3, comprises of Loh Yin Heng, Wang Bo, Jeanette Sim Yu, Hang Jin Guang, and Ananya Agarwal.

## Description
Our objective is to build a food delivery robot that seamlessly navigates through a restaurant, collects an order, and delivers it to the designated table. Our design is based on the Turtlebot 3 platform and utilizes ROS 2 Foxy as its operating system. We employ the MQTT messaging protocol for inter-system communication.

To achieve our objective, we have built a dispenser equipped with a keypad that allows for the selection of the table. Our dispenser is powered by an ESP32 microcontroller, which serves as the brain of the dispenser. This allows for seamless communication between the dispenser and the robot, and enables us to easily modify and update the dispenser's functionality.

This repository houses the code for robot navigation, MQTT communication and dispenser functions.
You may check out our project report under the [/Documentations](https://github.com/yinheng996/r2table_nav/tree/main/Documentations) folder for more details on our robot and its mission.

## File Organisation
TODO: update after finalised

## Preparation
### Installation
1. Follow the instructions here to setup ROS 2 Foxy on laptop and TurtleBot. <br/>
    (You may stop once you are able to teleoperate the TurtleBot from your computer.) <br/> 
    https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. Follow the instructions here to download, install and setup MQTT X on your laptop. <br/>
    (We are using a cloud-based MQTT desktop client for this project (MQTT X) for better consistency.) <br/> 
    https://github.com/emqx/MQTTX/blob/main/docs/manual.md <br/>
   Refer to our project configurations [here](https://github.com/yinheng996/r2table_nav/blob/main/Documentations/MQTT%20X%20Setup%20Config.pdf)
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
6. Refer to our [/Hardware/Electrical Circuits](https://github.com/yinheng996/r2table_nav/tree/main/Hardware/Electrical%20Circuits) folder for the schematics of the circuits that we use to replicate our robot system. <br/>

__Important__: Please note that our project has been developed using Ubuntu 20.04.4, ROS2 Foxy, Arduino 2.1.0 and Python 3.6, as well as a DOIT ESP32 Devkit V1 Board and a Robotis Co. TurtleBot3 Burger with Raspberry Pi 3B+. As such, modifications may be necessary when using other software systems or hardware platforms.

### Setup

1. To setup your Ubuntu Machine, clone our repository into your Home directory. Compile the workspace. <br/>

    ```
    cd colcon_ws/src/auto_nav
    git clone https://github.com/yinheng996/r2table_nav.git 
    cd colcon_ws
    colcon build
2. To setup your TurtleBot, copy the `/py_pubsub` folder onto the Raspberry Pi. Compile the workspace. <br/>
    
    ```
    ssh ubuntu@<RPi IP address>
    scp -r <path to r2table_nav directory>/py_pubsub ubuntu@<RPi IP address>:~/turtlebot_ws/src 
    cd turtlebot3_ws
    colcon build 
3. To setup the ESP32, we will use the Arduino IDE.
    1. Open the code for the ESP32 in the Arduino IDE. This will be located in the repository that you cloned in step 1. In the code, make any necessary changes or modifications.
    2. Connect the ESP32 to your computer using a USB cable.
    3. In the Arduino IDE, select the correct board and serial port. To do this, navigate to `Tools` -> `Board` and select `ESP32 Dev Module`. Then, navigate to `Tools` -> `Port` and select the correct serial port.
    4. To upload the code onto the ESP32, click on the `Upload` button in the Arduino IDE and hold down BOOT button on esp32, release the button once u see "connecting" in the serial monitor.
    5. Once the upload is complete, disconnect the ESP32 from your computer and power it using an external power source. The code should now be running on the ESP32.
4. To setup MQTT X, launch the MQTT X Desktop Client
    1. After the connection to ESP32 is successful, click the `New Subscription` button in the lower left corner to add New Topics. <br/>
    2. Add the topics `docking` and `table_num`
## System check

#### For your TurtleBot
We need to test if the limit switch is connected to the RPi and if the RPi is able to publish its status using [limit_switch.py](https://github.com/yinheng996/r2table_nav/blob/main/py_pubsub_robot/limit_switch.py)
##### In one terminal
    ssh ubuntu@<RPi IP address>
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
##### In another terminal
    ssh ubuntu@<RPi IP address>
    cd <path to r2table_nav directory>/py_pubsub
    python3 limit_switch.py
The terminal should start publishing the limit switch state, press and release the limit switch and check if the publisher updates the limit switch state.
If not, reboot your TurtleBot and repeat the TurtleBot system Check.
#### For your Laptop
We need to run a factory test on the TurtleBot from the Laptop to test if the laptop is able to communicate with the RPi and if all components of the TurtleBot is functional using [r2factory_test.py](https://github.com/yinheng996/r2table_nav/blob/main/table_nav/table_nav/r2factory_test.py).
##### In one terminal
    cd <path to r2table_nav directory>/table_nav/table_nav
    python3 r2factory_test.py
Follow the instructions printed on your terminal, and if everything works out fine, it means the system is ready to go.
#### For your ESP32
We need to run a factory test on the ESP32. <br/>
   Launch the ESP32 factory test scripts in our [/ESP32_test](https://github.com/yinheng996/r2table_nav/tree/main/ESP32_test) folder onto your Arduino IDE, then upload and run on your ESP32 module. <br/>
   Do this individually for each script. <br/>
   If your ESP32 module passes all the tests, it is ready to go.
#### For your MQTT X
We need to check if the MQTT is able to received messages from the ESP32. <br/>
   Launch the MQTT desktop client. <br/>
   Check for messages received from the ESP32 Publisher. If nothing is received, refresh the desktop client. <br/>

## Operation Parameters
From lines 15 to 39 of [/table_nav/table_nav/r2tcheckpt_nav.py](https://github.com/yinheng996/r2table_nav/blob/main/table_nav/table_nav/r2tcheckpt_nav.py)

The explanation of all the parameters are as commented below. These are the values which we found work the best for us in our use case. Please change these values if you require a different behavior of the robot.

    # constants
    rotation_speed_fast = 0.5 # speed of fast rotation, mainly for regular turns
    rotation_speed_slow = 0.2 # speed of slow rotation, mainly for calibrations
    moving_speed = 0.18 # speed of linear movements
    lidar_offset = 0.193 # distance from lidar to front of robot
    lidar_offset_b = 0.094 # ditance from lidar to back of robot
    disp_from_wall = 0.50 # distance from centre of dispenser to wall
    stop_distance = 0.1 # distance to stop from Table with front facing table
    stop_distance += lidar_offset 
    stop_distance_b = 0.1 # distance to stop from Table with back facing table
    stop_distance_b += lidar_offset_b 

    centre_of_rotation_f_offset = 0.152 # distance from centre of rotation to front of robot
    centre_of_rotation_b_offset = 0.13 # distance from centre of rotation to back of robot

    move_dist_f = lidar_offset + 0.14 + 0.04 + 0.03 # movement offset at speed 0.18 from front
    move_dist_corf = move_dist_f - centre_of_rotation_f_offset # movement offset at speed 0.18 from centre of rotation from front
    move_dist_b = lidar_offset_b + 0.13 + 0.04 +0.04 + 0.03 # movement offset at speed -0.18 from back
    move_dist_corb = move_dist_b - centre_of_rotation_b_offset # movement offset at speed -0.18 from centre of rotation from back

    angle_sweep = 30 # angle to sweep for Table
    front_angles = range(-angle_sweep, angle_sweep+1,1)
    back_angles = range(180-angle_sweep, 180+angle_sweep+1,1)
    scanfile = 'lidar.txt' # file to store lidar data logged
    mapfile = 'map.txt' # file to store map data logged

From lines 41 to 47 of [/table_nav/table_nav/r2tcheckpt_nav.py](https://github.com/yinheng996/r2table_nav/blob/main/table_nav/table_nav/r2tcheckpt_nav.py)

Please input your MQTT configurations, especially your username and password.

    # MQTT variables
    mqtt_broker = "broker.emqx.io"
    mqtt_port = 1883
    mqtt_username = "idpgrp3" # username for mqtt broker
    mqtt_password = "turtlebot" # password for mqtt broker
    mqtt_topic_tablenum = "table_num" # topic to subscribe to for table number
    mqtt_topic_docking = "docking" # topic to subscribe to for docking

## Running the Code
A total of 3 terminals would be required to run the code, in addition to the MQTT X desktop client. Ensure that Wifi connection is established and the ESP32, Laptop and TurtleBot are all in the same Wifi connection.
##### In Terminal 1: To bring up TurtleBot3
    ssh ubuntu@<RPi IP address>
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
##### In Terminal 2: To host Bot Limit Switch Publisher
    ssh ubuntu@<RPi IP address>
    cd <path to r2table_nav directory>/py_pubsub_robot
    python3 limit_switch.py
##### In Terminal 3: To run navigation code
    cd <path to r2table_nav directory>/table_nav/table_nav
    python3 r2tcheckpt_nav.py
#### In MQTT X:
    Launch MQTT X and select `connect`.
The system is ready to go. Press a number from 1-6 on the keypad for Food Delivery.
