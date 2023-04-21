# Food Delivery 

This is an open-source project 

EG2310 Fundamentals of System Design Project, AY22/23,
Innovation and Design Programme (iDP), National University of Singapore (NUS)

Studio 2 Group 3
Loh Yin Heng, Wang Bo, Jeanette Sim Yu, Hang Jin Guang, Ananya Agarwal

## Description

## Operating Instructions
### Installation
1. Follow the instructions here to setup ROS 2 Foxy on laptop and turtlebot https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. Follow the instructions here to setup MQTT on your laptop

1. Clone our repository into your Home directory.

    git clone https://github.com/Magmanat/r2auto_nav.git
    
2. Shift the entire folder hardware_bringup into your turtlebot3_ws using scp, then build the workspace, for example
 

    scp -r path_to_r2auto_nav/turtlebot3_ws/src/hardware_bringup ubuntu@(ip-address-of-pi):~/turtlebot3/src
    ssh ubuntu@(ip-address-of-pi)
    cd turtlebot3_ws
    colcon build
  
3. cd into colcon_ws and colcon build the workspace to setup the ros package on your laptop
  

    cd path_to_r2auto_nav/colcon_ws
    colcon build
  
  ## System check
Firstly, we will use the factory_test package to ensure that everything is running properly, we will ssh into the turtlebot and startup all the hardware publishers and subscribers.

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
