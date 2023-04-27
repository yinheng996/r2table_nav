import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
from paho.mqtt import client as mqtt_client
from std_msgs.msg import Bool, String

# Constants
FAST_ROTATION_SPEED = 0.5 # speed of fast rotation, mainly for regular turns
SLOW_ROTATION_SPEED = 0.2 # speed of slow rotation, mainly for calibrations
MOVING_SPEED = 0.18 # speed of linear movements

LIDAR_FRONT_OFFSET = 0.193 # distance from lidar to front of robot
LIDAR_BACK_OFFSET = 0.094 # distance from lidar to back of robot
DISPENSER_WALL_DISTANCE = 0.50 # distance from centre of dispenser to wall
STOP_DISTANCE = 0.1 + LIDAR_FRONT_OFFSET # distance to stop from Table

CORF_OFFSET = 0.152 # distance from centre of rotation to front of robot
CORB_OFFSET = 0.13 # distance from centre of rotation to back of robot

DOCKING_DIST = 0.178 # distance to stop from docking station

MOVE_DIST_F = LIDAR_FRONT_OFFSET + 0.21 # movement offset at speed 0.18 from front
MOVE_DIST_CORF = MOVE_DIST_F - CORF_OFFSET # movement offset at speed 0.18 from centre of rotation from front
MOVE_DIST_B = LIDAR_BACK_OFFSET + 0.24 # movement offset at speed -0.18 from back
MOVE_DIST_CORB = MOVE_DIST_B - CORB_OFFSET # movement offset at speed -0.18 from centre of rotation from back

SWEEP_ANGLE = 30 # angle to sweep for Table
ANGLE_RANGE = range(-SWEEP_ANGLE, SWEEP_ANGLE + 1) # range of angles to sweep for Table

LIDAR_DATA_FILE = 'lidar.txt' # file to store lidar data logged
MAP_DATA_FILE = 'map.txt' # file to store map data logged

CAN_DROP = 4 # seconds to wait before can is dropped

# MQTT variables
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_USERNAME = "idpgrp3" # username for MQTT broker
MQTT_PASSWORD = "turtlebot" # password for MQTT broker
MQTT_TOPIC_TABLENUM = "table_num" # topic to subscribe to for table number
MQTT_TOPIC_DOCKING = "docking" # topic to subscribe to for docking

table_num = 0 
docking = False

occ_bins = [-1, 0, 100, 101] 

# Line following dictionary
line_dict = {
    '': 'No data',
    '0': 'Forward',
    '1': 'Left',
    '2': 'Right',
    '3': 'Stop'
}

def connect_mqtt() -> mqtt_client:

    def on_connect(client, userdata, flags, rc):
        client.subscribe(MQTT_TOPIC_TABLENUM)
        client.subscribe(MQTT_TOPIC_DOCKING)
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    # create MQTT client
    client = mqtt_client.Client('HolyMQTT')
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.connect(MQTT_BROKER, MQTT_PORT, 65535)
    return client

def subscribe(client: mqtt_client):

    last_msg = None

    def on_message(client, userdata, msg):
        nonlocal last_msg
        global table_num, docking
        last_msg = msg

        topic_mapping = {
            MQTT_TOPIC_TABLENUM: lambda payload: int(payload.decode()[-2]),
            MQTT_TOPIC_DOCKING: lambda payload: payload.decode()
        }

        try:
            table_num, docking = [
                topic_mapping[msg.topic](msg.payload) 
                for topic in topic_mapping 
                if topic == msg.topic
            ]
        except Exception as e:
            print(f"An error occurred: {e}")


    client.subscribe(MQTT_TOPIC_TABLENUM)
    client.on_message = on_message

def run():

    client = connect_mqtt()
    subscribe(client)
    while True:
        last_num = table_num
        client.loop()
        #create condition to check if table num is not the same and is not 0
        if table_num != 0 and table_num != last_num:
            
            table_nav = TableNav()
            time.sleep(CAN_DROP)
            table_nav.nav(table_num)
            table_nav.destroy_node()
    

def euler_from_quaternion(x, y, z, w): # code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class TableNav(Node):

    def __init__(self): 
        super().__init__('table_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(OccupancyGrid,'map',self.occ_callback,qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(LaserScan,'scan',self.scan_callback,qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        # create subscriber to track bot limit switch
        self.bot_limit_subscription = self.create_subscription(Bool,'/bot_limit_switch',self.bot_limit_callback,10)
        self.bot_limit_subscription  # prevent unused variable warning
        self.bot_limit = False

        #create subscriber to track IR sensor that is a String
        self.ir_sensor_subscription = self.create_subscription(String,'/bot_IR',self.ir_sensor_callback,10)
        self.ir_sensor_subscription  # prevent unused variable warning
        self.ir_sensor = ''

    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def bot_limit_callback(self, msg):
        # to return True value when limit switch is pressed, False otherwise
        self.bot_limit = msg.data
    
    def ir_sensor_callback(self, msg):
        # to return True value when IR sensor is triggered, False otherwise
        self.ir_sensor = msg.data
        # self.get_logger().info('IR sensor: %s' % self.ir_sensor)

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):

        if abs(rot_angle) > 5:
            rotation_speed = FAST_ROTATION_SPEED
        else:
            rotation_speed = SLOW_ROTATION_SPEED

        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotation_speed
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        
        self.publisher_.publish(twist)


    # all-in-one function for linear movements
    # first input == direction of movement (forward or backward)
    # second input == angle to check (0, 90, 180, 270)
    # third input == check if more or less than the input distance (more or less)
    # fourth input == distance to check
    # fifth input == point of reference (lidar or front, back, centre of rotation from front, centre of rotation from back or lidar)
    def move_til(self, direction, angle, more_less, dist, pt_of_ref):

        self.get_logger().info('Moving %s until distance at %s degrees is %s than %s from %s' \
                               % (direction, angle, more_less, dist, pt_of_ref))
        
        move_dict = {
            'forward': 1, 
            'backward': -1, 
            'more': True, 
            'less': False
            }

        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x, twist.angular.z = move_dict[direction] * MOVING_SPEED, 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

        # convert distance to float
        dist = float(dist)

        #rebasing the distance based on the point of reference
        if pt_of_ref == 'front':
            dist += MOVE_DIST_F
        elif pt_of_ref == 'back':
            dist += MOVE_DIST_B
        elif pt_of_ref == 'centre of rotation from front':
            dist += MOVE_DIST_CORF
        elif pt_of_ref == 'centre of rotation from back':
            dist += MOVE_DIST_CORB
        
        self.get_logger().info('Linear movement initiated')
        
        # create parameter to check distance
        check_dist = self.laser_range[angle]
        
        while math.isnan(check_dist) or (((not math.isnan(check_dist)) and (check_dist< dist)) == move_dict[more_less]):
            #allow the callback functions to run
            rclpy.spin_once(self)            
            check_dist = self.laser_range[angle]

            # log the info
            self.get_logger().info('Distance at %s degrees: %f' % (angle, check_dist))
        
        # stop moving
        self.stopbot()

    # function to simplify right angle rotations
    def right_angle_rotate(self, orientation):

        self.get_logger().info('Turning 90 degrees %s' % orientation)

        turn_dict = {
            'clockwise': 270,
            'anticlockwise': 90
            }
        
        self.rotatebot(turn_dict[orientation])

    def stop_at_table(self, side_facing_table):

        self.get_logger().info('Stopping at table')

        # get the laser range data for front angles
        laser_front = self.laser_range[ANGLE_RANGE]

        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x, twist.angular.z = 0.05, 0.0
        self.publisher_.publish(twist)

        while rclpy.ok():
            # get the indices of elements less than stop distance
            indices = [i for i, x in enumerate(laser_front) if not math.isnan(x) and x < float(STOP_DISTANCE)]
            if len(indices) > 0:
                print("Arrived")
                # stop moving
                self.stopbot()
                break

            # allow callback functions to run
            rclpy.spin_once(self)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x, twist.angular.z = 0.0, 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

    
    # calibration using triangulation
    def calibrate(self, direction):
        max_attempts = 50 # to avoid infinite recursion

        calib_dict = {'R': [250, 270, 290, 'right'], \
                      'L': [70, 90, 110, 'left'], \
                      'F': [340, 0, 20, 'front'], \
                      'B': [160, 180, 200, 'back']}
        
        for attempt in range(max_attempts):
            rclpy.spin_once(self)
            self.get_logger.info(f"Aligning with {calib_dict[direction][3]} wall")
            if (abs((self.laser_range[calib_dict[direction][0]]/1.064) - self.laser_range[calib_dict[direction][1]])) <= 0.01 and \
                (abs((self.laser_range[calib_dict[direction][2]]/1.064) - self.laser_range[calib_dict[direction][1]])) <= 0.01:
                self.get_logger().info('Aligned with %s wall' % calib_dict[direction][3])
                break

            else:
                if self.laser_range[calib_dict[direction][0]]/1.064 > self.laser_range[calib_dict[direction][1]] or \
                    self.laser_range[calib_dict[direction][1]] > self.laser_range[calib_dict[direction][2]]/1.064:
                    self.rotatebot(0.5)

                elif self.laser_range[calib_dict[direction][2]]/1.064 > self.laser_range[calib_dict[direction][1]] or \
                    self.laser_range[calib_dict[direction][1]] > self.laser_range[calib_dict[direction][0]]/1.064:
                    self.rotatebot(-0.5)

            self.get_logger().info('Failed to align after %d attempts' % max_attempts)
    
    def undocking(self):
        # move backwards at 0.10 speed until distance at 0 degrees is more than 0.3m
        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x, twist.angular.z = -0.2, 0.0
        time.sleep(1)
        self.publisher_.publish(twist)
        
        #dist is distance from tip of the robot 
        dist =  0.30 + LIDAR_FRONT_OFFSET
        check_dist =  self.laser_range[0]

        # check if distance at 0 degrees is more than 0.3m
        # ignore nan
        # stop moving if distance at 0 degrees is more than 0.3m           
        while math.isnan(check_dist) or ((not math.isnan(check_dist)) and (check_dist<dist)):

            #allow the callback functions to run
            rclpy.spin_once(self)
            check_dist = self.laser_range[0]
            
            # log the info
            self.get_logger().info('Distance from dispenser: %f' % (check_dist))

        self.stopbot()

    def docking(self):

        self.get_logger().info('Docking')
        self.calibrate('F')
        self.check_dist(DISPENSER_WALL_DISTANCE)
        
        # rotate 90 degrees anticlockwise
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')

        # move forward until distance at 0 degrees is less than 0.2m
        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x,twist.angular.z = 0.05,0.0
        self.publisher_.publish(twist)

        start_time = time.monotonic()
        while math.isnan(self.laser_range[0]) or self.laser_range[0] > 0.40:
            rclpy.spin_once(self)
            if time.monotonic() - start_time > 10:  # timeout after 10 seconds
                self.get_logger().info('Timeout: Failed to dock')
                return

        self.line_follow()
        self.get_logger().info('Docked')

    def check_dist(self, limit):
        # remove NaN values from laser_range array
        x = self.laser_range.copy()
        x[np.isnan(x)] = np.inf

        # create Twist object
        twist = Twist()

        # ensure distance at 0 degrees is limit
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # calculate distance error and set twist velocities
            error = round(x[0] - LIDAR_FRONT_OFFSET + CORF_OFFSET, 3) - limit
            twist.linear.x = 0.01 if error > 0.01 else (-0.01 if error < -0.01 else 0.0)
            twist.angular.z = 0.0

            # publish Twist object and break out of the loop if the error is within tolerance
            if abs(error) < 0.01:
                twist.linear.x = 0.0
                self.publisher_.publish(twist)
                break

            # publish Twist object and continue looping
            self.publisher_.publish(twist)
            
        self.get_logger().info(str(x[0] - LIDAR_FRONT_OFFSET + CORF_OFFSET))
    
    # function to locate table 6
    # table 6 is located at the top left quadrant of the robot
    # robot navigates to it by finding the shortest distance in the quadrant and rotating to that angle, then proceeds towards it
    # robot then waits for the limit switch to be depressed
    # then returns to its study position to its starting position
    def locate_table6(self, starting_angle, ending_angle):
        angle = np.nanargmin(self.laser_range[starting_angle:ending_angle])

        self.get_logger().info(f'Table located: {angle} degrees')

        self.rotatebot(angle)
        self.stop_at_table('front')
        self.stopbot()

        while self.bot_limit == 1:
            self.get_logger().info('Waiting for can to be picked up')
            rclpy.spin_once(self)   

        self.rotatebot(-angle)

    # use IR sensor for line following
    def line_follow(self):
        rclpy.spin_once(self)
        self.get_logger().info('Line following')

        while self.laser_range[0] > DOCKING_DIST or math.isnan(self.laser_range[0]):
            rclpy.spin_once(self)
            self.get_logger().info(str(self.laser_range[0]))
            
            twist = Twist()
            if line_dict[self.ir_sensor] == 'Right':
                twist.linear.x, twist.angular.z = 0.01, -0.1
            elif line_dict[self.ir_sensor] == 'Left':
                twist.linear.x, twist.angular.z = 0.01, 0.1
            elif line_dict[self.ir_sensor] == 'Forward':
                twist.linear.x, twist.angular.z = 0.02, 0.0
                    
            self.publisher_.publish(twist)   

        self.stopbot()


    # defining individual checkpoints
    def from_dock_to_1(self):
        self.undocking()
        self.move_til('backward', 0, 'more', 0.60, 'front')
        self.calibrate('R')
        self.move_til('backward', 180, 'less', 0.40, 'centre of rotation from back')

    def from_1_to_dock(self):
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 1.2, 'front')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 0.5, 'front')
        self.right_angle_rotate('clockwise')
        self.docking()
    
    def calib_1(self):
        self.calibrate('F')
        self.check_dist(0.5)
        
    def from_1_to_2(self):
        self.right_angle_rotate('clockwise')
        self.calibrate('L')
        self.check_dist(0.25)
        self.right_angle_rotate('clockwise')
        self.calibrate('B')
        self.move_til('forward', 0, 'less', 1.20, 'centre of rotation from front')

    def from_1_to_3(self):
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.move_til('forward', 0, 'less', 1.20, 'centre of rotation from front')
        self.rotatebot(10)
        self.move_til('forward', 0, 'less', 0.4, 'centre of rotation from front')

    def from_2_to_1(self):
        self.rotatebot(10)
        self.move_til('forward', 0, 'less', 0.4, 'front')

    def from_2_to_3(self): 
        self.rotatebot(10)
        self.move_til('forward', 0, 'less', 0.4, 'centre of rotation from front')

    def from_3_to_1(self):
        self.calibrate('L')
        self.check_dist(0.25)
        self.right_angle_rotate('clockwise')
        self.calibrate('B')

        self.move_til('forward', 180, 'more', 1.20, 'centre of rotation from back')
        self.rotatebot(20)
        self.move_til('forward', 0, 'less', 0.4, 'front')

    def calib_3(self):
        self.calibrate('F')
        self.check_dist(0.38)

    def from_3_to_4(self):
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 0.35, 'front')

    def calib_4(self):
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')

    def from_4_to_5(self):
        self.move_til('forward', 180, 'more', 0.50, 'back')

    def calib_5(self):
        self.right_angle_rotate('clockwise')
        self.calibrate('F')
        self.check_dist(0.22)
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 1.4, 'centre of rotation from front')
        

    # defining individual functions for each table
    def to_table1(self):

        self.from_dock_to_1()
        self.right_angle_rotate('anticlockwise')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('L')

        self.stop_at_table('front')

    def from_table1(self):

        self.move_til('backward', 0, 'more', 0.30, 'front')
        self.right_angle_rotate('anticlockwise')

        self.calib_1()
        self.from_1_to_dock()

    def to_table2(self):

        self.from_dock_to_1()
        self.right_angle_rotate('clockwise')
        self.calib_1()
        self.from_1_to_2()
        self.right_angle_rotate('anticlockwise')
        self.check_dist(0.40)
        self.stop_at_table('front')

    def from_table2(self):

        self.check_dist(0.40)
        self.right_angle_rotate('anticlockwise')

        self.from_2_to_1()

        self.calib_1()
        self.from_1_to_dock()

    def to_table3(self):

        self.from_dock_to_1()
        self.right_angle_rotate('clockwise')
        self.calib_1()

        self.from_1_to_2()
        self.right_angle_rotate('clockwise')

        self.check_dist(0.40)
        self.stop_at_table('front')

    def from_table3(self):

        self.check_dist(0.40)
        self.right_angle_rotate('clockwise')

        self.from_2_to_1()

        self.calib_1()
        self.from_1_to_dock()
    
    def to_table4(self):

        self.from_dock_to_1()

        self.from_1_to_3()
        self.calib_3()
        self.right_angle_rotate('clockwise')

        self.calibrate('L')
        self.stop_at_table('front')

    def from_table4(self):

        self.from_3_to_1()
        self.calib_1()
        self.from_1_to_dock()

    def to_table6(self):

        self.from_dock_to_1()

        self.from_1_to_3()
        self.calib_3()

        self.from_3_to_4()
        self.calib_4()

        self.from_4_to_5()
        self.calib_5()

    def from_table6(self):

        self.move_til('backward', 180, 'less', 0.50, 'centre of rotation from back')
        self.right_angle_rotate('anticlockwise')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('F')
        self.check_dist(0.38)
        self.right_angle_rotate('clockwise')
        self.calibrate('L')

        self.move_til('forward', 180, 'more', 1.0, 'back')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('F')
        self.check_dist(0.37)
        self.right_angle_rotate('clockwise')
        self.calibrate('L')
        self.move_til('forward', 0, 'less', 0.4, 'front')

        self.from_3_to_1()
        self.calib_1
        self.from_1_to_dock()

    # table 5 isolated so no need to use checkpoints
    def to_table5(self): 

        self.undocking()
        self.right_angle_rotate('clockwise')
        self.calibrate('F')

        self.move_til('backward', 0, 'more', 1.0, 'front')
        self.right_angle_rotate('anticlockwise')
        self.move_til('forward', 0, 'less', 0.4, 'centre of rotation from front')
        self.calibrate('F')
        self.check_dist(0.4)
        self.right_angle_rotate('anticlockwise')

        self.move_til('forward', 0, 'less', 0.25, 'front')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')

        self.move_til('forward', 180, 'more', 1.3, 'center of rotation from back')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('F')
        self.check_dist(0.4)
        self.right_angle_rotate('clockwise')
        self.calibrate('L')

        self.stop_at_table('front')

    def from_table5(self): 

        self.move_til('backward', 0, 'more', 0.3, 'front')
        self.calibrate('L')

        self.move_til('backward', 180, 'less', 0.7, 'center of rotation from back')
        self.right_angle_rotate('anticlockwise')

        self.move_til('forward', 180, 'more', 1.55, 'center of rotation from back')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('F')
        self.move_til('backward', 0, 'more', 0.4, 'centre of rotation from front')
        # self.check_dist(0.6)
        self.right_angle_rotate('clockwise')
        self.calibrate('L')

        self.move_til('forward', 0, 'less', 0.4, 'front')
        self.docking()
    
    # governing function to process table number input and execute corresponding table function
    def nav(self, table_num):        

        # dictionary containing table number as key and corresponding table function as value
        nav_dict = {1: (self.to_table1, self.from_table1), 
                    2: (self.to_table2, self.from_table2), 
                    3: (self.to_table3, self.from_table3), 
                    4: (self.to_table4, self.from_table4), 
                    5: (self.to_table5, self.from_table5), 
                    6: (self.to_table6, self.from_table6)}

        try:
            self.get_logger().info('Table number: %d' % table_num)

            while rclpy.ok():
                if self.laser_range.size != 0:
                    nav_dict[table_num][0]()

                    if table_num == 6: self.locate_table6(0, 90)
                    else: 
                        rclpy.spin_once(self)
                        while self.bot_limit == 1:
                            print(self.bot_limit)
                            self.get_logger().info('Waiting for can to be picked up')
                            rclpy.spin_once(self)


                    time.sleep(3)
                    nav_dict[table_num][1]()

                    break
                
                # allow the callback functions to run
                rclpy.spin_once(self)
            
        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)
    run()

if __name__ == '__main__': main()