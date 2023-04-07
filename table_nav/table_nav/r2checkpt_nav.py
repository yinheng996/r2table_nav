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

# constants
rotation_speed = 0.2
occ_bins = [-1, 0, 100, 101] 
lidar_offset = 0.195
lidar_offset_b = 0.15
stop_distance = 0.1 + lidar_offset
stop_distance_b = 0.1 + lidar_offset_b
centre_of_rotation_f_offset = 0.15
centre_of_rotation_b_offset = 0.12
homing_from_wall = 0.5
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
back_angles = range(150, 210, 1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

# MQTT variables
mqtt_broker = "broker.emqx.io"
mqtt_port = 1883
mqtt_username = "idpgrp3"
mqtt_password = "turtlebot"
mqtt_topic_tablenum = "table_num"
mqtt_topic_docking = "docking"
table_num = 0 
docking = False

def connect_mqtt() -> mqtt_client:

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    # create MQTT client
    client = mqtt_client.Client('HolyMQTT')
    client.username_pw_set(mqtt_username, mqtt_password)
    client.on_connect = on_connect
    client.connect(mqtt_broker, mqtt_port, 65535)
    return client

def subscribe(client: mqtt_client):

    last_msg = None

    def on_message(client, userdata, msg):
        nonlocal last_msg
        global docking
        global table_num

        if last_msg is None or (msg.payload != last_msg.payload):
            last_msg = msg

            try:
                #decodeing msg.payload.decode() to get the table number
                #the second last character is the table number
                if msg.topic == mqtt_topic_tablenum:
                    table_num = int(msg.payload.decode()[-2])
                    table_nav = TableNav()
                    time.sleep(8)
                    table_nav.nav(table_num)
                    table_nav.destroy_node()

                if msg.topic == mqtt_topic_docking:
                    docking = msg.payload.decode()[-2] == '1'

            except Exception as e:
                print(f"An error occurred: {e}")

    client.subscribe(mqtt_topic_tablenum)
    client.subscribe(mqtt_topic_docking)
    client.on_message = on_message

def run():

    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()

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
        # self.bot_limit_subscription = self.create_subscription(Bool,'/bot_limit_switch',self.bot_limit_callback,10)
        # self.bot_limit_subscription  # prevent unused variable warning
        # self.bot_limit = False

        # create subscriber to track LDR for line following
        # self.ldr_subscription = self.create_subscription(Bool,'/ldr',self.ldr_callback,10)
        # self.ldr_subscription  # prevent unused variable warning
        # self.ldr = False

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
        #to return True value when limit switch is pressed, False otherwise
        pass


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # using complex numbers to avoid problems when the angles go from 360 to 0, or from -180 to 180
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
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

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
        self.get_logger().info('Moving %s until distance at %s degrees is %s than %s from %s' % (direction, angle, more_less, dist, pt_of_ref))
        
        move_dict = {'forward': 1, 'backward': -1, 'more': True, 'less': False}

        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x = move_dict[direction] * 0.20  # initial speed
        twist.angular.z = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)
        
        #rebasing the distance based on the point of reference
        if pt_of_ref == 'front':
            dist += lidar_offset +0.25
        elif pt_of_ref == 'back':
            dist += lidar_offset_b + 0.15
        elif pt_of_ref == 'centre of rotation from front':
            dist += (centre_of_rotation_f_offset-lidar_offset)
        elif pt_of_ref == 'centre of rotation from back':
            dist += (centre_of_rotation_b_offset-lidar_offset_b)
        
        print('Distance to check: %s' % dist)
        
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

        turn_dict = {'clockwise': 270,
                    'anticlockwise': 90}
        
        self.rotatebot(turn_dict[orientation])

    # function to locate table 6
    # table 6 is located at the top left quadrant of the robot
    # robot navigates to it by finding the shortest distance in the quadrant and rotating to that angle, then proceeds towards it
    # robot then waits for the limit switch to be depressed
    # then returns to its study position to its starting position
    def locate_table6(self, starting_angle, ending_angle):
        angle = np.nanargmin(self.laser_range[starting_angle:ending_angle])
        
        
        # instead of moving moving diagonally to the table, we will move in an L-shape, calculated using trigonometry
        # detected distance is the hypotenuse 
        # we will move forward by the adjacent side, then rotate 90 degrees anticlockwise, then move forward until distance at 0 degrees is less than 0.3m
        hypotenuse = self.laser_range[angle]
        adjacent = hypotenuse * math.cos(angle)
        opposite = hypotenuse * math.sin(angle)

        self.get_logger().info('Table located: %d %f m' % (angle, hypotenuse))

        #case 1: opposite is small enuf that there is no need to move laterally
        #case 1a: opposite is small enuf that there is no need to rotate at all

        #case 2: adjacent is small enuf that there is no need to move forward
        #case 2a: adjacent is small enuf that there is no need to rotate at all

        #case 3: both are small enuf that there is no need to move forward or laterally
        #case 3a: both are small enuf that there is no need to rotate at all
        
        # move forward by the adjacent side
        self.move_til('forward', 0, 'less', self.laser_range[0]-adjacent-lidar_offset-0.07, 'center of rotation from front')
        # rotate 90 degrees anticlockwise
        self.right_angle_rotate('anticlockwise')
        # move forward until distance at 0 degrees is less than 0.1m
        self.stop_at_table('front')
        self.stopbot()

    def stop_at_table(self, side_facing_table):

        self.get_logger().info('Stopping at table')

        if side_facing_table == 'front': 
            comparison_range = front_angles
            stop = stop_distance + 0.02
            running_speed = 0.08
            print("Front facing table")

        else:
            comparison_range = back_angles
            stop = stop_distance_b 
            running_speed = -0.08
            print("Back facing table")

        # check if approaching table
        lri = (self.laser_range[comparison_range]<float(stop)).nonzero()

        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x,twist.angular.z = running_speed,0.0
        time.sleep(1)
        self.publisher_.publish(twist)

        while rclpy.ok():

            if(len(lri[0])>0):
                print("Arrived")
                #stop moving
                self.stopbot()
                break

            # allow callback functions to run
            rclpy.spin_once(self)
            lri = (self.laser_range[comparison_range]<float(stop)).nonzero()

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x, twist.angular.z = 0.0, 0.0
        time.sleep(1)
        self.publisher_.publish(twist)


    def calibrate(self, direction):
        max_attempts = 200 # to avoid infinite recursion

        calib_dict = {'R': [250, 270, 290, 'right'], \
                      'L': [70, 90, 110, 'left'], \
                      'F': [340, 0, 20, 'front'], \
                      'B': [160, 180, 200, 'back']}
        
        for attempt in range(max_attempts):
            print(f"Aligning with {calib_dict[direction][3]} wall...")
            if (abs((self.laser_range[calib_dict[direction][0]]/1.064) - self.laser_range[calib_dict[direction][1]])) <= 0.01 and \
                (abs((self.laser_range[calib_dict[direction][2]]/1.064) - self.laser_range[calib_dict[direction][1]])) <= 0.01:
                print("Aligned")
                break

            else:
                if self.laser_range[calib_dict[direction][0]]/1.064 > self.laser_range[calib_dict[direction][1]] or \
                    self.laser_range[calib_dict[direction][1]] > self.laser_range[calib_dict[direction][2]]/1.064:
                    print("Tilting left")
                    self.rotatebot(0.5)
                    rclpy.spin_once(self)

                elif self.laser_range[calib_dict[direction][2]]/1.064 > self.laser_range[calib_dict[direction][1]] or \
                    self.laser_range[calib_dict[direction][1]] > self.laser_range[calib_dict[direction][0]]/1.064:
                    print("Tilting right")
                    self.rotatebot(-0.5)
                    rclpy.spin_once(self)

        else:
            print("Failed to align after {} attempts".format(max_attempts))
    
    def undocking(self):
        # move backwards at 0.10 speed until distance at 0 degrees is more than 0.3m
        # create Twist object, publish movement
        twist = Twist()
        
        twist.linear.x = -0.10
        twist.angular.z = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)
        
        #dist is distance from tip of the robot 
        dist =  0.10 + lidar_offset
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

        # ensure distance at 0 degrees is 0.5m
        while rclpy.ok():

            self.get_logger().info(str(self.laser_range[0] - lidar_offset + centre_of_rotation_f_offset))

            # create Twist object, publish movement
            twist = Twist()
            if round((self.laser_range[0] - lidar_offset + centre_of_rotation_f_offset),3) > 0.51: 
                twist.linear.x,twist.angular.z = 0.01,0.0 
            elif round((self.laser_range[0] - lidar_offset + centre_of_rotation_f_offset),3) < 0.49: 
                twist.linear.x,twist.angular.z = -0.01,0.0
            else:
                twist.linear.x, twist.angular.z = 0.0, 0.0
                break
            time.sleep(1)
            self.publisher_.publish(twist)
            rclpy.spin_once(self)
        
        # rotate 90 degrees anticlockwise
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')

        # move forward until distance at 0 degrees is less than 0.2m
        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x,twist.angular.z = 0.05,0.0
        time.sleep(1)
        self.publisher_.publish(twist)

        while math.isnan(self.laser_range[0]) or self.laser_range[0] > 0.07: #to replace with check dist limit switch
            rclpy.spin_once(self)
            print(self.laser_range[0])

        self.stopbot()
        self.get_logger().info('Docked')

    def checkpoint_1A_in_(self):
        self.undocking()
        pass
    
    def checkpoint_1A_out_(self):
        self.docking()
        pass

    def checkpoint_1B_in_(self):
        pass

    def checkpoint_1B_out_(self):
        pass

    def checkpoint_1C_in_(self):
        pass

    def checkpoint_1C_out_(self):
        pass

    def checkpoint_1D_in_(self):
        pass

    def checkpoint_1D_out_(self):
        pass

    def checkpoint_2A_in_(self):
        pass

    def checkpoint_2A_out_(self):
        pass

    def checkpoint_2B_in_(self):
        pass

    def checkpoint_2B_out_(self):
        pass

    def checkpoint_2C_in_(self):
        pass

    def checkpoint_2C_out_(self):
        pass


    # defining individual functions for each table
    def to_table1(self):
        self.move_til('backward', 180, 'less', 0.30, 'back') 
        self.stop_at_table('back')

    def from_table1(self):
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 1.2, 'front')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 0.5, 'front')
        self.right_angle_rotate('clockwise')
        self.docking()

    def to_table2(self):
        self.move_til('backward', 0, 'more', 0.5, 'front')
        self.calibrate('R')
        self.move_til('backward', 180, 'less', 0.45, 'back')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.move_til('forward', 0, 'less', 1.15, 'front')
        self.right_angle_rotate('anticlockwise')
        self.stop_at_table('front')

    def from_table2(self):
        self.move_til('backward', 0, 'more', 0.10, 'front')
        self.right_angle_rotate('anticlockwise')
        self.move_til('forward', 0, 'less', 0.4, 'front')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 1.2, 'front')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 0.5, 'front')
        self.right_angle_rotate('clockwise')
        self.docking()

    def to_table6(self):
        self.move_til('backward', 0, 'more', 0.5, 'front')
        self.calibrate('R')
        self.move_til('backward', 180, 'less', 0.30, 'back')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.move_til('forward', 0, 'less', 0.60, 'center of rotation from front')
        self.right_angle_rotate('anticlockwise')
        self.move_til('forward', 0, 'less', 0.50, 'center of rotation from front')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')
        self.move_til('forward', 180, 'more', 0.80, 'back')
        self.calibrate('R')

    def from_table6(self):
        self.move_til('backward', 0, 'less', 0.40, 'centre of rotation from back')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('L')
        self.move_til('forward', 0, 'less', 0.30, 'front')
        self.right_angle_rotate('clockwise')
        self.calibrate('L')
        self.move_til('forward', 0, 'less', 0.4, 'front')
        self.right_angle_rotate('clockwise')
        self.calibrate('B')
        self.move_til('forward', 0, 'less', 0.5, 'front')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 1.2, 'front')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 0.5, 'front')
        self.right_angle_rotate('clockwise')
        self.docking()

    def to_table3(self):
        self.undocking()
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.move_til('forward', 180, 'more', 1.2, 'back')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.stop_at_table('front')

    def to_table4(self):
        self.undocking()
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.move_til('forward', 180, 'more', 1.2, 'back')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 1.1, 'front')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.stop_at_table('front')

    def to_table5(self):
        self.undocking()
        self.right_angle_rotate('anticlockwise')
        self.calibrate('B')
        self.move_til('forward', 180, 'more', 1.2, 'back')
        self.calibrate('R')
        self.move_til('forward', 0, 'less', 0.65, 'center of rotation from front')
        self.rotatebot(88)
        self.calibrate('R')
        self.stop_at_table('front')

    def from_table3_4_5(self):
        self.move_til('backward',180, 'less', 0.6, 'back')
        self.right_angle_rotate('clockwise')
        self.calibrate('R')
        self.move_til('backward', 180, 'less', 0.4, 'back')
        self.right_angle_rotate('anticlockwise')
        self.right_angle_rotate('anticlockwise')
        self.calibrate('F')
        self.docking()

    
    # governing function to process table number input and execute corresponding table function
    def nav(self, table_num):

        # dictionary containing table number as key and corresponding table function as value
        to_dict = {1: self.to_table1, \
                   2: self.to_table2, \
                   3: self.to_table3, \
                   4: self.to_table4, \
                   5: self.to_table5, \
                   6: self.to_table6}
        
        from_dict = {1: self.from_table1, \
                     2: self.from_table2, \
                     3: self.from_table3_4_5, \
                     4: self.from_table3_4_5, \
                     5: self.from_table3_4_5, \
                     6: self.from_table6}

        try:
            while rclpy.ok():
                if self.laser_range.size != 0:

                    self.get_logger().info('Table number: %d' % table_num)

                    #to include check bot limit switch status
                    to_dict[table_num]()
                    if table_num == 6:
                        self.locate_table6(0, 90)
                    time.sleep(3)
                    #to include check bot limit switch status
                    from_dict[table_num]()

                    #to include check dispenser limit switch status
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

    #Option 1: testing with MQTT
    #run()

    #Option 2: testing with manual input
    print("Starting node")
    table_num = int(input("Enter table number: "))
    table_nav = TableNav()
    table_nav.nav(table_num)
    table_nav.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()