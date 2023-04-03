import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
#from laptopLS import robot_LS_state
import numpy as np
import math
import cmath
import time

# constants
rotation_speed = 0.1 
occ_bins = [-1, 0, 100, 101] 
lidar_offset = 0.2 
stop_distance = 0.1 + lidar_offset
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

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

        # create subscriber to receive table number through MQTT
        # self.table_subscription = self.create_subscription(Int,'/table_number',self.table_callback,10)
        # self.table_subscription  # prevent unused variable warning
        self.table_number = 0

        # create subscriber to track bot limit switch
        self.bot_limit_subscription = self.create_subscription(Bool,'/bot_limit_switch',self.bot_limit_callback,10)
        self.bot_limit_subscription  # prevent unused variable warning

        #create subscriber to track dispenser limit switch
        # self.disp_limit_subscription = self.create_subscription(Bool,'/disp_limit_switch',self.dist_limit_callback,10)
        # self.disp_limit_subscription  # prevent unused variable warning
        # self.disp_limit = False

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

    def table_callback(self, msg):
        #to return table number jeanette
        pass

    def bot_limit_callback(self, msg):
        #to return True value when limit switch is pressed, False otherwise
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.bot_limit = msg.data
        print(self.bot_limit)
        pass

    def disp_limit_callback(self, msg):
        #to return True value when limit switch is pressed, False otherwise
        pass

    def ldr_callback(self, msg):
        #to return True value when LDR detects line, False otherwise
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

    def calibrateR(self):
        print(self.laser_range[250],self.laser_range[270],self.laser_range[290])
        if ((self.laser_range[250]/1.064) - self.laser_range[270]) <= 0.001 and ((self.laser_range[290]/1.064) - self.laser_range[270]) <= 0.001:
            print(self.laser_range[250],self.laser_range[270],self.laser_range[290])
            print("Aligned")
            return
        else:
            #print("differences", self.laser_range[np.nanargmin(self.laser_range[250:290])], self.laser_range[270])
            print(np.nanargmin(self.laser_range[250:269]),np.nanargmin(self.laser_range[270:290]))
            if self.laser_range[250] > self.laser_range[270] or self.laser_range[270] > self.laser_range[290]:
                #print(self.laser_range[270],np.nanargmin(self.laser_range[250:290]))
                print("Turning left")
                self.rotatebot(1)
                #self.rotatebot(360 - (270 - np.nanargmin(self.laser_range[250:290])))
            elif self.laser_range[290] > self.laser_range[270] or self.laser_range[270] > self.laser_range[250]:
                print("Turning right")
                self.rotatebot(-1)
                #self.rotatebot(np.nanargmin(self.laser_range[250:290]) - 270)
        return self.calibrateR()
    
    def bot_limit_pressed(self):
        if self.bot_limit == True: 
            return
        else:
            print("waiting for LS")
            time.sleep(3)
            return self.bot_limit_pressed()

    # all-in-one function for linear movements
    # first input == direction of movement (forward or backward)
    # second input == angle to check (0, 90, 180, 270)
    # third == check if more or less than the input distance (more or less)
    # fourth == distance to check
    def move_til(self, direction, angle, more_less, dist):

        self.get_logger().info('Moving %s until distance at %s degrees is %s than %f' % (direction, angle, more_less, dist))

        move_dict = {'forward': 0.07, 
                    'backward': -0.07,
                    'more': True,
                    'less': False}

        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x,twist.angular.z = move_dict[direction],0.0
        time.sleep(1)
        self.publisher_.publish(twist)

        self.get_logger().info('Linear movement initiated')

        # account for offset in distance measurement
        dist += lidar_offset

        # create parameter to check distance
        check_dist = self.laser_range[angle]
        while math.isnan(check_dist) or (((not math.isnan(check_dist)) and (check_dist < dist)) == move_dict[more_less]):

            #allow the callback functions to run
            rclpy.spin_once(self)
            check_dist = self.laser_range[angle]

            # log the info
            self.get_logger().info('Distance at %s degrees: %f' % (angle, check_dist))

        # stop moving
        twist.linear.x = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

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
        
        # move forward by the adjacent side
        self.move_til('forward', 0, 'less', self.laser_range[0]-adjacent-lidar_offset-0.2)
        # rotate 90 degrees anticlockwise
        self.right_angle_rotate('anticlockwise')
        # move forward until distance at 0 degrees is less than 0.1m
        self.stop_at_table()
        self.stopbot()

    def stop_at_table(self):
        # check if approaching table
        lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()

        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x,twist.angular.z = 0.05,0.0
        time.sleep(1)
        self.publisher_.publish(twist)
        
        while rclpy.ok():
            if(len(lri[0])>0):

                #stop moving
                self.stopbot()
                break

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # create Twist object, publish movement
        twist = Twist()
        twist.linear.x, twist.angular.z = 0.0, 0.0
        time.sleep(1)
        self.publisher_.publish(twist)


    # defining individual functions for each table
    def to_table1(self):
        self.move_til('backward', 0,'more', 1.0)
        for i in range(2):
            self.right_angle_rotate('clockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'less', 0.5)
        self.stop_at_table()

    def from_table1(self):
        self.move_til('backward', 0, 'more', 1.0)
        for i in range(2):
            self.right_angle_rotate('anticlockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'more', 1.0)

    def to_table2(self):
        self.move_til('backward', 0,'more', 1.0)
        for i in range(2):
            self.right_angle_rotate('clockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'less', 0.7)
        self.right_angle_rotate('clockwise')
        self.move_til('forward', 0, 'less', 1.2)
        self.right_angle_rotate('anticlockwise')
        self.stop_at_table()

    def from_table2(self):
        self.move_til('backward', 0, 'more', 1.0)
        self.right_angle_rotate('anticlockwise')
        self.move_til('forward', 0, 'less', 0.8)
        self.right_angle_rotate('anticlockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'less', 1.0)

    def to_table3(self):
        self.move_til('backward', 0, 'more', 1.0)
        self.right_angle_rotate('anticlockwise')
        self.calibrateR()
        self.move_til('forward', 180, 'more', 1.2)
        self.right_angle_rotate('anticlockwise')
        self.stop_at_table()

    def to_table4(self):
        self.move_til('backward', 0, 'more', 1.0)
        self.right_angle_rotate('anticlockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'less', 2.3)
        self.right_angle_rotate('anticlockwise')
        self.stop_at_table()

    def to_table5(self):
        self.move_til('backward', 0, 'more', 1.0)
        self.right_angle_rotate('anticlockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'less', 0.5)
        self.right_angle_rotate('anticlockwise')
        self.calibrateR()
        self.stop_at_table()

    def from_table3_4_5(self):
        self.move_til('backward',180, 'less', 1.0)
        self.right_angle_rotate('anticlockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'less', 0.5)
        self.right_angle_rotate('anticlockwise')

    def to_table6(self):
        self.move_til('backward', 0,'more', 1.0)
        for i in range(2):
            self.right_angle_rotate('clockwise')
        self.calibrateR()
        self.move_til('forward', 0, 'less', 0.7)
        self.right_angle_rotate('clockwise')
        self.move_til('forward', 0, 'less', 0.5)
        self.right_angle_rotate('anticlockwise')
        self.move_til('forward', 0, 'less', 1.4)
        self.right_angle_rotate('anticlockwise')
        self.move_til('forward', 0, 'less', 1.4)
        self.right_angle_rotate('clockwise')

    def from_table6(self):
        self.move_til('backward', 0, 'less', 0.42)
        self.right_angle_rotate('anticlockwise')
        self.move_til('forward', 0, 'less', 0.5)
        self.right_angle_rotate('clockwise')
        self.move_til('forward', 0, 'less', 0.5)
        self.right_angle_rotate('anticlockwise')

    #def table1(self):
        #     self.get_logger().info('Navigating to Table 1')
        #     try:
        #         while rclpy.ok():
        #             if self.laser_range.size != 0:
                        
        #                 #to include while loop to check bot status
        #                 self.to_table1()
        #                 #to include check limit switch status
        #                 self.from_table1()
        #                 #to include update bot status

        #             # allow the callback functions to run
        #             rclpy.spin_once(self)

        #     except Exception as e:
        #         print(e)
            
        #     # Ctrl-c detected
        #     finally:
        #         self.stopbot()
    
    # governing function to process table number input and execute corresponding table function
    def nav(self):
        # dictionary containing table number as key and corresponding table function as value
        to_dict = {1: self.to_table1, 2: self.to_table2, 3: self.to_table3, 4: self.to_table4, 5: self.to_table5, 6: self.to_table6}
        from_dict = {1: self.from_table1, 2: self.from_table2, 3: self.from_table3_4_5, 4: self.from_table3_4_5, 5: self.from_table3_4_5, 6: self.from_table6}

        try:
            while rclpy.ok():
                # to include if table_num!=0
                if self.laser_range.size != 0:
                    self.bot_limit_pressed()
                    #self.calibrateR()
                    #break
                    #calibrate & check bot limit switch status 
                    '''self.calibrateR()
                    self.bot_limit_pressed()
                    to_dict[self.table_number]()
                    if self.table_number == 6:
                        self.locate_table6(0, 90)
                    #to include check bot limit switch status
                    from_dict[self.table_number]()
                    #insert docking here
                    #to include check dispenser limit switch status
                    break #to instead include function to wait for next order'''
                
                # allow the callback functions to run
                rclpy.spin_once(self)
            
        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)
    table_nav = TableNav()
    table_nav.nav()

    # destroy the node explicitly
    table_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()