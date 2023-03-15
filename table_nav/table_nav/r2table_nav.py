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

# constants
rotatechange = 0.1 
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.10
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
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


    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
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


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
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
        twist.angular.z = c_change_dir * rotatechange
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


    def bck_til_more_than(self, dist):

        self.get_logger().info('Moving backward until distance at 0 degrees is more than %f' % dist)
        # allow the callback functions to run

        # get the distance at 0 degrees
        dist_at_0 = self.laser_range[0]
        # log the info
        self.get_logger().info('Distance at 0 degrees: %f' % dist_at_0)
        # create Twist object
        twist = Twist()
        # set linear speed to -0.3, and angular speed to 0, so the TurtleBot moves backward
        twist.linear.x,twist.angular.z = -0.03,0.0
        time.sleep(1)
        self.publisher_.publish(twist)


        # while the distance at 0 degrees is less than dist
        while(dist_at_0 < dist):
            # allow the callback functions to run
            rclpy.spin_once(self)
            # get the distance at 0 degrees
            dist_at_0 = self.laser_range[0]
            # log the info
            self.get_logger().info('Distance at 0 degrees: %f' % dist_at_0)

        # stop moving
        twist.linear.x = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

    def bck_til_less_than(self, dist):
        self.get_logger().info('Moving backward until distance at 180 degrees is less than %f' % dist)
        # create Twist object
        twist = Twist()
        # set linear speed to -0.03, and angular speed to 0, so the TurtleBot moves backward
        twist.linear.x,twist.angular.z = -0.03,0.0
        time.sleep(1)
        self.publisher_.publish(twist)

        # get the distance at 0 degrees
        dist_at_0 = self.laser_range[180]
        # log the info
        self.get_logger().info('Distance: %f' % dist_at_0)
        # while the distance at 180 degrees is less than dist
        while(dist_at_0 > dist):
            # allow the callback functions to run
            rclpy.spin_once(self)
            # get the distance at 0 degrees
            dist_at_0 = self.laser_range[0]
            # log the info
            self.get_logger().info('Distance: %f' % dist_at_0)

        # stop moving
        twist.linear.x = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

    def fwd_til_less_than(self, dist):
        self.get_logger().info('Moving forward until distance at 0 degrees is less than %f' % dist)
        # create Twist object
        twist = Twist()
        # set linear speed to 0.03, and angular speed to 0, so the TurtleBot moves forward
        twist.linear.x, twist.angular.z = 0.03, 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

        # get the distance at 0 degrees
        dist_at_0 = self.laser_range[0]
        # log the info
        self.get_logger().info('Distance: %f' % dist_at_0)
        # while the distance at 0 degrees is less than dist
        while(dist_at_0 > dist):
            # allow the callback functions to run
            rclpy.spin_once(self)
            # get the distance at 0 degrees
            dist_at_0 = self.laser_range[0]
            # log the info
            self.get_logger().info('Distance: %f' % dist_at_0)

        # stop moving
        twist.linear.x = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)

    def clkwise_90(self):
        self.get_logger().info('Turning 90 degrees clockwise')
        self.rotatebot(270)

    def aclkwise_90(self):
        self.get_logger().info('Turning 90 degrees anticlockwise')
        self.rotatebot(90)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)


    def table1(self):

        self.get_logger().info('In table1')

        try:

            while rclpy.ok():
                if self.laser_range.size != 0:
                    #move backwards into main axis until distance at 0 degrees is more than 0.7
                    self.bck_til_more_than(0.7)

                    #calibrate
                    #self.calibrate(left_wall=True)

                    #rotate 90 degrees anticlockwise
                    self.aclkwise_90()

                    #move forward until distance at 0 degrees is less than 0.5
                    self.fwd_til_less_than(0.5)

                    #rotate 90 degrees anticlockwise
                    self.aclkwise_90()

                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()

                # allow the callback functions to run
                rclpy.spin_once(self)


        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)
    table_nav = TableNav()
    table_nav.get_logger().info('In main')
    # rclpy.spin(table_nav)

    #receive input from user to know which table to go to, then execute corresponding table function
    #table_dict = {1: table_nav.table1, 2: table_nav.table2, 3: table_nav.table3, 4: table_nav.table4, 5: table_nav.table5}
    #selection = int(input(" Please enter the table number you would like to go to (1/2/3/4/5): "))
    #execute table function based on user input
    #table_dict[selection]()

    #execute function to navigate to table 1
    table_nav.table1()

    # destroy the node explicitly
    table_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()