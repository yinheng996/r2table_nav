import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

scanfile = 'lidar.txt'

class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def lidar_calib(self):
        self.get_logger().info('Lidar calibration initiated')
        self.angle = int(input('Enter angle to calibrate: '))
        try:
            while rclpy.ok():
                if self.laser_range.size != 0:
                    
                    #reports distance in the specified angle
                    self.get_logger().info('Distance at %s degrees: %s' % (self.angle, self.laser_range[self.angle]))

                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            pass

def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()
    scanner.lidar_calib()
    # destroy the node explicitly
    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()