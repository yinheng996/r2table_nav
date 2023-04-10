import rclpy
import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from std_msgs.msg import Bool

# Set up the GPIO mode and pin number
GPIO.setmode(GPIO.BCM)
limit_switch_pin = 23

# Set up the limit switch as an input
GPIO.setup(limit_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Read the status of the limit switch
# limit_switch_status = GPIO.input(limit_switch_pin)

class SwitchPublisher(Node):

    def __init__(self):
        super().__init__('Switch_publisher')
        self.publisher_ = self.create_publisher(Bool, '/bot_limit_switch', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Bool()
        msg.data = bool(GPIO.input(limit_switch_pin))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    Switch_publisher = SwitchPublisher()

    rclpy.spin(Switch_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Switch_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()