import rclpy
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
from rclpy.node import Node
from std_msgs.msg import String
Left_IR = 17 #GPIO 17 -> Left IR out
Right_IR = 9 #GPIO 9 -> Right IR out
#setup
GPIO.setup(Left_IR,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Right_IR,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

class IRPublisher(Node):

    def __init__(self):
        super().__init__('IR_publisher')
        self.publisher_ = self.create_publisher(String, '/bot_IR', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        # 0=both not detected; 1=left detected; 2=right detected; 3=both detected
        if (not GPIO.input(Left_IR)) and (not GPIO.input(Right_IR)):
            msg.data = "0"
        elif GPIO.input(Left_IR) and (not GPIO.input(Right_IR)):
            msg.data = "1"
        elif (not GPIO.input(Left_IR)) and GPIO.input(Right_IR):
            msg.data = "2"
        elif GPIO.input(Left_IR) and GPIO.input(Right_IR):
            msg.data = "3"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        print(GPIO.input(Left_IR),GPIO.input(Right_IR))

def main(args=None):
    rclpy.init(args=args)
    IR_publisher = IRPublisher()
    rclpy.spin(IR_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    IR_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()