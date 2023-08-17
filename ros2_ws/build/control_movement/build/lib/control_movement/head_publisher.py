import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class HeadPublisher(Node):
    def __init__(self):
        super().__init__('head_publisher')
        self.publisher_ = self.create_publisher(String, 'head_publisher', 10)
        timer_period = 0.5
        self.timer =self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

