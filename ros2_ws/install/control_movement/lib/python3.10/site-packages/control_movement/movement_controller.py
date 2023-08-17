import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from .head_publisher import HeadPublisher

class MovementController(Node):
    def __init__(self):
        super().__init__('controler_input')
        self.publishers = {}

    def add_publisher(self, name, publisher):
        publishers[name] = publisher

    def move_head():
        pass

def main():
    rclpy.init(args=args)
    move_controller = MovementController()
    head_publisher = HeadPublisher()
    move_controller.add_publisher(head_publisher)

    rclpy.spin(head_publisher)
    head_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()