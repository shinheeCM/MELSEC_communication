import rclpy
from rclpy.node import Node
import time

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello, World!')
        time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
