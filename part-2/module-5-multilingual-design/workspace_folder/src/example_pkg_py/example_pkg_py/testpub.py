#Daniel Yanke
#10/3/2024

import rclpy
from rclpy.node import Node

from example_test_interfaces.msg import ExampleTest

class PyPublisher(Node):
    def __init__(self):
        super().__init__("py_publisher")
        self.publisher = self.create_publisher(ExampleTest, "test_topic", 10)
        self.count = 0
        self.timer = self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        msg = ExampleTest()
        msg.unity_id = "djyanke"
        msg.seq_number = self.count
        self.count += 1
        msg.language = "Python"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.unity_id}', '{msg.seq_number}'")

def main(args = None):
    rclpy.init(args = args)
    python_publisher = PyPublisher()
    rclpy.spin(python_publisher)
    python_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
