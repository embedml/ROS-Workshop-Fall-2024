#Daniel Yanke
#10/3/2024

import rclpy
from rclpy.node import Node

from example_test_interfaces.msg import ExampleTest

class PySubscriber(Node):
    def __init__(self):
        super().__init__('py_subscriber')
        self.subscriber = self.create_subscription(
            ExampleTest,
            'test_topic',
            self.subscription_callback,
            10
        )
    def subscription_callback(self, msg):
        self.get_logger().info(f"I heard: '{msg.unity_id}', '{msg.seq_number}' from {msg.language} node")

def main(args = None):
    rclpy.init(args = args)
    python_subscriber = PySubscriber()
    rclpy.spin(python_subscriber)
    python_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()