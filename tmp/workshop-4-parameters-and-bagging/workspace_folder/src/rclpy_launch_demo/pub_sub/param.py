import rclpy      # ROS CLIENT LIBRARIES
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterEvent, SetParametersResult
from rcl_interfaces.msg import Parameter
import random


class ParamPublisher(Node):

    def __init__(self, 
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True):
        super().__init__('param_publisher')
        self.publisher_ = self.create_publisher(PointStamped, 'param_waypoint', 10)
        # CREATE LOCAL GOAL POINT VARIABLE 
        self.my_param_waypoint = PointStamped()
        self.timer_waypoint_pub = self.create_timer(1/5, self.waypoint_publisher)
        self.declare_parameter('my_float', value=42.0)
        self.my_float_param = self.get_parameter('my_float')

    def waypoint_publisher(self):
        msg = self.my_param_waypoint
        msg.header.stamp = self.get_clock().now().to_msg() 
        # self.get_logger().info('my_float:%0.2f' % self.get_parameter('my_float').value)
        msg.point.x = self.get_parameter('my_float').value
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    param_publisher = ParamPublisher()

    rclpy.spin(param_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    param_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
