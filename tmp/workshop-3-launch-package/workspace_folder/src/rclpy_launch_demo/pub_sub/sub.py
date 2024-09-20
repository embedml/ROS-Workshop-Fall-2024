import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class GoalSubscriber(Node):

    def __init__(self):
        super().__init__('goal_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'goal_waypoint',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('x:%0.2f, y:%0.2f, z:%0.2f' % (msg.x, msg.y, msg.z))


def main(args=None):
    rclpy.init(args=args)

    goal_subscriber = GoalSubscriber()

    rclpy.spin(goal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
