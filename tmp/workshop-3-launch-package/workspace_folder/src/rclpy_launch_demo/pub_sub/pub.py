import rclpy      # ROS CLIENT LIBRARIES
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        # PUBLISH
        self.publisher_ = self.create_publisher(Point, 'goal_waypoint', 10)
        # SUBSCRIBE
        self.subscription = self.create_subscription(
            String,
            'key_press',
            self.key_callback,
            10)
        goal_pub_timer_period = 1.0  # SECONDS
        self.timer_goal_pub = self.create_timer(goal_pub_timer_period, self.timer_callback)
        # CREATE LOCAL GOAL POINT VARIABLE 
        self.my_goal_waypoint = Point()
        self.allowed_keys = ['a', 's', 'd', 'w', ' ']
        

    def timer_callback(self):
        print(self.my_goal_waypoint)
        self.publisher_.publish(self.my_goal_waypoint)

    def key_callback(self, msg):
        key = msg.data
        # USE AN ASSERT STATEMENT TO CONFIRM KEY IS ALLOWED
        assert key in self.allowed_keys

        # USE KEYS TO MODIFY GOAL WAYPOINT 
        if key == 'a':
            self.my_goal_waypoint.x -= 1.0
        if key == 'd':
            self.my_goal_waypoint.x += 1.0
        if key == 's':
            self.my_goal_waypoint.y -= 1.0
        if key == 'w':
            self.my_goal_waypoint.y += 1.0




def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    rclpy.spin(goal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
