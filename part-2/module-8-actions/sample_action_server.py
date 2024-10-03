import time
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_workshop.action import VerboseMoveBase
from geometry_msgs.msg import PoseStamped


class MoveBaseActionServer(Node):

    def __init__(self):
        super().__init__('move_base_action_server')
        self._action_server = ActionServer(
            self,
            VerboseMoveBase,
            'move_base',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = VerboseMoveBase.Feedback()

        #kP = 0.1
        kP = 0.00001  # a very small gain.

        current_position = feedback_msg.base_pose.pose.position
        goal_position = goal_handle.request.target_pose.pose.position

        while (not self.is_close_enough(
                            current_position,
                            goal_position)):
            # ADJUST POSITION
            current_position.x += kP * (goal_position.x - current_position.x)
            current_position.y += kP * (goal_position.y - current_position.y)
            current_position.z += kP * (goal_position.z - current_position.z)
            feedback_msg.base_pose.header.stamp = self.get_clock().now().to_msg() 
            feedback_msg.base_pose.header.frame_id = '/world'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()

        result = VerboseMoveBase.Result()
        return result

    def is_close_enough(self, current_position, goal_position):
        goal_tolerance = 0.1  # METERS
        dist = math.sqrt((current_position.x - goal_position.x)**2 +
               (current_position.y - goal_position.y)**2 +
               (current_position.z - goal_position.z)**2)
        return dist < goal_tolerance



def main(args=None):
    rclpy.init(args=args)

    move_base_action_server = MoveBaseActionServer()

    rclpy.spin(move_base_action_server)


if __name__ == '__main__':
    main()
