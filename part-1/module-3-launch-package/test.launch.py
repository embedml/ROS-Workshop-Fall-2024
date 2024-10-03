#!/usr/bin/env python
# from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            # package='rclpy_launch_demo',
            namespace='',
            executable='pub',
            name='my_pub_name'
        ),
        Node(
            package='rclpy_launch_demo',
            namespace='',
            # executable='sub',
            name='my_sub_name'
        )
            ]
        )
