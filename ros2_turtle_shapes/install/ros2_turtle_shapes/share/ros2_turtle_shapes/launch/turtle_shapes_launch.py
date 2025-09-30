#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim_node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Start turtle_commander
        Node(
            package='ros2_turtle_shapes',
            executable='turtle_commander',
            name='turtle_commander',
            output='screen'
        ),
        
        # Start shape_node
        Node(
            package='ros2_turtle_shapes',
            executable='shape_node',
            name='shape_node',
            output='screen'
        ),
    ])