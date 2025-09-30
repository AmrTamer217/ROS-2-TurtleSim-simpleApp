#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        Node(
            package='ros2_turtle_shapes',
            executable='turtle_commander',
            name='turtle_commander',
            output='screen'
        ),
        # Run shape_node in its own xterm window
        ExecuteProcess(
            cmd=['xterm', '-hold', '-e', 'ros2', 'run', 'ros2_turtle_shapes', 'shape_node'],
            output='screen'
        )
    ])
