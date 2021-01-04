#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import AnonName, ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution as Join

pkg = 'path_score'


def generate_launch_description():
    return LaunchDescription([
        Node(
            package=pkg,
            executable='main',
            name='main'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name=AnonName('rviz2'),
            arguments=['-d', Join([ThisLaunchFileDir(), 'main.rviz'])],
            output={'both': 'log'}
        )
    ])
