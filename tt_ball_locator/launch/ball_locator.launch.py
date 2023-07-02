#!/usr/bin/env python3
import os
import launch
from launch_ros.actions import Node

def generate_launch_description():

    # Your launch configuration here

    return launch.LaunchDescription([
        Node(
            package='tt_ball_locator', 
            node_executable='tt_ball_locator.py',
            name='tt_ball_locator',
        )
    ])
