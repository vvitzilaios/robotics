import os
import launch
from launch_ros.actions import Node


def generate_launch_description():
    yml = os.path.join('tt_table_explorer', 'config', 'params.yaml')

    return launch.LaunchDescription([
        Node(
            package='tt_table_explorer',
            executable='tt_table_explorer.py',
            name='tt_table_explorer',
            parameters=[yml]
        ),
    ])
