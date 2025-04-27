#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate the URDF file and load it
    urdf_file_name = 'robot-xarm.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('xarmrob'),
        'urdf',
        urdf_file_name
    )
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # Locate the params YAML
    params_file_name = 'robot_xarm_info.yaml'
    params_file = os.path.join(
        get_package_share_directory('xarmrob'),
        'config',
        params_file_name
    )

    return LaunchDescription([
        # Your hand-tracking endpoint node
        Node(
            package='xarmrob',
            executable='endpoint_vision_smooth',
            name='endpoint_vision',
            output='screen',
            parameters=[params_file]
        ),

        # Robot state publisher for transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description}
            ],
            arguments=[urdf_path]
        ),
    ])


generate_launch_description()
