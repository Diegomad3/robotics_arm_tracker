from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_descr = LaunchDescription([
        Node(
            package='xarmrob',
            executable='command_xarm',
            name='command_xarm'
        ),
        Node(
            package='xarmrob',
            executable='cmd_sliders',
            name='cmd_sliders'
        )
    ])
    
    return launch_descr
    

