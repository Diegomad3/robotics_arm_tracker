from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_descr = LaunchDescription([
        Node(
            package='xarmrob',
            executable='command_xarm',
            name='command_xarm'
        )
    ## NOTE this file is set up to only run the "command_arm" code.
    ## THE USER mus also run "cmd_manual" or some other command node to interface with it.
    ])
    
    return launch_descr
    

