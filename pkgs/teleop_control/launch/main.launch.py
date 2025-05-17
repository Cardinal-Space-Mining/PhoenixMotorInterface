from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            name = 'teleop_control',
            package = 'teleop_control',
            executable = 'main',
            output = 'screen',
            # parameters = []
        )
    ])
