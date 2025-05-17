from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            name = 'test_control',
            package = 'test_control',
            executable = 'main',
            output = 'screen',
            # parameters = []
        )
    ])
