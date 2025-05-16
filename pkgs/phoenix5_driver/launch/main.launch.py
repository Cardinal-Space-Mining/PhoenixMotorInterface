from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            name = 'phoenix5_driver',
            package = 'phoenix5_driver',
            executable = 'main',
            output = 'screen',
            # parameters = []
        )
    ])
