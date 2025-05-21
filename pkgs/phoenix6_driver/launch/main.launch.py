from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("arduino_device", default_value="/dev/ttyACM0"),
        Node(
            name = 'phoenix6_driver',
            package = 'phoenix6_driver',
            executable = 'main',
            output = 'screen',
            parameters = [
                {'arduino_device': LaunchConfiguration("arduino_device", default="/dev/ttyACM0") }
            ]
        )
    ])
