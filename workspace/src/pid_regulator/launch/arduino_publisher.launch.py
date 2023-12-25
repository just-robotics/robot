from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial',
            executable='serial',
            name='serial'
        ),
        Node(
            package='pid_regulator',
            executable='arduino_publisher',
            name='arduino_publisher'
        ),
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
        )
    ])
