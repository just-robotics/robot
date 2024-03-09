import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


package_name = 'drive_controller'

rviz_file = os.path.join(get_package_share_directory(package_name), 'config', 'odom_rviz.rviz')
params_file = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')

rviz = Node(
    package='rviz2',
    namespace='',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file]
)

drive_controller = Node(
    package=package_name,
    executable='drive_controller',
    name='drive_controller',
    parameters=[params_file],
)

ros_serial = Node(
    package=package_name,
    executable='drive_controller',
    name='ros_serial',
    parameters=[params_file],
)

keyboard_handler = Node(
    package=package_name,
    executable='drive_controller',
    name='keyboard_handler',
    parameters=[params_file],
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(rviz)
    ld.add_action(drive_controller)
    # ld.add_action(ros_serial)
    # ld.add_action(keyboard_handler)
    
    return ld
