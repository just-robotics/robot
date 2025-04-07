import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


package_name = 'drive_controller'

params_file = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')

drive_controller = Node(
    package=package_name,
    executable='drive_controller',
    name='drive_controller',
    parameters=[params_file],
)


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(drive_controller)
    return ld
