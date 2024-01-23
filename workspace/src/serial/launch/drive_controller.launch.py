import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


package_name = 'serial'

serial_params = os.path.join(get_package_share_directory(package_name), 'config', 'drive_controller_params.yaml')

serial = Node(
    package=package_name,
    executable='serial',
    name='serial',
    parameters=[serial_params]
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(serial)
    
    return ld
