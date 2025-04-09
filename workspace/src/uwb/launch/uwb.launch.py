import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


package_name = 'uwb'

params_file = os.path.join(get_package_share_directory(package_name), 'config', 'serial_params.yaml')

uwb_node = Node(
    package=package_name,
    executable='uwb',
    name='uwb_node',
    parameters=[params_file],
)


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(uwb_node)
    return ld
