import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


package_name = 'robot'

slam_toolobox_config = os.path.join(get_package_share_directory(package_name), 'config', 'slam_tof_params.yaml')

tof_transform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_static_transform_publisher',
    arguments=[
        '--x', '0.000',
        '--y', '0.000',
        '--z', '0.252',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_footprint',
        '--child-frame-id', 'tof',
    ]
)

slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]
        ),
        launch_arguments={'slam_params_file': slam_toolobox_config}.items()
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(tof_transform)
    ld.add_action(slam_toolbox)
    
    return ld
