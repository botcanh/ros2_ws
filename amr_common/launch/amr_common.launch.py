import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('amr_common')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'hardware.launch.py')
        )
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'odometry.launch.py')
        )
    )

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            )
        ),
        launch_arguments={'port': '8765'}.items(),
    )

    return LaunchDescription([
        hardware_launch,
        odometry_launch,
        foxglove_launch,
    ])
