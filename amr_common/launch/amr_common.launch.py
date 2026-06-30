import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    return LaunchDescription([
        hardware_launch,
        odometry_launch,
    ])
