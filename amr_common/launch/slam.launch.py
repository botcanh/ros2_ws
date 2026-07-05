import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('amr_common'),
        'config',
        'slam_toolbox_params.yaml',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={'slam_params_file': slam_params}.items(),
    )

    return LaunchDescription([slam])
