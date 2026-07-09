import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('amr_common'),
        'config',
        'nav2_params.yaml',
    )

    map_file = LaunchConfiguration('map')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/ros2_ws/src/maps/my_map.yaml',
        description='Full path to the map yaml file',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        declare_map,
        nav2,
    ])
