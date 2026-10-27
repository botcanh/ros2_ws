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

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            )
        ),
        # The web app calls services that may not be up yet (map_server while
        # Nav2 starts, nav cancel in MANUAL). With the defaults such a call
        # blocks rosbridge's main thread forever and freezes every topic.
        launch_arguments={
            'port': '9090',
            'call_services_in_new_thread': 'true',
            'default_call_service_timeout': '5.0',
            'send_action_goals_in_new_thread': 'true',
        }.items(),
    )

    return LaunchDescription([
        hardware_launch,
        odometry_launch,
        foxglove_launch,
        rosbridge_launch,
    ])
