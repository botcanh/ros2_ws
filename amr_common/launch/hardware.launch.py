import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bridge_node = Node(
        package='esp32_bridge',
        executable='bridge_node',
        name='esp32_bridge',
        output='screen',
    )

    hclidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hclidar_driver_ros2'),
                'launch',
                'hclidar_launch.py',
            )
        )
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('amr_common'),
                'launch',
                'static_tf.launch.py',
            )
        )
    )

    return LaunchDescription([
        bridge_node,
        hclidar_launch,
        static_tf_launch,
    ])
