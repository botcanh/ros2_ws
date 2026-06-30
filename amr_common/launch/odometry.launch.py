from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    odom_node = Node(
        package='esp32_bridge',
        executable='odom_node',
        name='odom_node',
        output='screen',
    )

    return LaunchDescription([
        odom_node,
    ])
