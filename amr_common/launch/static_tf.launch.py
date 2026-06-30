from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # laser_frame is 15 cm behind base_link (robot's +x is forward),
    # 2 cm up -- same height offset as the old transform this replaces.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '--x', '-0.15',
            '--y', '0',
            '--z', '0.02',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ],
    )

    return LaunchDescription([
        laser_tf,
    ])
