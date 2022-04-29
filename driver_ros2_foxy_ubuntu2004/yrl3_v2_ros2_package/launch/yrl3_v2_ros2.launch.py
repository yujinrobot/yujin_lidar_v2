import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_yaml = os.path.join(
        get_package_share_directory('yrl3_v2_ros2_package'),
        'config',
        'lidar_params.yaml'
    )
    config_rviz = os.path.join(
        get_package_share_directory('yrl3_v2_ros2_package'),
        'config',
        'yrl3_v2_rviz2.rviz'
    )
    return LaunchDescription([
        Node(
            package="yrl3_v2_ros2_package",
            executable="yrl3_v2_ros2_node",
            name="yrl3_v2_ros2_node",
            output="screen",
            emulate_tty=True,
            parameters=[config_yaml]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            emulate_tty=True,
            arguments=['-d', config_rviz]
        )
    ])

