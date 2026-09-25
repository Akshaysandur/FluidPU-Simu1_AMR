"""Start slam_toolbox's online-async SLAM node against our AMR's /scan.

Mirrors the real slam_toolbox package's own online_async_launch.py
(/opt/ros/humble/share/slam_toolbox/launch/online_async_launch.py), pointed
at our own mapper_params_online_async.yaml instead of its stock defaults.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = os.path.join(
        get_package_share_directory('amr_simulation'),
        'config', 'mapper_params_online_async.yaml'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),
        slam_toolbox_node,
    ])
