"""FMS: starts fleet_manager (station commands -> Nav2 waypoint runs).
Requires localization + navigation.launch.py to be running.
Trigger:  ros2 service call /fms/station_1 std_srvs/srv/Trigger
     or:  ros2 topic pub --once /fms/command std_msgs/msg/String "{data: station_1}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('amr_navigation')
    return LaunchDescription([
        Node(
            package='amr_navigation',
            executable='fleet_manager.py',
            name='fleet_manager',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'fleet_manager.yaml'),
                {'graph_file': os.path.join(share, 'config', 'lab_graph.json')},
            ],
        ),
    ])
