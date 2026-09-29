"""Multi-AMR fleet manager (fleet_manager_multi.py): runs all robots' missions at the same time with
right-of-way handling. Requires sim_multi.launch.py + navigation_multi.launch.py to be running.
Trigger:  ros2 service call /fms/fleet_mission std_srvs/srv/Trigger
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
            executable='fleet_manager_multi.py',
            name='fleet_manager',
            output='screen',
            parameters=[
                {'use_sim_time': True,
                 'graph_file': os.path.join(share, 'config', 'fleet_graph.yaml'),   # nodes, edges, stations, parking
                 'robots_file': os.path.join(share, 'config', 'multi_robot.yaml'),
                 'map_yaml': os.path.join(share, 'maps', 'room_map_world.yaml'),
                 'keepout_yaml': os.path.join(share, 'maps', 'lanes_keepout.yaml')},
            ],
        ),
    ])
