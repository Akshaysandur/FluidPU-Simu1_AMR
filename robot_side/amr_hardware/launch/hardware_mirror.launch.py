"""Brings up the real robot for mirror mode: run this ON the Raspberry Pi, after the dev machine's Webots +
Nav2 (sim.launch.py + navigation.launch.py) are already running and driving the simulated robot.

Starts: dynamixel_drive (mirrors /cmd_vel onto the real wheels), imu_node, ultrasonic_node (the real-world
safety net), and the SLAMTEC LiDAR via the standard sllidar_ros2 package (installed separately, see README).
None of these feed back into the dev machine's Nav2 - see the package docstring in dynamixel_drive.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('amr_hardware')
    params = os.path.join(share, 'config', 'hardware.yaml')

    nodes = [
        Node(package='amr_hardware', executable='dynamixel_drive', name='dynamixel_drive',
             output='screen', parameters=[params]),
        Node(package='amr_hardware', executable='imu_node', name='imu_node',
             output='screen', parameters=[params]),
        Node(package='amr_hardware', executable='ultrasonic_node', name='ultrasonic_node',
             output='screen', parameters=[params]),
    ]

    try:
        lidar_share = get_package_share_directory('sllidar_ros2')
        lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_share, 'launch', 'sllidar_a2m12_launch.py')),
            launch_arguments={'serial_port': '/dev/ttyUSB1', 'serial_baudrate': '256000',
                              'frame_id': 'lidar_link'}.items())
        nodes.append(lidar)
    except Exception:
        pass   # sllidar_ros2 not installed yet - the other nodes still come up; see README for apt/build steps

    return LaunchDescription(nodes)
