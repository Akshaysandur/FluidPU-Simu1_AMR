"""Launch TurtleBot3 Burger inside this project's own Webots world
(amr_simulation/webots/worlds/simulation.wbt), with this project's own
ecosystem layered on top: RViz and twist_mux.

This is a deliberate fork of webots_ros2_turtlebot's own robot_launch.py
rather than an IncludeLaunchDescription of it verbatim (unlike the first
TurtleBot3 integration pass - see decisions-and-gotchas.md). Two things
required this:
  1. robot_launch.py resolves its 'world' argument via
     PathJoinSubstitution([package_dir, 'worlds', world]) - always inside
     webots_ros2_turtlebot's own package share directory, with no way to
     point it at a world file living anywhere else, INCLUDING as an
     absolute path (confirmed: PathJoinSubstitution.perform() joins with
     str(Path(*components)), and Path() does NOT drop earlier components
     for a later absolute one the way os.path.join does - '/a'/'/b'
     stays '/a/b', not '/b'). So reaching our own simulation.wbt requires
     building our own WebotsLauncher call instead.
  2. robot_launch.py also hardcodes its ros2_control_params path to its
     own installed resource/ros2control.yml, with no launch argument to
     override it - and disabling diffdrive_controller's enable_odom_tf
     (see amr_navigation/launch/navigation.launch.py for why: rf2o_laser_
     odometry is now the sole odom->base_link TF broadcaster) requires a
     different params file.
Everything else below (robot_state_publisher, the static base_footprint
publisher, the WebotsController driver, the two controller_manager
spawners, and the wait-for-connection gating) is copied from
robot_launch.py unchanged - only the world path and the ros2_control_params
path differ. The optional turtlebot3_navigation2/turtlebot3_cartographer
blocks from the original are dropped: this project always launches its own
amr_navigation/navigation.launch.py separately, and never sets the
'nav'/'slam' arguments that would have activated those blocks anyway.
"""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir_desc = get_package_share_directory('amr_description')
    package_dir_sim = get_package_share_directory('amr_simulation')

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_path = os.path.join(package_dir_desc, 'rviz', 'viewport_config.rviz')

    # TURTLEBOT3_MODEL is read by turtlebot3-family nodes/launch files
    # (e.g. turtlebot3_teleop) that branch on robot model - the driver
    # below hardcodes 'burger' internally regardless, but this is set for
    # every other TurtleBot3 package in this workspace, not just this one.
    turtlebot3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    webots = WebotsLauncher(
        world=os.path.join(package_dir_sim, 'webots', 'worlds', 'simulation.wbt'),
        mode=mode,
        ros2_supervisor=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    # Project-owned fork (amr_simulation/resource/turtlebot_webots.urdf),
    # not webots_ros2_turtlebot's installed copy - adds explicit <device>
    # entries for the four sonar_* DistanceSensor nodes in simulation.wbt's
    # extensionSlot, so they publish on predictable topic names instead of
    # webots_ros2_driver's node-relative "~/<device_name>" default. See
    # that file's own header comment for the full reasoning.
    robot_description_path = os.path.join(
        package_dir_sim, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(
        package_dir_sim, 'config', 'ros2control.yaml')

    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params,
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom'),
        ],
        respawn=True,
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # twist_mux: merges velocity commands from two channels (see
    # amr_navigation/config/twist_mux.yaml for the current priorities -
    #   teleop=100, navigation=10).
    # Output goes to /cmd_vel_premonitor, NOT straight to /cmd_vel anymore -
    # amr_navigation/launch/navigation.launch.py's collision_monitor_node
    # now sits between this and the driver, as the last, source-agnostic
    # safety check (it must see commands from EVERY source - teleop
    # included - not just Nav2's, which is why it's spliced in after
    # twist_mux rather than before it).
    twist_mux_params = os.path.join(
        get_package_share_directory('amr_navigation'), 'config', 'twist_mux.yaml'
    )
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/cmd_vel_premonitor')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),

        turtlebot3_model_env,
        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        turtlebot_driver,
        waiting_nodes,

        rviz,
        twist_mux_node,

        # Kill all nodes once the Webots simulation has exited.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=Shutdown())],
            )
        ),
    ])
