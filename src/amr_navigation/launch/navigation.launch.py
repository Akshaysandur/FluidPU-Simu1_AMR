"""Full Nav2 autonomy stack: static map + AMCL localization, Smac global
planning, DWB local control (nose-forward-only, matching TurtleBot3
Burger's non-holonomic diff-drive), and behavior/BT execution -
coordinated by nav2_lifecycle_manager so every server comes up active
automatically. Run alongside amr_simulation/sim.launch.py.

odom -> base_link TF comes from rf2o_laser_odometry (scan-matching), not
diffdrive_controller's wheel encoders. TurtleBot3's diffdrive_controller
still runs and still publishes wheel odometry on plain /odom, but its TF
broadcast is disabled (enable_odom_tf: false in
amr_simulation/config/ros2control.yaml) so only one node ever broadcasts
odom->base_link. rf2o's own solver-failure handling was hardened earlier
in this project (CLaserOdometry2DNode::process() now skips publish() and
holds the last known pose on a failed convergence instead of republishing
stale data under a fresh timestamp), so this reinstatement doesn't carry
the same risk that motivated dropping RF2O when TurtleBot3 was integrated."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml_path = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_params_file = os.path.join(
        get_package_share_directory('amr_navigation'),
        'config', 'nav2_params.yaml'
    )

    default_nav_to_pose_bt_xml = os.path.join(
        get_package_share_directory('amr_navigation'),
        'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
        }],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        ros_arguments=['--log-level', 'WARN'],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        ros_arguments=['--log-level', 'WARN'],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        ros_arguments=['--log-level', 'WARN'],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        }],
        ros_arguments=['--log-level', 'WARN'],
    )

    # VelocitySmoother sits between controller_server and twist_mux: takes
    # /cmd_vel_raw (DWB's raw, step-changing output), clamps it to the
    # acc/decel budget in nav2_params.yaml's velocity_smoother block, and
    # publishes /cmd_vel_smoothed - which twist_mux.yaml's "navigation"
    # channel now points at instead of /cmd_vel_raw directly. Is a
    # nav2_util::LifecycleNode (confirmed via `strings` on
    # libvelocity_smoother_core.so), so it needs lifecycle_manager bring-up
    # like every other server here - added to node_names below.
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_raw'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
        ],
    )

    # CollisionMonitor: real nav2_util::LifecycleNode (confirmed via its
    # header, nav2_collision_monitor/collision_monitor_node.hpp), needs
    # lifecycle_manager bring-up like every other server here - added to
    # node_names below. Sits after twist_mux (see sim.launch.py's
    # twist_mux_node comment) as the last, source-agnostic safety net:
    # reads the four sonar_* ultrasonic topics (nav2_collision_monitor's
    # own dedicated "range" source type - its real header docstring reads
    # "Implementation for IR/ultrasound range sensor source", built for
    # exactly this) directly, independent of the costmap/planner entirely,
    # and either slows or hard-stops whatever velocity twist_mux produced -
    # from Nav2 OR teleop - before it ever reaches the driver. This is
    # deliberately separate from nav2_params.yaml's local_costmap.
    # range_layer (which feeds the SAME four sensors into the costmap for
    # MPPI to plan around): that path is a normal planning input MPPI can
    # weigh against other critics and route around, this one is a
    # dedicated last-line reflex that can't be out-voted by path-following
    # priorities.
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # Classifies /cmd_vel_raw (controller_server's pre-mux output) into a
    # human-readable maneuver state, logged only on state change. Watches
    # Nav2's own output specifically - it will not see teleop's commands,
    # since that's a separate twist_mux input channel this node isn't
    # subscribed to.
    maneuver_telemetry_node = Node(
        package='amr_navigation',
        executable='maneuver_telemetry_node.py',
        name='maneuver_telemetry_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rf2o_params_file = os.path.join(
        get_package_share_directory('amr_navigation'),
        'config', 'rf2o_params.yaml'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # RF2O's first scan callback looks up base_link -> laser_frame via TF
    # (CLaserOdometry2DNode.cpp's setLaserPoseFromTf(), called once on the
    # first scan) and does not retry or abort if that lookup fails - it
    # silently continues with a garbage laser-to-base transform for the
    # rest of the run. This launch file has no dependency on
    # amr_simulation/sim.launch.py's robot_state_publisher actually being
    # up yet, so a few seconds' delay gives its static TF time to
    # propagate first.
    rf2o_node_delayed = TimerAction(period=3.0, actions=[rf2o_node])

    # Two lifecycle managers, deliberately staggered. controller_server's costmap
    # message filters call tf2 waitForTransform() when a /scan arrives before map->odom
    # exists; if AMCL's first map->odom lands in that window, tf2's BufferCore deadlocks
    # (listener thread holds the transformable-requests lock and waits for the request
    # lock that the filter thread holds - confirmed with gdb on controller_server).
    # The listener then never sees another transform, and every NavigateToPose "succeeds"
    # instantly with the robot not moving. Bringing AMCL up FIRST, so map->odom is already
    # being published when the costmaps subscribe, removes the window.
    def lifecycle_manager(name, node_names):
        return Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'bond_respawn_max_duration': 10.0,
                'attempt_respawn_reconnection': True,
                'node_names': node_names,
            }],
        )

    localization_manager = lifecycle_manager(
        'lifecycle_manager_localization', ['map_server', 'amcl'])
    navigation_manager = lifecycle_manager(
        'lifecycle_manager_navigation',
        ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
         'velocity_smoother', 'collision_monitor'])
    navigation_manager_delayed = TimerAction(period=12.0, actions=[navigation_manager])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            # amr_simulation/sim.launch.py spawns TurtleBot3 Burger inside
            # this project's own simulation.wbt world (not
            # webots_ros2_turtlebot's reference world), so this project's
            # own pre-existing map - saved against that same wall layout -
            # is the correct default again.
            # room_map_hires.yaml: same walls, same 3.10m x 2.55m extents,
            # same origin as room_map.yaml - just 0.025 m/px instead of
            # 0.05. StaticLayer forces the global costmap to the map's
            # resolution, so this is the only way to give the global planner
            # a grid fine enough to resolve this layout's 353mm passages.
            # See that file's header for the full reasoning; point this back
            # at room_map.yaml to revert.
            default_value=os.path.join(get_package_share_directory('amr_navigation'), 'maps', 'room_map_world.yaml'),
            # room_map_world: rasterised from simulation.wbt itself (see maps/make_world_map.py),
            # map frame == Webots world frame. Replaces the SLAM room_map_hires, which was
            # 3-8 % undersized versus the world and lacked the Station-1 wall and pillar.
            description='Full path to the saved map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 params file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),

        # rf2o_node_delayed intentionally NOT launched: /scan has a negative
        # angle_increment (Webots clockwise), which rf2o mirrors -> wrong odom.
        # odom->base_link now comes from diffdrive_controller (wheel odometry).
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        velocity_smoother_node,
        collision_monitor_node,
        maneuver_telemetry_node,
        localization_manager,
        navigation_manager_delayed,
    ])
