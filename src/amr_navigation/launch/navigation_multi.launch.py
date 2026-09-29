"""Two-AMR Nav2: a complete, independent Nav2 stack per robot (namespaces /amr1, /amr2), using the same
tuned nav2_params.yaml as the single-robot navigation.launch.py (map server, AMCL, Smac planner, Regulated
Pure Pursuit, behavior server, bt_navigator, velocity smoother, collision monitor).

How the single-robot parameters are reused: for every robot, nav2_params.yaml is loaded, every top-level
node key gets a '/<ns>/' prefix, absolute topic names (/scan, /odom, /cmd_vel_*, /ultrasonic/*) get the
same prefix, and AMCL's initial pose is replaced by that robot's spawn pose (multi_robot.yaml). Every node
remaps /tf -> /<ns>/tf and /tf_static -> /<ns>/tf_static (private TF tree per robot, see sim_multi.launch.py).

Each robot sees the other one through its own lidar (an obstacle in its costmaps) and the collision
monitor; deliberate right-of-way handling (who yields, where to) is done by fleet_manager_multi.py.

Bring-up order per robot is the same as navigation.launch.py (AMCL first, then the navigation servers
12 s later - the tf2 costmap-filter deadlock workaround); the second robot's navigation manager starts 3 s
after the first one to spread the start-up CPU load.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPICS = {'/scan', '/odom', '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_smoothed', '/cmd_vel_premonitor',
          '/cmd_vel_teleop'}


def namespaced(value, ns):
    if isinstance(value, dict):
        return {k: namespaced(v, ns) for k, v in value.items()}
    if isinstance(value, list):
        return [namespaced(v, ns) for v in value]
    if isinstance(value, str) and (value in TOPICS or value.startswith('/ultrasonic/')):
        return f'/{ns}{value}'
    return value


def robot_params(params, ns, pose):
    body = namespaced(params, ns)
    amcl = body['amcl']['ros__parameters']
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': pose['x'], 'y': pose['y'], 'z': 0.0, 'yaw': pose['yaw']}
    out_dir = os.path.join(tempfile.gettempdir(), 'amr_multi')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'nav2_{ns}.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump({f'/{ns}/{node}': v for node, v in body.items()}, f)
    return path


def generate_launch_description():
    nav_dir = get_package_share_directory('amr_navigation')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_xml = os.path.join(nav_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')

    with open(os.path.join(nav_dir, 'config', 'nav2_params.yaml')) as f:
        base_params = yaml.safe_load(f)
    with open(os.path.join(nav_dir, 'config', 'multi_robot.yaml')) as f:
        robots = yaml.safe_load(f)['robots']

    actions = []
    for idx, (ns, pose) in enumerate(robots.items()):
        params = robot_params(base_params, ns, pose)
        tf_remap = [('/tf', f'/{ns}/tf'), ('/tf_static', f'/{ns}/tf_static')]

        def node(package, executable, name, extra_params=None, remappings=None, warn=False):
            return Node(
                package=package, executable=executable, name=name, namespace=ns, output='screen',
                parameters=[params, {'use_sim_time': use_sim_time}, *(extra_params or [])],
                remappings=tf_remap + (remappings or []),
                ros_arguments=['--log-level', 'WARN'] if warn else [],
            )

        map_server = node('nav2_map_server', 'map_server', 'map_server',
                          extra_params=[{'yaml_filename': map_yaml}])
        amcl = node('nav2_amcl', 'amcl', 'amcl', warn=True)
        planner = node('nav2_planner', 'planner_server', 'planner_server', warn=True)
        controller = node('nav2_controller', 'controller_server', 'controller_server', warn=True,
                          remappings=[('cmd_vel', 'cmd_vel_raw')])
        behavior = node('nav2_behaviors', 'behavior_server', 'behavior_server',
                        remappings=[('cmd_vel', 'cmd_vel_raw')])
        bt_nav = node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', warn=True,
                      extra_params=[{'default_nav_to_pose_bt_xml': bt_xml}])
        smoother = node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                        remappings=[('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')])
        collision = node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor')

        def lifecycle_manager(name, node_names):
            return Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name,
                namespace=ns, output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                             'bond_respawn_max_duration': 10.0, 'attempt_respawn_reconnection': True,
                             'node_names': node_names}],
            )

        localization = lifecycle_manager('lifecycle_manager_localization', ['map_server', 'amcl'])
        navigation = lifecycle_manager(
            'lifecycle_manager_navigation',
            ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
             'velocity_smoother', 'collision_monitor'])

        actions += [map_server, amcl, planner, controller, behavior, bt_nav, smoother, collision,
                    localization, TimerAction(period=12.0 + 3.0 * idx, actions=[navigation])]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(nav_dir, 'maps', 'room_map_world.yaml'),
            description='Full path to the saved map yaml file (map frame == Webots world frame)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Webots) clock'),
        *actions,
    ])
