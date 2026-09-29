"""Two-AMR simulation: one Webots world (simulation_multi.wbt) with TWO TurtleBot3 Burgers,
'amr1' and 'amr2'. Each robot gets its own ROS namespace and its own TF tree:

    /amr1/{scan,odom,cmd_vel,ultrasonic/*,tf,tf_static,...}      /amr2/{...}

Frames are NOT prefixed (both robots use map/odom/base_link) because each namespace has a private
/tf topic (every node below remaps /tf -> /<ns>/tf, /tf_static -> /<ns>/tf_static). That is the
standard Nav2 multi-robot layout and keeps every single-robot parameter (frame names) unchanged.

Per robot this is the same chain as sim.launch.py, namespaced:
  WebotsController(extern controller, robot_name=<ns>) -> ros2_control (diffdrive + joint_state) ->
  robot_state_publisher + static base_link->base_footprint -> twist_mux -> /<ns>/cmd_vel_premonitor
(navigation_multi.launch.py then adds Nav2 and the collision monitor after twist_mux.)

The single-robot files (sim.launch.py, simulation.wbt, turtlebot_webots.urdf, ros2control.yaml) are
untouched; this launch reads them and rewrites the per-node parameter keys for each namespace.
RViz is not started (two robots would need a per-robot config); use the Webots window.
"""

import os
import tempfile

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

TOPICS = {'/scan', '/odom', '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_smoothed', '/cmd_vel_premonitor',
          '/cmd_vel_teleop'}


def namespaced(value, ns):
    """Prefix the absolute topic names of the single-robot params with /<ns>."""
    if isinstance(value, dict):
        return {k: namespaced(v, ns) for k, v in value.items()}
    if isinstance(value, list):
        return [namespaced(v, ns) for v in value]
    if isinstance(value, str) and (value in TOPICS or value.startswith('/ultrasonic/')):
        return f'/{ns}{value}'
    return value


def write_params(name, ns, params):
    """Write {'/<ns>/<node>': body} for every top-level node of `params`; return the file path."""
    out_dir = os.path.join(tempfile.gettempdir(), 'amr_multi')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'{name}_{ns}.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump({f'/{ns}/{node}': namespaced(body, ns) for node, body in params.items()}, f)
    return path


def generate_launch_description():
    sim_dir = get_package_share_directory('amr_simulation')
    nav_dir = get_package_share_directory('amr_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')

    with open(os.path.join(nav_dir, 'config', 'multi_robot.yaml')) as f:
        robots = list(yaml.safe_load(f)['robots'])
    with open(os.path.join(sim_dir, 'config', 'ros2control.yaml')) as f:
        ros2control = yaml.safe_load(f)
    with open(os.path.join(nav_dir, 'config', 'twist_mux.yaml')) as f:
        twist_mux = yaml.safe_load(f)

    # The controller's own TF frame prefixing (default: namespace) must be off: frames stay odom/base_link.
    ros2control['diffdrive_controller']['ros__parameters']['tf_frame_prefix_enable'] = False

    webots = WebotsLauncher(
        world=os.path.join(sim_dir, 'webots', 'worlds', 'simulation_multi.wbt'),
        mode=mode,
        ros2_supervisor=True,
    )

    urdf = os.path.join(sim_dir, 'resource', 'turtlebot_webots_multi.urdf')
    actions = [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        webots,
        webots._supervisor,
    ]

    for ns in robots:
        tf_remap = [('/tf', f'/{ns}/tf'), ('/tf_static', f'/{ns}/tf_static')]

        robot_state_publisher = Node(
            package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
            output='screen', remappings=tf_remap,
            parameters=[{'robot_description': '<robot name=""><link name=""/></robot>',
                         'use_sim_time': use_sim_time}],
        )
        footprint_publisher = Node(
            package='tf2_ros', executable='static_transform_publisher', namespace=ns, output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            remappings=tf_remap,
        )

        cm = f'/{ns}/controller_manager'
        spawner_args = ['--controller-manager', cm, '--controller-manager-timeout', '50']
        diffdrive_spawner = Node(package='controller_manager', executable='spawner', namespace=ns,
                                 output='screen', arguments=['diffdrive_controller'] + spawner_args)
        joint_state_spawner = Node(package='controller_manager', executable='spawner', namespace=ns,
                                   output='screen', arguments=['joint_state_broadcaster'] + spawner_args)

        driver = WebotsController(
            robot_name=ns,
            namespace=ns,
            parameters=[
                {'robot_description': urdf, 'use_sim_time': use_sim_time,
                 'set_robot_state_publisher': True},
                write_params('ros2control', ns, ros2control),
            ],
            remappings=[
                (f'/{ns}/diffdrive_controller/cmd_vel_unstamped', f'/{ns}/cmd_vel'),
                (f'/{ns}/diffdrive_controller/odom', f'/{ns}/odom'),
            ] + tf_remap,
            respawn=True,
        )

        twist_mux_node = Node(
            package='twist_mux', executable='twist_mux', name='twist_mux', namespace=ns,
            output='screen',
            parameters=[write_params('twist_mux', ns, twist_mux), {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_out', f'/{ns}/cmd_vel_premonitor')],
        )

        actions += [robot_state_publisher, footprint_publisher, driver,
                    WaitForControllerConnection(target_driver=driver,
                                                nodes_to_start=[diffdrive_spawner, joint_state_spawner]),
                    twist_mux_node]

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='realtime', description='Webots startup mode'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Webots) clock'),
        *actions,
        # Kill all nodes once the Webots simulation has exited.
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=webots,
                                        on_exit=[launch.actions.EmitEvent(event=Shutdown())])),
    ])
