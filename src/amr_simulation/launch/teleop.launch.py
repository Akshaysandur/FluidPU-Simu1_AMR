"""Keyboard teleoperation for the TurtleBot3 Burger AMR.

No remapping needed: teleop_twist_keyboard's default output topic is
already '/cmd_vel' (its 'stamped' parameter defaults to False, so it
publishes plain geometry_msgs/Twist - verified against the installed
teleop_twist_keyboard.py), and amr_simulation/launch/sim.launch.py's
WebotsController driver already remaps diffdrive_controller's real input
topic to that same plain '/cmd_vel' name. This bypasses twist_mux entirely
and writes straight to the controller, same as before under the old
drivetrains - an existing design choice (manual override takes priority),
not something changed here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Run directly (no xterm wrapper): ros2 launch does not redirect a child
    # node's stdin, so teleop_twist_keyboard's raw termios keyboard capture
    # works fine reading from the terminal ros2 launch itself was started in.
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),
        teleop_node,
    ])
