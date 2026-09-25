from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('amr_description')
    default_urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'robots', 'amr.urdf.xacro'])
    default_rviz_path = PathJoinSubstitution([pkg_share, 'rviz', 'viewport_config.rviz'])

    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='amr',
        description='Name of the robot (also selects config/<robot_name>/controllers.yaml)')
    declare_prefix = DeclareLaunchArgument(
        'prefix', default_value='',
        description='Prefix for robot joints and links')
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo', default_value='false', choices=['true', 'false'],
        description='Whether to load the Gazebo Sim / ros2_control plugins')
    declare_urdf_model = DeclareLaunchArgument(
        'urdf_model', default_value=default_urdf_path,
        description='Absolute path to the robot xacro/urdf file')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file', default_value=default_rviz_path,
        description='Absolute path to the RViz config file')
    declare_jsp_gui = DeclareLaunchArgument(
        'jsp_gui', default_value='true', choices=['true', 'false'],
        description='Launch joint_state_publisher_gui for interactive joint sliders')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', choices=['true', 'false'],
        description='Whether to start RViz')

    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('urdf_model'), ' ',
            'robot_name:=', LaunchConfiguration('robot_name'), ' ',
            'prefix:=', LaunchConfiguration('prefix'), ' ',
            'use_gazebo:=', LaunchConfiguration('use_gazebo'),
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    )

    return LaunchDescription([
        declare_robot_name,
        declare_prefix,
        declare_use_gazebo,
        declare_urdf_model,
        declare_rviz_config,
        declare_jsp_gui,
        declare_use_rviz,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
