from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')

    robot_launch = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'launch', 'robot.launch.py'])
    controllers_cfg = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'config', 'controllers.yaml'])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file', controllers_cfg,
                   '--controller-manager-timeout', '30',
                   '--switch-timeout', '30'])

    mecanum_drive_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mecanum_drive_controller',
                   '--param-file', controllers_cfg,
                   '--controller-manager-timeout', '30',
                   '--switch-timeout', '30',
                   '--controller-ros-args',
                   '-r /mecanum_drive_controller/reference:=/cmd_vel '
                   '-r /mecanum_drive_controller/odometry:=/odom '
                   '-r /mecanum_drive_controller/tf_odometry:=/tf'])

    # Start the mecanum controller after the broadcaster to avoid concurrent
    # controller switches on the hardware manager.
    spawn_mecanum_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start rviz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={'sim': 'false', 'rviz': rviz}.items(),
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controllers_cfg, {'use_sim_time': False}],
            remappings=[('~/robot_description', '/robot_description')],
        ),

        joint_state_broadcaster_spawner,
        spawn_mecanum_after_broadcaster,
    ])
