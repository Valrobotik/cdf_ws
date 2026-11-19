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
    gz_launch_path = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    world_path = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'world.sdf'])
    bridge_cfg_path = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'config', 'ros_gz_example_bridge.yaml'])
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

    # gz_ros2_control creates the controller_manager; start the mecanum
    # controller once the broadcaster is active to avoid concurrent switches.
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
            launch_arguments={'sim': 'true', 'rviz': rviz}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [world_path, ' -r -s'],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
            launch_arguments=[
                          ('topic', 'robot_description'),
                          ('entity_name', 'robot'),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
            launch_arguments=[('topic', 'mcc/robot_description'),
                              ('entity_name', 'mcc_camera'),
                              ('x', '0.0'), ('y', '1.0'), ('z', '1.5'),
                              ('R', '0.0'), ('P', '1.1'), ('Y', '-1.57')],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                ['config_file:=', bridge_cfg_path]],
            parameters=[{'use_sim_time': True}],
        ),

        joint_state_broadcaster_spawner,
        spawn_mecanum_after_broadcaster,
    ])
