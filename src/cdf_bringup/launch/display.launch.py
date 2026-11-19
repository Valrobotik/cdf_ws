from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')

    urdf = Command(['xacro ',
                    PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'description', 'model.urdf.xacro']),
                    ' use_sim:=', sim])
    urdf_cam = Command(['xacro ',
                        PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'description', 'mcc_camera.urdf.xacro'])])

    gz_launch_path = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    world_path = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'world.sdf'])

    rz_bridge_cfg_path = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'config', 'ros_gz_example_bridge.yaml'])
    ekf_cfg_path = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'config', 'ekf.yaml'])

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

    # Activate the mecanum controller only once the broadcaster is up, to avoid
    # concurrent controller switches between the real and simulated manager.
    spawn_mecanum_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run in Gazebo with gz_ros2_control (true) or on the real hardware (false)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start rviz'),

        # ------------------------------------------------------------------
        # Simulation only: Gazebo, model spawning and the Gazebo bridge.
        # ------------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            condition=IfCondition(sim),
            launch_arguments={
                'gz_args': [world_path, ' -r -s'],
                # 'gz_args': 'empty.sdf -r',
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
            condition=IfCondition(sim),
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
            condition=IfCondition(sim),
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
                ['config_file:=', rz_bridge_cfg_path]],
            parameters=[{'use_sim_time': sim}],
            condition=IfCondition(sim),
        ),

        # ------------------------------------------------------------------
        # Common to simulation and real hardware.
        # ------------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf, 'use_sim_time': sim}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_mcc',
            output='screen',
            namespace='mcc',
            parameters=[{'robot_description': urdf_cam, 'use_sim_time': sim}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_cfg_path, {'use_sim_time': sim}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'robot.rviz',])],
            parameters=[{'use_sim_time': sim}],
        ),

        # ------------------------------------------------------------------
        # Real hardware only: controller_manager hosting stepper_foc_1_hw.
        # In simulation gz_ros2_control provides its own controller_manager.
        # ------------------------------------------------------------------
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[controllers_cfg, {'use_sim_time': sim}],
            remappings=[('~/robot_description', '/robot_description')],
            condition=UnlessCondition(sim),
        ),

        # ------------------------------------------------------------------
        # Controllers: loaded from the real ros2_control_node or from the
        # gz_ros2_control plugin, depending on `sim`.
        # ------------------------------------------------------------------
        joint_state_broadcaster_spawner,
        spawn_mecanum_after_broadcaster,
    ])
