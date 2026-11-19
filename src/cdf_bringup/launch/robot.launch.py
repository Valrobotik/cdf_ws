from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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
    ekf_cfg_path = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'config', 'ekf.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Publish the Gazebo robot model and use sim time (true), or the real hardware model (false)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start rviz'),

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
    ])
