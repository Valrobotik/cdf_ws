from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')

    sim_launch = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'launch', 'sim.launch.py'])
    hardware_launch = PathJoinSubstitution([FindPackageShare('cdf_bringup'), 'launch', 'hardware.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Run in Gazebo with gz_ros2_control (true) or on the real hardware (false)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start rviz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            condition=IfCondition(sim),
            launch_arguments={'rviz': rviz}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            condition=UnlessCondition(sim),
            launch_arguments={'rviz': rviz}.items(),
        ),
    ])
