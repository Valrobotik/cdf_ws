from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('rviz')

    nav2_bringup_share = FindPackageShare('nav2_bringup')
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('bnav'), 'config', 'nav2_params.yaml'])
    default_map_file = PathJoinSubstitution(
        [FindPackageShare('bnav'), 'maps', 'map.yaml'])

    navigation_core_launch_path = PathJoinSubstitution(
        [FindPackageShare('bnav'), 'launch', 'navigation_core.launch.py'])
    rviz_launch_path = PathJoinSubstitution(
        [nav2_bringup_share, 'launch', 'rviz_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Use the Gazebo clock and simulation if true'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Full path to the nav2 params file'),
        DeclareLaunchArgument(
            'map', default_value=default_map_file,
            description='Full path to the static map yaml file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically bring nodes to the active state'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch the standard Nav2 RViz view alongside navigation'),

        # One Nav2 lifecycle manager for the servers this robot uses. The
        # optional route/collision/docking/following servers are not started;
        # they are unused here and a stuck optional server blocks Nav2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_core_launch_path),
            launch_arguments={
                'sim': sim,
                'params_file': params_file,
                'map': map_yaml_file,
                'autostart': autostart,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_path),
            condition=IfCondition(use_rviz),
            launch_arguments={
                # rviz_launch.py defaults to the 'navigation' namespace, while
                # this launch keeps the Nav2 nodes in the root namespace.
                'namespace': '',
                'use_sim_time': sim,
            }.items(),
        ),
        Node(
            package='bnav',
            executable='box_obstacles_node.py',
            name='box_obstacles',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('bnav'), 'config', 'box_obstacles.yaml']),
                {'use_sim_time': sim},
            ],
        ),
        Node(
            package='bnav',
            executable='alternate_goals_mission',
            name='alternate_goals_mission',
            output='screen',
            parameters=[{'use_sim_time': sim}],
        ),

    ])
