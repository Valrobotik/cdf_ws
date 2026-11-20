from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    default_params_file = PathJoinSubstitution(
        [FindPackageShare('bnav'), 'config', 'nav2_params.yaml'])
    default_map_file = PathJoinSubstitution(
        [FindPackageShare('bnav'), 'maps', 'map.yaml'])

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav_node(package, executable, name, extra_remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[params_file, {'use_sim_time': sim}],
            remappings=remappings + list(extra_remappings),
        )

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

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

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {'yaml_filename': map_yaml_file, 'use_sim_time': sim},
            ],
            remappings=remappings,
        ),
        nav_node(
            'nav2_controller', 'controller_server', 'controller_server',
            [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node(
            'nav2_behaviors', 'behavior_server', 'behavior_server',
            [('cmd_vel', 'cmd_vel_nav')]),
        nav_node(
            'nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
            [('cmd_vel', 'cmd_vel_nav')]),
        nav_node(
            'nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node(
            'nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),

        # One lifecycle manager for every server this stack starts. The
        # optional route/collision/docking/following servers are deliberately
        # not launched: this robot does not use them, and a stuck optional
        # server blocks the entire Nav2 lifecycle.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav2',
            output='screen',
            parameters=[{
                'use_sim_time': sim,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
