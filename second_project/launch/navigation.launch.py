from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')

    world_file = os.path.join(pkg_dir, 'world', 'task2.world')
    map_file   = os.path.join(pkg_dir, 'map',    'map.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav2.rviz')
    csv_file    = os.path.join(pkg_dir, 'csv',    'goals.csv')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),

        # ── Stage simulator ──────────────────────────────────────────────────
        # Package: stage_ros2_stageros  (confirmed from source + demo launch)
        # The world file is passed as a CLI argument, NOT as a ROS parameter.
        # Parameters that Stage reads via declare_parameter() are listed below.
        
        Node(
            package='stage_ros2_stageros',
            executable='stageros',
            name='stageros',
            output='screen',
            arguments=[world_file], 
            parameters=[{
                'use_sim_time': True,
                'base_watchdog_timeout': 0.2,
                'is_depth_canonical': True,
                'use_model_names': False,
                'delay_odom_tf_by_one_update': False,
            }]
        ),

        # ── Map server ───────────────────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file
            }]
        ),

        # ── AMCL ─────────────────────────────────────────────────────────────
        # scan_topic is set to /base_scan in nav2_params.yaml because that is
        # the topic name hardcoded in the stageros source (kBaseScan = "base_scan").
        # Stage TF chain: odom → base_footprint → base_link → base_laser_link
        # AMCL base_frame_id must be base_footprint (not base_link) to match.
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Planner server ───────────────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            ros_arguments=[
                '--log-level', 'planner_server:=INFO',
                '--log-level', 'global_costmap.global_costmap:=WARN',
            ],
        ),


        # ── Smoother server ──────────────────────────────────────────────────
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Controller server (DWB local planner) ────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
             ros_arguments=[
                '--log-level', 'ERROR', # DWB is very verbose at INFO level, especially with the Stage laser scan topic
            ],
        ),

        # ── Behavior server ──────────────────────────────────────────────────
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── BT Navigator ─────────────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            ros_arguments=[
              '--log-level', 'ERROR',
            ],
        ),

        # ── Lifecycle manager ────────────────────────────────────────────────
        # Delayed by 5 s so that all nav2 nodes are already spinning before
        # the lifecycle manager attempts to transition them to "active".
        # With use_sim_time=true the wall-clock startup can be slower than
        # expected; bond_timeout=30 gives enough margin.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': [
                            'map_server',
                            'amcl',
                            'planner_server',
                            'smoother_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator',
                        ],
                        'bond_timeout': 30.0,
                        'attempt_respawn_reopen': True,
                        'bond_respawn_max_duration': 10.0,
                    }]
                ),
            ]
        ),

        # ── Goal publisher ───────────────────────────────────────────────────
        # Delayed by 10 s so that nav2 is fully active before the first goal
        # is sent via the NavigateToPose action server.
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='second_project',
                    executable='goal_publisher',
                    name='goal_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'csv_path': csv_file
                    }]
                ),
            ]
        ),

        # ── RViz2 ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])