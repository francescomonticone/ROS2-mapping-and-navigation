from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def make_slam_node(context, *args, **kwargs):
    # Read the selected SLAM mode from the launch argument at runtime.
    slam_mode = LaunchConfiguration('slam_mode').perform(context).lower()

    # Select the correct SLAM Toolbox executable depending on the chosen mode.
    if slam_mode == 'sync':
        executable = 'sync_slam_toolbox_node'
    elif slam_mode == 'async':
        executable = 'async_slam_toolbox_node'
    else:
        raise RuntimeError("slam_mode must be either 'async' or 'sync'")

    # Create the SLAM Toolbox node with the chosen executable and the shared parameter file.
    return [
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mode': 'mapping',
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                },
            ],
        )
    ]


def generate_launch_description():
    # Get the installed share directory of the package.
    pkg_share = FindPackageShare('second_project')

    # Default parameter file for SLAM Toolbox mapping mode.
    default_params = PathJoinSubstitution([
        pkg_share, 'config', 'slam_toolbox_mapping.yaml'
    ])

    # Default RViz configuration file for mapping visualization.
    default_rviz = PathJoinSubstitution([
        pkg_share, 'rviz', 'mapping.rviz'
    ])

    # Convert the 3D pointcloud from the bag into a 2D LaserScan for SLAM.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/ugv/rslidar_points'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            #TF
            'target_frame': 'rslidar',

            # Allow a small TF delay tolerance when reading transforms from the bag.
            'transform_tolerance': 0.5, #high tolerance, Tf arrives with delay

            # Start with permissive height limits so the scan is not too aggressively filtered.
            'min_height': -0.1,
            'max_height': 1.0,

            # Keep a full 360 degree scan around the robot.
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,

            # Filter out invalid or too distant measurements.
            'range_min': 0.1,
            'range_max': 6.0, #max distance of laser

            # Keep scan timing coherent with bag playback.
            'scan_time': 0.1,
            'queue_size': 50, #high value

            # Publish infinite values for missing returns.
            'use_inf': True,
            'inf_epsilon': 1.0,
        }]
    )

    # RViz is started only when the rviz argument is true.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        # Enable or disable RViz from the command line.
        DeclareLaunchArgument('rviz', default_value='true'),

        # Select SLAM Toolbox mode: async is usually better for this project.
        DeclareLaunchArgument('slam_mode', default_value='async'),

        # Use bag time instead of wall clock time.
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Topic names and frames used by SLAM Toolbox.
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_frame', default_value='UGV_odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='UGV_base_link'),

        # External YAML file with the SLAM Toolbox parameters.
        DeclareLaunchArgument('slam_params_file', default_value=default_params),

        # External RViz config file.
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        # Start the converter before SLAM so the scan topic exists.
        pointcloud_to_laserscan,

        # Start SLAM Toolbox with the selected mode.
        OpaqueFunction(function=make_slam_node),

        # Start RViz for visualization.
        rviz,
    ])