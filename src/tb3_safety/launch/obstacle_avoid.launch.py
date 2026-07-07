from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('front_half_angle_deg', default_value='28.0'),
        DeclareLaunchArgument('side_sector_min_deg', default_value='30.0'),
        DeclareLaunchArgument('side_sector_max_deg', default_value='120.0'),
        DeclareLaunchArgument('avoid_distance', default_value='0.95'),
        DeclareLaunchArgument('clear_distance', default_value='1.10'),
        DeclareLaunchArgument('forward_front_min', default_value='0.85'),
        DeclareLaunchArgument('emergency_distance', default_value='0.45'),
        DeclareLaunchArgument('side_clear_distance', default_value='0.45'),
        DeclareLaunchArgument('side_emergency_distance', default_value='0.28'),
        DeclareLaunchArgument('clear_confirm_frames', default_value='8'),
        DeclareLaunchArgument('turn_time_sec', default_value='1.20'),
        DeclareLaunchArgument('forward_time_sec', default_value='2.80'),
        DeclareLaunchArgument('turn_speed', default_value='0.32'),
        DeclareLaunchArgument('emergency_turn_speed', default_value='0.42'),
        DeclareLaunchArgument('forward_speed', default_value='0.12'),
        DeclareLaunchArgument('rejoin_speed', default_value='0.030'),
        DeclareLaunchArgument('search_rejoin_speed', default_value='0.020'),
        DeclareLaunchArgument('search_rejoin_turn_speed', default_value='0.07'),
        DeclareLaunchArgument('rejoin_kp', default_value='0.0025'),
        DeclareLaunchArgument('rejoin_max_ang', default_value='0.18'),
        DeclareLaunchArgument('line_rejoin_error_thresh', default_value='75.0'),
        DeclareLaunchArgument('line_rejoin_confirm_frames', default_value='5'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
    ]

    avoid_node = Node(
        package='tb3_safety',
        executable='obstacle_avoid',
        name='obstacle_avoid',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'front_half_angle_deg': LaunchConfiguration('front_half_angle_deg'),
            'side_sector_min_deg': LaunchConfiguration('side_sector_min_deg'),
            'side_sector_max_deg': LaunchConfiguration('side_sector_max_deg'),
            'avoid_distance': LaunchConfiguration('avoid_distance'),
            'clear_distance': LaunchConfiguration('clear_distance'),
            'forward_front_min': LaunchConfiguration('forward_front_min'),
            'emergency_distance': LaunchConfiguration('emergency_distance'),
            'side_clear_distance': LaunchConfiguration('side_clear_distance'),
            'side_emergency_distance': LaunchConfiguration('side_emergency_distance'),
            'clear_confirm_frames': LaunchConfiguration('clear_confirm_frames'),
            'turn_time_sec': LaunchConfiguration('turn_time_sec'),
            'forward_time_sec': LaunchConfiguration('forward_time_sec'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'emergency_turn_speed': LaunchConfiguration('emergency_turn_speed'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'rejoin_speed': LaunchConfiguration('rejoin_speed'),
            'search_rejoin_speed': LaunchConfiguration('search_rejoin_speed'),
            'search_rejoin_turn_speed': LaunchConfiguration('search_rejoin_turn_speed'),
            'rejoin_kp': LaunchConfiguration('rejoin_kp'),
            'rejoin_max_ang': LaunchConfiguration('rejoin_max_ang'),
            'line_rejoin_error_thresh': LaunchConfiguration('line_rejoin_error_thresh'),
            'line_rejoin_confirm_frames': LaunchConfiguration('line_rejoin_confirm_frames'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }],
    )

    return LaunchDescription(arguments + [avoid_node])
