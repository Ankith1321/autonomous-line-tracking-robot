from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_obstacle'),
        DeclareLaunchArgument('front_half_angle_deg', default_value='25.0'),
        DeclareLaunchArgument('side_sector_min_deg', default_value='35.0'),
        DeclareLaunchArgument('side_sector_max_deg', default_value='100.0'),
        DeclareLaunchArgument('avoid_distance', default_value='0.35'),
        DeclareLaunchArgument('clear_distance', default_value='0.45'),
        DeclareLaunchArgument('emergency_distance', default_value='0.18'),
        DeclareLaunchArgument('stop_time_sec', default_value='0.30'),
        DeclareLaunchArgument('turn_time_sec', default_value='1.40'),
        DeclareLaunchArgument('forward_time_sec', default_value='0.90'),
        DeclareLaunchArgument('min_turn_away_time_sec', default_value='0.75'),
        DeclareLaunchArgument('turn_back_time_sec', default_value='0.90'),
        DeclareLaunchArgument('turn_speed', default_value='0.30'),
        DeclareLaunchArgument('forward_speed', default_value='0.03'),
        DeclareLaunchArgument('rejoin_speed', default_value='0.035'),
        DeclareLaunchArgument('search_rejoin_speed', default_value='0.025'),
        DeclareLaunchArgument('search_rejoin_turn_speed', default_value='0.08'),
        DeclareLaunchArgument('rejoin_kp', default_value='0.0025'),
        DeclareLaunchArgument('rejoin_max_ang', default_value='0.18'),
        DeclareLaunchArgument('line_rejoin_error_thresh', default_value='80.0'),
        DeclareLaunchArgument('line_rejoin_confirm_frames', default_value='5'),
        DeclareLaunchArgument('max_rejoin_time_sec', default_value='6.0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
    ]

    avoid_node = Node(
        package='tb3_safety',
        executable='obstacle_avoid',
        name='obstacle_avoid',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'front_half_angle_deg': LaunchConfiguration('front_half_angle_deg'),
            'side_sector_min_deg': LaunchConfiguration('side_sector_min_deg'),
            'side_sector_max_deg': LaunchConfiguration('side_sector_max_deg'),
            'avoid_distance': LaunchConfiguration('avoid_distance'),
            'clear_distance': LaunchConfiguration('clear_distance'),
            'emergency_distance': LaunchConfiguration('emergency_distance'),
            'stop_time_sec': LaunchConfiguration('stop_time_sec'),
            'turn_time_sec': LaunchConfiguration('turn_time_sec'),
            'forward_time_sec': LaunchConfiguration('forward_time_sec'),
            'min_turn_away_time_sec': LaunchConfiguration('min_turn_away_time_sec'),
            'turn_back_time_sec': LaunchConfiguration('turn_back_time_sec'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'rejoin_speed': LaunchConfiguration('rejoin_speed'),
            'search_rejoin_speed': LaunchConfiguration('search_rejoin_speed'),
            'search_rejoin_turn_speed': LaunchConfiguration('search_rejoin_turn_speed'),
            'rejoin_kp': LaunchConfiguration('rejoin_kp'),
            'rejoin_max_ang': LaunchConfiguration('rejoin_max_ang'),
            'line_rejoin_error_thresh': LaunchConfiguration('line_rejoin_error_thresh'),
            'line_rejoin_confirm_frames': LaunchConfiguration('line_rejoin_confirm_frames'),
            'max_rejoin_time_sec': LaunchConfiguration('max_rejoin_time_sec'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }],
    )

    return LaunchDescription(arguments + [avoid_node])
