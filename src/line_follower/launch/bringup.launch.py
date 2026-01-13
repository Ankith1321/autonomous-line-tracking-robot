from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_safety',
            executable='obstacle_avoid',
            name='obstacle_avoid',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'cmd_vel_topic': '/cmd_vel_avoid'},
                {'front_half_angle_deg': 25.0},
                {'side_sector_deg': 60.0},
                {'avoid_distance': 0.55},
                {'clear_distance': 0.75},
                {'forward_speed': 0.07},
                {'turn_speed': 0.9},
                {'publish_rate_hz': 20.0},
            ],
        ),

        Node(
            package='line_follower',
            executable='line_detector',
            name='line_detector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='line_follower',
            executable='line_controller',
            name='line_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/cmd_vel', '/cmd_vel_line')],
        ),

        Node(
            package='line_follower',
            executable='supervisor',
            name='supervisor',
            output='screen',
            parameters=[
                {'line_topic': '/cmd_vel_line'},
                {'avoid_topic': '/cmd_vel_avoid'},
                {'output_topic': '/cmd_vel'},
                {'rate_hz': 20.0},
            ],
        ),
    ])
