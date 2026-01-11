from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_safety',
            executable='obstacle_stop',
            name='obstacle_stop',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'cmd_vel_topic': '/cmd_vel'},
                {'front_half_angle_deg': 20.0},
                {'stop_distance': 0.35},
                {'forward_speed': 0.08},
                {'publish_rate_hz': 10.0},
            ],
        )
    ])
