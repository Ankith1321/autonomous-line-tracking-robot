from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    env_qt = SetEnvironmentVariable(name="QT_X11_NO_MITSHM", value="1")

    # Gazebo TB3 autorace world (has line track)
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "turtlebot3_autorace_2020.launch.py")
        )
    )

    detector = Node(
        package="line_follower",
        executable="line_detector",
        name="line_detector",
        output="screen",
        parameters=[{
            "roi_start": 0.60,
            "line_is_dark": True,
            "use_adaptive": True,
            "adaptive_block": 31,
            "adaptive_c": 5,
            "kernel_size": 5,
        }],
    )

    controller = Node(
        package="line_follower",
        executable="line_controller",
        name="line_controller",
        output="screen",
        parameters=[{
            "linear_x": 0.08,
            "k_p": 0.01,
            "max_ang_z": 1.0,
            "steer_sign": 1.0,
        }],
        remappings=[
            ("/cmd_vel", "/cmd_vel_raw"),
        ],
    )

    return LaunchDescription([
        env_qt,
        gazebo,
        detector,
        controller,
    ])
