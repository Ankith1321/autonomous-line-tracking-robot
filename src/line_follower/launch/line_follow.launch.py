from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cam = Node(
        package="image_tools",
        executable="cam2image",
        name="cam2image",
        output="screen",
        arguments=["--ros-args", "-r", "image:=/camera/image_raw"],
    )

    detector = Node(
        package="line_follower",
        executable="line_detector",
        name="line_detector",
        output="screen",
        parameters=[
            {
                "roi_start": 0.60,
                "line_is_dark": True,
                "use_adaptive": True,
                "adaptive_block": 31,
                "adaptive_c": 5,
                "kernel_size": 5,
            }
        ],
    )

    controller = Node(
        package="line_follower",
        executable="controller",
        name="line_controller",
        output="screen",
        parameters=[
            {
                "linear_x": 0.08,
                "k_p": 0.01,
                "max_ang_z": 1.0,
                "steer_sign": 1.0,
            }
        ],
    )

    return LaunchDescription([cam, detector, controller])
