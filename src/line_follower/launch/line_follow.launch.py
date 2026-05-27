from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    env_qt = SetEnvironmentVariable(name="QT_X11_NO_MITSHM", value="1")

    # --- Common ---
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    # --- Detector tuning ---
    roi_start_arg = DeclareLaunchArgument("roi_start", default_value="0.80")
    line_is_dark_arg = DeclareLaunchArgument("line_is_dark", default_value="false")
    use_adaptive_arg = DeclareLaunchArgument("use_adaptive", default_value="true")
    adaptive_block_arg = DeclareLaunchArgument("adaptive_block", default_value="51")
    adaptive_c_arg = DeclareLaunchArgument("adaptive_c", default_value="2")
    kernel_size_arg = DeclareLaunchArgument("kernel_size", default_value="5")
    fixed_thresh_arg = DeclareLaunchArgument("fixed_thresh", default_value="150")
    min_nonzero_arg = DeclareLaunchArgument("min_nonzero", default_value="300")
    max_fill_ratio_arg = DeclareLaunchArgument("max_fill_ratio", default_value="0.75")
    min_contour_area_arg = DeclareLaunchArgument("min_contour_area", default_value="250")

    # --- Controller tuning ---
    linear_x_arg = DeclareLaunchArgument("linear_x", default_value="0.04")
    k_p_arg = DeclareLaunchArgument("k_p", default_value="0.01")
    max_ang_z_arg = DeclareLaunchArgument("max_ang_z", default_value="1.0")
    steer_sign_arg = DeclareLaunchArgument("steer_sign", default_value="1.0")

    detector = Node(
        package="line_follower",
        executable="line_detector",
        name="line_detector",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "roi_start": LaunchConfiguration("roi_start"),
            "line_is_dark": LaunchConfiguration("line_is_dark"),
            "use_adaptive": LaunchConfiguration("use_adaptive"),
            "adaptive_block": LaunchConfiguration("adaptive_block"),
            "adaptive_c": LaunchConfiguration("adaptive_c"),
            "kernel_size": LaunchConfiguration("kernel_size"),
            "fixed_thresh": LaunchConfiguration("fixed_thresh"),
            "min_nonzero": LaunchConfiguration("min_nonzero"),
            "max_fill_ratio": LaunchConfiguration("max_fill_ratio"),
            "min_contour_area": LaunchConfiguration("min_contour_area"),
        }],
    )

    controller = Node(
        package="line_follower",
        executable="line_controller",
        name="line_controller",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "linear_x": LaunchConfiguration("linear_x"),
            "k_p": LaunchConfiguration("k_p"),
            "max_ang_z": LaunchConfiguration("max_ang_z"),
            "steer_sign": LaunchConfiguration("steer_sign"),
        }],
        # IMPORTANT: do NOT remap /cmd_vel here.
        # Your controller should publish /cmd_vel_raw directly and mux will publish /cmd_vel.
    )

    return LaunchDescription([
        env_qt,
        use_sim_time_arg,
        roi_start_arg,
        line_is_dark_arg,
        use_adaptive_arg,
        adaptive_block_arg,
        adaptive_c_arg,
        kernel_size_arg,
        fixed_thresh_arg,
        min_nonzero_arg,
        max_fill_ratio_arg,
        min_contour_area_arg,
        linear_x_arg,
        k_p_arg,
        max_ang_z_arg,
        steer_sign_arg,
        detector,
        controller,
    ])
