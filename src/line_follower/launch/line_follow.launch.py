from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    env_qt = SetEnvironmentVariable(name="QT_X11_NO_MITSHM", value="1")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    roi_start_arg = DeclareLaunchArgument("roi_start", default_value="0.60")
    line_is_dark_arg = DeclareLaunchArgument("line_is_dark", default_value="false")
    use_adaptive_arg = DeclareLaunchArgument("use_adaptive", default_value="true")
    adaptive_block_arg = DeclareLaunchArgument("adaptive_block", default_value="51")
    adaptive_c_arg = DeclareLaunchArgument("adaptive_c", default_value="2")
    kernel_size_arg = DeclareLaunchArgument("kernel_size", default_value="5")
    fixed_thresh_arg = DeclareLaunchArgument("fixed_thresh", default_value="150")
    use_hsv_arg = DeclareLaunchArgument("use_hsv", default_value="true")
    hsv_lower_h_arg = DeclareLaunchArgument("hsv_lower_h", default_value="15")
    hsv_lower_s_arg = DeclareLaunchArgument("hsv_lower_s", default_value="80")
    hsv_lower_v_arg = DeclareLaunchArgument("hsv_lower_v", default_value="80")
    hsv_upper_h_arg = DeclareLaunchArgument("hsv_upper_h", default_value="40")
    hsv_upper_s_arg = DeclareLaunchArgument("hsv_upper_s", default_value="255")
    hsv_upper_v_arg = DeclareLaunchArgument("hsv_upper_v", default_value="255")
    min_nonzero_arg = DeclareLaunchArgument("min_nonzero", default_value="50")
    max_fill_ratio_arg = DeclareLaunchArgument("max_fill_ratio", default_value="0.60")
    min_contour_area_arg = DeclareLaunchArgument("min_contour_area", default_value="250.0")
    lost_sentinel_arg = DeclareLaunchArgument("lost_sentinel", default_value="-1.0")
    ema_alpha_arg = DeclareLaunchArgument("ema_alpha", default_value="0.25")
    max_contour_jump_arg = DeclareLaunchArgument("max_contour_jump", default_value="120.0")
    contour_switch_confirm_frames_arg = DeclareLaunchArgument(
        "contour_switch_confirm_frames", default_value="3"
    )

    linear_x_arg = DeclareLaunchArgument("linear_x", default_value="0.04")
    k_p_arg = DeclareLaunchArgument("k_p", default_value="0.01")
    max_ang_z_arg = DeclareLaunchArgument("max_ang_z", default_value="1.0")
    steer_sign_arg = DeclareLaunchArgument("steer_sign", default_value="1.0")
    search_w_arg = DeclareLaunchArgument("search_w", default_value="0.35")
    search_linear_x_arg = DeclareLaunchArgument("search_linear_x", default_value="0.02")
    min_linear_x_arg = DeclareLaunchArgument("min_linear_x", default_value="0.02")
    slowdown_error_arg = DeclareLaunchArgument("slowdown_error", default_value="80.0")
    turn_in_place_error_arg = DeclareLaunchArgument("turn_in_place_error", default_value="240.0")
    error_deadband_arg = DeclareLaunchArgument("error_deadband", default_value="10.0")
    angular_alpha_arg = DeclareLaunchArgument("angular_alpha", default_value="0.35")
    lost_timeout_arg = DeclareLaunchArgument("lost_timeout_sec", default_value="6.0")

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
            "use_hsv": LaunchConfiguration("use_hsv"),
            "hsv_lower_h": LaunchConfiguration("hsv_lower_h"),
            "hsv_lower_s": LaunchConfiguration("hsv_lower_s"),
            "hsv_lower_v": LaunchConfiguration("hsv_lower_v"),
            "hsv_upper_h": LaunchConfiguration("hsv_upper_h"),
            "hsv_upper_s": LaunchConfiguration("hsv_upper_s"),
            "hsv_upper_v": LaunchConfiguration("hsv_upper_v"),
            "min_nonzero": LaunchConfiguration("min_nonzero"),
            "max_fill_ratio": LaunchConfiguration("max_fill_ratio"),
            "min_contour_area": LaunchConfiguration("min_contour_area"),
            "lost_sentinel": LaunchConfiguration("lost_sentinel"),
            "ema_alpha": LaunchConfiguration("ema_alpha"),
            "max_contour_jump": LaunchConfiguration("max_contour_jump"),
            "contour_switch_confirm_frames": LaunchConfiguration("contour_switch_confirm_frames"),
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
            "search_w": LaunchConfiguration("search_w"),
            "search_linear_x": LaunchConfiguration("search_linear_x"),
            "lost_sentinel": LaunchConfiguration("lost_sentinel"),
            "min_linear_x": LaunchConfiguration("min_linear_x"),
            "slowdown_error": LaunchConfiguration("slowdown_error"),
            "turn_in_place_error": LaunchConfiguration("turn_in_place_error"),
            "error_deadband": LaunchConfiguration("error_deadband"),
            "angular_alpha": LaunchConfiguration("angular_alpha"),
            "lost_timeout_sec": LaunchConfiguration("lost_timeout_sec"),
        }],
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
        use_hsv_arg,
        hsv_lower_h_arg,
        hsv_lower_s_arg,
        hsv_lower_v_arg,
        hsv_upper_h_arg,
        hsv_upper_s_arg,
        hsv_upper_v_arg,
        min_nonzero_arg,
        max_fill_ratio_arg,
        min_contour_area_arg,
        lost_sentinel_arg,
        ema_alpha_arg,
        max_contour_jump_arg,
        contour_switch_confirm_frames_arg,
        linear_x_arg,
        k_p_arg,
        max_ang_z_arg,
        steer_sign_arg,
        search_w_arg,
        search_linear_x_arg,
        min_linear_x_arg,
        slowdown_error_arg,
        turn_in_place_error_arg,
        error_deadband_arg,
        angular_alpha_arg,
        lost_timeout_arg,
        detector,
        controller,
    ])
