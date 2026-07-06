# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**TurtleBot3 Waffle Pi Autonomous Line Following with Obstacle Avoidance** — A ROS 2 (Humble) simulation and control system that enables a TurtleBot3 robot to follow a yellow line while avoiding static obstacles using LiDAR and camera inputs.

### Key Components

- **line_follower** — Python ROS 2 package for camera-based yellow line detection and P-control steering
  - `line_detector.py` — Processes camera frames (HSV/adaptive thresholding) to locate yellow line; publishes line error to `/line_error`
  - `controller.py` — PID steering controller that subscribes to `/line_error` and publishes `/cmd_vel_raw`
  
- **tb3_safety** — Python ROS 2 package for LiDAR-based obstacle avoidance using a 6-state FSM
  - `obstacle_avoid.py` — Main node; states: IDLE → STOP → TURN_AWAY → FORWARD_AROUND → TURN_BACK → REJOIN_LINE
  - Publishes to `/cmd_vel_obstacle`
  
- **cmd_vel_mux.py** — Arbitration layer that prioritizes commands from line controller and obstacle avoidance
  - Subscribes to `/cmd_vel_raw` (line controller) and `/cmd_vel_obstacle` (obstacle avoidance)
  - Publishes merged velocity to `/cmd_vel` with timeout-based fallbacks

### Topic Flow

```
/camera/image_raw → line_detector → /line_error → line_controller → /cmd_vel_raw
/scan → obstacle_avoid → /cmd_vel_obstacle
/cmd_vel_raw + /cmd_vel_obstacle → cmd_vel_mux.py → /cmd_vel
```

## Building and Running

### Build
```bash
cd /ws
colcon build
```

### Setup Environment
```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

### Run the Full Demo (Recommended)
```bash
/ws/run_final_demo.sh
```
Starts Gazebo, line detector, line controller, obstacle avoidance, cmd_vel_mux, RViz, and world markers. Runtime: 30–35 seconds per demo cycle.

### Run Individual Components

**Gazebo with yellow line world:**
```bash
/ws/start_yellow_line_world.sh
```

**Line following (detector + controller):**
```bash
ros2 launch line_follower line_follow.launch.py
```

**Obstacle avoidance:**
```bash
ros2 launch tb3_safety obstacle_avoid.launch.py
```

**RViz visualization:**
```bash
rviz2 -d /ws/rviz/yellow_line_demo.rviz
```

**Command velocity mux:**
```bash
python3 /ws/cmd_vel_mux.py
```

## Monitoring and Debugging

### Watch Active Topics
```bash
ros2 topic echo /line_error          # Line detection error (~0 when centered)
ros2 topic echo /cmd_vel_raw         # Line controller output
ros2 topic echo /cmd_vel_obstacle    # Obstacle avoidance output
ros2 topic echo /cmd_vel             # Final muxed command to robot
ros2 topic list                      # List all active topics
ros2 node list                       # List all active nodes
```

### Check Logs
```bash
tail -f /ws/logs/autonomy_line.log        # Line detector/controller logs
tail -f /ws/logs/safety_obstacle.log      # Obstacle avoidance logs
tail -f /ws/logs/cmd_vel_mux.log          # Mux arbitration logs
```

### Stop Demo
```bash
/ws/stop_final_demo.sh
```

## Key Files and Directories

- **`/ws/worlds/yellow_line_obstacle_demo.world`** — Main Gazebo world definition (single yellow line with obstacles)
- **`/ws/src/line_follower/`** — Line detection and control package
  - `launch/line_follow.launch.py` — Configurable launch file with HSV thresholds, control gains, detection ROI, etc.
  - `line_follower/line_detector.py` — Camera processing logic
  - `line_follower/controller.py` — P-control steering logic
  
- **`/ws/src/tb3_safety/`** — Obstacle avoidance package
  - `launch/obstacle_avoid.launch.py` — FSM parameters (thresholds, timings, speeds, angles)
  - `tb3_safety/obstacle_avoid.py` — 6-state FSM implementation
  
- **`/ws/cmd_vel_mux.py`** — Command arbitration script
- **`/ws/rviz/yellow_line_demo.rviz`** — RViz configuration for visualization
- **`/ws/run_final_demo.sh`** — Master startup script

## Common Development Tasks

### Tune Line Detection Parameters
Edit the HSV thresholds and ROI in `/ws/src/line_follower/launch/line_follow.launch.py`:
- `hsv_lower_h`, `hsv_upper_h` — Hue range for yellow
- `hsv_lower_s`, `hsv_upper_s` — Saturation range
- `hsv_lower_v`, `hsv_upper_v` — Value (brightness) range
- `roi_start` — Where to start ROI on image (0.0 to 1.0, top to bottom)
- `adaptive_block`, `adaptive_c` — Adaptive thresholding kernel

Then rebuild and re-run.

### Tune Line Control Gains
In `/ws/src/line_follower/launch/line_follow.launch.py`:
- `linear_x` — Forward velocity (m/s)
- `k_p` — Proportional steering gain
- `max_ang_z` — Max angular velocity (rad/s)
- `search_w`, `search_linear_x` — Behavior when line is lost

### Adjust Obstacle Avoidance Behavior
In `/ws/src/tb3_safety/launch/obstacle_avoid.launch.py`:
- Distance thresholds: `avoid_distance`, `clear_distance`, `emergency_distance`, `hard_stop_distance`
- Angular sectors: `front_half_angle_deg`, `side_sector_min_deg`, `side_sector_max_deg`
- FSM timing: `turn_time_sec`, `forward_time_sec`, `forward_burst_time_sec`, `turn_back_time_sec`
- Speeds: `turn_speed`, `forward_speed`, `rejoin_speed`
- Line rejoin control: `rejoin_kp`, `rejoin_max_ang`, `line_rejoin_error_thresh`

### Rebuild After Code Changes
```bash
cd /ws
colcon build --packages-select line_follower  # Line follower only
colcon build --packages-select tb3_safety     # Obstacle avoidance only
colcon build                                  # All packages
```

### Run Tests
```bash
colcon test --packages-select line_follower
colcon test --packages-select tb3_safety
```

Tests check code style (flake8, pep257) and copyright headers. Results in `/ws/log/`.

## Architecture Notes

- **FSM Design**: Obstacle avoidance uses a 6-state finite state machine to handle approach, avoidance, and line-rejoin phases. Transitions are based on LiDAR sector distances and timing counters.
- **Mux Logic**: `cmd_vel_mux.py` prioritizes obstacle avoidance if recent (`obstacle_timeout_sec`), otherwise uses line control. Falls back to zero velocity if both timeout.
- **Line Detection**: Uses HSV color space for robust yellow detection; contour-based centroid calculation yields line error. Adaptive thresholding improves robustness to lighting.
- **Simulation Environment**: Gazebo runs in headless server mode; RViz provides visualization of robot state, camera feed, and scan data.

## Dependencies

- ROS 2 Humble
- `cv2` (OpenCV) for image processing
- `geometry_msgs`, `sensor_msgs`, `std_msgs` (ROS message types)
- `launch`, `launch_ros` (ROS 2 launch framework)
- Gazebo and TurtleBot3 simulation packages

## Notes for Future Work

- All launch parameters have sensible defaults but are designed to be overridable at runtime. Use `ros2 launch <pkg> <file.launch.py> key:=value` syntax.
- Logs are written to `/ws/logs/` with rotation. Old logs can be cleaned up to free disk space.
- The yellow line world is the single active demonstration; other world files are archived.
- ROS 2 uses wall-clock time by default; simulation mode (`use_sim_time:=true`) syncs to Gazebo's simulated time.
