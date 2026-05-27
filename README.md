# ROS 2 Humble TurtleBot3 Line Following Demo

## Project Goal

This workspace runs a TurtleBot3 simulation in ROS 2 Humble that follows a visible line using the camera while using LaserScan data as a simple safety stop input.

The goal of the current demo is a slow, observable, end-to-end pipeline for debugging perception, control, safety, and command arbitration inside the VS Code Dev Container.

## Architecture And Topic Flow

Gazebo/TurtleBot3 publishes sensor data:

```text
/camera/image_raw
/scan
```

The line-following pipeline is:

```text
/camera/image_raw
  -> line_detector
  -> /line_error
  -> line_controller
  -> /cmd_vel_raw
```

The safety pipeline is:

```text
/scan
  -> obstacle_stop
  -> /cmd_vel_obstacle   only when an obstacle is inside stop_distance
```

The final command pipeline is:

```text
/cmd_vel_raw + /cmd_vel_obstacle
  -> cmd_vel_mux.py
  -> /cmd_vel
  -> TurtleBot3 diff drive in Gazebo
```

The mux gives priority to recent `/cmd_vel_obstacle` messages. When no recent obstacle command exists, it uses `/cmd_vel_raw`. If both inputs are stale, it publishes a zero Twist.

## Packages And Nodes

### Custom Packages

`line_follower`

- `line_detector`: subscribes to `/camera/image_raw`; publishes `/line_error`, `/line_mask`, and `/line_mask_nonzero`.
- `line_controller`: subscribes to `/line_error`; publishes `/cmd_vel_raw`.
- `supervisor`: older/alternate command arbitration node, not used by the final demo.
- `simple_node`: minimal test node.

`tb3_safety`

- `obstacle_stop`: subscribes to `/scan`; publishes zero Twist on `/cmd_vel_obstacle` only while an obstacle is inside the configured stop distance.
- `obstacle_avoid`: timed obstacle avoidance primitive; present in the package but not used by the final demo.

### Support Packages

The workspace also contains TurtleBot3 packages and simulation support:

- `turtlebot3_bringup`
- `turtlebot3_description`
- `turtlebot3_gazebo`
- `turtlebot3_navigation2`
- `turtlebot3_cartographer`
- `turtlebot3_teleop`
- `turtlebot3_msgs`

## Final Demo

Run the final demo from inside the VS Code Dev Container:

```bash
/ws/run_final_demo.sh
```

The script starts:

1. Gazebo/TurtleBot3 using `/ws/start_gazebo.sh`
2. Waits for `/camera/image_raw` and `/scan`
3. Line detector and line controller with tuned debug-safe parameters
4. Obstacle stop safety node with reduced simulation stop distance
5. `cmd_vel_mux.py`

Logs are written to:

```bash
/ws/logs/gazebo.log
/ws/logs/autonomy_line.log
/ws/logs/safety_obstacle.log
/ws/logs/cmd_vel_mux.log
```

Process IDs are saved to:

```bash
/ws/logs/final_demo.pids
```

## Stop Command

Stop the demo from inside the VS Code Dev Container:

```bash
/ws/stop_final_demo.sh
```

The stop script does not use Docker commands. It stops project/demo processes, Gazebo/TurtleBot3 simulation processes, waits briefly, and prints the remaining ROS node list.

## Debug Commands

Watch key topics:

```bash
ros2 topic echo /line_error
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel_obstacle
ros2 topic echo /cmd_vel
```

Watch logs:

```bash
tail -f /ws/logs/gazebo.log
tail -f /ws/logs/autonomy_line.log
tail -f /ws/logs/safety_obstacle.log
tail -f /ws/logs/cmd_vel_mux.log
```

Inspect graph state:

```bash
ros2 node list
ros2 topic list
```

Useful visual/debug topics from the detector:

```bash
ros2 topic echo /line_mask_nonzero
ros2 topic hz /camera/image_raw
ros2 topic hz /scan
```

## Current Final Demo Tuning

The current `/ws/run_final_demo.sh` uses these line-following values:

```bash
ros2 launch line_follower line_follow.launch.py \
  use_sim_time:=true \
  roi_start:=0.60 \
  fixed_thresh:=100 \
  min_nonzero:=50 \
  k_p:=0.003 \
  max_ang_z:=0.10 \
  linear_x:=0.005 \
  search_w:=0.05 \
  search_linear_x:=0.003
```

Detector tuning:

- `roi_start:=0.60`: searches the lower 40 percent of the camera image.
- `fixed_thresh:=100`: lowers the grayscale threshold so the line is easier to detect in the current scene.
- `min_nonzero:=50`: allows a small number of line pixels to count as a valid detection.

Controller tuning:

- `linear_x:=0.005`: very slow forward speed for debugging.
- `k_p:=0.003`: low proportional steering gain.
- `max_ang_z:=0.10`: caps angular speed for smoother observation.
- `search_w:=0.05`: slow rotation while searching for a lost line.
- `search_linear_x:=0.003`: very slow forward motion during search.

Safety tuning:

```bash
ros2 launch tb3_safety obstacle_stop.launch.py use_sim_time:=true stop_distance:=0.20
```

- `stop_distance:=0.20`: reduced for simulation testing so the robot does not stop too early.

Mux behavior:

- `cmd_vel_mux.py` listens to `/cmd_vel_raw` and `/cmd_vel_obstacle`.
- It publishes final `/cmd_vel`.
- Obstacle commands have priority while recent.
- Line commands are used when obstacle commands are absent or stale.

## Implemented

- Gazebo/TurtleBot3 simulation startup.
- Camera input through `/camera/image_raw`.
- Line mask generation and line error publication.
- Slow P-controller for line following through `/cmd_vel_raw`.
- LaserScan-based obstacle stop through `/cmd_vel_obstacle`.
- Command mux that arbitrates obstacle stop over line following.
- Dev-container-safe final run and stop scripts.
- Log files and PID tracking for final demo runs.

## Known Limitations

- Line detection currently uses a simple grayscale threshold path; some launch arguments such as `line_is_dark`, `use_adaptive`, `adaptive_block`, and `adaptive_c` are exposed but the current detector threshold function does not use all of them.
- Tuning is scene-specific. Lighting, camera pose, ROI, and line color can require different `roi_start`, `fixed_thresh`, and validity thresholds.
- The robot is intentionally tuned very slowly for debugging, not for performance.
- `obstacle_stop` is a stop-only safety behavior. It does not plan around obstacles.
- The older `supervisor` flow exists in the codebase but the final demo uses `cmd_vel_mux.py`.
- The stop script uses strong `pkill -9` patterns for stubborn Gazebo/TurtleBot3 processes, which is practical for this dev container but not a production lifecycle manager.

## Future Work

- Improve line detection with a real selectable threshold mode, such as bright-line, dark-line, adaptive threshold, or HSV color filtering.
- Add launch-level presets for different tracks and lighting conditions.
- Replace stop-only safety with a true obstacle avoidance or local planning behavior.
- Add automated tests for detector parameter wiring and mux priority behavior.
- Add RViz/Gazebo visualization notes for `/line_mask` and robot state.
- Consolidate older launch paths so there is one documented bringup architecture.
