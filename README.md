# TurtleBot3 Camera-Based Line Follower with Static Obstacle Avoidance

This ROS 2 Humble workspace implements autonomous line following combined with a finite state machine (FSM) safety controller for static obstacle avoidance on a TurtleBot3 Waffle Pi. The system uses a visual PID loop for line tracking, Lidar scan sectors for obstacle detours, and a control multiplexer for seamless transitions.

---

## System Topology & Data Flow

```text
               +------------------+
               |   Gazebo /scan   |
               +--------+---------+
                        |
                        v
               +--------+---------+
               |  obstacle_avoid  +---------------+
               +--------+---------+               |
                        | (cmd_vel_obstacle)      | (safety_state)
                        v                         v
+----------+   +--------+---------+     +---------+--------+
| Camera   |-->|  line_detector   |     |   cmd_vel_mux    |
+----------+   +--------+---------+     +---------+--------+
                        | (line_error)            ^
                        v                         |
               +--------+---------+               |
               | line_controller  +---------------+
               +------------------+ (cmd_vel_raw)
```

1. **Line Detection**: The `line_detector` node processes `/camera/image_raw` using OpenCV (HSV thresholding, morphological filtering, and contour selection) to compute the centroid deviation from the center of the frame (`/line_error`).
2. **Line Control**: The `line_controller` node uses a PID controller (proportional steering with velocity slowdown at sharp curves) to publish raw control inputs (`/cmd_vel_raw`).
3. **Obstacle Avoidance**: The `obstacle_avoid` node processes `/scan` Lidar data. When an obstacle is detected in the front sector, it publishes to `/safety_state` and executes a multi-phase bypass trajectory (Turn Away, Shift Out, Drive Past, and Rejoin).
4. **Command Mux**: The `cmd_vel_mux` node arbitrates control between `/cmd_vel_raw` and `/cmd_vel_obstacle` based on the active `/safety_state`, publishing the output to `/cmd_vel` with zero handover latency.

---

## Directory Structure

```text
ws/
├── .dev_archive/                  # Deprecated scripts, archives, and media files
├── rviz/                          # RViz visualization configuration profiles
│   └── yellow_line_demo.rviz
├── worlds/                        # Gazebo simulation environments
│   └── yellow_line_obstacle_demo.world
├── run_final_demo.sh              # Production orchestrator (starts all nodes, simulator, and RViz)
├── stop_final_demo.sh             # Graceful teardown of Gazebo and ROS 2 nodes
├── collect_demo_debug.sh          # Diagnosis helper (saves report to /ws/logs/demo_debug_report.txt)
├── start_yellow_line_world.sh     # Headless Gazebo & robot spawn script
└── src/                           # ROS 2 source packages
    ├── line_follower/             # Vision-based line tracking package
    │   ├── launch/
    │   │   └── line_follow.launch.py
    │   └── line_follower/
    │       ├── controller.py      # Line tracking velocity PID controller
    │       └── line_detector.py   # OpenCV image processing node
    └── tb3_safety/                # Safety monitoring and bypass maneuvers package
        ├── launch/
        │   └── obstacle_avoid.launch.py
        └── tb3_safety/
            ├── cmd_vel_mux.py     # Command velocity priority multiplexer
            ├── obstacle_avoid.py  # FSM safety detour and bypass node
            └── world_markers.py   # RViz visualization marker publisher
```

---

## Installation & Build

### Dependencies
- ROS 2 Humble
- Gazebo 11
- OpenCV 4
- `cv_bridge`
- TurtleBot3 simulation packages (included in `src/`)

### Compilation
From the workspace root, compile the packages:
```bash
colcon build --symlink-install
```
Source the environment:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Execution Guide

### 1. Launch the Simulation and Autonomy Stack
Execute the unified orchestration script. This starts the Gazebo environment, spawns the robot, launches the line follower and obstacle avoidance nodes, and opens RViz:
```bash
./run_final_demo.sh
```

### 2. Monitoring & Debugging
Tail node logs in real time:
```bash
tail -f logs/autonomy_line.log
tail -f logs/safety_obstacle.log
tail -f logs/cmd_vel_mux.log
```
Check control messages:
```bash
ros2 topic echo /line_error
ros2 topic echo /cmd_vel
```

### 3. Graceful Shutdown
Shut down all processes, including simulator instances, RViz, and background nodes:
```bash
./stop_final_demo.sh
```
