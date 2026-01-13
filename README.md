# Autonomous Line Tracking Robot (ROS 2)

## Overview
This project implements an **Autonomous Line Tracking Robot** using **ROS 2 (Humble)** and **Gazebo simulation**.  
The robot detects and follows a line using a **camera-based computer vision pipeline** and uses **LiDAR data for obstacle awareness**.

The system follows a **modular ROS 2 architecture**, separating perception, control, and safety components.  
All development and testing are performed in **simulation** using **TurtleBot3 (Burger)**.

---

## Project Objectives
- Detect a line using a camera
- Compute steering error from the detected line position
- Control robot motion to follow the line autonomously
- Detect obstacles using LiDAR data
- Simulate the complete system in Gazebo
- Follow ROS 2 best practices (nodes, topics, launch files, parameters)

---

## System Architecture (Implemented)

Camera  
↓  
Line Detector Node  
↓  
Line Controller Node  
↓  
`/cmd_vel`  
↓  
TurtleBot3 (Gazebo)

Debug & Visualization Topics:
- `/line_mask`
- `/line_error`
- `/scan`
- `/tf`

---

## Implemented ROS 2 Nodes

### Camera Node
- Publishes raw camera images
- Topic: `/camera/image_raw`

### Line Detector Node
- Subscribes to camera images
- Uses ROI-based adaptive thresholding
- Computes line centroid and lateral error
- Publishes:
  - `/line_error`
  - `/line_mask`
  - `/line_mask_nonzero`

### Line Controller Node
- Proportional (P) controller
- Converts line error into velocity commands
- Publishes:
  - `/cmd_vel`
- Parameters:
  - `linear_x`
  - `k_p`
  - `max_ang_z`
  - `steer_sign`

### Obstacle Detection Node (Standalone)
- Uses LiDAR (`/scan`)
- Detects obstacles in front of the robot
- Stops the robot when an obstacle is detected
- Verified independently

---

## Features Completed

### ROS 2 Environment
- ROS 2 Humble running inside Docker
- Colcon workspace successfully built
- Modular package structure

### Camera & Vision
- Camera image stream verified
- Line mask visualization working
- Real-time line detection

### Line Following Control
- P-controller implemented
- Velocity commands published at runtime
- Tunable parameters for control behavior

### Gazebo Simulation
- TurtleBot3 (Burger) spawned successfully
- LiDAR laser rays visualized (blue lines)
- Odometry, TF, and scan topics active

### Launch & Bringup
- Unified launch files to start:
  - Camera
  - Line detector
  - Line controller
- Shell scripts for:
  - Gazebo startup
  - Autonomous system startup
  - Clean shutdown

---

## Current Project Status

Overall completion: **~75–80%**

| Component                     | Status        |
|--------------------------------|---------------|
| ROS 2 Setup                    | Complete      |
| Camera Pipeline                | Complete      |
| Line Detection                 | Complete      |
| Line Following Controller      | Complete      |
| Gazebo Simulation              | Complete      |
| Obstacle Detection (Standalone)| Complete      |
| System Integration             | Partial       |
| Full Autonomous Demo           | Pending       |
| Documentation                  | In Progress   |

---

## Known Limitations / Pending Work
- Arbitration between line-following and obstacle-avoidance commands
- Dedicated Gazebo world with a visible line track
- Final end-to-end autonomous demonstration
- Further parameter tuning for smoother motion
- Minor code cleanup and polishing

---

## How to Run the Project

### Start Gazebo Simulation
```bash
xhost +local:docker
docker exec -it ros2_humble bash -lc "
source /opt/ros/humble/setup.bash &&
export TURTLEBOT3_MODEL=burger &&
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
"


### Start Autonomous Line Following
```bash
docker exec -it ros2_humble bash -lc "
source /opt/ros/humble/setup.bash &&
source /ws/install/setup.bash &&
ros2 launch line_follower bringup.launch.py
"
---

