#!/usr/bin/env bash
set +e

source /opt/ros/humble/setup.bash 2>/dev/null || true
source /ws/install/setup.bash 2>/dev/null || true

echo "[STOP] Stopping final demo processes..."

pkill -f "ros2 launch" || true
pkill -f "line_detector" || true
pkill -f "line_controller" || true
pkill -f "obstacle_stop" || true
pkill -f "obstacle_avoid" || true
pkill -f "cmd_vel_mux.py" || true
pkill -f "gzserver" || true
pkill -f "gzclient" || true
pkill -f "gazebo" || true
pkill -f "robot_state_publisher" || true
pkill -f "spawn_entity" || true

echo "[STOP] Force-stopping stubborn Gazebo/TurtleBot3 simulation processes..."
pkill -9 -f "gzserver" || true
pkill -9 -f "gzclient" || true
pkill -9 -f "gazebo" || true
pkill -9 -f "turtlebot3" || true
pkill -9 -f "camera_driver" || true
pkill -9 -f "obstacle_avoid" || true
pkill -9 -f "robot_state_publisher" || true
pkill -9 -f "spawn_entity" || true
pkill -9 -f "parameter_bridge" || true

if [ -f /ws/logs/final_demo.pids ]; then
  echo "[STOP] Removing stale PID file: /ws/logs/final_demo.pids"
  rm -f /ws/logs/final_demo.pids || true
fi

sleep 3

echo "[STOP] Remaining ROS nodes:"
ros2 node list 2>/dev/null || true
