#!/usr/bin/env bash
set +e

source /opt/ros/humble/setup.bash 2>/dev/null || true
source /ws/install/setup.bash 2>/dev/null || true

OUT=${1:-/ws/logs/demo_debug_report.txt}
mkdir -p /ws/logs
rm -f "$OUT"

{
  echo "===== DEMO DEBUG REPORT ====="
  date
  echo

  echo "===== ROS NODES ====="
  ros2 node list 2>&1
  echo

  echo "===== ROS TOPICS ====="
  ros2 topic list 2>&1
  echo

  echo "===== ACTIVE PARAMS: obstacle_avoid ====="
  for p in avoid_distance clear_distance emergency_distance forward_time_sec forward_speed rejoin_speed line_rejoin_error_thresh line_rejoin_confirm_frames; do
    ros2 param get /obstacle_avoid "$p" 2>&1
  done
  echo

  echo "===== RECENT /line_error SAMPLES ====="
  timeout 4 ros2 topic echo /line_error 2>&1
  echo

  echo "===== RECENT /cmd_vel_raw SAMPLES ====="
  timeout 4 ros2 topic echo /cmd_vel_raw 2>&1
  echo

  echo "===== RECENT /cmd_vel_obstacle SAMPLES ====="
  timeout 4 ros2 topic echo /cmd_vel_obstacle 2>&1
  echo

  echo "===== RECENT /cmd_vel SAMPLES ====="
  timeout 4 ros2 topic echo /cmd_vel 2>&1
  echo

  echo "===== AUTONOMY LINE LOG TAIL ====="
  tail -120 /ws/logs/autonomy_line.log 2>&1
  echo

  echo "===== OBSTACLE AVOID LOG TAIL ====="
  tail -160 /ws/logs/safety_obstacle.log 2>&1
  echo

  echo "===== CMD VEL MUX LOG TAIL ====="
  tail -80 /ws/logs/cmd_vel_mux.log 2>&1
  echo

  echo "===== GAZEBO LOG TAIL ====="
  tail -80 /ws/logs/gazebo.log 2>&1
} | tee "$OUT"

echo "Saved report to: $OUT"
