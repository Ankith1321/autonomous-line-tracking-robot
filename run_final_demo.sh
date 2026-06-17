#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

LOG_DIR=/ws/logs
PID_FILE=${LOG_DIR}/final_demo.pids

wait_for_topic() {
  local topic="$1"
  local timeout_sec="${2:-90}"
  local waited=0

  echo "[MONITOR] Waiting for ${topic}..."
  while ! ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; do
    if [ "${waited}" -ge "${timeout_sec}" ]; then
      echo "[MONITOR] ERROR: timed out waiting for ${topic} after ${timeout_sec}s"
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "[MONITOR] Found ${topic}"
}

echo "[MONITOR] Stopping any previous final demo processes..."
/ws/stop_final_demo.sh

mkdir -p "${LOG_DIR}"
: > "${PID_FILE}"

echo "[GAZEBO] Starting Gazebo/TurtleBot3..."
bash /ws/start_gazebo.sh > "${LOG_DIR}/gazebo.log" 2>&1 &
echo "gazebo:$!" >> "${PID_FILE}"

echo "[MONITOR] Waiting for Gazebo sensor topics..."
wait_for_topic /camera/image_raw 120
wait_for_topic /scan 120

echo "[AUTONOMY_LINE] Starting line detector/controller..."
ros2 launch line_follower line_follow.launch.py   use_sim_time:=true   use_hsv:=true   roi_start:=0.60   min_nonzero:=50   hsv_lower_h:=15   hsv_lower_s:=80   hsv_lower_v:=80   hsv_upper_h:=40   hsv_upper_s:=255   hsv_upper_v:=255   min_contour_area:=250.0   max_contour_jump:=120.0   contour_switch_confirm_frames:=3   ema_alpha:=0.25   k_p:=0.003   steer_sign:=-1.0   max_ang_z:=0.20   linear_x:=0.03   min_linear_x:=0.01   search_w:=0.10   search_linear_x:=0.01   error_deadband:=12.0   angular_alpha:=0.30   > "${LOG_DIR}/autonomy_line.log" 2>&1 &
echo "autonomy_line:$!" >> "${PID_FILE}"
sleep 2

echo "[SAFETY_OBSTACLE] Starting obstacle avoidance node..."
ros2 launch tb3_safety obstacle_avoid.launch.py   use_sim_time:=true   front_half_angle_deg:=25.0   side_sector_min_deg:=35.0   side_sector_max_deg:=100.0   avoid_distance:=0.35   clear_distance:=0.45   emergency_distance:=0.18   stop_time_sec:=0.30   turn_time_sec:=1.40   forward_time_sec:=0.90   turn_speed:=0.30   forward_speed:=0.03   > "${LOG_DIR}/safety_obstacle.log" 2>&1 &
echo "safety_obstacle:$!" >> "${PID_FILE}"
sleep 1

echo "[CMD_VEL_MUX] Starting command velocity mux..."
bash /ws/start_mux.sh > "${LOG_DIR}/cmd_vel_mux.log" 2>&1 &
echo "cmd_vel_mux:$!" >> "${PID_FILE}"
sleep 2

echo "[MONITOR] Final demo started. PIDs saved to ${PID_FILE}"

echo "[MONITOR] ROS nodes:"
ros2 node list || true

echo "[MONITOR] ROS topics:"
ros2 topic list || true

cat <<'MSG'
[MONITOR] Watch logs:
  tail -f /ws/logs/gazebo.log
  tail -f /ws/logs/autonomy_line.log
  tail -f /ws/logs/safety_obstacle.log
  tail -f /ws/logs/cmd_vel_mux.log

[MONITOR] Debug topics:
  ros2 topic echo /line_error
  ros2 topic echo /cmd_vel_raw
  ros2 topic echo /cmd_vel_obstacle
  ros2 topic echo /cmd_vel

[MONITOR] Stop demo:
  /ws/stop_final_demo.sh
MSG
