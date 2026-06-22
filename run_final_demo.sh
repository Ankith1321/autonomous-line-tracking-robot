#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

LOG_DIR=/ws/logs
PID_FILE=${LOG_DIR}/final_demo.pids
GAZEBO_START_SCRIPT=/ws/start_yellow_line_world.sh
RVIZ_CONFIG=/ws/rviz/yellow_line_demo.rviz
MARKER_SCRIPT=/ws/world_markers.py
DYNAMIC_SCRIPT=/ws/dynamic_obstacle.py

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

record_pid() {
  local label="$1"
  local pid="$2"
  echo "${label}:${pid}" >> "${PID_FILE}"
}

echo "[MONITOR] Stopping any previous final demo processes..."
/ws/stop_final_demo.sh

mkdir -p "${LOG_DIR}"
: > "${PID_FILE}"

echo "[GAZEBO] Starting Gazebo/TurtleBot3 Waffle Pi world..."
bash "${GAZEBO_START_SCRIPT}" > "${LOG_DIR}/gazebo.log" 2>&1 &
record_pid "gazebo" "$!"

echo "[MONITOR] Waiting for Gazebo sensor topics..."
wait_for_topic /camera/image_raw 120
wait_for_topic /scan 120

echo "[AUTONOMY_LINE] Starting line detector/controller..."
ros2 launch line_follower line_follow.launch.py   use_sim_time:=true   use_hsv:=true   roi_start:=0.60   min_nonzero:=50   hsv_lower_h:=15   hsv_lower_s:=80   hsv_lower_v:=80   hsv_upper_h:=40   hsv_upper_s:=255   hsv_upper_v:=255   min_contour_area:=250.0   max_contour_jump:=120.0   contour_switch_confirm_frames:=3   ema_alpha:=0.25   k_p:=0.0025   steer_sign:=-1.0   max_ang_z:=0.18   linear_x:=0.040   min_linear_x:=0.016   search_w:=0.07   search_linear_x:=0.012   error_deadband:=12.0   angular_alpha:=0.35   > "${LOG_DIR}/autonomy_line.log" 2>&1 &
record_pid "autonomy_line" "$!"
sleep 2

echo "[OBSTACLE_AVOID] Starting static obstacle avoidance..."
ros2 launch tb3_safety obstacle_avoid.launch.py   use_sim_time:=true   front_half_angle_deg:=30.0   side_sector_min_deg:=30.0   side_sector_max_deg:=120.0   avoid_distance:=1.00   clear_distance:=1.15   emergency_distance:=0.50   stop_time_sec:=0.0   turn_time_sec:=1.60   forward_time_sec:=1.35   min_turn_away_time_sec:=1.00   turn_back_time_sec:=1.05   turn_speed:=0.25   forward_speed:=0.035   rejoin_speed:=0.03   search_rejoin_speed:=0.025   search_rejoin_turn_speed:=0.08   rejoin_kp:=0.0025   rejoin_max_ang:=0.18   line_rejoin_error_thresh:=75.0   line_rejoin_confirm_frames:=5   max_rejoin_time_sec:=6.0   > "${LOG_DIR}/safety_obstacle.log" 2>&1 &
record_pid "obstacle_avoid" "$!"
sleep 1

echo "[CMD_VEL_MUX] Starting command velocity mux..."
bash /ws/start_mux.sh > "${LOG_DIR}/cmd_vel_mux.log" 2>&1 &
record_pid "cmd_vel_mux" "$!"
sleep 1

echo "[RVIZ] Starting RViz..."
rviz2 -d "${RVIZ_CONFIG}" > "${LOG_DIR}/rviz.log" 2>&1 &
record_pid "rviz" "$!"
sleep 1

echo "[MARKERS] Starting RViz world marker publisher..."
python3 "${MARKER_SCRIPT}" > "${LOG_DIR}/world_markers.log" 2>&1 &
record_pid "markers" "$!"
sleep 1

echo "[DYNAMIC_OBSTACLE] Starting moving obstacle controller..."
python3 "${DYNAMIC_SCRIPT}" > "${LOG_DIR}/dynamic_obstacle.log" 2>&1 &
record_pid "dynamic_obstacle" "$!"
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
  tail -f /ws/logs/rviz.log
  tail -f /ws/logs/world_markers.log
  tail -f /ws/logs/dynamic_obstacle.log

[MONITOR] Debug topics:
  ros2 topic list
  ros2 node list
  ros2 topic echo /line_error
  ros2 topic echo /cmd_vel

[MONITOR] Stop demo:
  /ws/stop_final_demo.sh
MSG
