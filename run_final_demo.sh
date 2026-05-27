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
ros2 launch line_follower line_follow.launch.py \
  use_sim_time:=true \
  roi_start:=0.60 \
  fixed_thresh:=100 \
  min_nonzero:=50 \
  k_p:=0.003 \
  max_ang_z:=0.10 \
  linear_x:=0.005 \
  search_w:=0.05 \
  search_linear_x:=0.003 \
  > "${LOG_DIR}/autonomy_line.log" 2>&1 &
echo "autonomy_line:$!" >> "${PID_FILE}"
sleep 2

echo "[SAFETY_OBSTACLE] Starting obstacle stop safety node..."
ros2 launch tb3_safety obstacle_stop.launch.py use_sim_time:=true stop_distance:=0.20 > "${LOG_DIR}/safety_obstacle.log" 2>&1 &
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
