#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

WORLD_FILE=/ws/worlds/yellow_line_obstacle_demo.world
ROBOT_MODEL=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf
ROBOT_START_X=0.0
ROBOT_START_Y=0.0
ROBOT_START_YAW=0.0

cleanup() {
  kill "${GZSERVER_PID:-}" "${GZCLIENT_PID:-}" "${RSP_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch gazebo_ros gzserver.launch.py world:="${WORLD_FILE}" &
GZSERVER_PID=$!

ros2 launch gazebo_ros gzclient.launch.py &
GZCLIENT_PID=$!

sleep 5

ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true &
RSP_PID=$!

ros2 run gazebo_ros spawn_entity.py \
  -entity turtlebot3_waffle_pi \
  -file "${ROBOT_MODEL}" \
  -x "${ROBOT_START_X}" \
  -y "${ROBOT_START_Y}" \
  -z 0.01 \
  -Y "${ROBOT_START_YAW}"

wait "${GZSERVER_PID}" "${GZCLIENT_PID}" "${RSP_PID}"
