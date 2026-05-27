#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch tb3_safety obstacle_stop.launch.py use_sim_time:=true
