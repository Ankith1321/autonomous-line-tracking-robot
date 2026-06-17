#!/usr/bin/env bash
set -e
docker exec -it ros2_humble bash -lc "
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch line_follower bringup.launch.py
"
