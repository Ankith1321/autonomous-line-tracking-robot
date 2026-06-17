#!/usr/bin/env bash
set -e
docker exec -it ros2_humble bash -lc "
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
"
