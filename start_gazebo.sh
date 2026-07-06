#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

exec /ws/start_yellow_line_world.sh
