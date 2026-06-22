#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
python3 /ws/cmd_vel_mux.py
