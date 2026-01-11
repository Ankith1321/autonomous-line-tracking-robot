#!/bin/bash
set -e

# Ensure container is running
docker start ros2_humble >/dev/null 2>&1 || true

# Ensure X permits root-in-container (autostart should do this; fallback kept)
xhost +SI:localuser:root >/dev/null 2>&1 || true

# Start Gazebo GUI in background (force DISPLAY and Qt safe mode)
docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  ros2_humble \
  bash -lc "nohup gzclient >/tmp/gzclient.log 2>&1 & disown"
