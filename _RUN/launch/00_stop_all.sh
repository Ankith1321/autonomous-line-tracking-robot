#!/usr/bin/env bash
set +e

echo "=== HOST: kill Gazebo GUI/server if any ==="
pkill -f gzclient || true
pkill -f gzserver || true
pkill -f gazebo || true

echo "=== CONTAINER: kill ROS launches + nodes + Gazebo ==="
docker exec -it ros2_humble bash -lc "
set +e
pkill -f 'ros2 launch' || true
pkill -f supervisor || true
pkill -f obstacle_stop || true
pkill -f line_controller || true
pkill -f line_detector || true
pkill -f gzclient || true
pkill -f gzserver || true
pkill -f gazebo || true
rm -f /tmp/gazebo* /tmp/gzserver* /tmp/gzclient* 2>/dev/null || true
sleep 1
echo '--- Remaining gz/ros processes (should be empty) ---'
ps aux | egrep 'gzserver|gzclient|gazebo|ros2 launch|obstacle_stop|line_controller|line_detector|supervisor' | grep -v egrep || true
"
