# ROS 2 Humble TurtleBot3 Waffle Pi Final Demo

## Goal

This workspace runs a TurtleBot3 Waffle Pi in Gazebo for camera-based yellow line following with static obstacle avoidance and return to line following.

Active topic flow:

```text
/camera/image_raw -> line_detector -> /line_error -> line_controller -> /cmd_vel_raw
/scan -> obstacle_avoid -> /cmd_vel_obstacle
/cmd_vel_raw + /cmd_vel_obstacle -> cmd_vel_mux.py -> /cmd_vel
```

## Active Gazebo World

The single active final demo world is:

```bash
/ws/worlds/yellow_line_obstacle_demo.world
```

Use these launch commands:

```bash
/ws/start_yellow_line_world.sh
/ws/start_gazebo.sh
/ws/run_final_demo.sh
/ws/stop_final_demo.sh
```

`/ws/start_gazebo.sh` is a wrapper around `/ws/start_yellow_line_world.sh`, so both use the same world.

## Final Demo Runtime

`/ws/run_final_demo.sh` starts:

1. Gazebo with the single simple yellow-line demo world
2. Line detector and line controller
3. `obstacle_avoid` for static obstacle avoidance
4. `cmd_vel_mux.py`
5. RViz using `/ws/rviz/yellow_line_demo.rviz`
6. `world_markers.py` for `/demo_world_markers`

## Active Safety Node

The active obstacle node is:

```bash
/ws/src/tb3_safety/tb3_safety/obstacle_avoid.py
```

The final demo does not use `obstacle_stop`.

## Gazebo-Only Command

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
/ws/start_yellow_line_world.sh
```

## RViz Command

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
rviz2 -d /ws/rviz/yellow_line_demo.rviz
```

## Debug Commands

```bash
ros2 topic list
ros2 node list
ros2 topic echo /line_error
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel_obstacle
ros2 topic echo /cmd_vel
tail -f /ws/logs/autonomy_line.log
tail -f /ws/logs/safety_obstacle.log
tail -f /ws/logs/cmd_vel_mux.log
```
