# TurtleBot3 Waffle Pi Final Demo Handoff

Workspace: `/ws`  
ROS version: ROS 2 Humble  
Robot model: TurtleBot3 Waffle Pi only  
Demo goal: camera-based yellow line following with LiDAR-based static obstacle avoidance and line rejoin.

## Main Demo Goal

The demo should show this clear behavior:

1. TurtleBot3 Waffle Pi starts on a yellow line.
2. Camera detects the yellow line using HSV.
3. Line controller follows the yellow line using `/line_error`.
4. LiDAR detects a static obstacle near the line.
5. Obstacle avoidance takes priority over line following.
6. Robot briefly stops/turns away before collision.
7. Robot moves around the obstacle.
8. Robot detects the yellow line again.
9. Robot returns control to normal line following.

The priority is reliability and clarity for a proposal/demo, not a hard maze or complex world.

## Current Architecture

Do not redesign this unless absolutely necessary.

Topics:

- Gazebo camera publishes: `/camera/image_raw`
- Gazebo LiDAR publishes: `/scan`
- Line detector publishes: `/line_error`
- Line controller publishes: `/cmd_vel_raw`
- Obstacle avoidance publishes: `/cmd_vel_obstacle`
- Command mux publishes final command: `/cmd_vel`
- TurtleBot3 listens to: `/cmd_vel`

Command priority:

```text
/cmd_vel_obstacle  -> highest priority when recent
/cmd_vel_raw       -> normal line following
/cmd_vel           -> final robot velocity
```

## Main Files

### `/ws/worlds/yellow_line_obstacle_demo.world`

Active Gazebo world.

Current design:

- Open grey floor.
- No boundary walls.
- No maze.
- No dynamic obstacle.
- One straight yellow line.
- One static box obstacle near the line.

Current important world values:

```xml
<model name="yellow_line_open_straight">
  <pose>0.0 0.0 0.012 0 0 0</pose>
  <size>7.2 0.16 0.014</size>
</model>

<model name="obstacle_box_static_offset">
  <pose>-0.20 0.42 0.18 0 0 0.10</pose>
  <size>0.30 0.30 0.36</size>
</model>
```

Reason for current obstacle placement:

- It is close enough to be detected by LiDAR.
- It is visible as a real obstacle in Gazebo/RViz.
- It is not so close that the robot gets trapped in repeated rejoin/avoidance loops.

### `/ws/start_yellow_line_world.sh`

Starts Gazebo and spawns TurtleBot3 Waffle Pi.

Important values:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ROBOT_START_X=-3.0
ROBOT_START_Y=0.0
ROBOT_START_YAW=0.0
```

Robot starts at the beginning of the straight yellow line facing forward.

### `/ws/run_final_demo.sh`

Starts the full final demo sequentially:

1. Stops previous processes.
2. Starts Gazebo world.
3. Waits for `/camera/image_raw`.
4. Waits for `/scan`.
5. Starts line detector/controller.
6. Starts obstacle avoidance.
7. Starts command mux.
8. Starts RViz.
9. Starts world marker publisher.

Current line follower parameters:

```bash
use_sim_time:=true
use_hsv:=true
roi_start:=0.60
min_nonzero:=50
hsv_lower_h:=15
hsv_lower_s:=80
hsv_lower_v:=80
hsv_upper_h:=40
hsv_upper_s:=255
hsv_upper_v:=255
min_contour_area:=250.0
max_contour_jump:=120.0
contour_switch_confirm_frames:=3
ema_alpha:=0.25
max_fill_ratio:=0.70
k_p:=0.0025
steer_sign:=-1.0
max_ang_z:=0.18
linear_x:=0.040
min_linear_x:=0.018
search_w:=0.07
search_linear_x:=0.012
error_deadband:=12.0
angular_alpha:=0.35
```

Current obstacle avoidance parameters:

```bash
front_half_angle_deg:=25.0
side_sector_min_deg:=30.0
side_sector_max_deg:=120.0
avoid_distance:=0.95
clear_distance:=1.05
emergency_distance:=0.50
side_clearance_min:=0.35
stop_time_sec:=0.10
turn_time_sec:=1.25
forward_time_sec:=1.80
min_turn_away_time_sec:=0.75
turn_back_time_sec:=0.95
turn_speed:=0.25
forward_speed:=0.040
rejoin_speed:=0.035
search_rejoin_speed:=0.025
search_rejoin_turn_speed:=0.08
rejoin_kp:=0.0025
rejoin_block_distance:=0.50
rejoin_release_distance:=0.58
rejoin_block_confirm_frames:=3
rejoin_block_turn_sec:=0.80
rejoin_max_ang:=0.18
line_rejoin_error_thresh:=35.0
line_rejoin_confirm_frames:=8
max_rejoin_time_sec:=6.0
```

### `/ws/src/line_follower/line_follower/line_detector.py`

Detects yellow line from `/camera/image_raw`.

Important behavior:

- Uses HSV thresholding.
- Publishes `/line_error`.
- `line_error = tracked_cx - image_center`.
- Publishes `lost_sentinel = -1.0` when line is lost.
- Publishes `/line_mask` and `/line_mask_nonzero` for debugging.

Current issue solved:

- Yellow line used to be too thick, giving high fill ratio near the rejection limit.
- Line width was reduced from `0.20` to `0.16`.
- `max_fill_ratio` was increased to `0.70`.

### `/ws/src/line_follower/line_follower/controller.py`

Converts `/line_error` into `/cmd_vel_raw`.

Important behavior:

```python
angular_z = steer_sign * k_p * error
```

Current sign:

```bash
steer_sign:=-1.0
```

This appeared correct from the logs: when line is centered, angular output is near zero.

### `/ws/src/tb3_safety/tb3_safety/obstacle_avoid.py`

Static obstacle avoidance state machine.

Current state machine:

```text
NORMAL
STOP
TURN_AWAY
FORWARD_AROUND
TURN_BACK
REJOIN_LINE
RELEASE_TO_LINE_FOLLOWER
```

Important safety behavior:

- In `NORMAL`, it does not publish obstacle commands unless obstacle is detected.
- If `front_min < avoid_distance`, obstacle avoidance takes control.
- If `front_min < emergency_distance`, robot does not move forward.
- No negative linear velocity is published.
- Obstacle commands are published to `/cmd_vel_obstacle`.

Important rejoin behavior added:

```text
If line is at far camera edge:
  abs(line_error) > 180 -> turn in place, no forward motion

If line is off-center but visible:
  abs(line_error) > 100 -> slow arc back to line

If line is reasonably visible:
  normal guided rejoin
```

Reason:

Earlier, when the line was barely visible at the far edge, the robot kept driving forward while steering at max angle. That made it move beside the line or get trapped near the obstacle. The new behavior prevents forward motion when the line is too far from center.

Important rejoin release logic:

- Release only after line is centered for enough frames.
- Uses `line_rejoin_error_thresh:=35.0`.
- Uses `line_rejoin_confirm_frames:=8`.
- Uses `rejoin_release_distance:=0.58` instead of requiring the full `clear_distance` during rejoin.

Reason:

The robot was previously centered on the line again but still seeing the obstacle around `0.59-0.67m`, causing repeated `LINE_CENTERED_BUT_FRONT_BLOCKED` loops. `rejoin_release_distance` allows release when the robot is safely past/near the obstacle without demanding an unrealistic front clearance.

### `/ws/src/tb3_safety/launch/obstacle_avoid.launch.py`

Launch file for obstacle avoidance node.

Important addition:

```python
DeclareLaunchArgument('rejoin_release_distance', default_value='0.58')
```

And parameter mapping:

```python
'rejoin_release_distance': LaunchConfiguration('rejoin_release_distance')
```

### `/ws/cmd_vel_mux.py`

Muxes raw line following and obstacle avoidance commands.

Behavior:

- `/cmd_vel_obstacle` has priority only if recent.
- Obstacle timeout: `0.35s`.
- If no recent obstacle command, mux uses `/cmd_vel_raw`.
- If neither command is recent, publishes zero.

This file was checked and was working correctly.

### `/ws/world_markers.py`

Publishes RViz markers on `/demo_world_markers`.

Important behavior:

- Parses `/ws/worlds/yellow_line_obstacle_demo.world`.
- Publishes markers for models beginning with:
  - `yellow_line_`
  - `wall_`
  - `obstacle_`
- Uses transient local QoS.
- Frame: `odom`.

This makes RViz mirror the Gazebo world instead of maintaining separate hardcoded coordinates.

### `/ws/collect_demo_debug.sh`

Collects debug output into:

```bash
/ws/logs/demo_debug_report.txt
```

Important behavior:

```bash
rm -f "$OUT"
```

This deletes the old debug report before creating a new one.

Use it while the demo is still running.

## Problems Encountered And Solutions

### Problem 1: World was too complex

Symptoms:

- Robot triggered obstacle avoidance because of walls.
- Startup rotation occurred.
- Rejoin behavior was hard to debug.

Cause:

- Closed arena/walls and complex routes caused LiDAR false triggers.

Solution:

- Removed boundary walls.
- Removed maze/branches.
- Used open floor with one straight yellow line.

Current status:

- Startup line following is stable.

### Problem 2: Yellow line was too thick

Symptoms:

- Line detector showed `fill` near `0.57`.
- It sometimes exceeded `max_fill_ratio` and published lost sentinel.

Cause:

- Yellow line was occupying too much ROI.

Solution:

- Reduced line width to `0.16`.
- Increased `max_fill_ratio` to `0.70`.

Current expected behavior:

- Detector fill is around `0.45`, which is stable.

### Problem 3: Dynamic obstacle did not move

Symptoms:

- Dynamic obstacle log repeatedly said waiting for `/set_entity_state`.
- Dynamic obstacle never moved.

Cause:

- Gazebo state service/plugin was not reliably available in this setup.

Solution:

- Removed dynamic obstacle entirely for version 1.
- Deleted `dynamic_obstacle.py`.
- Removed dynamic obstacle startup/stop references.

Current status:

- Static obstacle only.

### Problem 4: Obstacle too far outside line

Symptoms:

- Robot appeared to just follow the line without meaningful avoidance.

Cause:

- Obstacle was placed too far from the robot path.

Solution attempted:

- Moved obstacle closer.

Outcome:

- Too close became worse, so final placement is a compromise.

Current placement:

```xml
<pose>-0.20 0.42 0.18 0 0 0.10</pose>
<size>0.30 0.30 0.36</size>
```

### Problem 5: Medium obstacle was too hard

Symptoms:

- Robot got trapped in repeated rejoin/blocked loops.
- Logs showed line error around `-200` to `-250`.
- Robot saw front obstacle around `0.62-0.73m` repeatedly.

Cause:

- Obstacle was too close to the yellow line.
- Rejoin logic tried to move forward even when line was only visible at the far camera edge.

Solution:

- Moved obstacle to a safer offset.
- Added far-edge rejoin logic:
  - turn in place if `abs(line_error) > 180`.
  - slow arc if `abs(line_error) > 100`.
- Added `rejoin_release_distance`.

Current status:

- This should reduce trapping and make the demo more reliable.

### Problem 6: Debug report looked stale

Symptoms:

- User saw old debug report timestamp after running again.

Cause:

- VS Code tab may show cached old contents.
- Debug script may not have been run while demo was active.

Solution:

- Confirmed `collect_demo_debug.sh` deletes old report before writing.
- Run debug collection while demo is still running.
- Reopen/refresh VS Code file tab or use `head` command.

Correct check:

```bash
head -3 /ws/logs/demo_debug_report.txt
ls -l /ws/logs/demo_debug_report.txt
```

## Current Recommended Run Commands

Start full demo:

```bash
/ws/stop_final_demo.sh
/ws/run_final_demo.sh
```

Collect debug while demo is still running:

```bash
/ws/collect_demo_debug.sh
cat /ws/logs/demo_debug_report.txt
```

Check report freshness:

```bash
head -3 /ws/logs/demo_debug_report.txt
ls -l /ws/logs/demo_debug_report.txt
```

Watch logs live:

```bash
tail -f /ws/logs/autonomy_line.log
```

```bash
tail -f /ws/logs/safety_obstacle.log
```

```bash
tail -f /ws/logs/cmd_vel_mux.log
```

Manual topic checks:

```bash
ros2 topic echo /line_error
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel_obstacle
ros2 topic echo /cmd_vel
```

Manual node/topic checks:

```bash
ros2 node list
ros2 topic list
```

## Build And Validation Commands

After changing `tb3_safety`:

```bash
cd /ws
colcon build --packages-select tb3_safety --symlink-install
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

Validation commands:

```bash
bash -n /ws/start_yellow_line_world.sh
bash -n /ws/run_final_demo.sh
bash -n /ws/stop_final_demo.sh
bash -n /ws/collect_demo_debug.sh
python3 -m py_compile /ws/world_markers.py /ws/cmd_vel_mux.py /ws/src/tb3_safety/tb3_safety/obstacle_avoid.py /ws/src/tb3_safety/launch/obstacle_avoid.launch.py
xmllint --noout /ws/worlds/yellow_line_obstacle_demo.world
gz sdf -k /ws/worlds/yellow_line_obstacle_demo.world
```

## How To Interpret Debug Report

### Good signs

Line detector:

```text
fill around 0.40 to 0.50
last_error near 0 on straight line
```

Mux:

```text
mux source=RAW
mux source=OBSTACLE
mux source=RAW
```

Obstacle avoidance:

```text
OBSTACLE_DETECTED
TURN_AWAY
FORWARD_AROUND
TURN_BACK
SEARCH_LINE_FORWARD
LINE_CENTERED
RELEASE_TO_LINE_FOLLOWER
```

### Bad signs

Line far at camera edge:

```text
line_error around +/-200
```

Obstacle loop:

```text
REJOIN_FRONT_BLOCKED repeated many times
EMERGENCY_TURN repeated many times
```

Mux stuck in obstacle:

```text
mux source=OBSTACLE
```

and never returns to RAW.

## Current Working Percentage Estimate

This is an engineering estimate, not a guarantee.

- Easy baseline with obstacle far outside: about 85-90% stable, but weak demo value.
- Too-hard medium obstacle: about 40-60% stable, not demo safe.
- Current structured compromise: expected about 75-85% stable.

The current version is intended to prioritize a reliable final demo over maximum difficulty.

## Recommended Next Steps

1. Test current structured setup exactly as-is.
2. Do not move the obstacle closer until this version passes reliably.
3. If it still loops, collect debug while demo is running.
4. Tune only one thing at a time:
   - obstacle `y` position, or
   - `forward_time_sec`, or
   - `rejoin_release_distance`.
5. Avoid reintroducing walls, 8-shape, dynamic obstacle, or multiple obstacles until the straight-line static obstacle demo is stable.

## Important Rule

For version 1, keep it simple:

```text
one open world
one yellow line
one static obstacle
no walls
no dynamic obstacle
no maze
no 8-shape
```

Once this is reliable, complexity can be added slowly.
