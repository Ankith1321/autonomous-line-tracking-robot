#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_in_sector(angle, start, end):
    angle = normalize_angle(angle)
    start = normalize_angle(start)
    end = normalize_angle(end)
    if start <= end:
        return start <= angle <= end
    return angle >= start or angle <= end


def sector_min(ranges, angle_min, angle_inc, start_angle, end_angle):
    minimum = float('inf')
    for idx, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = angle_min + idx * angle_inc
        if angle_in_sector(angle, start_angle, end_angle):
            minimum = min(minimum, value)
    return minimum


class ObstacleAvoid(Node):
    IDLE = 0
    TURN_AWAY = 1
    FORWARD_AROUND = 2
    REJOIN_LINE = 3

    STATE_NAMES = {
        IDLE: 'IDLE',
        TURN_AWAY: 'TURN_AWAY',
        FORWARD_AROUND: 'FORWARD_AROUND',
        REJOIN_LINE: 'REJOIN_LINE',
    }

    def __init__(self):
        super().__init__('obstacle_avoid')

        params = {
            'front_half_angle_deg': 28.0,
            'side_sector_min_deg': 30.0,
            'side_sector_max_deg': 120.0,
            'avoid_distance': 0.90,
            'clear_distance': 1.05,
            'forward_front_min': 0.75,
            'emergency_distance': 0.40,
            'side_clear_distance': 0.38,
            'side_emergency_distance': 0.25,
            'clear_confirm_frames': 8,
            'turn_time_sec': 1.80,
            'forward_time_sec': 3.50,
            'turn_speed': 0.45,
            'emergency_turn_speed': 0.55,
            'forward_speed': 0.10,
            'rejoin_speed': 0.09,
            'search_rejoin_speed': 0.08,
            'search_rejoin_turn_speed': 0.35,
            'rejoin_kp': 0.0025,
            'rejoin_max_ang': 0.35,
            'line_rejoin_error_thresh': 75.0,
            'line_rejoin_confirm_frames': 5,
            'publish_rate_hz': 20.0,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)

        self.front_half_angle = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        self.side_sector_min = math.radians(float(self.get_parameter('side_sector_min_deg').value))
        self.side_sector_max = math.radians(float(self.get_parameter('side_sector_max_deg').value))
        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.forward_front_min = float(self.get_parameter('forward_front_min').value)
        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.side_clear_distance = float(self.get_parameter('side_clear_distance').value)
        self.side_emergency_distance = float(self.get_parameter('side_emergency_distance').value)
        self.clear_confirm_frames = max(1, int(self.get_parameter('clear_confirm_frames').value))
        self.turn_time = max(0.1, float(self.get_parameter('turn_time_sec').value))
        self.forward_time = max(0.1, float(self.get_parameter('forward_time_sec').value))
        self.turn_speed = abs(float(self.get_parameter('turn_speed').value))
        self.emergency_turn_speed = abs(float(self.get_parameter('emergency_turn_speed').value))
        self.forward_speed = max(0.0, float(self.get_parameter('forward_speed').value))
        self.rejoin_speed = max(0.0, float(self.get_parameter('rejoin_speed').value))
        self.search_rejoin_speed = max(0.0, float(self.get_parameter('search_rejoin_speed').value))
        self.search_rejoin_turn_speed = abs(float(self.get_parameter('search_rejoin_turn_speed').value))
        self.rejoin_kp = float(self.get_parameter('rejoin_kp').value)
        self.rejoin_max_ang = abs(float(self.get_parameter('rejoin_max_ang').value))
        self.line_rejoin_error_thresh = float(self.get_parameter('line_rejoin_error_thresh').value)
        self.line_rejoin_confirm_frames = max(1, int(self.get_parameter('line_rejoin_confirm_frames').value))
        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.dt = 1.0 / publish_rate

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_obstacle', 10)
        self.state_pub = self.create_publisher(Int32, '/safety_state', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Float32, '/line_error', self.on_line_error, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.last_scan = None
        self.line_error = None
        self.state = self.IDLE
        self.state_started_at = self.get_clock().now()
        self.state_until = self.state_started_at
        self.turn_dir = 1.0
        self.clear_count = 0
        self.line_rejoin_count = 0
        self.last_motion_log_time = None
        self.turn_timeout_warned = False
        
        self.angle_turned = 0.0
        self.forward_phase = 1
        self.phase_until = self.state_started_at
        self.phase_min_until = self.state_started_at

        self.get_logger().info('Obstacle bypass node initialized.')

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_line_error(self, msg: Float32):
        if math.isfinite(msg.data) and msg.data != -1.0:
            self.line_error = float(msg.data)
        else:
            self.line_error = None

    def scan_distances(self):
        front = sector_min(
            self.last_scan.ranges,
            self.last_scan.angle_min,
            self.last_scan.angle_increment,
            -self.front_half_angle,
            self.front_half_angle,
        )
        left = sector_min(
            self.last_scan.ranges,
            self.last_scan.angle_min,
            self.last_scan.angle_increment,
            self.side_sector_min,
            self.side_sector_max,
        )
        right = sector_min(
            self.last_scan.ranges,
            self.last_scan.angle_min,
            self.last_scan.angle_increment,
            -self.side_sector_max,
            -self.side_sector_min,
        )
        return front, left, right

    def state_name(self):
        return self.STATE_NAMES.get(self.state, 'UNKNOWN')

    def direction_name(self):
        return 'LEFT' if self.turn_dir > 0.0 else 'RIGHT'

    def search_direction_name(self):
        return 'RIGHT' if self.turn_dir > 0.0 else 'LEFT'

    def choose_turn_direction(self, ranges, angle_min, angle_inc):
        # Steer towards the sector with greater clearance to avoid obstacle corner-clipping
        left_front = sector_min(ranges, angle_min, angle_inc, 0.0, math.radians(60.0))
        right_front = sector_min(ranges, angle_min, angle_inc, math.radians(-60.0), 0.0)

        if left_front >= right_front:
            self.turn_dir = 1.0
        else:
            self.turn_dir = -1.0

        self.get_logger().info(
            f'Bypass direction: {self.direction_name()} (L: {left_front:.2f}m, R: {right_front:.2f}m)'
        )

    def obstacle_side_clearance(self, left, right):
        return right if self.turn_dir > 0.0 else left

    def publish_cmd(self, linear_x, angular_z, front, left, right, obstacle_side):
        msg = Twist()
        msg.linear.x = max(0.0, float(linear_x))
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self.log_motion(front, left, right, obstacle_side, msg.linear.x, msg.angular.z)

    def log_motion(self, front, left, right, obstacle_side, linear_x, angular_z):
        now = self.get_clock().now()
        if self.last_motion_log_time is not None:
            dt = (now - self.last_motion_log_time).nanoseconds * 1e-9
            if dt < 0.20:
                return
        self.last_motion_log_time = now
        self.get_logger().info(
            f'state={self.state_name()} front_min={front:.2f} left_clearance={left:.2f} '
            f'right_clearance={right:.2f} obstacle_side_clearance={obstacle_side:.2f} '
            f'chosen_direction={self.direction_name()} line_error={self.line_error} '
            f'linear.x={linear_x:.3f} angular.z={angular_z:.3f}'
        )

    def enter_state(self, state, duration_sec=0.0, reason=''):
        now = self.get_clock().now()
        self.state = state
        self.state_started_at = now
        self.state_until = now + Duration(seconds=max(0.0, duration_sec))
        
        self.clear_count = 0
        self.line_rejoin_count = 0
        self.turn_timeout_warned = False
        self.line_error = None
        self.last_motion_log_time = None
        
        if state == self.TURN_AWAY:
            self.angle_turned = 0.0

        if state == self.FORWARD_AROUND:
            self.forward_phase = 1
            self.phase_until = now + Duration(seconds=4.00)
            self.get_logger().info('FSM -> FORWARD_AROUND (Phase 1)')
            
        log_msg = f'FSM -> {self.state_name()} (Dir: {self.direction_name()})'
        if reason:
            log_msg += f' | Reason: {reason}'
        self.get_logger().info(log_msg)
        self.state_pub.publish(Int32(data=int(self.state)))

    def line_centered(self):
        return self.line_error is not None and abs(self.line_error) < self.line_rejoin_error_thresh

    def on_timer(self):
        if self.last_scan is None:
            return

        self.state_pub.publish(Int32(data=int(self.state)))

        now = self.get_clock().now()
        front, left, right = self.scan_distances()

        # Use narrow aperture during detour to prevent false triggers from side walls
        path_front = sector_min(
            self.last_scan.ranges,
            self.last_scan.angle_min,
            self.last_scan.angle_increment,
            -math.radians(12.0),
            math.radians(12.0),
        )

        if self.state == self.IDLE:
            if front < self.avoid_distance:
                self.choose_turn_direction(self.last_scan.ranges, self.last_scan.angle_min, self.last_scan.angle_increment)
                obstacle_side = self.obstacle_side_clearance(left, right)
                self.get_logger().warn(
                    f'OBSTACLE_DETECTED front_min={front:.2f} left_clearance={left:.2f} '
                    f'right_clearance={right:.2f} obstacle_side_clearance={obstacle_side:.2f} '
                    f'chosen_direction={self.direction_name()}'
                )
                self.enter_state(self.TURN_AWAY, self.turn_time, 'initial_turn')
            return

        obstacle_side = self.obstacle_side_clearance(left, right)
        emergency = front < self.emergency_distance or obstacle_side < self.side_emergency_distance

        if self.state == self.TURN_AWAY:
            angular = self.emergency_turn_speed if emergency else self.turn_speed
            self.publish_cmd(0.0, self.turn_dir * angular, front, left, right, obstacle_side)
            self.angle_turned += angular * self.dt

            if front > self.clear_distance and obstacle_side > self.side_emergency_distance and self.angle_turned >= math.radians(45.0):
                self.clear_count += 1
            else:
                self.clear_count = 0

            if self.clear_count >= self.clear_confirm_frames:
                self.enter_state(self.FORWARD_AROUND, self.forward_time, 'front_and_side_clear_confirmed')
                return

            if now >= self.state_until:
                if front > self.clear_distance and self.angle_turned >= math.radians(45.0):
                    self.enter_state(self.FORWARD_AROUND, self.forward_time, 'turn_timeout_front_clear_fallback')
                    return
                elif not self.turn_timeout_warned:
                    self.turn_timeout_warned = True
                    self.get_logger().warn(
                        f'TURN_AWAY timeout reached but front still blocked (front={front:.2f}) or min turn angle not reached (angle={self.angle_turned*180.0/math.pi:.1f} deg). Continuing turn.'
                    )
            return

        if self.state == self.FORWARD_AROUND:
            if emergency:
                self.publish_cmd(0.0, self.turn_dir * self.emergency_turn_speed, front, left, right, obstacle_side)
                self.enter_state(self.TURN_AWAY, self.turn_time, 'emergency_during_forward')
                return

            if self.forward_phase == 1:
                # Phase 1: Shift out (drive straight at angle)
                if path_front <= self.forward_front_min:
                    self.publish_cmd(0.0, self.turn_dir * self.turn_speed, front, left, right, obstacle_side)
                    self.enter_state(self.TURN_AWAY, self.turn_time, 'front_blocked_during_shift_out')
                    return
                
                self.publish_cmd(self.forward_speed, 0.0, front, left, right, obstacle_side)
                if now >= self.phase_until:
                    self.forward_phase = 2
                    deg = self.angle_turned * 180.0 / math.pi
                    self.get_logger().info(f'FORWARD_AROUND: transitioning to Phase 2 (turn parallel, angle to correct = {deg:.1f} deg)')
                return

            elif self.forward_phase == 2:
                # Phase 2: Turn parallel (turn opposite to turn_dir)
                angular_z = -self.turn_dir * self.turn_speed
                self.publish_cmd(0.0, angular_z, front, left, right, obstacle_side)
                self.angle_turned -= self.turn_speed * self.dt
                if self.angle_turned <= 0.0:
                    self.forward_phase = 3
                    self.phase_until = now + Duration(seconds=9.00)
                    self.phase_min_until = now + Duration(seconds=3.00)
                    self.get_logger().info('FORWARD_AROUND: transitioning to Phase 3 (drive past)')
                return

            elif self.forward_phase == 3:
                # Phase 3: Drive parallel to the line to clear the obstacle
                if path_front <= self.forward_front_min:
                    self.publish_cmd(0.0, self.turn_dir * self.turn_speed, front, left, right, obstacle_side)
                    self.enter_state(self.TURN_AWAY, self.turn_time, 'front_blocked_during_drive_past')
                    return
                
                self.publish_cmd(self.forward_speed, 0.0, front, left, right, obstacle_side)
                
                if now >= self.phase_min_until and obstacle_side > 0.85:
                    self.enter_state(self.REJOIN_LINE, reason='obstacle_side_cleared_dynamic')
                    return
                
                if now >= self.phase_until:
                    self.enter_state(self.REJOIN_LINE, reason='forward_timeout')
                return

        if self.state == self.REJOIN_LINE:
            if emergency:
                self.publish_cmd(0.0, self.turn_dir * self.emergency_turn_speed, front, left, right, obstacle_side)
                self.enter_state(self.TURN_AWAY, self.turn_time, 'emergency_during_rejoin')
                return

            # If line is re-acquired, check if obstacle is physically cleared before transitioning to IDLE
            if self.line_error is not None:
                self.line_rejoin_count += 1
                if self.line_rejoin_count >= self.line_rejoin_confirm_frames:
                    if front > self.avoid_distance and obstacle_side > self.side_clear_distance:
                        self.publish_cmd(0.0, 0.0, front, left, right, obstacle_side)
                        self.enter_state(self.IDLE, reason='line_reacquired_and_obstacle_cleared')
                        return
                    else:
                        # Steer along the line using rejoin parameters but do not hand back control to autonomy yet
                        linear = self.rejoin_speed
                        angular = max(-self.rejoin_max_ang, min(self.rejoin_max_ang, -self.rejoin_kp * self.line_error))
                        self.publish_cmd(linear, angular, front, left, right, obstacle_side)
                        return
                else:
                    # Keep searching/sweeping towards the expected line direction while we confirm the line
                    linear = self.search_rejoin_speed
                    angular = -self.turn_dir * self.search_rejoin_turn_speed
                    self.publish_cmd(linear, angular, front, left, right, obstacle_side)
                    return
            else:
                self.line_rejoin_count = 0
                # Keep searching/sweeping towards the expected line direction
                linear = self.search_rejoin_speed
                angular = -self.turn_dir * self.search_rejoin_turn_speed
                self.publish_cmd(linear, angular, front, left, right, obstacle_side)
                return


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
