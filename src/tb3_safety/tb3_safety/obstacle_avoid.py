#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


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

        self.declare_parameter('front_half_angle_deg', 28.0)
        self.declare_parameter('side_sector_min_deg', 30.0)
        self.declare_parameter('side_sector_max_deg', 120.0)
        self.declare_parameter('avoid_distance', 0.95)
        self.declare_parameter('clear_distance', 1.10)
        self.declare_parameter('forward_front_min', 0.85)
        self.declare_parameter('emergency_distance', 0.45)
        self.declare_parameter('side_clear_distance', 0.45)
        self.declare_parameter('side_emergency_distance', 0.28)
        self.declare_parameter('clear_confirm_frames', 3)
        self.declare_parameter('turn_time_sec', 1.20)
        self.declare_parameter('forward_time_sec', 1.50)
        self.declare_parameter('turn_speed', 0.32)
        self.declare_parameter('emergency_turn_speed', 0.42)
        self.declare_parameter('forward_speed', 0.035)
        self.declare_parameter('rejoin_speed', 0.030)
        self.declare_parameter('search_rejoin_speed', 0.020)
        self.declare_parameter('search_rejoin_turn_speed', 0.07)
        self.declare_parameter('rejoin_kp', 0.0025)
        self.declare_parameter('rejoin_max_ang', 0.18)
        self.declare_parameter('line_rejoin_error_thresh', 75.0)
        self.declare_parameter('line_rejoin_confirm_frames', 5)
        self.declare_parameter('publish_rate_hz', 20.0)

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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_obstacle', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Float32, '/line_error', self.on_line_error, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

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

        self.get_logger().info(
            'ObstacleAvoid simplified with side clearance: '
            f'avoid={self.avoid_distance:.2f} clear={self.clear_distance:.2f} '
            f'forward_front_min={self.forward_front_min:.2f} side_clear={self.side_clear_distance:.2f}'
        )

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

    def choose_turn_direction(self, left, right):
        self.turn_dir = 1.0 if left >= right else -1.0

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
        if state == self.TURN_AWAY:
            self.clear_count = 0
            self.turn_timeout_warned = False
        if state == self.REJOIN_LINE:
            self.line_rejoin_count = 0
        if state == self.IDLE:
            self.line_rejoin_count = 0
            self.clear_count = 0
            self.turn_timeout_warned = False
        msg = f'state_change={self.state_name()} chosen_direction={self.direction_name()}'
        if reason:
            msg += f' transition_reason={reason}'
        self.get_logger().info(msg)

    def line_centered(self):
        return self.line_error is not None and abs(self.line_error) < self.line_rejoin_error_thresh

    def on_timer(self):
        if self.last_scan is None:
            return

        now = self.get_clock().now()
        front, left, right = self.scan_distances()

        if self.state == self.IDLE:
            if front < self.avoid_distance:
                self.choose_turn_direction(left, right)
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
            if front > self.clear_distance and obstacle_side > self.side_clear_distance:
                self.clear_count += 1
            else:
                self.clear_count = 0

            if self.clear_count >= self.clear_confirm_frames:
                self.enter_state(self.FORWARD_AROUND, self.forward_time, 'front_and_side_clear_confirmed')
                return

            if now >= self.state_until and not self.turn_timeout_warned:
                self.turn_timeout_warned = True
                self.get_logger().warn(
                    f'TURN_AWAY timeout diagnostic: front_min={front:.2f} '
                    f'obstacle_side_clearance={obstacle_side:.2f}; continuing turn'
                )
            return

        if self.state == self.FORWARD_AROUND:
            if emergency:
                self.publish_cmd(0.0, self.turn_dir * self.emergency_turn_speed, front, left, right, obstacle_side)
                self.enter_state(self.TURN_AWAY, self.turn_time, 'emergency_during_forward')
                return

            if front <= self.forward_front_min or obstacle_side <= self.side_clear_distance:
                self.publish_cmd(0.0, self.turn_dir * self.turn_speed, front, left, right, obstacle_side)
                self.enter_state(self.TURN_AWAY, self.turn_time, 'front_or_side_not_clear_for_forward')
                return

            self.publish_cmd(self.forward_speed, 0.0, front, left, right, obstacle_side)
            if now >= self.state_until:
                self.enter_state(self.REJOIN_LINE, reason='forward_complete')
            return

        if self.state == self.REJOIN_LINE:
            if emergency:
                self.publish_cmd(0.0, self.turn_dir * self.emergency_turn_speed, front, left, right, obstacle_side)
                self.enter_state(self.TURN_AWAY, self.turn_time, 'emergency_during_rejoin')
                return

            if self.line_error is not None:
                linear = self.rejoin_speed
                angular = max(-self.rejoin_max_ang, min(self.rejoin_max_ang, -self.rejoin_kp * self.line_error))
            else:
                linear = self.search_rejoin_speed
                angular = -self.turn_dir * self.search_rejoin_turn_speed

            self.publish_cmd(linear, angular, front, left, right, obstacle_side)

            if self.line_centered():
                self.line_rejoin_count += 1
                if self.line_rejoin_count >= self.line_rejoin_confirm_frames:
                    self.enter_state(self.IDLE, reason='line_centered_confirmed')
                    return
            else:
                self.line_rejoin_count = 0


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
