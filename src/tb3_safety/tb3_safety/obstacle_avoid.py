#!/usr/bin/env python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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


def sector_min(ranges, angle_min, angle_inc, a0, a1):
    if not ranges or angle_inc == 0.0:
        return float('inf')

    minimum = float('inf')
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue

        angle = angle_min + index * angle_inc
        if angle_in_sector(angle, a0, a1):
            minimum = min(minimum, value)
    return minimum


class ObstacleAvoid(Node):
    IDLE = 0
    STOP = 1
    TURN_AWAY = 2
    FORWARD_AROUND = 3
    TURN_BACK = 4
    SEARCH_LINE_FORWARD = 5

    def __init__(self):
        super().__init__('obstacle_avoid')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_obstacle')
        self.declare_parameter('line_error_topic', '/line_error')
        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('side_sector_min_deg', 30.0)
        self.declare_parameter('side_sector_max_deg', 120.0)
        self.declare_parameter('avoid_distance', 0.95)
        self.declare_parameter('clear_distance', 1.05)
        self.declare_parameter('emergency_distance', 0.50)
        self.declare_parameter('side_clearance_min', 0.35)
        self.declare_parameter('stop_time_sec', 0.10)
        self.declare_parameter('turn_time_sec', 1.50)
        self.declare_parameter('forward_time_sec', 6.00)
        self.declare_parameter('forward_min_distance', 0.32)
        self.declare_parameter('min_turn_away_time_sec', 0.60)
        self.declare_parameter('turn_back_time_sec', 3.00)
        self.declare_parameter('turn_back_yaw_tolerance_deg', 5.0)
        self.declare_parameter('turn_speed', 0.35)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('rejoin_speed', 0.12)
        self.declare_parameter('search_rejoin_speed', 0.10)
        self.declare_parameter('search_rejoin_turn_speed', 0.15)
        self.declare_parameter('rejoin_kp', 0.0025)
        self.declare_parameter('rejoin_block_distance', 0.50)
        self.declare_parameter('rejoin_release_distance', 0.58)
        self.declare_parameter('rejoin_block_confirm_frames', 3)
        self.declare_parameter('rejoin_block_turn_sec', 0.60)
        self.declare_parameter('rejoin_max_ang', 0.18)
        self.declare_parameter('line_rejoin_error_thresh', 35.0)
        self.declare_parameter('line_rejoin_confirm_frames', 8)
        self.declare_parameter('max_rejoin_time_sec', 5.0)
        self.declare_parameter('line_lost_sentinel', -1.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.line_error_topic = self.get_parameter('line_error_topic').value
        self.front_half_angle = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        self.side_sector_min = math.radians(float(self.get_parameter('side_sector_min_deg').value))
        self.side_sector_max = math.radians(float(self.get_parameter('side_sector_max_deg').value))
        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.side_clearance_min = float(self.get_parameter('side_clearance_min').value)
        self.stop_time = float(self.get_parameter('stop_time_sec').value)
        self.turn_time = float(self.get_parameter('turn_time_sec').value)
        self.forward_time = float(self.get_parameter('forward_time_sec').value)
        self.forward_min_distance = float(self.get_parameter('forward_min_distance').value)
        self.min_turn_away_time = float(self.get_parameter('min_turn_away_time_sec').value)
        self.turn_back_time = float(self.get_parameter('turn_back_time_sec').value)
        self.turn_back_yaw_tolerance = math.radians(
            float(self.get_parameter('turn_back_yaw_tolerance_deg').value)
        )
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.rejoin_speed = float(self.get_parameter('rejoin_speed').value)
        self.search_rejoin_speed = float(self.get_parameter('search_rejoin_speed').value)
        self.search_rejoin_turn_speed = float(self.get_parameter('search_rejoin_turn_speed').value)
        self.rejoin_kp = float(self.get_parameter('rejoin_kp').value)
        self.rejoin_block_distance = float(self.get_parameter('rejoin_block_distance').value)
        self.rejoin_release_distance = float(self.get_parameter('rejoin_release_distance').value)
        self.rejoin_block_confirm_frames = max(1, int(self.get_parameter('rejoin_block_confirm_frames').value))
        self.rejoin_block_turn_sec = float(self.get_parameter('rejoin_block_turn_sec').value)
        self.rejoin_max_ang = float(self.get_parameter('rejoin_max_ang').value)
        self.line_rejoin_error_thresh = float(self.get_parameter('line_rejoin_error_thresh').value)
        self.line_rejoin_confirm_frames = max(1, int(self.get_parameter('line_rejoin_confirm_frames').value))
        self.max_rejoin_time = float(self.get_parameter('max_rejoin_time_sec').value)
        self.line_lost_sentinel = float(self.get_parameter('line_lost_sentinel').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.line_sub = self.create_subscription(Float32, self.line_error_topic, self.on_line_error, 10)
        self.timer = self.create_timer(1.0 / max(1.0, publish_rate), self.on_timer)

        self.last_scan = None
        self.last_pose = None
        self.last_yaw = None
        self.forward_start_pose = None
        self.turn_away_start_yaw = None
        self.last_line_error = self.line_lost_sentinel
        self.line_valid = False
        self.line_centered_frames = 0
        self.state = self.IDLE
        self.state_started = self.get_clock().now()
        self.state_until = self.state_started
        self.turn_dir = 1.0
        self.logged_stop = False
        self.rejoin_timeout_logged = False
        self.logged_line_steering = False
        self.last_cmd_log_time = self.get_clock().now()
        self.last_scan_log_time = self.get_clock().now()
        self.rejoin_turn_until = self.get_clock().now()
        self.rejoin_blocked_count = 0

        self.get_logger().info(
            'ObstacleAvoid running: '
            f'avoid={self.avoid_distance:.2f}, clear={self.clear_distance:.2f}, '
            f'emergency={self.emergency_distance:.2f}, turn={self.turn_time:.2f}s, '
            f'forward_min_distance={self.forward_min_distance:.2f}m, '
            f'forward_time_cap={self.forward_time:.2f}s, rejoin_speed={self.rejoin_speed:.3f}'
        )

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_odom(self, msg: Odometry):
        self.last_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.last_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def distance_traveled(self, reference_pose):
        if reference_pose is None or self.last_pose is None:
            return 0.0
        dx = self.last_pose[0] - reference_pose[0]
        dy = self.last_pose[1] - reference_pose[1]
        return math.hypot(dx, dy)

    def on_line_error(self, msg: Float32):
        err = float(msg.data)
        self.last_line_error = err
        self.line_valid = math.isfinite(err) and err != self.line_lost_sentinel
        if self.state == self.SEARCH_LINE_FORWARD and self.line_valid and abs(err) <= self.line_rejoin_error_thresh:
            self.line_centered_frames += 1
        elif self.state == self.SEARCH_LINE_FORWARD:
            self.line_centered_frames = 0

    def enter_state(self, state, duration_sec=0.0):
        self.state = state
        self.state_started = self.get_clock().now()
        self.state_until = self.state_started + Duration(seconds=duration_sec)
        if state != self.STOP:
            self.logged_stop = False
        if state == self.SEARCH_LINE_FORWARD:
            self.line_centered_frames = 0
            self.rejoin_timeout_logged = False
            self.logged_line_steering = False
            self.rejoin_turn_until = self.state_until
            self.rejoin_blocked_count = 0
        if state == self.FORWARD_AROUND:
            self.forward_start_pose = self.last_pose
        if state == self.TURN_AWAY:
            self.turn_away_start_yaw = self.last_yaw

    def scan_distances(self):
        msg = self.last_scan
        front = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -self.front_half_angle, self.front_half_angle,
        )
        front_left = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            0.0, self.front_half_angle,
        )
        front_right = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -self.front_half_angle, 0.0,
        )
        left = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            self.side_sector_min, self.side_sector_max,
        )
        right = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -self.side_sector_max, -self.side_sector_min,
        )
        return front, front_left, front_right, left, right

    def choose_turn(self, front_left, front_right, left, right):
        front_delta = 0.08
        if front_left + front_delta < front_right:
            self.turn_dir = -1.0
            return 'front_obstacle_left_turn_right'
        if front_right + front_delta < front_left:
            self.turn_dir = 1.0
            return 'front_obstacle_right_turn_left'
        self.turn_dir = 1.0 if left >= right else -1.0
        return 'side_clearance_left' if self.turn_dir > 0.0 else 'side_clearance_right'

    def state_name(self):
        names = {
            self.IDLE: 'NORMAL',
            self.STOP: 'STOP',
            self.TURN_AWAY: 'TURN_AWAY',
            self.FORWARD_AROUND: 'FORWARD_AROUND',
            self.TURN_BACK: 'TURN_BACK',
            self.SEARCH_LINE_FORWARD: 'REJOIN_LINE',
        }
        return names.get(self.state, f'UNKNOWN({self.state})')

    def publish(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = max(0.0, float(linear_x))
        cmd.angular.z = float(angular_z)
        self.pub.publish(cmd)

        now = self.get_clock().now()
        if self.state != self.IDLE and (now - self.last_cmd_log_time).nanoseconds * 1e-9 >= 0.5:
            self.last_cmd_log_time = now
            self.get_logger().info(
                f'CMD state={self.state_name()} linear.x={cmd.linear.x:.3f} '
                f'angular.z={cmd.angular.z:.3f}'
            )

    def start_turn_away(self):
        self.enter_state(self.TURN_AWAY, self.turn_time)
        if self.turn_dir > 0.0:
            self.get_logger().info('TURN_AWAY_LEFT chosen_direction=left')
        else:
            self.get_logger().info('TURN_AWAY_RIGHT chosen_direction=right')

    def release_to_line_follower(self):
        self.get_logger().info('RELEASE_TO_LINE_FOLLOWER')
        self.enter_state(self.IDLE)

    def rejoin_elapsed(self, now):
        return (now - self.state_started).nanoseconds * 1e-9

    def on_timer(self):
        if self.last_scan is None:
            return

        now = self.get_clock().now()
        front, front_left, front_right, left, right = self.scan_distances()

        if (now - self.last_scan_log_time).nanoseconds * 1e-9 >= 1.0:
            self.last_scan_log_time = now
            self.get_logger().info(
                f'DIAG state={self.state_name()} front_min={front:.2f} '
                f'front_left={front_left:.2f} front_right={front_right:.2f} '
                f'left_clearance={left:.2f} right_clearance={right:.2f} '
                f'publishing_cmd={self.state != self.IDLE}'
            )

        if self.state not in (self.IDLE, self.STOP, self.TURN_AWAY) and front < self.emergency_distance:
            reason = self.choose_turn(front_left, front_right, left, right)
            self.get_logger().warn(
                f'EMERGENCY_TURN front_min={front:.2f} state={self.state_name()} reason={reason}'
            )
            self.start_turn_away()
            self.publish(0.0, self.turn_dir * self.turn_speed)
            return

        if self.state == self.IDLE:
            if front < self.avoid_distance:
                reason = self.choose_turn(front_left, front_right, left, right)
                side = 'left' if self.turn_dir > 0.0 else 'right'
                self.get_logger().warn(
                    f'OBSTACLE_DETECTED front_min={front:.2f} front_left={front_left:.2f} '
                    f'front_right={front_right:.2f} left_clearance={left:.2f} '
                    f'right_clearance={right:.2f} chosen_direction={side} reason={reason}'
                )
                if self.stop_time <= 0.0:
                    self.start_turn_away()
                    self.publish(0.0, self.turn_dir * self.turn_speed)
                else:
                    self.enter_state(self.STOP, self.stop_time)
                    self.publish(0.0, 0.0)
            return

        if self.state == self.STOP:
            if not self.logged_stop:
                self.get_logger().info('STOP')
                self.logged_stop = True
            self.publish(0.0, 0.0)
            if now >= self.state_until:
                self.start_turn_away()
            return

        if self.state == self.TURN_AWAY:
            self.publish(0.0, self.turn_dir * self.turn_speed)
            turn_elapsed = (now - self.state_started).nanoseconds * 1e-9
            if turn_elapsed >= self.min_turn_away_time and front > self.avoid_distance:
                self.enter_state(self.FORWARD_AROUND, self.forward_time)
                self.get_logger().info('FORWARD_AROUND')
            elif now >= self.state_until and front <= self.avoid_distance:
                self.state_until = now + Duration(seconds=0.5)
                self.get_logger().warn(
                    f'TURN_AWAY_EXTEND front_min={front:.2f}; waiting for front sector to clear'
                )
            return

        if self.state == self.FORWARD_AROUND:
            side_min = min(left, right)
            if front < self.avoid_distance or side_min < self.side_clearance_min:
                reason = self.choose_turn(front_left, front_right, left, right)
                self.get_logger().warn(
                    f'FORWARD_BLOCKED front={front:.2f} left={left:.2f} right={right:.2f} reason={reason}'
                )
                self.start_turn_away()
                self.publish(0.0, self.turn_dir * self.turn_speed)
                return
            self.publish(self.forward_speed, 0.0)
            traveled = self.distance_traveled(self.forward_start_pose)
            distance_met = traveled >= self.forward_min_distance
            time_capped = now >= self.state_until
            if distance_met or time_capped:
                if time_capped and not distance_met:
                    self.get_logger().warn(
                        f'FORWARD_AROUND_TIME_CAP traveled={traveled:.2f}m '
                        f'< required={self.forward_min_distance:.2f}m; proceeding on time cap'
                    )
                else:
                    self.get_logger().info(f'FORWARD_AROUND_CLEARED traveled={traveled:.2f}m')
                self.enter_state(self.TURN_BACK, self.turn_back_time)
                self.get_logger().info('TURN_BACK')
            return

        if self.state == self.TURN_BACK:
            self.publish(0.0, -self.turn_dir * self.turn_speed)
            yaw_restored = False
            if self.turn_away_start_yaw is not None and self.last_yaw is not None:
                yaw_error = normalize_angle(self.last_yaw - self.turn_away_start_yaw)
                yaw_restored = abs(yaw_error) <= self.turn_back_yaw_tolerance
            time_capped = now >= self.state_until
            if yaw_restored or time_capped:
                if time_capped and not yaw_restored:
                    self.get_logger().warn('TURN_BACK_TIME_CAP heading not fully restored; proceeding on time cap')
                self.enter_state(self.SEARCH_LINE_FORWARD)
                self.get_logger().info('SEARCH_LINE_FORWARD')
            return

        if self.state == self.SEARCH_LINE_FORWARD:
            if now < self.rejoin_turn_until:
                self.publish(0.0, self.turn_dir * self.turn_speed)
                return

            if front < self.emergency_distance or front < self.rejoin_block_distance:
                self.rejoin_blocked_count += 1
            else:
                self.rejoin_blocked_count = 0

            if self.rejoin_blocked_count >= self.rejoin_block_confirm_frames:
                reason = self.choose_turn(front_left, front_right, left, right)
                self.get_logger().warn(
                    f'REJOIN_FRONT_BLOCKED front_min={front:.2f}; '
                    f'count={self.rejoin_blocked_count}; '
                    f'latched turn before rejoin reason={reason}'
                )
                self.line_centered_frames = 0
                self.logged_line_steering = False
                self.rejoin_blocked_count = 0
                self.rejoin_turn_until = now + Duration(seconds=self.rejoin_block_turn_sec)
                self.publish(0.0, self.turn_dir * self.turn_speed)
                return

            if self.line_valid:
                angular_z = self.clamp(-self.rejoin_kp * self.last_line_error, -self.rejoin_max_ang, self.rejoin_max_ang)
                abs_line_error = abs(self.last_line_error)
                if abs_line_error > 180.0:
                    linear_x = 0.0
                    rejoin_mode = 'TURN_IN_PLACE_FAR_EDGE'
                elif abs_line_error > 100.0:
                    linear_x = min(self.search_rejoin_speed, self.rejoin_speed)
                    rejoin_mode = 'SLOW_ARC_TO_LINE'
                else:
                    linear_x = self.rejoin_speed
                    rejoin_mode = 'GUIDED_REJOIN'
                self.get_logger().info(
                    f'REJOIN_STEER mode={rejoin_mode} line_error={self.last_line_error:.1f} '
                    f'linear.x={linear_x:.3f} angular.z={angular_z:.3f} '
                    f'rejoin_blocked_count={self.rejoin_blocked_count}'
                )
                self.publish(linear_x, angular_z)
                if not self.logged_line_steering:
                    self.get_logger().info('LINE_VISIBLE_STEERING_TO_REJOIN')
                    self.logged_line_steering = True
                if self.line_centered_frames >= self.line_rejoin_confirm_frames:
                    if front > self.rejoin_release_distance:
                        self.get_logger().info('LINE_CENTERED')
                        self.release_to_line_follower()
                        return
                    reason = self.choose_turn(front_left, front_right, left, right)
                    self.get_logger().warn(
                        f'LINE_CENTERED_BUT_FRONT_BLOCKED front_min={front:.2f}; '
                        f'turning before release reason={reason}'
                    )
                    self.line_centered_frames = 0
                    self.rejoin_blocked_count = 0
                    self.rejoin_turn_until = now + Duration(seconds=self.rejoin_block_turn_sec)
                    self.publish(0.0, self.turn_dir * self.turn_speed)
                    return
            else:
                self.publish(self.search_rejoin_speed, -self.turn_dir * self.search_rejoin_turn_speed)
                self.logged_line_steering = False

            if self.rejoin_elapsed(now) >= self.max_rejoin_time and not self.rejoin_timeout_logged:
                if self.line_valid:
                    self.get_logger().warn('REJOIN_TIMEOUT line visible but not centered; continuing guided rejoin')
                else:
                    self.get_logger().warn('REJOIN_TIMEOUT no centered line yet; continuing slow search')
                self.rejoin_timeout_logged = True
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
