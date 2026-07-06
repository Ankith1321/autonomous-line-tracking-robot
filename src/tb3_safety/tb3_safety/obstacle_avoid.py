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
    minimum = float('inf')
    for idx, val in enumerate(ranges):
        if not math.isfinite(val) or val <= 0.0:
            continue
        angle = angle_min + idx * angle_inc
        if angle_in_sector(angle, a0, a1):
            minimum = min(minimum, val)
    return minimum


class ObstacleAvoid(Node):
    IDLE = 0
    STOP = 1
    TURN = 2
    FORWARD = 3
    TURN_BACK = 4
    SEARCH_LINE = 5

    def __init__(self):
        super().__init__('obstacle_avoid')
        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('side_sector_max_deg', 170.0)
        self.declare_parameter('lidar_to_hull_margin', 0.20)
        self.declare_parameter('avoid_distance', 0.95)
        self.declare_parameter('emergency_distance', 0.25)
        self.declare_parameter('stop_time_sec', 0.60)
        self.declare_parameter('back_off_speed', 0.12)
        self.declare_parameter('turn_direction_hysteresis_m', 0.10)
        self.declare_parameter('turn_speed', 0.30)
        self.declare_parameter('turn_time_sec', 2.30)
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('forward_distance_m', 2.20)
        self.declare_parameter('forward_time_sec', 15.00)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('search_turn_speed', 0.12)
        self.declare_parameter('line_search_error_threshold', 20.0)
        self.declare_parameter('line_search_confirm_count', 8)
        self.declare_parameter('search_max_yaw_deviation_deg', 130.0)
        self.declare_parameter('search_timeout_sec', 45.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Get parameters
        self.front_half_angle = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        self.side_sector_max = math.radians(float(self.get_parameter('side_sector_max_deg').value))
        self.lidar_to_hull_margin = float(self.get_parameter('lidar_to_hull_margin').value)
        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.stop_time = float(self.get_parameter('stop_time_sec').value)
        self.back_off_speed = float(self.get_parameter('back_off_speed').value)
        self.turn_direction_hysteresis = float(self.get_parameter('turn_direction_hysteresis_m').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_time = float(self.get_parameter('turn_time_sec').value)
        self.turn_time_cap = self.turn_time * 1.6
        self.intended_turn_angle = self.turn_speed * self.turn_time
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.forward_distance = float(self.get_parameter('forward_distance_m').value)
        self.forward_time = float(self.get_parameter('forward_time_sec').value)
        self.yaw_tolerance = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self.search_turn_speed = float(self.get_parameter('search_turn_speed').value)
        self.line_search_error_threshold = float(self.get_parameter('line_search_error_threshold').value)
        self.line_search_confirm_count = int(self.get_parameter('line_search_confirm_count').value)
        self.search_max_yaw_deviation = math.radians(float(self.get_parameter('search_max_yaw_deviation_deg').value))
        self.search_timeout = float(self.get_parameter('search_timeout_sec').value)
        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_obstacle', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.line_error_sub = self.create_subscription(Float32, '/line_error', self.on_line_error, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

        self.last_scan = None
        self.line_error = None
        self.line_seen_count = 0
        self.state = self.IDLE
        self.state_until = self.get_clock().now()
        self.turn_dir = 1.0
        self.current_yaw = None
        self.target_yaw = None
        self.current_x = None
        self.current_y = None
        self.forward_start_x = None
        self.forward_start_y = None
        self.turn_start_yaw = None
        self.search_start_yaw = None
        self.search_dir = 1.0
        self.search_entered_time = None

        self.get_logger().info(
            f'ObstacleAvoid: avoid={self.avoid_distance:.2f}m, '
            f'emergency={self.emergency_distance:.2f}m, '
            f'turn={self.turn_time:.2f}s, forward={self.forward_distance:.2f}m'
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def on_line_error(self, msg: Float32):
        if math.isfinite(msg.data) and msg.data != -1.0:
            self.line_error = float(msg.data)
        else:
            self.line_error = None

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def in_sector(self, angle, start, end):
        return angle_in_sector(angle, start, end)

    def scan_distances(self):
        scan = self.last_scan
        front = sector_min(scan.ranges, scan.angle_min, scan.angle_increment, -self.front_half_angle, self.front_half_angle)
        left = sector_min(scan.ranges, scan.angle_min, scan.angle_increment, self.front_half_angle, self.side_sector_max)
        right = sector_min(scan.ranges, scan.angle_min, scan.angle_increment, -self.side_sector_max, -self.front_half_angle)
        return (front - self.lidar_to_hull_margin, left - self.lidar_to_hull_margin, right - self.lidar_to_hull_margin)

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def enter_state(self, state: int, duration_sec: float = 0.0):
        self.state = state
        self.state_until = self.get_clock().now() + Duration(seconds=duration_sec)
        if state == self.SEARCH_LINE:
            self.search_start_yaw = self.current_yaw
            self.search_dir = -self.turn_dir if self.turn_dir != 0.0 else 1.0
            self.search_entered_time = self.get_clock().now()

    def has_line(self) -> bool:
        return self.line_error is not None and abs(self.line_error) <= self.line_search_error_threshold

    def distance_travelled(self) -> float:
        if self.current_x is None or self.forward_start_x is None:
            return 0.0
        return math.hypot(self.current_x - self.forward_start_x, self.current_y - self.forward_start_y)

    def select_turn_direction(self, left: float, right: float):
        if left >= right + self.turn_direction_hysteresis:
            self.turn_dir = 1.0
        elif right >= left + self.turn_direction_hysteresis:
            self.turn_dir = -1.0

    def on_timer(self):
        if self.last_scan is None:
            return
        now = self.get_clock().now()
        front, left, right = self.scan_distances()

        if self.state == self.IDLE:
            self.publish_cmd(0.0, 0.0)
            if front < self.avoid_distance:
                self.target_yaw = self.current_yaw
                self.select_turn_direction(left, right)
                self.get_logger().warn(f'OBSTACLE front={front:.2f} left={left:.2f} right={right:.2f}')
                self.enter_state(self.STOP, self.stop_time)
            return

        if self.state == self.STOP:
            self.publish_cmd(-self.back_off_speed, 0.0)
            if now >= self.state_until:
                self.turn_start_yaw = self.current_yaw
                self.enter_state(self.TURN, self.turn_time_cap)
            return

        if self.state == self.TURN:
            if min(front, left, right) < self.emergency_distance:
                self.select_turn_direction(left, right)
                self.get_logger().warn(f'EMERGENCY during turn front={front:.2f}')
                self.enter_state(self.STOP, self.stop_time)
                return
            self.publish_cmd(0.0, self.turn_dir * self.turn_speed)
            rotated_enough = (self.turn_start_yaw is not None and self.current_yaw is not None and
                            abs(normalize_angle(self.current_yaw - self.turn_start_yaw)) >= self.intended_turn_angle)
            if rotated_enough or now >= self.state_until:
                self.forward_start_x = self.current_x
                self.forward_start_y = self.current_y
                self.enter_state(self.FORWARD, self.forward_time)
            return

        if self.state == self.FORWARD:
            if min(front, left, right) < self.emergency_distance:
                self.select_turn_direction(left, right)
                self.get_logger().warn(f'EMERGENCY forward front={front:.2f}')
                self.enter_state(self.STOP, self.stop_time)
                return
            self.publish_cmd(self.forward_speed, 0.0)
            if self.distance_travelled() >= self.forward_distance or now >= self.state_until:
                self.enter_state(self.TURN_BACK)
            return

        if self.state == self.TURN_BACK:
            if self.current_yaw is None or self.target_yaw is None:
                self.enter_state(self.SEARCH_LINE)
                return
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) <= self.yaw_tolerance:
                self.enter_state(self.SEARCH_LINE)
                return
            self.publish_cmd(0.0, math.copysign(self.turn_speed, yaw_error))
            return

        if self.state == self.SEARCH_LINE:
            if self.has_line():
                self.line_seen_count += 1
                if self.line_seen_count >= self.line_search_confirm_count:
                    self.get_logger().info('LINE reacquired')
                    self.line_seen_count = 0
                    self.enter_state(self.IDLE)
                    return
            else:
                self.line_seen_count = 0

            if self.search_entered_time is not None:
                searched_for = (now - self.search_entered_time).nanoseconds / 1e9
                if searched_for >= self.search_timeout:
                    self.get_logger().warn(f'SEARCH timeout after {searched_for:.1f}s')
                    self.line_seen_count = 0
                    self.enter_state(self.IDLE)
                    return

            yaw_dev = 0.0
            if self.current_yaw is not None and self.search_start_yaw is not None:
                yaw_dev = normalize_angle(self.current_yaw - self.search_start_yaw)

            if yaw_dev >= self.search_max_yaw_deviation:
                self.search_dir = -1.0
            elif yaw_dev <= -self.search_max_yaw_deviation:
                self.search_dir = 1.0
            elif self.line_error is not None:
                self.search_dir = -math.copysign(1.0, self.line_error)

            self.publish_cmd(0.0, self.search_dir * self.search_turn_speed)


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
