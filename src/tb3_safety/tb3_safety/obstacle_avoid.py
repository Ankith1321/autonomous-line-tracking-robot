#!/usr/bin/env python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


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
    TURN = 2
    FORWARD = 3
    RETURN_TO_LINE = 4

    def __init__(self):
        super().__init__('obstacle_avoid')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_obstacle')
        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('side_sector_min_deg', 35.0)
        self.declare_parameter('side_sector_max_deg', 100.0)
        self.declare_parameter('avoid_distance', 0.35)
        self.declare_parameter('clear_distance', 0.45)
        self.declare_parameter('emergency_distance', 0.18)
        self.declare_parameter('stop_time_sec', 0.30)
        self.declare_parameter('turn_time_sec', 1.40)
        self.declare_parameter('forward_time_sec', 0.90)
        self.declare_parameter('turn_speed', 0.30)
        self.declare_parameter('forward_speed', 0.03)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value)
        )
        self.side_sector_min = math.radians(
            float(self.get_parameter('side_sector_min_deg').value)
        )
        self.side_sector_max = math.radians(
            float(self.get_parameter('side_sector_max_deg').value)
        )
        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.stop_time = float(self.get_parameter('stop_time_sec').value)
        self.turn_time = float(self.get_parameter('turn_time_sec').value)
        self.forward_time = float(self.get_parameter('forward_time_sec').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.timer = self.create_timer(1.0 / max(1.0, publish_rate), self.on_timer)

        self.last_scan = None
        self.state = self.IDLE
        self.state_until = self.get_clock().now()
        self.turn_dir = 1.0
        self.logged_stop = False

        self.get_logger().info(
            'ObstacleAvoid running: '
            f'avoid={self.avoid_distance:.2f}, clear={self.clear_distance:.2f}, '
            f'emergency={self.emergency_distance:.2f}, stop={self.stop_time:.2f}s, '
            f'turn={self.turn_time:.2f}s @ {self.turn_speed:.2f} rad/s, '
            f'forward={self.forward_time:.2f}s @ {self.forward_speed:.2f} m/s'
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def enter_state(self, state, duration_sec=0.0):
        self.state = state
        self.state_until = self.get_clock().now() + Duration(seconds=duration_sec)
        if state != self.STOP:
            self.logged_stop = False

    def scan_distances(self):
        msg = self.last_scan
        front = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -self.front_half_angle, self.front_half_angle,
        )
        left = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            self.side_sector_min, self.side_sector_max,
        )
        right = sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -self.side_sector_max, -self.side_sector_min,
        )
        return front, left, right

    def choose_turn(self, left, right):
        self.turn_dir = 1.0 if left >= right else -1.0

    def publish(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.pub.publish(cmd)

    def on_timer(self):
        if self.last_scan is None:
            return

        now = self.get_clock().now()
        front, left, right = self.scan_distances()

        if self.state == self.IDLE:
            if front < self.avoid_distance:
                self.choose_turn(left, right)
                self.get_logger().warn(
                    f'OBSTACLE_DETECTED front={front:.2f} left={left:.2f} right={right:.2f}'
                )
                self.enter_state(self.STOP, self.stop_time)
            return

        if self.state == self.STOP:
            if not self.logged_stop:
                self.get_logger().info('STOP')
                self.logged_stop = True
            self.publish(0.0, 0.0)
            if now >= self.state_until:
                self.enter_state(self.TURN, self.turn_time)
                if self.turn_dir > 0.0:
                    self.get_logger().info('TURN_LEFT')
                else:
                    self.get_logger().info('TURN_RIGHT')
            return

        if self.state == self.TURN:
            self.publish(0.0, self.turn_dir * self.turn_speed)
            if now >= self.state_until:
                self.enter_state(self.FORWARD, self.forward_time)
                self.get_logger().info('MOVE_FORWARD')
            return

        if self.state == self.FORWARD:
            if front < self.emergency_distance:
                self.get_logger().warn(
                    f'EMERGENCY_STOP front={front:.2f} left={left:.2f} right={right:.2f}'
                )
                self.enter_state(self.STOP, self.stop_time)
                return
            self.publish(self.forward_speed, 0.0)
            if now >= self.state_until:
                self.enter_state(self.RETURN_TO_LINE)
                self.get_logger().info('RETURN_TO_LINE')
            return

        if self.state == self.RETURN_TO_LINE:
            self.enter_state(self.IDLE)
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
