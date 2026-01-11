#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleStop(Node):
    """
    Simple safety controller:
    - Subscribes to /scan
    - Finds the closest obstacle in a front sector (+/- angle)
    - Publishes /cmd_vel: stops if too close, otherwise drives forward
    - Logs only when state changes (STOP <-> GO) to avoid spam
    """

    def __init__(self):
        super().__init__('obstacle_stop')

        # Parameters (tunable)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('front_half_angle_deg', 20.0)   # +/- degrees around forward direction
        self.declare_parameter('stop_distance', 0.35)          # meters
        self.declare_parameter('forward_speed', 0.08)          # m/s
        self.declare_parameter('publish_rate_hz', 10.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').get_parameter_value().double_value
        )
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # State
        self.latest_min_front = None  # float or None
        self.is_stopped = False

        # ROS I/O
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            f"ObstacleStop running. scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}, "
            f"front_half_angle_deg={math.degrees(self.front_half_angle):.1f}, "
            f"stop_distance={self.stop_distance:.2f}, forward_speed={self.forward_speed:.2f}"
        )

    def on_scan(self, msg: LaserScan):
        """
        Compute the minimum finite range in the front sector.
        TurtleBot3 LaserScan in Gazebo typically has angle=0 as forward.
        """
        min_front = float('inf')

        angle = msg.angle_min
        for r in msg.ranges:
            a = self._wrap_to_pi(angle)
            if -self.front_half_angle <= a <= self.front_half_angle:
                if math.isfinite(r):
                    min_front = min(min_front, r)
            angle += msg.angle_increment

        # If nothing finite, keep inf (meaning clear within sensor max range)
        self.latest_min_front = min_front

    @staticmethod
    def _wrap_to_pi(a: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def on_timer(self):
        twist = Twist()

        # If no scan received yet, publish zero for safety
        if self.latest_min_front is None:
            self.pub.publish(twist)
            return

        should_stop = self.latest_min_front < self.stop_distance

        if should_stop:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

            if not self.is_stopped:
                self.get_logger().warn(f"STOP: obstacle at {self.latest_min_front:.2f} m")
                self.is_stopped = True
        else:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.pub.publish(twist)

            if self.is_stopped:
                # If min_front is inf, format will show "inf" which is fine.
                self.get_logger().info(f"GO: path clear (min_front={self.latest_min_front:.2f})")
                self.is_stopped = False


def main():
    rclpy.init()
    node = ObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
