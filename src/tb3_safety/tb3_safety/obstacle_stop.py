#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleStop(Node):
    """
    VETO-ONLY safety node:
    - Subscribes to /scan
    - Computes closest obstacle in front sector (+/- angle)
    - Publishes /cmd_vel_obstacle:
        * STOP (0,0) when obstacle is too close
        * nothing when clear, so the mux can use /cmd_vel_raw
    """

    def __init__(self):
        super().__init__('obstacle_stop')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_obstacle')
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').get_parameter_value().double_value
        )
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # State
        self.latest_min_front = None
        self.is_stopped = False

        # ROS I/O
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            f"ObstacleStop (VETO) running. scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}, "
            f"front_half_angle_deg={math.degrees(self.front_half_angle):.1f}, "
            f"stop_distance={self.stop_distance:.2f}"
        )

    def on_scan(self, msg: LaserScan):
        min_front = float('inf')

        angle = msg.angle_min
        for r in msg.ranges:
            a = self._wrap_to_pi(angle)
            if -self.front_half_angle <= a <= self.front_half_angle:
                if math.isfinite(r):
                    min_front = min(min_front, r)
            angle += msg.angle_increment

        self.latest_min_front = min_front

    @staticmethod
    def _wrap_to_pi(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def on_timer(self):
        # If no scan yet, do not claim the obstacle channel.
        if self.latest_min_front is None:
            return

        should_stop = self.latest_min_front < self.stop_distance

        if should_stop:
            out = Twist()
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.pub.publish(out)

            if not self.is_stopped:
                self.get_logger().warn(f"STOP: obstacle at {self.latest_min_front:.2f} m")
                self.is_stopped = True
        else:
            if self.is_stopped:
                self.get_logger().info(
                    f"CLEAR: path clear (min_front={self.latest_min_front:.2f})"
                )
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
