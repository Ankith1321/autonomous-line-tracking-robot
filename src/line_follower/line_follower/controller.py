#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')

        self.declare_parameter('linear_x', 0.08)
        self.declare_parameter('min_linear_x', 0.02)
        self.declare_parameter('k_p', 0.004)
        self.declare_parameter('max_ang_z', 0.30)
        self.declare_parameter('steer_sign', -1.0)
        self.declare_parameter('lost_sentinel', -1000.0)
        self.declare_parameter('search_w', 0.18)
        self.declare_parameter('search_linear_x', 0.0)
        self.declare_parameter('slowdown_error', 80.0)
        self.declare_parameter('turn_in_place_error', 240.0)
        self.declare_parameter('error_deadband', 10.0)
        self.declare_parameter('angular_alpha', 0.35)

        self.linear_x = float(self.get_parameter('linear_x').value)
        self.min_linear_x = float(self.get_parameter('min_linear_x').value)
        self.k_p = float(self.get_parameter('k_p').value)
        self.max_ang_z = float(self.get_parameter('max_ang_z').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.lost_sentinel = float(self.get_parameter('lost_sentinel').value)
        self.search_w = float(self.get_parameter('search_w').value)
        self.search_linear_x = float(self.get_parameter('search_linear_x').value)
        self.slowdown_error = float(self.get_parameter('slowdown_error').value)
        self.turn_in_place_error = float(self.get_parameter('turn_in_place_error').value)
        self.error_deadband = max(0.0, float(self.get_parameter('error_deadband').value))
        self.angular_alpha = self.clamp(
            float(self.get_parameter('angular_alpha').value), 0.0, 1.0
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.err_sub = self.create_subscription(Float32, '/line_error', self.on_error, 10)
        self.last_turn_sign = 1.0
        self.last_angular_z = 0.0

        self.get_logger().info(
            f'Controller started: cmd_vel=/cmd_vel_raw, linear_x={self.linear_x}, '
            f'min_linear_x={self.min_linear_x}, k_p={self.k_p}, max_ang_z={self.max_ang_z}, '
            f'error_deadband={self.error_deadband}, angular_alpha={self.angular_alpha}'
        )

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def on_error(self, msg: Float32) -> None:
        err = float(msg.data)
        cmd = Twist()

        if not math.isfinite(err) or err == self.lost_sentinel:
            cmd.linear.x = self.search_linear_x
            cmd.angular.z = self.last_turn_sign * self.search_w
            self.last_angular_z = cmd.angular.z
            self.cmd_pub.publish(cmd)
            return

        if abs(err) <= self.error_deadband:
            target_angular_z = 0.0
        else:
            target_angular_z = self.clamp(
                self.steer_sign * self.k_p * err,
                -self.max_ang_z,
                self.max_ang_z,
            )

        angular_z = (
            self.last_angular_z
            + self.angular_alpha * (target_angular_z - self.last_angular_z)
        )
        angular_z = self.clamp(angular_z, -self.max_ang_z, self.max_ang_z)
        self.last_angular_z = angular_z
        if abs(angular_z) > 1e-4:
            self.last_turn_sign = 1.0 if angular_z > 0.0 else -1.0

        abs_error = abs(err)
        if abs_error >= self.turn_in_place_error:
            linear_x = 0.0
        elif abs_error <= self.slowdown_error:
            linear_x = self.linear_x
        else:
            span = max(1.0, self.turn_in_place_error - self.slowdown_error)
            ratio = (abs_error - self.slowdown_error) / span
            linear_x = self.linear_x - ratio * (self.linear_x - self.min_linear_x)

        cmd.linear.x = max(0.0, linear_x)
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineController()
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
