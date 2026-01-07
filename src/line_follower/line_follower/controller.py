#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LineController(Node):
    def __init__(self):
        super().__init__("line_controller")

        # Parameters (easy to tune)
        self.declare_parameter("linear_x", 0.08)     # m/s
        self.declare_parameter("k_p", 0.012)          # proportional gain
        self.declare_parameter("max_ang_z", 1.0)     # rad/s
        self.declare_parameter("steer_sign", 1.0)    # set to -1.0 if steering is reversed

        self._linear_x = float(self.get_parameter("linear_x").value)
        self._k_p = float(self.get_parameter("k_p").value)
        self._max_ang_z = float(self.get_parameter("max_ang_z").value)
        self._steer_sign = float(self.get_parameter("steer_sign").value)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._err_sub = self.create_subscription(
            Float32,
            "/line_error",
            self._on_error,
            10,
        )

        self.get_logger().info(
            f"Controller started: linear_x={self._linear_x}, "
            f"k_p={self._k_p}, max_ang_z={self._max_ang_z}, "
            f"steer_sign={self._steer_sign}"
        )

    def _clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _on_error(self, msg: Float32) -> None:
        err = float(msg.data)

        twist = Twist()

        # If detector reports "no line", stop for safety.
        if err == -1.0:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            return

        # Proportional steering control
        ang = self._steer_sign * self._k_p * err
        ang = self._clamp(ang, -self._max_ang_z, self._max_ang_z)

        twist.linear.x = self._linear_x
        twist.angular.z = ang
        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass



if __name__ == "__main__":
    main()
