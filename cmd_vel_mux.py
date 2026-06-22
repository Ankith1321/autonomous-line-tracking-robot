#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('obstacle_timeout_sec', 0.35)
        self.declare_parameter('line_timeout_sec', 0.50)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('debug', False)

        self.obstacle_timeout = float(self.get_parameter('obstacle_timeout_sec').value)
        self.line_timeout = float(self.get_parameter('line_timeout_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.period = 1.0 / publish_hz if publish_hz > 0.0 else 0.05

        self.last_line = Twist()
        self.last_obs = Twist()
        self.last_line_time = None
        self.last_obs_time = None
        self.last_source = None

        self.create_subscription(Twist, '/cmd_vel_raw', self.on_line, 10)
        self.create_subscription(Twist, '/cmd_vel_obstacle', self.on_obs, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(
            f'cmd_vel_mux started | obstacle_timeout={self.obstacle_timeout:.2f}s '
            f'line_timeout={self.line_timeout:.2f}s publish_hz={1.0 / self.period:.1f}'
        )

    def on_line(self, msg: Twist):
        self.last_line = msg
        self.last_line_time = self.get_clock().now()

    def on_obs(self, msg: Twist):
        self.last_obs = msg
        self.last_obs_time = self.get_clock().now()

    def is_recent(self, timestamp, timeout_sec):
        if timestamp is None:
            return False
        age_sec = (self.get_clock().now() - timestamp).nanoseconds * 1e-9
        return age_sec <= timeout_sec

    def tick(self):
        obs_recent = self.is_recent(self.last_obs_time, self.obstacle_timeout)
        line_recent = self.is_recent(self.last_line_time, self.line_timeout)

        if obs_recent:
            out = self.last_obs
            source = 'OBSTACLE'
        elif line_recent:
            out = self.last_line
            source = 'RAW'
        else:
            out = Twist()
            source = 'ZERO'

        if source != self.last_source:
            self.last_source = source
            self.get_logger().info(
                f'mux source={source} v={out.linear.x:.3f} w={out.angular.z:.3f}'
            )
        elif self.debug:
            self.get_logger().info(
                f'mux source={source} v={out.linear.x:.3f} w={out.angular.z:.3f}'
            )

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
