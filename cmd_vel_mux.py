#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Tunables
        self.declare_parameter('obstacle_timeout_sec', 0.25)  # seconds
        self.declare_parameter('line_timeout_sec', 0.50)      # seconds
        self.declare_parameter('publish_hz', 20.0)            # Hz
        self.declare_parameter('debug', False)

        self.obstacle_timeout = float(self.get_parameter('obstacle_timeout_sec').value)
        self.line_timeout = float(self.get_parameter('line_timeout_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.period = 1.0 / publish_hz if publish_hz > 0.0 else 0.05

        # State
        self.last_line = Twist()
        self.last_obs = Twist()
        self.last_line_time = None
        self.last_obs_time = None

        # I/O
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_line, 10)
        self.create_subscription(Twist, '/cmd_vel_obstacle', self.on_obs, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(
            f"cmd_vel_mux started | obstacle_timeout={self.obstacle_timeout:.2f}s "
            f"line_timeout={self.line_timeout:.2f}s publish_hz={1.0/self.period:.1f}"
        )

    def on_line(self, msg: Twist):
        self.last_line = msg
        self.last_line_time = self.get_clock().now()

    def on_obs(self, msg: Twist):
        self.last_obs = msg
        self.last_obs_time = self.get_clock().now()

    def _is_recent(self, t, timeout_sec: float) -> bool:
        if t is None:
            return False
        now = self.get_clock().now()
        age_sec = (now - t).nanoseconds * 1e-9
        return age_sec <= timeout_sec

    def tick(self):
        obs_recent = self._is_recent(self.last_obs_time, self.obstacle_timeout)
        line_recent = self._is_recent(self.last_line_time, self.line_timeout)

        if obs_recent:
            out = self.last_obs
            src = "OBSTACLE"
        elif line_recent:
            out = self.last_line
            src = "LINE"
        else:
            out = Twist()  # stop if both are stale
            src = "STOP(stale)"

        if self.debug:
            self.get_logger().info(f"mux -> {src}: v={out.linear.x:.3f}, w={out.angular.z:.3f}")

        self.pub.publish(out)

def main():
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
