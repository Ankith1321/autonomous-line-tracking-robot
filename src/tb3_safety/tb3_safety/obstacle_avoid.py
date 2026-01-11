import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def sector_min(ranges, angle_min, angle_inc, a0, a1):
    n = len(ranges)
    i0 = int((a0 - angle_min) / angle_inc)
    i1 = int((a1 - angle_min) / angle_inc)
    i0 = max(0, min(n - 1, i0))
    i1 = max(0, min(n - 1, i1))
    if i0 > i1:
        i0, i1 = i1, i0

    m = float('inf')
    for i in range(i0, i1 + 1):
        r = ranges[i]
        if math.isfinite(r) and r > 0.0 and r < m:
            m = r
    return m


class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_avoid')
        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('side_sector_deg', 60.0)
        self.declare_parameter('avoid_distance', 0.55)
        self.declare_parameter('clear_distance', 0.75)
        self.declare_parameter('forward_speed', 0.07)
        self.declare_parameter('turn_speed', 0.9)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.front_half_angle = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        self.side_sector = math.radians(float(self.get_parameter('side_sector_deg').value))

        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.last_scan = None
        self.avoiding = False
        self.turn_dir = 1.0  # +1 left, -1 right

        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            f'ObstacleAvoid running. scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'avoid_distance={self.avoid_distance}, clear_distance={self.clear_distance}'
        )

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_timer(self):
        if self.last_scan is None:
            return

        msg = self.last_scan
        a_min = msg.angle_min
        a_inc = msg.angle_increment
        ranges = msg.ranges

        front_min = sector_min(ranges, a_min, a_inc, -self.front_half_angle, self.front_half_angle)
        left_min = sector_min(ranges, a_min, a_inc, self.front_half_angle, self.front_half_angle + self.side_sector)
        right_min = sector_min(ranges, a_min, a_inc, -(self.front_half_angle + self.side_sector), -self.front_half_angle)

        if not self.avoiding:
            if front_min < self.avoid_distance:
                self.avoiding = True
                self.turn_dir = 1.0 if left_min > right_min else -1.0
                self.get_logger().warn(
                    f'AVOID start: front={front_min:.2f} left={left_min:.2f} right={right_min:.2f} '
                    f'dir={"LEFT" if self.turn_dir > 0 else "RIGHT"}'
                )
        else:
            if front_min > self.clear_distance:
                self.avoiding = False
                self.get_logger().info(f'AVOID end: front={front_min:.2f}')

        out = Twist()
        if self.avoiding:
            out.linear.x = self.forward_speed
            out.angular.z = self.turn_dir * self.turn_speed
        else:
            out.linear.x = 0.0
            out.angular.z = 0.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
