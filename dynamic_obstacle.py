#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from visualization_msgs.msg import Marker, MarkerArray


class DynamicObstacle(Node):
    def __init__(self):
        super().__init__('demo_dynamic_obstacle')
        self.declare_parameter('entity_name', 'dynamic_obstacle_box')
        self.declare_parameter('x', 1.70)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('amplitude_y', 0.55)
        self.declare_parameter('z', 0.18)
        self.declare_parameter('delay_sec', 10.0)
        self.declare_parameter('period_sec', 20.0)
        self.declare_parameter('marker_topic', '/demo_world_markers')

        self.entity_name = self.get_parameter('entity_name').value
        self.x = float(self.get_parameter('x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.amplitude_y = float(self.get_parameter('amplitude_y').value)
        self.z = float(self.get_parameter('z').value)
        self.delay_sec = float(self.get_parameter('delay_sec').value)
        self.period_sec = max(1.0, float(self.get_parameter('period_sec').value))
        marker_topic = self.get_parameter('marker_topic').value

        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self.on_timer)
        self.pending_future = None
        self.service_ready_logged = False
        self.motion_started_logged = False
        self.last_wait_log_sec = -10.0
        self.last_motion_log_sec = -10.0
        self.get_logger().info(
            f'Dynamic obstacle active: entity={self.entity_name}, x={self.x:.2f}, '
            f'y=+/-{self.amplitude_y:.2f}, delay={self.delay_sec:.1f}s'
        )

    def elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def current_y(self):
        t = max(0.0, self.elapsed() - self.delay_sec)
        phase = -math.pi / 2.0 + 2.0 * math.pi * (t / self.period_sec)
        return self.center_y + self.amplitude_y * math.sin(phase)

    def on_timer(self):
        if not self.client.service_is_ready():
            self.wait_for_set_state_service()
            return

        if not self.service_ready_logged:
            self.get_logger().info('/set_entity_state service ready; dynamic obstacle controller is live')
            self.service_ready_logged = True

        self.finish_pending_request()
        if self.pending_future is not None:
            return

        y = -self.amplitude_y if self.elapsed() < self.delay_sec else self.current_y()
        self.set_pose(y)
        self.publish_marker(y)
        self.log_motion(y)

    def wait_for_set_state_service(self):
        now_sec = self.elapsed()
        if now_sec - self.last_wait_log_sec >= 2.0:
            self.get_logger().warn('Waiting for /set_entity_state service')
            self.last_wait_log_sec = now_sec
        self.client.wait_for_service(timeout_sec=0.1)

    def finish_pending_request(self):
        if self.pending_future is None or not self.pending_future.done():
            return

        try:
            result = self.pending_future.result()
        except Exception as exc:
            self.get_logger().warn(f'/set_entity_state request failed: {exc}')
            self.pending_future = None
            return

        if result is not None and not result.success:
            self.get_logger().warn(f'/set_entity_state rejected request: {result.status_message}')
        self.pending_future = None

    def set_pose(self, y):
        state = EntityState()
        state.name = self.entity_name
        state.reference_frame = 'world'
        state.pose.position.x = self.x
        state.pose.position.y = y
        state.pose.position.z = self.z
        state.pose.orientation.w = 1.0

        request = SetEntityState.Request()
        request.state = state
        self.pending_future = self.client.call_async(request)

    def log_motion(self, y):
        now_sec = self.elapsed()
        if now_sec < self.delay_sec:
            if not self.motion_started_logged and now_sec - self.last_motion_log_sec >= 2.0:
                remaining = max(0.0, self.delay_sec - now_sec)
                self.get_logger().info(
                    f'Dynamic obstacle holding at y={y:.2f}; motion starts in {remaining:.1f}s'
                )
                self.last_motion_log_sec = now_sec
            return

        if not self.motion_started_logged:
            self.get_logger().info('Dynamic obstacle motion started')
            self.motion_started_logged = True

        if now_sec - self.last_motion_log_sec >= 1.0:
            self.get_logger().info(f'Dynamic obstacle moving: x={self.x:.2f}, y={y:.2f}')
            self.last_motion_log_sec = now_sec

    def publish_marker(self, y):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dynamic_obstacle'
        marker.id = 500
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = y
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.36
        marker.color.r = 0.1
        marker.color.g = 0.1
        marker.color.b = 0.9
        marker.color.a = 1.0
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacle()
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
