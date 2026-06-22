#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

WORLD_FILE = Path('/ws/worlds/yellow_line_obstacle_demo.world')
MARKER_TOPIC = '/demo_world_markers'
FRAME_ID = 'odom'


class DemoWorldMarkers(Node):
    def __init__(self):
        super().__init__('demo_world_markers')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, MARKER_TOPIC, qos)
        self.markers = self.load_world_markers(WORLD_FILE)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f'Publishing {len(self.markers)} Gazebo-derived markers on {MARKER_TOPIC}'
        )

    def load_world_markers(self, world_file):
        root = ET.parse(world_file).getroot()
        markers = []
        marker_id = 0
        for model in root.findall('.//model'):
            name = model.get('name', '')
            if not self.is_demo_marker_model(name):
                continue

            pose = self.parse_pose(model.findtext('pose'))
            visual = model.find('./link/visual')
            if visual is None:
                continue

            geometry = visual.find('geometry')
            material = visual.find('material')
            marker_type, scale = self.parse_geometry(geometry)
            if marker_type is None:
                continue

            marker = self.make_marker(
                marker_id=marker_id,
                ns=self.namespace_for(name),
                marker_type=marker_type,
                pose=pose,
                scale=scale,
                color=self.parse_color(material, name),
            )
            markers.append(marker)
            marker_id += 1
        return markers

    def is_demo_marker_model(self, name):
        prefixes = ('yellow_line_', 'wall_', 'obstacle_')
        return name.startswith(prefixes)

    def namespace_for(self, name):
        if name.startswith('yellow_line_'):
            return 'yellow_line'
        if name.startswith('wall_'):
            return 'walls'
        return 'obstacles'

    def parse_pose(self, pose_text):
        values = [float(value) for value in (pose_text or '0 0 0 0 0 0').split()]
        values += [0.0] * (6 - len(values))
        return values[:6]

    def parse_geometry(self, geometry):
        if geometry is None:
            return None, None

        box = geometry.find('box')
        if box is not None:
            size = [float(value) for value in box.findtext('size').split()]
            return Marker.CUBE, size

        cylinder = geometry.find('cylinder')
        if cylinder is not None:
            radius = float(cylinder.findtext('radius'))
            length = float(cylinder.findtext('length'))
            return Marker.CYLINDER, [radius * 2.0, radius * 2.0, length]

        return None, None

    def parse_color(self, material, name):
        if material is not None:
            diffuse = material.findtext('diffuse') or material.findtext('ambient')
            if diffuse:
                color = [float(value) for value in diffuse.split()]
                color += [1.0] * (4 - len(color))
                return color[:4]

        if name.startswith('yellow_line_'):
            return [1.0, 0.85, 0.0, 1.0]
        if name.startswith('wall_'):
            return [0.7, 0.7, 0.7, 1.0]
        return [0.05, 0.05, 0.05, 1.0]

    def make_marker(self, marker_id, ns, marker_type, pose, scale, color):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        qx, qy, qz, qw = self.quaternion_from_rpy(pose[3], pose[4], pose[5])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    def quaternion_from_rpy(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        for marker in self.markers:
            marker.header.stamp = now
            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DemoWorldMarkers()
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
