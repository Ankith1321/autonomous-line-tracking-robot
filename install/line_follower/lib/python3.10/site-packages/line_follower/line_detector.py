#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")

        # Subscribing to the camera image
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.on_image,
            10,
        )

        # Publishing line error (pixels, signed)
        self.err_pub = self.create_publisher(Float32, "/line_error", 10)

        self.bridge = CvBridge()
        self.get_logger().info("LineDetector started: listening to camera images")

        # Always publish something so downstream controller is alive
        self._last_error = -1.0
        self._publish_period = 0.1  # 10 Hz
        self._publish_timer = self.create_timer(self._publish_period, self._publish_latest_error)

    def _publish_latest_error(self):
        msg = Float32()
        msg.data = float(self._last_error)
        self.err_pub.publish(msg)

    def on_image(self, msg: Image):
        # Convert ROS Image -> OpenCV image (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h, w, _ = frame.shape

        # Use only the bottom part of the image (where the line is usually visible)
        roi = frame[int(h * 0.50) : h, 0:w]

        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Threshold for a dark line on bright floor (adjust if needed)
        _, mask = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY_INV)

        # Compute centroid using image moments
        M = cv2.moments(mask)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])  # centroid x in ROI
            # error = centroid - image_center (positive means line is to the right)
            error = float(cx - (w / 2.0))
            self._last_error = error
        else:
            # No line detected -> publish -1.0 so controller stops safely
            self._last_error = -1.0


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
