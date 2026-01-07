#!/usr/bin/env python3

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge


class LineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")

        # ---- Parameters (tune without code changes later) ----
        self.declare_parameter("roi_start", 0.60)          # bottom 40% by default
        self.declare_parameter("line_is_dark", True)       # True: dark line on bright floor
        self.declare_parameter("use_adaptive", True)       # adaptive thresholding
        self.declare_parameter("adaptive_block", 31)       # must be odd
        self.declare_parameter("adaptive_c", 5)            # subtract constant
        self.declare_parameter("fixed_thresh", 140)        # used if use_adaptive=False
        self.declare_parameter("kernel_size", 5)           # morphology kernel size (odd recommended)

        self._roi_start = float(self.get_parameter("roi_start").value)
        self._line_is_dark = bool(self.get_parameter("line_is_dark").value)
        self._use_adaptive = bool(self.get_parameter("use_adaptive").value)
        self._adaptive_block = int(self.get_parameter("adaptive_block").value)
        self._adaptive_c = int(self.get_parameter("adaptive_c").value)
        self._fixed_thresh = int(self.get_parameter("fixed_thresh").value)
        self._kernel_size = int(self.get_parameter("kernel_size").value)

        # Enforce adaptive block odd and >= 3
        if self._adaptive_block < 3:
            self._adaptive_block = 3
        if self._adaptive_block % 2 == 0:
            self._adaptive_block += 1

        # ---- ROS I/O ----
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.on_image,
            10,
        )

        self.err_pub = self.create_publisher(Float32, "/line_error", 10)

        # Debug outputs
        self.mask_pub = self.create_publisher(Image, "/line_mask", 10)          # mono8 debug mask
        self.nonzero_pub = self.create_publisher(Int32, "/line_mask_nonzero", 10)

        self.bridge = CvBridge()
        self.get_logger().info(
            "LineDetector started. Subscribed: /camera/image_raw, Publishing: /line_error, /line_mask, /line_mask_nonzero"
        )

        # Publish a continuous stream for downstream controller
        self._last_error = -1.0
        self._last_nonzero = 0
        self._publish_timer = self.create_timer(0.1, self._publish_latest)  # 10 Hz

        # Throttled log
        self._last_log_t = 0.0

    def _publish_latest(self):
        self.err_pub.publish(Float32(data=float(self._last_error)))
        self.nonzero_pub.publish(Int32(data=int(self._last_nonzero)))

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w, _ = frame.shape

        # ROI
        roi_start_px = int(h * self._roi_start)
        roi_start_px = max(0, min(h - 1, roi_start_px))
        roi = frame[roi_start_px:h, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Thresholding
        if self._use_adaptive:
            mask = cv2.adaptiveThreshold(
                gray_blur,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV if self._line_is_dark else cv2.THRESH_BINARY,
                self._adaptive_block,
                self._adaptive_c,
            )
        else:
            # Fixed threshold
            mode = cv2.THRESH_BINARY_INV if self._line_is_dark else cv2.THRESH_BINARY
            _, mask = cv2.threshold(gray_blur, self._fixed_thresh, 255, mode)

        # Morphology cleanup
        k = max(1, self._kernel_size)
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Debug: publish mask
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except Exception:
            pass

        # Mask stats
        nonzero = int(cv2.countNonZero(mask))
        self._last_nonzero = nonzero

        # Compute centroid
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            self._last_error = float(cx - (w / 2.0))
        else:
            self._last_error = -1.0

        # Throttled log (once per second)
        now = time.monotonic()
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            self.get_logger().info(
                f"roi_start={self._roi_start:.2f}, line_is_dark={self._line_is_dark}, "
                f"use_adaptive={self._use_adaptive}, nonzero={nonzero}, last_error={self._last_error:.1f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
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
