#!/usr/bin/env python3
"""Debug yellow border detection from the simulated RGB camera."""

import argparse
import time


DEFAULT_IMAGE_TOPIC = "/cyberdog/rgb_camera/image_raw"


def image_msg_to_bgr(msg, cv2, np):
    encoding = msg.encoding.lower()
    width = msg.width
    height = msg.height

    if encoding in ("rgb8", "bgr8"):
        channels = 3
    elif encoding in ("rgba8", "bgra8"):
        channels = 4
    else:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, msg.step)
    image = row[:, : width * channels].reshape(height, width, channels)

    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


class YellowBorderNode:
    def __init__(self, rclpy, Node, Image, qos_profile_sensor_data, cv2, np, args):
        self.rclpy = rclpy
        self.cv2 = cv2
        self.np = np
        self.node = Node("yellow_border_debug")
        self.topic = args.image_topic
        self.show = args.show
        self.roi_top = args.roi_top
        self.min_pixels = args.min_pixels
        self.print_period = args.print_period
        self.last_print = 0.0

        self.sub = self.node.create_subscription(
            Image,
            self.topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.node.get_logger().info(f"subscribed image topic: {self.topic}")

    def on_image(self, msg):
        try:
            bgr = image_msg_to_bgr(msg, self.cv2, self.np)
        except ValueError as exc:
            self.node.get_logger().warn(str(exc))
            return

        height, width = bgr.shape[:2]
        y0 = int(height * self.roi_top)
        roi = bgr[y0:, :]
        hsv = self.cv2.cvtColor(roi, self.cv2.COLOR_BGR2HSV)

        lower = self.np.array([18, 70, 80], dtype=self.np.uint8)
        upper = self.np.array([40, 255, 255], dtype=self.np.uint8)
        mask = self.cv2.inRange(hsv, lower, upper)
        kernel = self.np.ones((5, 5), self.np.uint8)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_OPEN, kernel)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_CLOSE, kernel)

        _, xs = self.np.nonzero(mask)
        now = time.monotonic()
        if now - self.last_print >= self.print_period:
            self.last_print = now
            self.print_detection(width, xs)

        if self.show:
            self.show_overlay(bgr, mask, y0, xs)

    def print_detection(self, width, xs):
        pixel_count = int(xs.size)
        if pixel_count < self.min_pixels:
            print(f"[yellow] lost pixels={pixel_count}")
            return

        left = xs[xs < width * 0.5]
        right = xs[xs >= width * 0.5]
        left_x = float(self.np.median(left)) if left.size else None
        right_x = float(self.np.median(right)) if right.size else None

        if left_x is not None and right_x is not None:
            lane_center = 0.5 * (left_x + right_x)
            state = "both"
        elif left_x is not None:
            lane_center = left_x
            state = "left_only"
        else:
            lane_center = right_x
            state = "right_only"

        offset = (lane_center - width * 0.5) / (width * 0.5)
        print(
            f"[yellow] state={state:10s} offset={offset:+.3f} "
            f"pixels={pixel_count:5d} left={left_x} right={right_x}"
        )

    def show_overlay(self, bgr, mask, y0, xs):
        overlay = bgr.copy()
        overlay[y0:, :, 1] = self.np.maximum(overlay[y0:, :, 1], mask)
        height, width = overlay.shape[:2]
        self.cv2.line(overlay, (width // 2, y0), (width // 2, height - 1), (255, 0, 0), 1)
        if xs.size:
            cx = int(self.np.median(xs))
            self.cv2.line(overlay, (cx, y0), (cx, height - 1), (0, 0, 255), 1)
        self.cv2.imshow("yellow_border_debug", overlay)
        self.cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser(description="Detect yellow track borders from RGB camera images.")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--roi-top", type=float, default=0.45, help="Top crop ratio for ground ROI.")
    parser.add_argument("--min-pixels", type=int, default=500, help="Minimum yellow pixels before reporting lost.")
    parser.add_argument("--print-period", type=float, default=0.3, help="Seconds between debug prints.")
    parser.add_argument("--show", action="store_true", help="Show an OpenCV debug window.")
    args = parser.parse_args()

    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise SystemExit(
            "Missing ROS2/OpenCV Python dependency. In the container, try:\n"
            "  sudo apt update && sudo apt install -y python3-opencv ros-galactic-rclpy ros-galactic-sensor-msgs"
        ) from exc

    rclpy.init()
    detector = YellowBorderNode(rclpy, Node, Image, qos_profile_sensor_data, cv2, np, args)
    try:
        rclpy.spin(detector.node)
    except KeyboardInterrupt:
        pass
    finally:
        detector.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
