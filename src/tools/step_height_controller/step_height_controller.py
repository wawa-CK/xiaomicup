#!/usr/bin/env python3
"""
Depth-triggered step-height controller.

Subscriptions:
  - ROS2 depth Image topic (configurable)

Publishes:
  - LCM `robot_control_cmd` messages to change `step_height` when an obstacle/step is detected.

Usage:
  - Edit `config.yaml` in the same folder to change one or two parameters per run.
  - Run: `python3 step_height_controller.py --config config.yaml`

Dependencies:
  - python3, numpy, rclpy (ROS2), lcm, pyyaml
  - The repo's generated LCM python types are used (robot_control_cmd_lcmt).
"""
import os
import sys
import time
import argparse
import yaml
import math

try:
    import rclpy
    from sensor_msgs.msg import Image
except Exception:
    rclpy = None

import numpy as np


def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


class StepHeightController:
    def __init__(self, cfg):
        self.cfg = cfg
        # Add repo lcm types to path
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
        lcm_py_path = os.path.join(repo_root, 'src', 'cyberdog_locomotion', 'common', 'lcm_type', 'lcm')
        sys.path.insert(0, lcm_py_path)

        import lcm
        import robot_control_cmd_lcmt

        self.lcm = lcm.LCM(cfg.get('lcm_url', "udpm://239.255.76.67:7671?ttl=255"))
        self.robot_msg_cls = robot_control_cmd_lcmt.robot_control_cmd_lcmt

        # ROS2 setup
        if rclpy is None:
            raise RuntimeError('rclpy not available. Install ROS2 python client (rclpy)')

        rclpy.init()
        self.node = rclpy.create_node('step_height_controller')
        self.depth_topic = cfg.get('depth_topic', '/camera/depth/image_raw')
        self.encoding = cfg.get('encoding', None)
        self.roi = cfg.get('roi', {'x': 0.45, 'y': 0.6, 'w': 0.1, 'h': 0.2})
        self.threshold = float(cfg.get('distance_threshold', 0.5))
        self.high_step = float(cfg.get('high_step', 0.06))
        self.normal_step = float(cfg.get('normal_step', 0.02))
        self.gait_id = int(cfg.get('gait_id', 26))
        self.mode = int(cfg.get('mode', 11))
        self.vel_des = cfg.get('vel_des', [0.0, 0.0, 0.0])
        self.cooldown = float(cfg.get('publish_cooldown', 1.0))
        self.last_publish = 0.0
        self.state_high = False

        self.node.get_logger().info(f'Depth topic: {self.depth_topic}, threshold: {self.threshold} m')

        self.sub = self.node.create_subscription(Image, self.depth_topic, self.image_cb, 10)

    def image_to_array(self, msg: Image):
        # Support common depth encodings
        if getattr(msg, 'encoding', None) == '32FC1' or (self.encoding == '32FC1'):
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        elif getattr(msg, 'encoding', None) == '16UC1' or (self.encoding == '16UC1'):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
        else:
            # Try to interpret as float32 by default
            try:
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            except Exception:
                raise RuntimeError(f'Unsupported image encoding: {getattr(msg, "encoding", None)}')
        return arr

    def image_cb(self, msg: Image):
        try:
            arr = self.image_to_array(msg)
        except Exception as e:
            self.node.get_logger().error(f'Cannot parse depth image: {e}')
            return

        h, w = arr.shape
        rx = int(self.roi['x'] * w)
        ry = int(self.roi['y'] * h)
        rw = max(1, int(self.roi['w'] * w))
        rh = max(1, int(self.roi['h'] * h))
        x0 = max(0, rx - rw // 2)
        y0 = max(0, ry - rh // 2)
        x1 = min(w, x0 + rw)
        y1 = min(h, y0 + rh)

        roi_arr = arr[y0:y1, x0:x1]
        # filter zeros and NaNs
        roi_valid = roi_arr[np.isfinite(roi_arr) & (roi_arr > 0.001)]
        if roi_valid.size == 0:
            return

        min_depth = float(np.percentile(roi_valid, 10))
        mean_depth = float(np.mean(roi_valid))

        now = time.time()
        trigger = min_depth < self.threshold

        # hysteresis to avoid flapping
        if trigger and not self.state_high:
            self.state_high = True
            self.publish_step(self.high_step)
            self.last_publish = now
        elif not trigger and self.state_high:
            # only lower when cooldown passed
            if now - self.last_publish > self.cooldown:
                self.state_high = False
                self.publish_step(self.normal_step)
                self.last_publish = now

    def publish_step(self, step_val: float):
        msg = self.robot_msg_cls()
        msg.mode = self.mode
        msg.gait_id = self.gait_id
        msg.life_count = (msg.life_count + 1) & 0xFF
        msg.vel_des = [float(self.vel_des[0]), float(self.vel_des[1]), float(self.vel_des[2])]
        msg.step_height = [float(step_val), float(step_val)]
        msg.duration = 0
        self.lcm.publish('robot_control_cmd', msg.encode())
        self.node.get_logger().info(f'Published step_height={step_val:.3f} gait_id={self.gait_id}')

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c', default=os.path.join(os.path.dirname(__file__), 'config.yaml'))
    args = parser.parse_args()
    cfg = load_config(args.config)
    ctrl = StepHeightController(cfg)
    ctrl.spin()


if __name__ == '__main__':
    main()
