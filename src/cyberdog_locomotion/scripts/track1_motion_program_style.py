#!/usr/bin/env python3
"""Timed motion-program style baseline for track 1.

This script intentionally avoids state-estimator distance gating. It publishes a
smooth timed sequence similar to the official motion-program examples so it can
be compared against track1_stone_path.py.
"""

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
LOCO_ROOT = SCRIPT_DIR.parent
LCM_TYPE_DIR = LOCO_ROOT / "common" / "lcm_type" / "lcm"
sys.path.insert(0, str(LCM_TYPE_DIR))

CMD_LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
CHANNEL = "robot_control_cmd"

K_QP_STAND = 3
K_LOCOMOTION = 11
K_RECOVERY_STAND = 12

GAIT_STAND = 1
GAIT_TROT_SLOW = 27
DEFAULT_IMAGE_TOPIC = "/cyberdog/rgb_camera/image_raw"
VISION_SEGMENTS = {"stone_slabs", "turn_approach", "pre_turn_creep"}

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

@dataclass(frozen=True)
class Segment:
    name: str
    duration: float
    mode: int
    gait_id: int
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    body_height: float = 0.0
    step_height_front: float = 0.05
    step_height_rear: float = 0.05

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

class VisionCorrection:
    def __init__(self, args, cv2, np, rclpy, Node, Image, qos_profile_sensor_data):
        self.cv2 = cv2
        self.np = np
        self.rclpy = rclpy
        self.node = Node("track1_yellow_correction")
        self.roi_top = args.vision_roi_top
        self.min_pixels = args.vision_min_pixels
        self.yaw_gain = args.vision_yaw_gain
        self.yaw_limit = args.vision_yaw_limit
        self.max_age = args.vision_max_age
        self.print_period = args.vision_print_period
        self.deadband = args.vision_deadband
        self.last_print = 0.0
        self.offset = 0.0
        self.pixel_count = 0
        self.state = "lost"
        self.stamp = 0.0
        self.lock = threading.Lock()
        self._spin_thread = None
        self.sub = self.node.create_subscription(
            Image,
            args.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.node.get_logger().info(f"subscribed image topic: {args.image_topic}")

    def on_image(self, msg):
        try:
            bgr = image_msg_to_bgr(msg, self.cv2, self.np)
        except ValueError as exc:
            self.node.get_logger().warn(str(exc))
            return

        height, width = bgr.shape[:2]
        y0 = int(height * self.roi_top)
        hsv = self.cv2.cvtColor(bgr[y0:, :], self.cv2.COLOR_BGR2HSV)
        lower = self.np.array([18, 70, 80], dtype=self.np.uint8)
        upper = self.np.array([40, 255, 255], dtype=self.np.uint8)
        mask = self.cv2.inRange(hsv, lower, upper)
        kernel = self.np.ones((5, 5), self.np.uint8)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_OPEN, kernel)
        mask = self.cv2.morphologyEx(mask, self.cv2.MORPH_CLOSE, kernel)

        _, xs = self.np.nonzero(mask)
        pixel_count = int(xs.size)
        now = time.monotonic()
        if pixel_count < self.min_pixels:
            with self.lock:
                self.pixel_count = pixel_count
                self.state = "lost"
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
        with self.lock:
            self.offset = float(offset)
            self.pixel_count = pixel_count
            self.state = state
            self.stamp = now

    def correction(self):
        with self.lock:
            age = time.monotonic() - self.stamp if self.stamp else float("inf")
            if self.state == "lost" or age > self.max_age:
                return 0.0
            offset = self.offset
            state = self.state
            pixels = self.pixel_count

        if abs(offset) < self.deadband:
            yaw = 0.0
        else:
            yaw = clamp(self.yaw_gain * offset, -self.yaw_limit, self.yaw_limit)
        now = time.monotonic()
        if now - self.last_print >= self.print_period:
            self.last_print = now
            print(f"[track1-vision] state={state:10s} offset={offset:+.3f} pixels={pixels:5d} yaw_corr={yaw:+.3f}")
        return yaw

    def spin_background(self):
        def spin_node():
            try:
                self.rclpy.spin(self.node)
            except Exception:
                pass

        self._spin_thread = threading.Thread(target=spin_node, daemon=True)
        self._spin_thread.start()
        return self._spin_thread

    def shutdown(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass

    def join(self, timeout: float = 1.0):
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=timeout)

def build_plan(args):
    return [
        Segment("recover_from_lie", args.recover_duration, K_RECOVERY_STAND, 0),
        Segment("settle_qp_stand", args.settle_duration, K_QP_STAND, GAIT_STAND, body_height=0.25),
        Segment(
            "stone_slabs",
            args.stone_duration,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.stone_speed,
            body_height=args.stone_body_height,
            step_height_front=args.stone_step_front,
            step_height_rear=args.stone_step_rear,
        ),
        Segment(
            "turn_approach",
            args.approach_duration,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.approach_speed,
            step_height_front=args.approach_step_front,
            step_height_rear=args.approach_step_rear,
        ),
        Segment(
            "pre_turn_creep",
            args.pre_turn_settle,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.turn_prep_speed,
            step_height_front=args.turn_step_front,
            step_height_rear=args.turn_step_rear,
        ),
        Segment(
            "turn_left_entry",
            args.turn_entry_duration,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.turn_entry_speed,
            yaw_rate=args.turn_entry_yaw_rate,
            step_height_front=args.turn_step_front,
            step_height_rear=args.turn_step_rear,
        ),
        Segment(
            "turn_left",
            args.turn_duration,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.turn_forward_speed,
            yaw_rate=args.turn_yaw_rate,
            step_height_front=args.turn_step_front,
            step_height_rear=args.turn_step_rear,
        ),
        Segment(
            "turn_left_exit",
            args.turn_exit_duration,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.turn_exit_speed,
            yaw_rate=args.turn_exit_yaw_rate,
            step_height_front=args.turn_step_front,
            step_height_rear=args.turn_step_rear,
        ),
        Segment(
            "post_turn_stabilize",
            args.post_turn_settle,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.post_turn_speed,
            step_height_front=args.curve_step_front,
            step_height_rear=args.curve_step_rear,
        ),
        Segment(
            "curve_clear",
            args.curve_duration,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=args.curve_speed,
            yaw_rate=args.curve_yaw_rate,
            step_height_front=args.curve_step_front,
            step_height_rear=args.curve_step_rear,
        ),
        Segment("hold_stand", args.final_hold, K_LOCOMOTION, GAIT_STAND),
    ]

def build_msg(lcm_type, segment: Segment, life_count: int):
    msg = lcm_type.robot_control_cmd_lcmt()
    msg.mode = segment.mode
    msg.gait_id = segment.gait_id
    msg.contact = 0x0F
    msg.life_count = life_count
    msg.vel_des[0] = segment.vx
    msg.vel_des[1] = segment.vy
    msg.vel_des[2] = segment.yaw_rate
    msg.pos_des[2] = segment.body_height
    msg.step_height[0] = segment.step_height_front
    msg.step_height[1] = segment.step_height_rear
    msg.duration = 0
    msg.value = 0
    return msg

def publish_segment(lc, lcm_type, segment: Segment, rate_hz: float, life_count: int, dry_run: bool, vision=None) -> int:
    ticks = max(1, int(segment.duration * rate_hz))
    print(
        f"[track1-program] {segment.name:18s} {segment.duration:5.1f}s "
        f"mode={segment.mode:2d} gait={segment.gait_id:2d} "
        f"vx={segment.vx:+.2f} vy={segment.vy:+.2f} yaw={segment.yaw_rate:+.2f} "
        f"step={segment.step_height_front:.3f}/{segment.step_height_rear:.3f}"
    )
    if dry_run:
        return (life_count + ticks) & 0x7F

    next_time = time.monotonic()
    for _ in range(ticks):
        life_count = (life_count + 1) & 0x7F
        active_segment = segment
        if vision is not None and segment.name in VISION_SEGMENTS:
            yaw_corr = vision.correction()
            active_segment = Segment(
                segment.name,
                segment.duration,
                segment.mode,
                segment.gait_id,
                vx=segment.vx,
                vy=segment.vy,
                yaw_rate=clamp(segment.yaw_rate + yaw_corr, -0.18, 0.18),
                body_height=segment.body_height,
                step_height_front=segment.step_height_front,
                step_height_rear=segment.step_height_rear,
            )
        msg = build_msg(lcm_type, active_segment, life_count)
        lc.publish(CHANNEL, msg.encode())
        next_time += 1.0 / rate_hz
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return life_count

def main() -> int:
    parser = argparse.ArgumentParser(description="Run a timed motion-program style track-1 baseline.")
    parser.add_argument("--rate", type=float, default=20.0, help="LCM publish rate in Hz.")
    parser.add_argument("--start-life", type=int, default=0, help="Initial life_count value.")
    parser.add_argument("--dry-run", action="store_true", help="Print the plan without publishing LCM.")
    vision_group = parser.add_mutually_exclusive_group()
    vision_group.add_argument("--use-vision", dest="use_vision", action="store_true", help="Use RGB yellow-border yaw correction on straight segments.")
    vision_group.add_argument("--no-vision", dest="use_vision", action="store_false", help="Disable RGB yellow-border yaw correction.")
    parser.set_defaults(use_vision=True)
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--vision-roi-top", type=float, default=0.45)
    parser.add_argument("--vision-min-pixels", type=int, default=500)
    parser.add_argument("--vision-yaw-gain", type=float, default=-0.10)
    parser.add_argument("--vision-yaw-limit", type=float, default=0.08)
    parser.add_argument("--vision-deadband", type=float, default=0.04)
    parser.add_argument("--vision-max-age", type=float, default=0.8)
    parser.add_argument("--vision-print-period", type=float, default=0.5)

    parser.add_argument("--recover-duration", type=float, default=5.0)
    parser.add_argument("--settle-duration", type=float, default=3.0)
    parser.add_argument("--stone-duration", type=float, default=23.15)
    parser.add_argument("--stone-speed", type=float, default=0.16)
    parser.add_argument("--stone-step-front", type=float, default=0.143)
    parser.add_argument("--stone-step-rear", type=float, default=0.1728)
    parser.add_argument("--stone-body-height", type=float, default=-0.01)
    parser.add_argument("--approach-duration", type=float, default=1.0)
    parser.add_argument("--approach-speed", type=float, default=0.2)
    parser.add_argument("--approach-step-front", type=float, default=0.1018)
    parser.add_argument("--approach-step-rear", type=float, default=0.1228)
    parser.add_argument("--pre-turn-settle", type=float, default=1.0)
    parser.add_argument("--turn-prep-speed", type=float, default=0.02)
    parser.add_argument("--turn-entry-duration", type=float, default=1.0)
    parser.add_argument("--turn-entry-speed", type=float, default=0.01)
    parser.add_argument("--turn-entry-yaw-rate", type=float, default=0.22)
    parser.add_argument("--turn-duration", type=float, default=4.5)
    parser.add_argument("--turn-yaw-rate", type=float, default=0.50)
    parser.add_argument("--turn-forward-speed", type=float, default=0.02)
    parser.add_argument("--turn-exit-duration", type=float, default=0.8)
    parser.add_argument("--turn-exit-yaw-rate", type=float, default=0.12)
    parser.add_argument("--turn-exit-speed", type=float, default=0.03)
    parser.add_argument("--turn-step-front", type=float, default=0.1018)
    parser.add_argument("--turn-step-rear", type=float, default=0.1228)
    parser.add_argument("--post-turn-settle", type=float, default=1.2)
    parser.add_argument("--post-turn-speed", type=float, default=0.04)
    parser.add_argument("--curve-duration", type=float, default=9.0)
    parser.add_argument("--curve-speed", type=float, default=0.12)
    parser.add_argument("--curve-yaw-rate", type=float, default=0.0)
    parser.add_argument("--curve-step-front", type=float, default=0.06)
    parser.add_argument("--curve-step-rear", type=float, default=0.06)
    parser.add_argument("--final-hold", type=float, default=120.0)
    args = parser.parse_args()

    lc = None
    lcm_type = None
    vision = None
    rclpy = None
    if not args.dry_run:
        try:
            import lcm
        except ImportError as exc:
            raise SystemExit("python3-lcm is required: sudo apt install python3-lcm") from exc
        try:
            import robot_control_cmd_lcmt as lcm_type
        except ImportError as exc:
            raise SystemExit(
                "LCM Python types were not found. Run:\n"
                "  cd src/cyberdog_locomotion/scripts && ./make_types.sh -c"
            ) from exc
        lc = lcm.LCM(CMD_LCM_URL)
        if args.use_vision:
            try:
                import cv2
                import numpy as np
                import rclpy
                from rclpy.node import Node
                from rclpy.qos import qos_profile_sensor_data
                from sensor_msgs.msg import Image
            except ImportError as exc:
                raise SystemExit(
                    "Missing ROS2/OpenCV Python dependency. Try:\n"
                    "  sudo apt update && sudo apt install -y python3-opencv ros-galactic-rclpy ros-galactic-sensor-msgs"
                ) from exc
            rclpy.init()
            vision = VisionCorrection(args, cv2, np, rclpy, Node, Image, qos_profile_sensor_data)
            vision.spin_background()

    print(
        f"[track1-program] channel={CHANNEL} url={CMD_LCM_URL} "
        f"rate={args.rate:.1f}Hz vision={'on' if args.use_vision else 'off'}"
    )
    life_count = args.start_life & 0x7F
    interrupted = False
    try:
        for segment in build_plan(args):
            life_count = publish_segment(lc, lcm_type, segment, args.rate, life_count, args.dry_run, vision)
    except KeyboardInterrupt:
        interrupted = True
        print("\n[track1-program] interrupted; sending stand command")
        if not args.dry_run:
            try:
                life_count = publish_segment(
                    lc,
                    lcm_type,
                    Segment("interrupt_stand", 1.0, K_LOCOMOTION, GAIT_STAND),
                    args.rate,
                    life_count,
                    False,
                    None,
                )
            except KeyboardInterrupt:
                pass
    finally:
        if rclpy is not None:
            rclpy.shutdown()
        if vision is not None:
            vision.join()
            vision.shutdown()
    if interrupted:
        print("[track1-program] stopped")
        return 130
    print("[track1-program] done")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
