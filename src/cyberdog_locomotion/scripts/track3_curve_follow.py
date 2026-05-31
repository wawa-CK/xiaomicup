#!/usr/bin/env python3
"""Lightweight curve follower for track 3 S-bend using RGB or fisheye topics."""

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List


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
STATE_LCM_URL = "udpm://239.255.76.67:7669?ttl=255"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def low_pass(old_value: float, new_value: float, alpha: float) -> float:
    return (1.0 - alpha) * old_value + alpha * new_value


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
    step_height: float = 0.05
    pitch: float = 0.0


@dataclass
class CurveSample:
    near_offset: float = 0.0
    far_offset: float = 0.0
    confidence: float = 0.0
    pixel_count: int = 0
    state: str = "lost"
    stamp: float = 0.0


@dataclass(frozen=True)
class RelativeWaypoint:
    name: str
    forward: float
    lateral: float
    speed: float
    radius: float


@dataclass(frozen=True)
class WorldWaypoint:
    name: str
    x: float
    y: float
    speed: float
    radius: float


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0  
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    stamp: float = 0.0
    ready: bool = False

    def update(self, msg) -> None:
        self.x = float(msg.p[0])
        self.y = float(msg.p[1])
        self.roll = float(msg.rpy[0])
        self.pitch = float(msg.rpy[1])
        self.yaw = float(msg.rpy[2])
        self.stamp = time.monotonic()
        self.ready = True


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


class CurveVision:
    def __init__(self, args, cv2, np, rclpy, Node, Image, qos_profile_sensor_data):
        self.cv2 = cv2
        self.np = np
        self.rclpy = rclpy
        self.node = Node("track3_curve_vision")
        self.topic = args.image_topic
        self.roi_top = args.vision_roi_top
        self.near_row = args.vision_near_row
        self.far_row = args.vision_far_row
        self.band_half_height = args.vision_band_half_height
        self.min_pixels = args.vision_min_pixels
        self.min_component_pixels = args.vision_component_min_pixels
        self.max_age = args.vision_max_age
        self.print_period = args.vision_print_period
        self.debug_mask = args.vision_debug_mask
        self.debug_save = args.vision_save_debug
        self.debug_save_period = max(1, args.vision_debug_save_period)
        self.debug_dir = Path(args.vision_debug_dir).expanduser()
        self.debug_dir.mkdir(parents=True, exist_ok=True)
        self.window_failed = False
        self.frame_index = 0
        self.last_print = 0.0
        self.lock = threading.Lock()
        self.sample = CurveSample()

        self.sub = self.node.create_subscription(
            Image,
            self.topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.node.get_logger().info(f"subscribed image topic: {self.topic}")

    def on_image(self, msg):
        self.frame_index += 1
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
        mask = self._keep_lowest_component(mask)

        roi_h, roi_w = mask.shape[:2]
        near_y = clamp(int(roi_h * self.near_row), 0, roi_h - 1)
        far_y = clamp(int(roi_h * self.far_row), 0, roi_h - 1)
        near_center, near_pixels = self._band_center(mask, near_y)
        far_center, far_pixels = self._band_center(mask, far_y)
        pixel_count = int(mask.sum() // 255)
        now = time.monotonic()

        if near_center is None or far_center is None or pixel_count < self.min_pixels:
            state = "lost"
            confidence = 0.0
            near_offset = 0.0
            far_offset = 0.0
        else:
            image_center = roi_w * 0.5
            near_offset = (near_center - image_center) / image_center
            far_offset = (far_center - image_center) / image_center
            confidence = min(1.0, (near_pixels + far_pixels) / max(1.0, self.min_pixels * 2.0))
            state = "tracking"

        with self.lock:
            self.sample = CurveSample(
                near_offset=float(near_offset),
                far_offset=float(far_offset),
                confidence=float(confidence),
                pixel_count=pixel_count,
                state=state,
                stamp=now,
            )

        if now - self.last_print >= self.print_period:
            self.last_print = now
            print(
                f"[track3-vision] state={state:8s} near={near_offset:+.3f} "
                f"far={far_offset:+.3f} conf={confidence:.2f} pixels={pixel_count:5d}"
            )

        overlay = None
        if self.debug_mask or self.debug_save:
            overlay = bgr.copy()
            self.cv2.rectangle(overlay, (0, y0), (width - 1, height - 1), (80, 80, 80), 1)
            self.cv2.line(overlay, (0, y0 + near_y), (width - 1, y0 + near_y), (0, 255, 0), 1)
            self.cv2.line(overlay, (0, y0 + far_y), (width - 1, y0 + far_y), (255, 0, 0), 1)

        if self.debug_mask and overlay is not None:
            if not self.window_failed:
                try:
                    self.cv2.imshow("track3_curve_follow", overlay)
                    self.cv2.imshow("track3_curve_mask", mask)
                    self.cv2.waitKey(1)
                except self.cv2.error:
                    self.window_failed = True
                    self.node.get_logger().warn(
                        f"OpenCV window unavailable, saving debug frames to: {self.debug_dir}"
                    )
        if self.debug_save and overlay is not None and self.frame_index % self.debug_save_period == 0:
            overlay_path = self.debug_dir / f"frame_{self.frame_index:06d}_overlay.png"
            mask_path = self.debug_dir / f"frame_{self.frame_index:06d}_mask.png"
            self.cv2.imwrite(str(overlay_path), overlay)
            self.cv2.imwrite(str(mask_path), mask)

    def _band_center(self, mask, center_y: int):
        top = max(0, center_y - self.band_half_height)
        bottom = min(mask.shape[0], center_y + self.band_half_height + 1)
        band = mask[top:bottom, :]
        ys, xs = self.np.nonzero(band)
        if xs.size == 0:
            return None, 0
        return 0.5 * (float(xs.min()) + float(xs.max())), int(xs.size)

    def _keep_lowest_component(self, mask):
        num_labels, labels, stats, _ = self.cv2.connectedComponentsWithStats(mask, 8)
        if num_labels <= 1:
            return mask

        width = mask.shape[1]
        image_center = width * 0.5
        best_label = 0
        best_score = -1.0
        for label in range(1, num_labels):
            area = int(stats[label, self.cv2.CC_STAT_AREA])
            if area < self.min_component_pixels:
                continue
            left = int(stats[label, self.cv2.CC_STAT_LEFT])
            top = int(stats[label, self.cv2.CC_STAT_TOP])
            comp_width = int(stats[label, self.cv2.CC_STAT_WIDTH])
            comp_height = int(stats[label, self.cv2.CC_STAT_HEIGHT])
            bottom = top + comp_height
            center_x = left + 0.5 * comp_width
            # Prefer the visible yellow region closest to the robot, then prefer centered ones.
            score = bottom * 10.0 + area * 0.02 - abs(center_x - image_center) * 0.5
            if score > best_score:
                best_score = score
                best_label = label

        if best_label == 0:
            return mask

        filtered = self.np.zeros_like(mask)
        filtered[labels == best_label] = 255
        return filtered

    def snapshot(self) -> CurveSample:
        with self.lock:
            return CurveSample(
                near_offset=self.sample.near_offset,
                far_offset=self.sample.far_offset,
                confidence=self.sample.confidence,
                pixel_count=self.sample.pixel_count,
                state=self.sample.state,
                stamp=self.sample.stamp,
            )

    def spin_background(self):
        def spin_node():
            try:
                self.rclpy.spin(self.node)
            except Exception:
                pass

        thread = threading.Thread(target=spin_node, daemon=True)
        thread.start()
        return thread

    def shutdown(self):
        self.node.destroy_node()


class StateMonitor:
    def __init__(self, rclpy, Node, state_lcm_type):
        self.rclpy = rclpy
        self.node = Node("track3_state_monitor")
        self.state_lcm_type = state_lcm_type
        self.lock = threading.Lock()
        self.state = RobotState()
        self.sub = None

    def attach(self, state_lc):
        self.sub = state_lc.subscribe("state_estimator", self._handle_state)

    def _handle_state(self, channel, data) -> None:
        msg = self.state_lcm_type.state_estimator_lcmt.decode(data)
        with self.lock:
            self.state.update(msg)

    def snapshot(self) -> RobotState:
        with self.lock:
            return RobotState(
                x=self.state.x,
                y=self.state.y,
                yaw=self.state.yaw,
                roll=self.state.roll,
                pitch=self.state.pitch,
                stamp=self.state.stamp,
                ready=self.state.ready,
            )


def build_msg(lcm_type, segment: Segment, life_count: int):
    msg = lcm_type.robot_control_cmd_lcmt()
    msg.mode = segment.mode
    msg.gait_id = segment.gait_id
    msg.contact = 0x0F
    msg.life_count = life_count
    msg.vel_des[0] = segment.vx
    msg.vel_des[1] = segment.vy
    msg.vel_des[2] = segment.yaw_rate
    msg.rpy_des[1] = segment.pitch
    msg.pos_des[2] = segment.body_height
    msg.step_height[0] = segment.step_height
    msg.step_height[1] = segment.step_height
    msg.duration = 0
    msg.value = 0
    return msg


def publish_hold(lc, lcm_type, segment: Segment, rate_hz: float, life_count: int, dry_run: bool) -> int:
    ticks = max(1, int(segment.duration * rate_hz))
    print(
        f"[track3] {segment.name:16s} {segment.duration:4.1f}s "
        f"vx={segment.vx:+.2f} vy={segment.vy:+.2f} yaw={segment.yaw_rate:+.2f}"
    )
    if dry_run:
        return (life_count + ticks) & 0x7F

    next_time = time.monotonic()
    for _ in range(ticks):
        life_count = (life_count + 1) & 0x7F
        lc.publish(CHANNEL, build_msg(lcm_type, segment, life_count).encode())
        next_time += 1.0 / rate_hz
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return life_count


def is_sample_valid(sample: CurveSample, max_age: float) -> bool:
    age = time.monotonic() - sample.stamp if sample.stamp else float("inf")
    return sample.state != "lost" and age <= max_age and sample.confidence > 0.05


def wait_for_state(state_lc, state: StateMonitor, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state_lc.handle_timeout(100)
        robot = state.snapshot()
        if robot.ready and time.monotonic() - robot.stamp <= 0.6:
            return robot
    return None


DEFAULT_STAGE3_WORLD_POINTS = [
    # Tuned from the user's measured Track 3 points:
    # P0 start ~= (-0.276, 4.310), then gradually enter the first right bend.
    ("s_entry_bias", 0.10, 4.85, 0.11, 0.28),
    ("s_right_arc_1", 0.42, 5.41, 0.12, 0.30),
    ("s_right_arc_2", 1.35, 5.58, 0.12, 0.30),
    ("s_right_arc_3", 2.27, 5.79, 0.12, 0.30),
    ("s_left_arc_1", 2.70, 6.28, 0.11, 0.28),
    ("s_left_arc_2", 2.99, 6.84, 0.10, 0.28),
    ("s_exit", 3.08, 6.95, 0.09, 0.26),
]

REFERENCE_STAGE3_START_X = -0.275993
REFERENCE_STAGE3_START_Y = 4.310233


def build_stage3_world_waypoints() -> List[WorldWaypoint]:
    return [
        WorldWaypoint(name=name, x=x, y=y, speed=speed, radius=radius)
        for name, x, y, speed, radius in DEFAULT_STAGE3_WORLD_POINTS
    ]


def build_stage3_shifted_world_waypoints(start: RobotState) -> List[WorldWaypoint]:
    shift_x = start.x - REFERENCE_STAGE3_START_X
    shift_y = start.y - REFERENCE_STAGE3_START_Y
    return [
        WorldWaypoint(
            name=name,
            x=x + shift_x,
            y=y + shift_y,
            speed=speed,
            radius=radius,
        )
        for name, x, y, speed, radius in DEFAULT_STAGE3_WORLD_POINTS
    ]


def build_relative_waypoints(args) -> List[RelativeWaypoint]:
    # Three-stage S template:
    # 1) right_turn: first right bend entrance
    # 2) mid_straight: short middle straight / transition
    # 3) left_exit: final left bend and exit alignment
    return [
        RelativeWaypoint("right_turn", args.right_turn_forward, args.right_turn_lateral, args.right_turn_speed, args.right_turn_radius),
        RelativeWaypoint("mid_straight", args.mid_straight_forward, args.mid_straight_lateral, args.mid_straight_speed, args.mid_straight_radius),
        RelativeWaypoint("left_exit", args.left_exit_forward, args.left_exit_lateral, args.left_exit_speed, args.left_exit_radius),
    ]


def build_world_waypoint(start: RobotState, item: RelativeWaypoint) -> WorldWaypoint:
    cos_yaw = math.cos(start.yaw)
    sin_yaw = math.sin(start.yaw)
    world_x = start.x + cos_yaw * item.forward - sin_yaw * item.lateral
    world_y = start.y + sin_yaw * item.forward + cos_yaw * item.lateral
    return WorldWaypoint(
        name=item.name,
        x=world_x,
        y=world_y,
        speed=item.speed,
        radius=item.radius,
    )


def vision_enabled_for_waypoint(waypoint_name: str, args) -> bool:
    if args.odom_only:
        return False
    if args.vision_straight_only:
        return waypoint_name == "mid_straight"
    return True


def blended_waypoint_target(current: WorldWaypoint, next_waypoint: WorldWaypoint, robot: RobotState, args):
    if next_waypoint is None:
        return current.x, current.y

    dx = next_waypoint.x - current.x
    dy = next_waypoint.y - current.y
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return current.x, current.y

    lookahead = min(args.path_lookahead, seg_len)
    ux = dx / seg_len
    uy = dy / seg_len
    path_x = current.x + ux * lookahead
    path_y = current.y + uy * lookahead

    dist_to_current = math.hypot(current.x - robot.x, current.y - robot.y)
    blend = 1.0 - clamp(dist_to_current / max(1e-6, args.path_blend_distance), 0.0, 1.0)
    blend = clamp(blend, 0.0, 1.0)
    return (
        current.x * (1.0 - blend) + path_x * blend,
        current.y * (1.0 - blend) + path_y * blend,
    )


def run_vision_fallback(lc, lcm_type, state_lc, rate_hz: float, life_count: int, dry_run: bool, vision: CurveVision, state: StateMonitor, args) -> int:
    ticks = max(1, int(args.follow_duration * rate_hz))
    filtered_yaw = 0.0
    filtered_vy = 0.0
    print(f"[track3] follow_curve      {args.follow_duration:4.1f}s mode=vision-fallback")
    if dry_run:
        return (life_count + ticks) & 0x7F

    target_yaw = state.snapshot().yaw
    next_time = time.monotonic()
    for _ in range(ticks):
        if state_lc is not None:
            state_lc.handle_timeout(0)
        sample = vision.snapshot()
        robot = state.snapshot()
        if not is_sample_valid(sample, args.vision_max_age):
            yaw_cmd = args.lost_yaw_rate
            vy_cmd = 0.0
            vx_cmd = args.lost_speed
        else:
            lateral_error = sample.near_offset
            heading_error = sample.far_offset - sample.near_offset
            yaw_error = wrap_angle(target_yaw - robot.yaw)
            yaw_raw = args.kp_lat * lateral_error + args.kp_head * heading_error
            yaw_raw += args.kp_imu * yaw_error
            vy_raw = args.kp_vy * lateral_error
            yaw_cmd = clamp(yaw_raw, -args.yaw_limit, args.yaw_limit)
            vy_cmd = clamp(vy_raw, -args.vy_limit, args.vy_limit)
            curve_strength = min(1.0, abs(heading_error) * args.curve_slow_gain)
            vx_cmd = args.base_speed - (args.base_speed - args.curve_speed) * curve_strength

        filtered_yaw = low_pass(filtered_yaw, yaw_cmd, args.filter_alpha)
        filtered_vy = low_pass(filtered_vy, vy_cmd, args.filter_alpha)

        life_count = (life_count + 1) & 0x7F
        seg = Segment(
            "follow_curve",
            0.0,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=vx_cmd,
            vy=filtered_vy,
            yaw_rate=filtered_yaw,
            body_height=args.body_height,
            step_height=args.step_height,
            pitch=args.body_pitch,
        )
        lc.publish(CHANNEL, build_msg(lcm_type, seg, life_count).encode())
        next_time += 1.0 / rate_hz
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return life_count


def run_curve_follow(lc, lcm_type, state_lc, rate_hz: float, life_count: int, dry_run: bool, vision: CurveVision, state: StateMonitor, args) -> int:
    ticks = max(1, int(args.follow_duration * rate_hz))
    filtered_yaw = 0.0
    filtered_vy = 0.0
    print(f"[track3] follow_curve      {args.follow_duration:4.1f}s mode=odom+imu+vision")
    if dry_run:
        return (life_count + ticks) & 0x7F

    start_state = wait_for_state(state_lc, state, args.state_wait_timeout)
    if start_state is None:
        if args.odom_only:
            print("[track3] state_estimator not ready, odom-only mode cannot continue")
            return life_count
        print("[track3] state_estimator not ready, fallback to vision-only mode")
        return run_vision_fallback(lc, lcm_type, state_lc, rate_hz, life_count, dry_run, vision, state, args)

    if args.use_world_path:
        if args.shift_world_path_to_start:
            waypoints = build_stage3_shifted_world_waypoints(start_state)
        else:
            waypoints = build_stage3_world_waypoints()
    else:
        templates = build_relative_waypoints(args)
        waypoint_idx = 0
        segment_start = start_state
        waypoints = [build_world_waypoint(segment_start, templates[waypoint_idx])]
    waypoint_idx = 0
    waypoint = waypoints[waypoint_idx]
    print(
        f"[track3-nav] enter waypoint={waypoint.name:8s} target=({waypoint.x:+.2f},{waypoint.y:+.2f}) "
        f"speed={waypoint.speed:.2f} radius={waypoint.radius:.2f}"
    )
    waypoint_start = time.monotonic()
    last_nav_print = 0.0
    next_time = time.monotonic()
    for _ in range(ticks):
        if state_lc is not None:
            state_lc.handle_timeout(0)
        sample = vision.snapshot()
        robot = state.snapshot()

        if (not robot.ready) or (time.monotonic() - robot.stamp > args.state_stale_timeout):
            if args.odom_only:
                print("[track3] state stale during odom-only path follow, stop handing over to exit_align")
                break
            print("[track3] state stale during path follow, fallback to cautious vision steering")
            return run_vision_fallback(lc, lcm_type, state_lc, rate_hz, life_count, dry_run, vision, state, args)

        dx = waypoint.x - robot.x
        dy = waypoint.y - robot.y
        distance = math.hypot(dx, dy)
        if distance <= waypoint.radius:
            print(f"[track3-nav] reached waypoint={waypoint.name} dist={distance:.2f}")
            waypoint_idx += 1
            if args.use_world_path:
                if waypoint_idx >= len(waypoints):
                    print("[track3-nav] final waypoint reached, handing over to exit_align")
                    break
                waypoint = waypoints[waypoint_idx]
                print(
                    f"[track3-nav] enter waypoint={waypoint.name:8s} target=({waypoint.x:+.2f},{waypoint.y:+.2f}) "
                    f"speed={waypoint.speed:.2f} radius={waypoint.radius:.2f}"
                )
                waypoint_start = time.monotonic()
                continue
            if waypoint_idx >= len(templates):
                print("[track3-nav] final waypoint reached, handing over to exit_align")
                break
            segment_start = robot
            waypoint = build_world_waypoint(segment_start, templates[waypoint_idx])
            print(
                f"[track3-nav] enter waypoint={waypoint.name:8s} target=({waypoint.x:+.2f},{waypoint.y:+.2f}) "
                f"speed={waypoint.speed:.2f} radius={waypoint.radius:.2f}"
            )
            waypoint_start = time.monotonic()
            continue

        if time.monotonic() - waypoint_start > args.waypoint_timeout:
            print(f"[track3-nav] timeout waypoint={waypoint.name} dist={distance:.2f}, forcing next phase")
            waypoint_idx += 1
            if args.use_world_path:
                if waypoint_idx >= len(waypoints):
                    print("[track3-nav] final waypoint timeout, handing over to exit_align")
                    break
                waypoint = waypoints[waypoint_idx]
                print(
                    f"[track3-nav] enter waypoint={waypoint.name:8s} target=({waypoint.x:+.2f},{waypoint.y:+.2f}) "
                    f"speed={waypoint.speed:.2f} radius={waypoint.radius:.2f}"
                )
                waypoint_start = time.monotonic()
                continue
            if waypoint_idx >= len(templates):
                print("[track3-nav] final waypoint timeout, handing over to exit_align")
                break
            segment_start = robot
            waypoint = build_world_waypoint(segment_start, templates[waypoint_idx])
            print(
                f"[track3-nav] enter waypoint={waypoint.name:8s} target=({waypoint.x:+.2f},{waypoint.y:+.2f}) "
                f"speed={waypoint.speed:.2f} radius={waypoint.radius:.2f}"
            )
            waypoint_start = time.monotonic()
            continue

        next_waypoint = waypoints[waypoint_idx + 1] if waypoint_idx + 1 < len(waypoints) else None
        aim_x, aim_y = blended_waypoint_target(waypoint, next_waypoint, robot, args)
        desired_yaw = math.atan2(aim_y - robot.y, aim_x - robot.x)
        heading_error = wrap_angle(desired_yaw - robot.yaw)
        yaw_cmd_goal = clamp(args.kp_goal * heading_error, -args.goal_yaw_limit, args.goal_yaw_limit)

        speed_scale = 1.0 - min(1.0, abs(heading_error) / args.heading_slow_angle) * args.heading_slow_factor
        pose_slow = min(0.45, abs(robot.roll) * args.roll_slow_gain + abs(robot.pitch) * args.pitch_slow_gain)
        vx_cmd = max(args.min_speed, waypoint.speed * speed_scale * (1.0 - pose_slow))

        yaw_assist = 0.0
        vy_cmd = 0.0
        use_vision = vision_enabled_for_waypoint(waypoint.name, args)
        if use_vision and is_sample_valid(sample, args.vision_max_age):
            vision_lat = sample.near_offset
            vision_head = sample.far_offset - sample.near_offset
            yaw_assist = args.kp_vision_lat * vision_lat + args.kp_vision_head * vision_head
            yaw_assist = clamp(yaw_assist, -args.vision_yaw_limit, args.vision_yaw_limit)
            vy_cmd = clamp(args.kp_vy * vision_lat, -args.vy_limit, args.vy_limit)
        elif use_vision and args.allow_blind_turn:
            yaw_assist = 0.0
        elif use_vision:
            vx_cmd = min(vx_cmd, args.lost_speed)

        yaw_roll = clamp(-robot.roll * args.kp_roll_yaw, -args.roll_yaw_limit, args.roll_yaw_limit)
        yaw_cmd = clamp(yaw_cmd_goal + yaw_assist + yaw_roll, -args.yaw_limit, args.yaw_limit)

        if time.monotonic() - last_nav_print >= args.nav_print_period:
            last_nav_print = time.monotonic()
            print(
                f"[track3-nav] wp={waypoint.name:8s} pos=({robot.x:+.2f},{robot.y:+.2f}) "
                f"target=({waypoint.x:+.2f},{waypoint.y:+.2f}) aim=({aim_x:+.2f},{aim_y:+.2f}) dist={distance:.2f} "
                f"yaw_err={heading_error:+.2f} vis={sample.state:8s} "
                f"vision_ctl={'on' if use_vision else 'off'}"
            )

        filtered_yaw = low_pass(filtered_yaw, yaw_cmd, args.filter_alpha)
        filtered_vy = low_pass(filtered_vy, vy_cmd, args.filter_alpha)

        life_count = (life_count + 1) & 0x7F
        seg = Segment(
            "follow_curve",
            0.0,
            K_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=vx_cmd,
            vy=filtered_vy,
            yaw_rate=filtered_yaw,
            body_height=args.body_height,
            step_height=args.step_height,
            pitch=args.body_pitch,
        )
        lc.publish(CHANNEL, build_msg(lcm_type, seg, life_count).encode())
        next_time += 1.0 / rate_hz
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return life_count


def wrap_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="Track 3 lightweight fisheye/RGB curve follower.")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--start-life", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    # Vision ROI: larger values mean "look lower" in the image, closer to the feet/front edge.
    parser.add_argument("--vision-roi-top", type=float, default=0.91)
    parser.add_argument("--vision-near-row", type=float, default=0.94)
    parser.add_argument("--vision-far-row", type=float, default=0.86)
    parser.add_argument("--vision-band-half-height", type=int, default=4)
    parser.add_argument("--vision-min-pixels", type=int, default=220)
    parser.add_argument("--vision-component-min-pixels", type=int, default=80)
    parser.add_argument("--vision-max-age", type=float, default=1.2)
    parser.add_argument("--vision-print-period", type=float, default=0.4)
    parser.add_argument("--vision-debug-mask", action="store_true")
    parser.add_argument("--vision-save-debug", action="store_true", default=True)
    parser.add_argument("--vision-debug-dir", default=str(SCRIPT_DIR / "track3_debug"))
    parser.add_argument("--vision-debug-save-period", type=int, default=8)
    parser.add_argument("--kp-lat", type=float, default=0.22)
    parser.add_argument("--kp-head", type=float, default=0.32)
    parser.add_argument("--kp-imu", type=float, default=0.55)
    parser.add_argument("--kp-vy", type=float, default=0.05)
    parser.add_argument("--kp-goal", type=float, default=1.15)
    parser.add_argument("--kp-vision-lat", type=float, default=0.10)
    parser.add_argument("--kp-vision-head", type=float, default=0.14)
    parser.add_argument("--kp-roll-yaw", type=float, default=0.12)
    parser.add_argument("--yaw-limit", type=float, default=0.22)
    parser.add_argument("--goal-yaw-limit", type=float, default=0.24)
    parser.add_argument("--vision-yaw-limit", type=float, default=0.12)
    parser.add_argument("--roll-yaw-limit", type=float, default=0.05)
    parser.add_argument("--vy-limit", type=float, default=0.05)
    parser.add_argument("--filter-alpha", type=float, default=0.35)
    parser.add_argument("--base-speed", type=float, default=0.16)
    parser.add_argument("--curve-speed", type=float, default=0.10)
    parser.add_argument("--curve-slow-gain", type=float, default=2.0)
    parser.add_argument("--lost-speed", type=float, default=0.06)
    parser.add_argument("--lost-yaw-rate", type=float, default=0.10)
    # Body pose: lower body and slight nose-down help the camera see nearer ground.
    parser.add_argument("--body-height", type=float, default=-0.02)
    parser.add_argument("--body-pitch", type=float, default=0.05)
    parser.add_argument("--step-height", type=float, default=0.055)
    parser.add_argument("--state-wait-timeout", type=float, default=5.0)
    parser.add_argument("--state-stale-timeout", type=float, default=0.6)
    parser.add_argument("--waypoint-timeout", type=float, default=12.0)
    parser.add_argument("--heading-slow-angle", type=float, default=0.55)
    parser.add_argument("--heading-slow-factor", type=float, default=0.55)
    parser.add_argument("--roll-slow-gain", type=float, default=0.70)
    parser.add_argument("--pitch-slow-gain", type=float, default=0.45)
    parser.add_argument("--min-speed", type=float, default=0.08)
    parser.add_argument("--allow-blind-turn", action="store_true")
    parser.add_argument("--nav-print-period", type=float, default=0.6)
    parser.add_argument("--odom-only", action="store_true")
    parser.add_argument("--vision-straight-only", action="store_true", default=True)
    parser.add_argument("--use-world-path", action="store_true", default=True)
    parser.add_argument("--shift-world-path-to-start", action="store_true", default=True)
    parser.add_argument("--path-lookahead", type=float, default=0.22)
    parser.add_argument("--path-blend-distance", type=float, default=0.95)
    # Stage 1: first right bend. If the robot cuts inside too early,
    # increase forward and make lateral less negative.
    parser.add_argument("--right-turn-forward", type=float, default=1.18)
    parser.add_argument("--right-turn-lateral", type=float, default=-0.42)
    parser.add_argument("--right-turn-speed", type=float, default=0.12)
    parser.add_argument("--right-turn-radius", type=float, default=0.22)
    # Stage 2: short middle straight. Increase forward to keep going before the left exit.
    parser.add_argument("--mid-straight-forward", type=float, default=1.89)
    parser.add_argument("--mid-straight-lateral", type=float, default=0.00)
    parser.add_argument("--mid-straight-speed", type=float, default=0.14)
    parser.add_argument("--mid-straight-radius", type=float, default=0.20)
    # Stage 3: final left bend and exit. Increase lateral to turn more left.
    parser.add_argument("--left-exit-forward", type=float, default=0.92)
    parser.add_argument("--left-exit-lateral", type=float, default=0.88)
    parser.add_argument("--left-exit-speed", type=float, default=0.12)
    parser.add_argument("--left-exit-radius", type=float, default=0.26)
    parser.add_argument("--exit-cruise-speed", type=float, default=0.12)
    parser.add_argument("--recover-duration", type=float, default=7.0)
    parser.add_argument("--settle-duration", type=float, default=2.0)
    parser.add_argument("--follow-duration", type=float, default=45.0)
    parser.add_argument("--exit-duration", type=float, default=0.5)
    parser.add_argument("--exit-speed", type=float, default=0.12)
    args = parser.parse_args()

    lc = None
    lcm_type = None
    state_lc = None
    state_type = None
    vision = None
    state = None
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
        try:
            import cv2
            import numpy as np
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
            import state_estimator_lcmt as state_type
        except ImportError as exc:
            raise SystemExit(
                "Missing ROS2/OpenCV Python dependency. Try:\n"
                "  sudo apt update && sudo apt install -y python3-opencv ros-galactic-rclpy ros-galactic-sensor-msgs"
            ) from exc
        lc = lcm.LCM(CMD_LCM_URL)
        state_lc = lcm.LCM(STATE_LCM_URL)
        rclpy.init()
        vision = CurveVision(args, cv2, np, rclpy, Node, Image, qos_profile_sensor_data)
        vision.spin_background()
        state = StateMonitor(rclpy, Node, state_type)
        state.attach(state_lc)

    print(f"[track3] channel={CHANNEL} url={CMD_LCM_URL} rate={args.rate:.1f}Hz mode=curve-follow")
    life_count = args.start_life & 0x7F
    try:
        life_count = publish_hold(
            lc,
            lcm_type,
            Segment("recover_from_lie", args.recover_duration, K_RECOVERY_STAND, 0),
            args.rate,
            life_count,
            args.dry_run,
        )
        life_count = publish_hold(
            lc,
            lcm_type,
            Segment("settle_qp_stand", args.settle_duration, K_QP_STAND, GAIT_STAND, body_height=0.25),
            args.rate,
            life_count,
            args.dry_run,
        )
        if vision is not None:
            life_count = run_curve_follow(lc, lcm_type, state_lc, args.rate, life_count, args.dry_run, vision, state, args)
        life_count = publish_hold(
            lc,
            lcm_type,
            Segment("exit_align", args.exit_duration, K_LOCOMOTION, GAIT_TROT_SLOW, vx=args.exit_speed, step_height=args.step_height),
            args.rate,
            life_count,
            args.dry_run,
        )
    except KeyboardInterrupt:
        print("\n[track3] interrupted")
        return 130
    finally:
        if vision is not None:
            vision.shutdown()
        if state is not None:
            state.node.destroy_node()
        if rclpy is not None:
            rclpy.shutdown()
    print("[track3] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
  
