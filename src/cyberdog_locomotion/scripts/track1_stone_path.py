#!/usr/bin/env python3
"""Closed-loop controller for track 1: stone path.

Run this from the cyberdog_sim workspace after the simulator and controller are
up. By default the script subscribes to state_estimator and drives by distance
and yaw targets. Use --open-loop to run the original timed baseline.
"""

import argparse
import math
import select
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
LOCO_ROOT = SCRIPT_DIR.parent
LCM_TYPE_DIR = LOCO_ROOT / "common" / "lcm_type" / "lcm"
sys.path.insert(0, str(LCM_TYPE_DIR))


CMD_LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
STATE_LCM_URL = "udpm://239.255.76.67:7669?ttl=255"
CHANNEL = "robot_control_cmd"
STATE_CHANNEL = "state_estimator"

K_QP_STAND = 3
K_LOCOMOTION = 11
K_RECOVERY_STAND = 12

GAIT_STAND = 1
GAIT_WALK = 6
GAIT_TROT_10_4 = 5
GAIT_TROT_10_5 = 9
GAIT_TROT_FAST = 10
GAIT_TROT_SLOW = 27

DRIVE_YAW_KP = 0.75
DRIVE_YAW_LIMIT = 0.32
DRIVE_LATERAL_KP = 0.65
DRIVE_LATERAL_LIMIT = 0.08
TURN_YAW_KP = 0.80
TURN_YAW_LIMIT = 0.28
TURN_FORWARD_SPEED = 0.02
POSE_ROLL_LIMIT = math.radians(32.0)
POSE_PITCH_LIMIT = math.radians(28.0)
PROGRESS_LOG_INTERVAL = 0.5


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


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    height: float = 0.0
    stamp: float = 0.0
    ready: bool = False

    def update(self, msg) -> None:
        self.x = float(msg.p[0])
        self.y = float(msg.p[1])
        self.height = float(msg.p[2])
        self.roll = float(msg.rpy[0])
        self.pitch = float(msg.rpy[1])
        self.yaw = float(msg.rpy[2])
        self.stamp = time.monotonic()
        self.ready = True


@dataclass(frozen=True)
class Waypoint:
    name: str
    distance: float
    speed: float
    gait_id: int = GAIT_TROT_SLOW
    yaw_delta: float = 0.0
    yaw_rate: float = 0.0
    lateral_offset: float = 0.0
    lateral_kp: float = DRIVE_LATERAL_KP
    lateral_limit: float = DRIVE_LATERAL_LIMIT
    step_height: float = 0.055
    body_height: float = 0.0
    yaw_kp: float = DRIVE_YAW_KP
    yaw_limit: float = DRIVE_YAW_LIMIT
    min_speed_scale: float = 0.30
    stall_timeout: float = 2.0
    stall_distance: float = 0.025
    stall_settle: bool = True
    timeout: float = 20.0
    min_duration: float = 0.0


@dataclass(frozen=True)
class ControlParams:
    drive_yaw_kp: float = DRIVE_YAW_KP
    drive_yaw_limit: float = DRIVE_YAW_LIMIT
    drive_lateral_kp: float = DRIVE_LATERAL_KP
    drive_lateral_limit: float = DRIVE_LATERAL_LIMIT
    turn_yaw_kp: float = TURN_YAW_KP
    turn_yaw_limit: float = TURN_YAW_LIMIT
    turn_forward_speed: float = TURN_FORWARD_SPEED
    turn_yaw_tolerance: float = math.radians(4.0)
    pose_roll_limit: float = POSE_ROLL_LIMIT
    pose_pitch_limit: float = POSE_PITCH_LIMIT


     
        if self.dry_run:
            self.life_count = (self.life_count + ticks) & 0x7F
            return
        next_time = time.monotonic()
        for _ in range(ticks):
            self.pump(0.0)
            self.publish(segment)
            next_time += 1.0 / self.rate_hz
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def drive_distance(self, waypoint: Waypoint) -> bool:
        print(
            f"[track1] {waypoint.name:16s} dist={waypoint.distance:.2f}m "
            f"speed={waypoint.speed:.2f} gait={waypoint.gait_id:2d} "
            f"step={waypoint.step_height:.3f} yaw_delta={math.degrees(waypoint.yaw_delta):+.1f}deg "
            f"lat={waypoint.lateral_offset:+.2f}m"
        )
        if self.dry_run:
            return True

        start_x = self.state.x
        start_y = self.state.y
        target_yaw = wrap_angle(self.state.yaw + waypoint.yaw_delta)
        forward_x = math.cos(target_yaw)
        forward_y = math.sin(target_yaw)
        left_x = -math.sin(target_yaw)
        left_y = math.cos(target_yaw)
        deadline = time.monotonic() + waypoint.timeout
        last_progress_dist = 0.0
        last_progress_time = time.monotonic()
        next_time = time.monotonic()
        next_log_time = time.monotonic() + PROGRESS_LOG_INTERVAL
        start_time = time.monotonic()
        traveled = 0.0
        completed = False
        while time.monotonic() < deadline:
            self.pump(0.0)
            now = time.monotonic()
            elapsed = now - start_time
            dx = self.state.x - start_x
            dy = self.state.y - start_y
            traveled = dx * forward_x + dy * forward_y
            lateral_error = waypoint.lateral_offset - (dx * left_x + dy * left_y)
            if self.pose_unstable():
                print(f"[track1] warning: pose unstable during {waypoint.name}, recovery stand")
                self.recover_and_settle()
                last_progress_dist = traveled
                last_progress_time = time.monotonic()
            if traveled >= waypoint.distance and elapsed >= waypoint.min_duration:
                completed = True
                break
            if traveled - last_progress_dist >= waypoint.stall_distance:
                last_progress_dist = traveled
                last_progress_time = time.monotonic()
            elif time.monotonic() - last_progress_time > waypoint.stall_timeout:
                print(f"[track1] warning: {waypoint.name} stalled")
                if waypoint.stall_settle:
                    self.hold(Segment("stall_settle", 0.0, K_QP_STAND, GAIT_STAND, body_height=0.25), 0.8)
                last_progress_dist = traveled
                last_progress_time = time.monotonic()
            yaw_error = wrap_angle(target_yaw - self.state.yaw)
            yaw_cmd = clamp(waypoint.yaw_kp * yaw_error + waypoint.yaw_rate, -waypoint.yaw_limit, waypoint.yaw_limit)
            vy_cmd = clamp(waypoint.lateral_kp * lateral_error, -waypoint.lateral_limit, waypoint.lateral_limit)
            speed_scale = clamp((waypoint.distance - traveled) / 0.45, waypoint.min_speed_scale, 1.0)
            if now >= next_log_time:
                print(
                    f"[track1] {waypoint.name:16s} progress={traveled:.2f}/{waypoint.distance:.2f}m "
                    f"time={elapsed:.1f}/{waypoint.min_duration:.1f}s "
                    f"lat_err={lateral_error:+.2f}m yaw_err={math.degrees(yaw_error):+.1f}deg "
                    f"cmd=(vx={waypoint.speed * speed_scale:+.2f}, vy={vy_cmd:+.2f}, yaw={yaw_cmd:+.2f}) "
                    f"rpy=({math.degrees(self.state.roll):+.1f},{math.degrees(self.state.pitch):+.1f})deg"
                )
                next_log_time = now + PROGRESS_LOG_INTERVAL
            seg = Segment(
                waypoint.name,
                0.0,
                K_LOCOMOTION,
                waypoint.gait_id,
                vx=waypoint.speed * speed_scale,
                vy=vy_cmd,
                yaw_rate=yaw_cmd,
                body_height=waypoint.body_height,
                step_height=waypoint.step_height,
            )
            self.publish(seg)
            next_time += 1.0 / self.rate_hz
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
        else:
            print(f"[track1] warning: {waypoint.name} timed out")
        elapsed = time.monotonic() - start_time
        status = "complete" if completed else "blocked"
        print(
            f"[track1] {waypoint.name:16s} end progress={traveled:.2f}/{waypoint.distance:.2f}m "
            f"time={elapsed:.1f}s status={status}"
        )
        return completed

    def turn_yaw(self, name: str, yaw_delta: float, timeout: float = 12.0) -> None:
        print(f"[track1] {name:16s} turn={math.degrees(yaw_delta):+.1f}deg")
        if self.dry_run:
            return
        target = wrap_angle(self.state.yaw + yaw_delta)
        deadline = time.monotonic() + timeout
        next_time = time.monotonic()
        while time.monotonic() < deadline:
            self.pump(0.0)
            if self.pose_unstable():
                print(f"[track1] warning: pose unstable during {name}, recovery stand")
                self.recover_and_settle()
                target = wrap_angle(self.state.yaw + yaw_delta)
            error = wrap_angle(target - self.state.yaw)
            if abs(error) < self.control.turn_yaw_tolerance:
                break
            yaw_cmd = clamp(self.control.turn_yaw_kp * error, -self.control.turn_yaw_limit, self.control.turn_yaw_limit)
            self.publish(
                Segment(
                    name,
                    0.0,
                    K_LOCOMOTION,
                    GAIT_TROT_SLOW,
                    vx=self.control.turn_forward_speed,
                    yaw_rate=yaw_cmd,
                    step_height=0.06,
                )
            )
            next_time += 1.0 / self.rate_hz
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
        else:
            print(f"[track1] warning: {name} timed out")

    def pose_unstable(self) -> bool:
        if not self.state.ready:
            return False
        return abs(self.state.roll) > self.control.pose_roll_limit or abs(self.state.pitch) > self.control.pose_pitch_limit

    def recover_and_settle(self) -> None:
        self.hold(Segment("recover_guard", 0.0, K_RECOVERY_STAND, 0), 3.0)
        self.hold(Segment("settle_guard", 0.0, K_QP_STAND, GAIT_STAND, body_height=0.25), 1.0)


# Tune these first in the standard Ubuntu 20.04 + Docker/Gazebo environment.
# Keep stone_entry and stone_exit slow enough that all four feet clear the slabs.
TRACK1_PLAN = [
    Segment("recover_from_lie", 5.0, K_RECOVERY_STAND, 0),
    Segment("settle_qp_stand", 1.5, K_QP_STAND, GAIT_STAND, body_height=0.24),
    Segment("enter_stones", 2.5, K_LOCOMOTION, GAIT_TROT_SLOW, vx=0.18, step_height=0.08),
    Segment("cross_stones", 6.0, K_LOCOMOTION, GAIT_TROT_SLOW, vx=0.22, step_height=0.06),
    Segment("pre_curve_align", 1.5, K_LOCOMOTION, GAIT_TROT_SLOW, vx=0.16, vy=-0.02, step_height=0.055),
    Segment("curve_left", 4.2, K_LOCOMOTION, GAIT_TROT_SLOW, vx=0.15, yaw_rate=0.38, step_height=0.055),
    Segment("curve_exit", 2.0, K_LOCOMOTION, GAIT_TROT_SLOW, vx=0.18, yaw_rate=0.10, step_height=0.05),
    Segment("finish_line", 1.8, K_LOCOMOTION, GAIT_TROT_SLOW, vx=0.20, step_height=0.05),
    Segment("hold_stand", 2.0, K_LOCOMOTION, GAIT_STAND, step_height=0.05),
]

# Track 1 dimensions from the 2026 Xiaomi Cup plan:
# stone slabs are 1.00 m long, 0.30 m wide, 0.05 m high, spaced by 0.20 m,
# with four slabs total. The robot starts across the slabs, then reaches the
# right-side bend and turns into the next segment. The commands use the robot
# pose at each segment boundary as the local origin, so different initial world
# poses do not change the plan.
CLOSED_LOOP_PLAN = [
    Waypoint(
        "stone_slabs_1_2",
        0.90,
        0.12,
        step_height=0.075,
        yaw_kp=0.95,
        yaw_limit=0.24,
        lateral_kp=0.75,
        lateral_limit=0.06,
        min_speed_scale=0.45,
        stall_timeout=3.0,
        timeout=24.0,
        min_duration=14.0,
    ),
    Segment("stone_mid_balance", 0.4, K_LOCOMOTION, GAIT_STAND, step_height=0.075),
    Waypoint(
        "stone_slabs_3_4",
        0.90,
        0.10,
        step_height=0.075,
        yaw_kp=0.95,
        yaw_limit=0.22,
        lateral_kp=0.70,
        lateral_limit=0.05,
        min_speed_scale=0.50,
        stall_timeout=2.5,
        timeout=28.0,
        min_duration=16.0,
    ),
    Waypoint(
        "turn_approach",
        0.25,
        0.14,
        step_height=0.055,
        yaw_kp=0.85,
        yaw_limit=0.24,
        lateral_kp=0.65,
        lateral_limit=0.07,
        min_speed_scale=0.40,
        timeout=10.0,
        min_duration=2.0,
    ),
    ("settle_before_turn", 0.8),
    ("turn_left", math.radians(86.0)),
    ("settle_after_turn", 0.8),
    Waypoint(
        "curve_clear",
        0.85,
        0.12,
        step_height=0.055,
        yaw_kp=0.95,
        yaw_limit=0.22,
        lateral_kp=0.70,
        lateral_limit=0.06,
        timeout=10.0,
    ),
]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


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
    msg.step_height[0] = segment.step_height
    msg.step_height[1] = segment.step_height
    msg.duration = 0
    msg.value = 0
    return msg


def publish_segment(lc, lcm_type, segment: Segment, rate_hz: float, life_count: int, dry_run: bool) -> int:
    period = 1.0 / rate_hz
    ticks = max(1, int(segment.duration * rate_hz))
    print(
        f"[track1] {segment.name:16s} {segment.duration:4.1f}s "
        f"mode={segment.mode:2d} gait={segment.gait_id:2d} "
        f"vx={segment.vx:+.2f} vy={segment.vy:+.2f} yaw={segment.yaw_rate:+.2f} "
        f"step={segment.step_height:.3f}"
    )
    if dry_run:
        return (life_count + ticks) & 0x7F

    next_time = time.monotonic()
    for _ in range(ticks):
        life_count = (life_count + 1) & 0x7F
        msg = build_msg(lcm_type, segment, life_count)
        lc.publish(CHANNEL, msg.encode())
        next_time += period
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
    return life_count


def run_open_loop(driver: Driver) -> None:
    print(f"[track1] channel={CHANNEL} url={CMD_LCM_URL} rate={driver.rate_hz:.1f}Hz mode=open-loop")
    for segment in TRACK1_PLAN:
        driver.life_count = publish_segment(driver.cmd_lc, driver.cmd_type, segment, driver.rate_hz, driver.life_count, driver.dry_run)
    print("[track1] done")


def run_closed_loop(driver: Driver) -> None:
    print(
        f"[track1] cmd={CMD_LCM_URL} state={STATE_LCM_URL} "
        f"rate={driver.rate_hz:.1f}Hz mode=closed-loop"
    )
    driver.hold(Segment("recover_from_lie", 0.0, K_RECOVERY_STAND, 0), 7.0)
    if not driver.wait_for_state(2.0):
        raise SystemExit("No state_estimator LCM received. Start the controller first and check LCM network.")
    driver.hold(Segment("settle_qp_stand", 0.0, K_QP_STAND, GAIT_STAND, body_height=0.25), 3.0)

    for step in CLOSED_LOOP_PLAN:
        if isinstance(step, Segment):
            driver.hold(step, step.duration)
        elif isinstance(step, Waypoint):
            if not driver.drive_distance(step):
                print(f"[track1] abort: {step.name} did not complete; hold stand and skip the turn")
                driver.hold(Segment("abort_hold", 0.0, K_LOCOMOTION, GAIT_STAND), 30.0)
                print("[track1] done")
                return
        else:
            name, value = step
            if name.startswith("settle_"):
                driver.hold(Segment(name, 0.0, K_LOCOMOTION, GAIT_STAND), value)
            else:
                driver.turn_yaw(name, value)

    driver.hold(Segment("hold_stand", 0.0, K_LOCOMOTION, GAIT_STAND), 30.0)
    print("[track1] done")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the CyberDog track-1 stone path plan.")
    parser.add_argument("--rate", type=float, default=20.0, help="LCM publish rate in Hz.")
    parser.add_argument("--start-life", type=int, default=0, help="Initial life_count value.")
    parser.add_argument("--dry-run", action="store_true", help="Print the plan without publishing LCM.")
    parser.add_argument("--open-loop", action="store_true", help="Use the original timed baseline.")
    args = parser.parse_args()

    cmd_lc = None
    state_lc = None
    cmd_type = None
    state_type = None
    if not args.dry_run:
        try:
            import lcm
        except ImportError as exc:
            raise SystemExit("python3-lcm is required: sudo apt install python3-lcm") from exc
        try:
            import robot_control_cmd_lcmt as cmd_type
            import state_estimator_lcmt as state_type
        except ImportError as exc:
            raise SystemExit(
                "LCM Python types were not found. Run:\n"
                "  cd src/cyberdog_locomotion/scripts && ./make_types.sh -c"
            ) from exc
        cmd_lc = lcm.LCM(CMD_LCM_URL)
        state_lc = lcm.LCM(STATE_LCM_URL)

    driver = Driver(cmd_lc, state_lc, cmd_type, state_type, args.rate, args.dry_run, args.start_life & 0x7F)
    if args.open_loop:
        run_open_loop(driver)
    else:
        run_closed_loop(driver)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
