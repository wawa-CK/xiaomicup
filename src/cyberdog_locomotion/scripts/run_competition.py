#!/usr/bin/env python3
"""Simple competition orchestrator for Track 1 -> Track 2 -> Track 3.

This keeps each track controller independent, while providing one single
entry-point for the "press start and run automatically" requirement.
"""

import argparse
import math
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
LINUX_SRC_ROOT = Path("/home/cyberdog_sim/src")
LINUX_STAGE1 = LINUX_SRC_ROOT / "cyberdog_locomotion" / "scripts" / "track1_motion_program_style.py"
LINUX_STAGE3 = LINUX_SRC_ROOT / "cyberdog_locomotion" / "scripts" / "track3_curve_follow.py"
LINUX_STAGE2_ROOT = LINUX_SRC_ROOT / "wild_glint_hunt"
LINUX_STAGE2_PARAMS = LINUX_STAGE2_ROOT / "wild_glint_hunt" / "config" / "competition_tuned_params.yaml"

DEFAULT_STAGE1_CMD = f"python3 {LINUX_STAGE1} --final-hold 0.5"
DEFAULT_STAGE2_SENSOR_CMD = ""
DEFAULT_STAGE2_CMD = (
    f"ros2 launch wild_glint_hunt sim_wild_glint_hunt.launch.py "
    f"params_file:={LINUX_STAGE2_PARAMS} "
    f"use_sim_time:=true "
    f"reset_robot:=false "
    f"sensor_delay:=1.0 "
    f"planner_delay:=3.0 "
    f"sim_gazebo_model_name:=cyberdog"
)
DEFAULT_STAGE3_CMD = (
    f"python3 {LINUX_STAGE3} --odom-only --fixed-world-path"
)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Stage2Monitor:
    def __init__(self, args):
        import rclpy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.node = rclpy.create_node("competition_stage2_monitor")
        self.odom = None
        self.odom_stamp = 0.0
        self.gazebo_pose = None
        self.gazebo_pose_stamp = 0.0
        self.success = False
        self.success_message = ""
        self.last_status_print = 0.0
        self.args = args

        self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.node.create_subscription(String, args.stage2_success_topic, self._on_success, 10)
        try:
            from gazebo_msgs.msg import LinkStates
            self.node.create_subscription(LinkStates, args.gazebo_link_states_topic, self._on_link_states, 10)
            self.node.get_logger().info(
                f"subscribed gazebo link states topic: {args.gazebo_link_states_topic}"
            )
        except ImportError:
            self.node.get_logger().warn("gazebo_msgs not available, stage1 handoff will not use link_states")

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_stamp = time.monotonic()

    def _on_link_states(self, msg):
        for index, name in enumerate(msg.name):
            if name not in self.args.gazebo_link_candidates:
                continue
            pose = msg.pose[index]
            yaw = yaw_from_quaternion(pose.orientation)
            self.gazebo_pose = (pose.position.x, pose.position.y, yaw)
            self.gazebo_pose_stamp = time.monotonic()
            return

    def _on_success(self, msg):
        self.success = True
        self.success_message = msg.data
        self.node.get_logger().info(f"received stage2 success message: {msg.data}")

    def spin_once(self, timeout_sec: float = 0.1):
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def current_pose(self, prefer_gazebo: bool = False):
        if prefer_gazebo and self.gazebo_pose is not None:
            x, y, yaw = self.gazebo_pose
            return x, y, yaw, "gazebo"
        if self.odom is not None:
            pos = self.odom.pose.pose.position
            yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
            return pos.x, pos.y, yaw, "odom"
        if self.gazebo_pose is not None:
            x, y, yaw = self.gazebo_pose
            return x, y, yaw, "gazebo"
        return None

    def pose_near(self, target_x, target_y, target_yaw, x_tol, y_tol, yaw_tol):
        pose = self.current_pose()
        if pose is None:
            return False, None
        x, y, yaw, source = pose
        dx = x - target_x
        dy = y - target_y
        dyaw = normalize_angle(yaw - target_yaw)
        ok = (
            abs(dx) <= x_tol and
            abs(dy) <= y_tol and
            abs(dyaw) <= yaw_tol
        )
        return ok, (x, y, yaw, dx, dy, dyaw, source)

    def pose_near_p0(self):
        return self.pose_near(
            self.args.p0_x,
            self.args.p0_y,
            self.args.p0_yaw,
            self.args.p0_x_tol,
            self.args.p0_y_tol,
            self.args.p0_yaw_tol,
        )

    def maybe_print_status(self):
        if time.monotonic() - self.last_status_print < self.args.monitor_print_period:
            return
        self.last_status_print = time.monotonic()
        near, pose = self.pose_near_p0()
        if pose is None:
            print("[total] stage2 monitor: waiting for /odom ...")
            return
        x, y, yaw, dx, dy, dyaw, source = pose
        print(
            f"[total] stage2 monitor: {source}=({x:+.3f},{y:+.3f}) yaw={yaw:+.3f} "
            f"err=({dx:+.3f},{dy:+.3f},{dyaw:+.3f}) near_p0={'yes' if near else 'no'} "
            f"success={'yes' if self.success else 'no'}"
        )

    def shutdown(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def popen_command(command: str, cwd: Path):
    print(f"[total] start: {command}")
    return subprocess.Popen(shlex.split(command), cwd=str(cwd))


def terminate_process(proc: subprocess.Popen, name: str, grace_sec: float = 5.0):
    if proc.poll() is not None:
        return
    print(f"[total] stopping {name} ...")
    try:
        proc.send_signal(signal.SIGINT)
    except Exception:
        proc.terminate()
    deadline = time.monotonic() + grace_sec
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    if proc.poll() is None:
        proc.kill()


def run_blocking_stage(command: str, cwd: Path, name: str) -> int:
    proc = popen_command(command, cwd)
    rc = proc.wait()
    print(f"[total] {name} exited with code {rc}")
    return rc


def run_stage2(args) -> int:
    import rclpy

    rclpy.init()
    monitor = Stage2Monitor(args)
    sensor_proc = None
    stage2_proc = None
    near_since = None
    start_time = time.monotonic()

    try:
        if args.stage2_sensor_cmd:
            sensor_proc = popen_command(args.stage2_sensor_cmd, LINUX_SRC_ROOT.parent)
            time.sleep(args.stage2_sensor_start_delay)
        stage2_proc = popen_command(args.stage2_cmd, LINUX_SRC_ROOT.parent)

        while True:
            monitor.spin_once(0.1)
            monitor.maybe_print_status()

            if stage2_proc.poll() is not None:
                print(f"[total] stage2 process exited with code {stage2_proc.returncode}")
                return stage2_proc.returncode

            near, pose = monitor.pose_near_p0()
            if near:
                if near_since is None:
                    near_since = time.monotonic()
                    print("[total] stage2 reached P0 tolerance, starting settle timer")
                elif time.monotonic() - near_since >= args.stage2_settle_sec:
                    print("[total] stage2 settled at P0, hand over to track3")
                    return 0
            else:
                near_since = None

            if args.require_stage2_success and not monitor.success:
                pass
            elif args.require_stage2_success and monitor.success and near:
                if near_since is None:
                    near_since = time.monotonic()

            if time.monotonic() - start_time > args.stage2_timeout:
                print("[total] stage2 timeout before reaching P0")
                return 124
    finally:
        if stage2_proc is not None:
            terminate_process(stage2_proc, "stage2")
        if sensor_proc is not None:
            terminate_process(sensor_proc, "stage2 sensors")
        monitor.shutdown()


def wait_for_stage1_handoff(args) -> int:
    import rclpy

    if args.skip_stage1_handoff_check:
        print("[total] stage1 handoff check skipped")
        return 0

    rclpy.init()
    monitor = Stage2Monitor(args)
    near_since = None
    start_time = time.monotonic()
    last_status_print = 0.0

    try:
        print(
            f"[total] checking stage1 handoff pose "
            f"target=({args.stage1_handoff_x:+.3f},{args.stage1_handoff_y:+.3f}) "
            f"yaw={args.stage1_handoff_yaw:+.3f}"
        )
        while True:
            monitor.spin_once(0.1)
            now = time.monotonic()
            near, pose = monitor.pose_near(
                args.stage1_handoff_x,
                args.stage1_handoff_y,
                args.stage1_handoff_yaw,
                args.stage1_handoff_x_tol,
                args.stage1_handoff_y_tol,
                args.stage1_handoff_yaw_tol,
            )

            if now - last_status_print >= args.monitor_print_period:
                last_status_print = now
                if pose is None:
                    print("[total] stage1 handoff: waiting for gazebo pose/odom ...")
                else:
                    x, y, yaw, dx, dy, dyaw, source = pose
                    print(
                        f"[total] stage1 handoff: {source}=({x:+.3f},{y:+.3f}) yaw={yaw:+.3f} "
                        f"err=({dx:+.3f},{dy:+.3f},{dyaw:+.3f}) "
                        f"near_target={'yes' if near else 'no'}"
                    )

            if near:
                if near_since is None:
                    near_since = now
                elif now - near_since >= args.stage1_handoff_settle_sec:
                    print("[total] stage1 handoff pose confirmed, starting stage2")
                    return 0
            else:
                near_since = None

            if now - start_time > args.stage1_handoff_timeout:
                print("[total] stage1 handoff timeout: track1 did not stop at stage2 entry")
                return 125
    finally:
        monitor.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Competition total controller")
    parser.add_argument("--stage1-cmd", default=DEFAULT_STAGE1_CMD)
    parser.add_argument("--stage2-sensor-cmd", default=DEFAULT_STAGE2_SENSOR_CMD)
    parser.add_argument("--stage2-cmd", default=DEFAULT_STAGE2_CMD)
    parser.add_argument("--stage3-cmd", default=DEFAULT_STAGE3_CMD)
    parser.add_argument("--skip-stage1", action="store_true")
    parser.add_argument("--skip-stage2", action="store_true")
    parser.add_argument("--skip-stage3", action="store_true")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--stage2-success-topic", default="hunt/success")
    parser.add_argument("--require-stage2-success", action="store_true")
    parser.add_argument("--stage2-timeout", type=float, default=330.0)
    parser.add_argument("--stage2-settle-sec", type=float, default=0.8)
    parser.add_argument("--stage2-sensor-start-delay", type=float, default=2.0)
    parser.add_argument("--monitor-print-period", type=float, default=1.0)
    parser.add_argument("--gazebo-link-states-topic", default="/gazebo/link_states")
    parser.add_argument(
        "--gazebo-link-candidates",
        nargs="+",
        default=["cyberdog::base_link", "robot::base_link", "base_link"],
    )
    parser.add_argument("--skip-stage1-handoff-check", action="store_true")
    parser.add_argument("--stage1-handoff-x", type=float, default=3.20)
    parser.add_argument("--stage1-handoff-y", type=float, default=0.70)
    parser.add_argument("--stage1-handoff-yaw", type=float, default=1.57)
    parser.add_argument("--stage1-handoff-x-tol", type=float, default=0.10)
    parser.add_argument("--stage1-handoff-y-tol", type=float, default=0.10)
    parser.add_argument("--stage1-handoff-yaw-tol", type=float, default=0.15)
    parser.add_argument("--stage1-handoff-timeout", type=float, default=5.0)
    parser.add_argument("--stage1-handoff-settle-sec", type=float, default=0.4)
    parser.add_argument("--p0-x", type=float, default=-0.275993)
    parser.add_argument("--p0-y", type=float, default=4.310233)
    parser.add_argument("--p0-yaw", type=float, default=1.538237)
    parser.add_argument("--p0-x-tol", type=float, default=0.05)
    parser.add_argument("--p0-y-tol", type=float, default=0.05)
    parser.add_argument("--p0-yaw-tol", type=float, default=0.10)
    args = parser.parse_args()

    if not args.skip_stage1:
        rc = run_blocking_stage(args.stage1_cmd, LINUX_SRC_ROOT.parent, "stage1")
        if rc != 0:
            return rc
        rc = wait_for_stage1_handoff(args)
        if rc != 0:
            return rc

    if not args.skip_stage2:
        rc = run_stage2(args)
        if rc != 0:
            return rc

    if not args.skip_stage3:
        rc = run_blocking_stage(args.stage3_cmd, LINUX_SRC_ROOT.parent, "stage3")
        if rc != 0:
            return rc

    print("[total] competition flow done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
