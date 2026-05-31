#!/usr/bin/env python3
"""
CyberDog 六赛段全自动竞赛控制器 — 无雷达全流程导航方案
纯视觉中线跟踪 + IMU/里程计 + TOF避障 + FSM任务控制
参考：小米杯比赛「无雷达全流程导航方案」
"""

import sys
import os
import time
import math
import threading
import signal
import traceback
import shutil
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import lcm
from robot_control_cmd_lcmt import robot_control_cmd_lcmt, robot_control_response_lcmt

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, Image
from tf2_ros import Buffer, TransformListener

try:
    from gazebo_msgs.msg import LinkStates
except Exception:
    LinkStates = None


def env_float(name, default, min_value=None, max_value=None):
    try:
        value = float(os.environ.get(name, default))
    except Exception:
        value = default
    if min_value is not None:
        value = max(min_value, value)
    if max_value is not None:
        value = min(max_value, value)
    return value


def env_int(name, default, min_value=None, max_value=None):
    try:
        value = int(os.environ.get(name, default))
    except Exception:
        value = default
    if min_value is not None:
        value = max(min_value, value)
    if max_value is not None:
        value = min(max_value, value)
    return value


LCM_CMD_URL = "udpm://239.255.76.67:7671?ttl=255"
LCM_RESP_URL = "udpm://239.255.76.67:7670?ttl=255"

MODE_PURE_DAMPER = 7
MODE_LOCOMOTION = 11
MODE_RECOVERY_STAND = 12
MODE_JUMP3D = 16
MODE_POS_INTERP = 21

GAIT_TROT_MEDIUM = 3
GAIT_TROT_10_4 = 5
GAIT_WALK = 6
GAIT_TROT_10_5 = 9
GAIT_TROT_FAST = 10
GAIT_TROT_SLOW = 27
GAIT_TROT_AUTO = 29
GAIT_TROT_VARIABLE = 26
GAIT_BOUND = 7
GAIT_TROT_SWING = 55
GAIT_TROT_PITCH = 57
GAIT_WALK_WAVE = 60
GAIT_USER_WALK_WAVE = 83
STAGE_RUN_GAIT = env_int("RACE_RUN_GAIT", GAIT_TROT_FAST, 1, 99)
if STAGE_RUN_GAIT == GAIT_USER_WALK_WAVE:
    STAGE_RUN_GAIT = GAIT_TROT_FAST

# 石板路使用今天凌晨稳定的 24-16 小跑步态；不要默认切到 Walk。
STONE_DEFAULT_GAIT = env_int("RACE_STONE_GAIT", GAIT_TROT_VARIABLE, 1, 99)
if STONE_DEFAULT_GAIT == GAIT_USER_WALK_WAVE:
    STONE_DEFAULT_GAIT = GAIT_TROT_VARIABLE
STAGE1_GAIT_NORMAL = STONE_DEFAULT_GAIT
STAGE1_GAIT_HIGH_STEP = env_int("RACE_STONE_HIGH_GAIT", STONE_DEFAULT_GAIT, 1, 99)
if STAGE1_GAIT_HIGH_STEP == GAIT_USER_WALK_WAVE:
    STAGE1_GAIT_HIGH_STEP = STONE_DEFAULT_GAIT
STAGE1_GAIT_RECONTACT = env_int("RACE_STONE_CONTACT_GAIT", STAGE1_GAIT_HIGH_STEP, 1, 99)
if STAGE1_GAIT_RECONTACT == GAIT_USER_WALK_WAVE:
    STAGE1_GAIT_RECONTACT = STAGE1_GAIT_HIGH_STEP

JUMP_POS_Y20 = 2
JUMP_POS_X30 = 4
JUMP_NEG_Y20 = 5
JUMP_DOWN_STAIR = 9

TOTAL_TIME_LIMIT = 900
CONTROL_RESPONSE_STALE_WARN = env_float("RACE_CONTROL_STALE_WARN", 2.0, 0.5, 10.0)
CONTROL_RESPONSE_STALE_FATAL = env_float("RACE_CONTROL_STALE_FATAL", 4.0, 1.0, 20.0)
CONTROL_RESPONSE_STARTUP_FATAL = env_float("RACE_CONTROL_STARTUP_FATAL", 8.0, 3.0, 30.0)
JUMP_RECOVERY_MIN_SEC = env_float("RACE_JUMP_RECOVERY_MIN_SEC", 0.80, 0.50, 2.50)
JUMP_RECOVERY_FALLBACK_SEC = env_float("RACE_JUMP_RECOVERY_FALLBACK_SEC", 1.35, 0.90, 4.00)
JUMP_RECOVERY_FORCE_RESET_SEC = env_float("RACE_JUMP_RECOVERY_FORCE_RESET_SEC", 1.80, 1.20, 5.00)
JUMP_HANDOFF_SEC = env_float("RACE_JUMP_HANDOFF_SEC", 0.25, 0.10, 1.20)

BASE_FRAME = os.environ.get("RACE_BASE_FRAME", "base_link")
ODOM_FRAME_CANDIDATES = tuple(
    f.strip() for f in os.environ.get("RACE_ODOM_FRAMES", "vodom,odom").split(",") if f.strip()
) or ("vodom", "odom")
POSE_SOURCE = os.environ.get("RACE_POSE_SOURCE", "gazebo").strip().lower()
if POSE_SOURCE not in ("gazebo", "tf", "auto"):
    POSE_SOURCE = "gazebo"
GAZEBO_POSE_STALE_SEC = env_float("RACE_GAZEBO_POSE_STALE_SEC", 0.35, 0.10, 1.00)
GAZEBO_ROBOT_LINK_CANDIDATES = tuple(
    f.strip() for f in os.environ.get(
        "RACE_GAZEBO_ROBOT_LINKS",
        "robot::base_link,robot::body,cyberdog::base_link,cyberdog::body,base_link,body",
    ).split(",") if f.strip()
) or ("robot::base_link", "base_link")

TOF_OBSTACLE_MIN = 0.20
TOF_OBSTACLE_MAX = 0.50

CENTERLINE_KP = 0.6
CENTERLINE_KD = 0.15
STAGE1_GOAL_STEER_WEIGHT = 0.22
STAGE1_LINE_HOLD_SEC = 0.85
SAFE_VX = env_float("RACE_SAFE_VX", 0.24, 0.03, 0.50)
SAFE_YAW = env_float("RACE_SAFE_YAW", 0.18, 0.05, 0.45)
STEP_FLAT = env_float("RACE_STEP_FLAT", 0.1018, 0.04, 0.18)
STEP_FLAT_REAR = env_float("RACE_STEP_FLAT_REAR", 0.1228, 0.04, 0.20)
STEP_STONE = env_float("RACE_STEP_STONE", 0.1418, 0.08, 0.18)
STEP_STONE_REAR = env_float("RACE_STEP_STONE_REAR", 0.1728, 0.08, 0.22)
BODY_HEIGHT = env_float("RACE_BODY_HEIGHT", 0.27, 0.18, 0.30)
SETTLE_TIME = env_float("RACE_SETTLE_TIME", 2.0, 0.3, 4.0)
S2_VX = env_float("RACE_S2_VX", 0.34, 0.04, 0.70)
S2_STEP = env_float("RACE_S2_STEP", 0.06, 0.03, 0.12)
S2_BODY = env_float("RACE_S2_BODY", 0.25, 0.18, 0.30)
STAGE1_HIGH_STEP_MIN_VX = env_float("RACE_STONE_MIN_VX", SAFE_VX, 0.018, 0.32)
STONE_APPROACH_VX = env_float("RACE_STONE_APPROACH_VX", SAFE_VX, 0.025, 0.42)
STONE_CRAWL_VX = env_float("RACE_STONE_CRAWL_VX", SAFE_VX, 0.020, 0.36)
STONE_STABLE_CRUISE_VX = env_float("RACE_STONE_STABLE_CRUISE_VX", SAFE_VX, 0.040, 0.44)
STONE_BALANCE_CRAWL_VX = env_float("RACE_STONE_BALANCE_CRAWL_VX", SAFE_VX, 0.020, 0.28)
STONE_IMPACT_RELIEF_VX = env_float("RACE_STONE_IMPACT_VX", 0.140, 0.010, 0.20)
STONE_STEPUP_VX = env_float("RACE_STONE_STEPUP_VX", SAFE_VX, 0.018, 0.28)
STONE_STEPUP_END_X = env_float("RACE_STONE_STEPUP_END_X", 1.12, 0.45, 1.40)
STONE_APPROACH_STEP_HEIGHT = env_float("RACE_STONE_APPROACH_STEP", STEP_FLAT, 0.06, 0.18)
STONE_APPROACH_STEP_HEIGHT_REAR = env_float("RACE_STONE_APPROACH_STEP_REAR", STEP_FLAT_REAR, 0.06, 0.20)
STONE_CLIMB_STEP_HEIGHT = env_float("RACE_STONE_STEP", STEP_STONE, 0.08, 0.18)
STONE_CLIMB_STEP_HEIGHT_REAR = env_float("RACE_STONE_STEP_REAR", STEP_STONE_REAR, 0.08, 0.22)
STONE_IMPACT_RELIEF_SEC = env_float("RACE_STONE_RELIEF_SEC", 0.20, 0.12, 0.70)
STONE_IMPACT_RAMP_SEC = env_float("RACE_STONE_RAMP_SEC", 0.12, 0.05, 0.45)
STONE_RECONTACT_FORWARD_VX = env_float("RACE_STONE_CONTACT_VX", min(SAFE_VX, 0.080), 0.02, 0.16)
STONE_RECONTACT_SIDE_VY = env_float("RACE_STONE_CONTACT_VY", 0.0, 0.0, 0.025)
STONE_RECONTACT_TOTAL_SEC = env_float("RACE_STONE_CONTACT_SEC", 0.0, 0.0, 0.30)
STONE_LANE_VY_GAIN = env_float("RACE_STONE_LANE_VY_GAIN", 0.16, 0.0, 0.22)
STONE_LANE_VY_LIMIT = env_float("RACE_STONE_LANE_VY_LIMIT", 0.060, 0.0, 0.075)
STONE_SIDE_DAMP_VY_GAIN = env_float("RACE_STONE_SIDE_DAMP_VY_GAIN", 0.36, 0.0, 0.55)
STONE_SIDE_DAMP_YAW_GAIN = env_float("RACE_STONE_SIDE_DAMP_YAW_GAIN", 0.58, 0.0, 0.90)
STONE_SIDE_DAMP_VY_LIMIT = env_float("RACE_STONE_SIDE_DAMP_VY_LIMIT", 0.048, 0.0, 0.070)
STONE_SIDE_DAMP_YAW_LIMIT = env_float("RACE_STONE_SIDE_DAMP_YAW_LIMIT", 0.115, 0.0, 0.180)
STONE_ROLL_VY_GAIN = env_float("RACE_STONE_ROLL_VY_GAIN", 0.072, 0.0, 0.120)
STONE_ROLL_YAW_GAIN = env_float("RACE_STONE_ROLL_YAW_GAIN", 0.22, 0.0, 0.38)
STONE_SIDE_SLIP_TRIGGER = env_float("RACE_STONE_SIDE_SLIP_TRIGGER", 0.070, 0.035, 0.12)
STONE_PAIR_SETTLE_SEC = env_float("RACE_STONE_PAIR_SETTLE_SEC", 0.0, 0.0, 1.00)
STONE_PAIR_SETTLE_Y = env_float("RACE_STONE_PAIR_SETTLE_Y", 0.16, 0.05, 0.35)
STONE_PAIR_SETTLE_VX = env_float("RACE_STONE_PAIR_SETTLE_VX", SAFE_VX, 0.012, 0.170)
STONE_PAIR_CENTER_VY_GAIN = env_float("RACE_STONE_PAIR_CENTER_VY_GAIN", 0.032, 0.0, 0.10)
STONE_PAIR_CENTER_VY_LIMIT = env_float("RACE_STONE_PAIR_CENTER_VY_LIMIT", 0.010, 0.0, 0.030)
STONE_FRONT_PAIR_WAIT_SEC = env_float("RACE_STONE_FRONT_PAIR_WAIT_SEC", 0.0, 0.0, 1.80)
STONE_FRONT_PAIR_WAIT_VX = env_float("RACE_STONE_FRONT_PAIR_WAIT_VX", 0.024, 0.0, 0.110)
STONE_FRONT_PAIR_HOLD_SEC = env_float("RACE_STONE_FRONT_PAIR_HOLD_SEC", 0.0, 0.0, 1.00)
STONE_FRONT_PAIR_PULSE_VX = env_float("RACE_STONE_FRONT_PAIR_PULSE_VX", 0.032, 0.0, 0.120)
STONE_FRONT_PAIR_PULSE_SEC = env_float("RACE_STONE_FRONT_PAIR_PULSE_SEC", 0.08, 0.04, 0.22)
STONE_FRONT_PAIR_PULSE_GAP_SEC = env_float("RACE_STONE_FRONT_PAIR_PULSE_GAP_SEC", 0.58, 0.18, 0.95)
STONE_FRONT_PAIR_CENTER_DEADBAND = env_float("RACE_STONE_FRONT_PAIR_CENTER_DEADBAND", 0.10, 0.0, 0.22)
STONE_FRONT_PAIR_UNLOAD_VY_GAIN = env_float("RACE_STONE_FRONT_PAIR_UNLOAD_VY_GAIN", 0.035, 0.0, 0.16)
STONE_FRONT_PAIR_UNLOAD_VY_LIMIT = env_float("RACE_STONE_FRONT_PAIR_UNLOAD_VY_LIMIT", 0.010, 0.0, 0.045)
STONE_FRONT_PAIR_UNLOAD_YAW_LIMIT = env_float("RACE_STONE_FRONT_PAIR_UNLOAD_YAW_LIMIT", 0.016, 0.0, 0.08)
STONE_FRONT_PAIR_YAW_GAIN = env_float("RACE_STONE_FRONT_PAIR_YAW_GAIN", 0.18, 0.0, 0.75)
STONE_FRONT_PAIR_STEP_FORWARD = env_float("RACE_STONE_FRONT_PAIR_STEP_FORWARD", 0.105, 0.020, 0.145)
STONE_FRONT_PAIR_STEP_LIFT = env_float("RACE_STONE_FRONT_PAIR_STEP_LIFT", 0.145, 0.060, 0.160)
STONE_FRONT_PAIR_STEP_MS = env_int("RACE_STONE_FRONT_PAIR_STEP_MS", 360, 180, 700)
STONE_FRONT_PAIR_STEP_HOLD_SEC = env_float("RACE_STONE_FRONT_PAIR_STEP_HOLD_SEC", 0.65, 0.18, 0.90)
STONE_FRONT_PAIR_STEPUP_VX = env_float("RACE_STONE_FRONT_PAIR_STEPUP_VX", 0.140, 0.010, 0.210)
# 赛道目标点按 race.world 中 2026 赛道模型的世界坐标设计：
# 起点石板路在 y≈0，随后沿 +y 方向依次经过寻珠区、S 弯、深隧区、独木桥和终点区。
# 赛题平面图中全场 400cm x 1600cm，对应仿真世界坐标约 x∈[-0.7,3.5], y∈[-0.6,15.8]。
STAGE1_CENTER_Y = 0.00
STAGE1_ENTRY_X = 0.60
STAGE1_MID_X = 1.50
STAGE1_EXIT_X = 2.20
STAGE1_EXIT_Y = 0.00
STAGE1_STONE_EXIT_POINT = (2.20, 0.00)
# STL 测得一二赛段净缺口为 x=2.778..3.378。
# 不能取几何中心 3.08：原地转向时前脚扫掠会碰到右侧 x=3.378 黄线。
# 按前脚前伸、半身宽和足端半径留余量后，base_link 转身中心取 x≈2.99。
STAGE1_GAP_X = env_float("RACE_STAGE1_GAP_X", 2.99, 2.96, 3.01)
STAGE1_DIRECT_STAGE2_X = env_float("RACE_STAGE1_DIRECT_STAGE2_X", 2.96, 2.86, 3.06)
STAGE1_DIRECT_STAGE2_Y = env_float("RACE_STAGE1_DIRECT_STAGE2_Y", 1.30, 1.20, 1.38)
# 红灰线交叉点在红色 x 轴上，y 必须为 0；到点后只允许转向缺口，不再继续回拉。
STAGE1_GAP_BELOW_Y = env_float("RACE_STAGE1_GAP_BELOW_Y", 0.00, -0.06, 0.06)
STAGE1_RED_GRAY_TURN_POINT = (STAGE1_GAP_X, STAGE1_GAP_BELOW_Y)
STAGE1_STAGE2_TURN_Y = env_float("RACE_STAGE1_STAGE2_TURN_Y", 1.30, 1.20, 1.38)
STAGE1_GAP_POINT = (STAGE1_GAP_X, STAGE1_STAGE2_TURN_Y)
STAGE1_GAP_BELOW_POINT = (STAGE1_GAP_X, STAGE1_GAP_BELOW_Y)
STAGE1_GAP_INNER_POINT = (STAGE1_GAP_X, STAGE1_STAGE2_TURN_Y)
STAGE1_GOAL_ENABLE_X = 0.60
STAGE1_MIN_EXIT_X = 2.20
STAGE1_MIN_EXIT_TIME = 3.0
STAGE1_MIN_STONE_HITS = 3
STAGE1_ODOM_ROCKROAD_X = 1.95
STONE_EDGE_X_MIN = env_float("RACE_STONE_EDGE_X_MIN", 0.00, 0.00, 0.75)
STONE_EDGE_X_MAX = env_float("RACE_STONE_EDGE_X_MAX", 0.45, 0.16, 0.95)
STONE_EDGE_JUMP_TRIGGER_X = env_float("RACE_STONE_EDGE_JUMP_TRIGGER_X", 0.00, 0.00, 0.90)
STONE_EDGE_JUMP_AFTER_X = env_float("RACE_STONE_EDGE_JUMP_AFTER_X", 0.00, 0.00, 0.35)
STONE_EDGE_JUMP_DONE_X = env_float("RACE_STONE_EDGE_JUMP_DONE_X", 0.70, 0.35, 1.35)
STONE_EDGE_JUMP_ALIGN_SEC = env_float("RACE_STONE_EDGE_JUMP_ALIGN_SEC", 0.18, 0.0, 0.60)
STONE_EDGE_JUMP_STONE_Y_MIN = env_float("RACE_STONE_EDGE_JUMP_STONE_Y_MIN", 0.10, -0.60, 0.80)
STONE_EDGE_JUMP_GAIT = env_int("RACE_STONE_EDGE_JUMP_GAIT", JUMP_POS_X30, 2, 5)
if STONE_EDGE_JUMP_GAIT not in (JUMP_POS_Y20, JUMP_POS_X30, JUMP_NEG_Y20):
    STONE_EDGE_JUMP_GAIT = JUMP_POS_X30
STONE_EDGE_JUMP_Y_TOL = env_float("RACE_STONE_EDGE_JUMP_Y_TOL", 0.32, 0.04, 0.42)
STONE_EDGE_JUMP_YAW_TOL = env_float("RACE_STONE_EDGE_JUMP_YAW_TOL", 0.22, 0.04, 0.52)
STONE_EDGE_JUMP_FORCE_YAW_TOL = env_float("RACE_STONE_EDGE_JUMP_FORCE_YAW_TOL", 0.22, 0.06, 0.52)
STONE_EDGE_JUMP_ROLL_TOL = env_float("RACE_STONE_EDGE_JUMP_ROLL_TOL", 0.18, 0.06, 0.30)
STONE_EDGE_JUMP_PITCH_TOL = env_float("RACE_STONE_EDGE_JUMP_PITCH_TOL", 0.18, 0.06, 0.30)
STONE_EDGE_JUMP_ALIGN_VX = env_float("RACE_STONE_EDGE_JUMP_ALIGN_VX", 0.0, 0.0, 0.040)
STONE_EDGE_JUMP_ALIGN_VY_LIMIT = env_float("RACE_STONE_EDGE_JUMP_ALIGN_VY_LIMIT", 0.020, 0.004, 0.040)
STONE_EDGE_JUMP_ALIGN_YAW_LIMIT = env_float("RACE_STONE_EDGE_JUMP_ALIGN_YAW_LIMIT", 0.42, 0.08, 0.55)
STONE_EDGE_JUMP_RECOVER_SEC = env_float("RACE_STONE_EDGE_JUMP_RECOVER_SEC", min(SETTLE_TIME, 2.50), 0.50, 2.50)
STONE_EDGE_JUMP_STAND_SEC = env_float("RACE_STONE_EDGE_JUMP_STAND_SEC", SETTLE_TIME, 0.60, 3.50)
STONE_EDGE_JUMP_DRIVE_SEC = env_float("RACE_STONE_EDGE_JUMP_DRIVE_SEC", 1.30, 0.50, 3.00)
STONE_EDGE_JUMP_DRIVE_DONE_X = env_float("RACE_STONE_EDGE_JUMP_DRIVE_DONE_X", 0.82, 0.55, 1.50)
STONE_EDGE_JUMP_DRIVE_VX = env_float("RACE_STONE_EDGE_JUMP_DRIVE_VX", SAFE_VX, 0.025, 0.240)
STONE_EDGE_JUMP_DRIVE_YAW_LIMIT = env_float("RACE_STONE_EDGE_JUMP_DRIVE_YAW_LIMIT", min(SAFE_YAW, 0.10), 0.0, 0.18)
STONE_EDGE_TRAP_START_X = env_float("RACE_STONE_EDGE_TRAP_START_X", 0.62, 0.45, 1.20)
STONE_EDGE_TRAP_END_X = env_float("RACE_STONE_EDGE_TRAP_END_X", 2.30, 1.30, 2.35)
STONE_EDGE_TRAP_VX = env_float("RACE_STONE_EDGE_TRAP_VX", SAFE_VX, 0.018, 0.280)
STONE_EDGE_TRAP_YAW_LIMIT = env_float("RACE_STONE_EDGE_TRAP_YAW_LIMIT", 0.055, 0.0, 0.12)
STONE_ENTRY_ALIGN_END_X = env_float("RACE_STONE_ENTRY_ALIGN_END_X", 1.02, 0.50, 1.20)
STONE_ENTRY_ALIGN_Y_TOL = env_float("RACE_STONE_ENTRY_ALIGN_Y_TOL", 0.045, 0.02, 0.14)
STONE_ENTRY_ALIGN_YAW_TOL = env_float("RACE_STONE_ENTRY_ALIGN_YAW_TOL", 0.060, 0.02, 0.18)
STONE_ENTRY_ALIGN_VX = env_float("RACE_STONE_ENTRY_ALIGN_VX", 0.040, 0.0, 0.100)
STONE_ENTRY_ALIGN_VY_GAIN = env_float("RACE_STONE_ENTRY_ALIGN_VY_GAIN", 0.16, 0.02, 0.25)
STONE_ENTRY_ALIGN_VY_LIMIT = env_float("RACE_STONE_ENTRY_ALIGN_VY_LIMIT", 0.020, 0.006, 0.045)
STONE_ENTRY_ALIGN_YAW_GAIN = env_float("RACE_STONE_ENTRY_ALIGN_YAW_GAIN", 0.52, 0.10, 0.90)
STONE_ENTRY_ALIGN_YAW_LIMIT = env_float("RACE_STONE_ENTRY_ALIGN_YAW_LIMIT", 0.060, 0.020, 0.14)
STONE_POST_ALIGN_START_X = env_float("RACE_STONE_POST_ALIGN_START_X", 0.55, 0.35, 1.40)
STONE_POST_ALIGN_END_X = env_float("RACE_STONE_POST_ALIGN_END_X", 2.65, 1.40, 2.80)
STONE_POST_ALIGN_Y_TOL = env_float("RACE_STONE_POST_ALIGN_Y_TOL", 0.030, 0.02, 0.18)
STONE_POST_ALIGN_YAW_TOL = env_float("RACE_STONE_POST_ALIGN_YAW_TOL", 0.055, 0.02, 0.20)
STONE_POST_ALIGN_VX = env_float("RACE_STONE_POST_ALIGN_VX", SAFE_VX, 0.015, 0.280)
STONE_POST_ALIGN_VY_GAIN = env_float("RACE_STONE_POST_ALIGN_VY_GAIN", 0.24, 0.02, 0.32)
STONE_POST_ALIGN_VY_LIMIT = env_float("RACE_STONE_POST_ALIGN_VY_LIMIT", 0.060, 0.008, 0.070)
STONE_POST_ALIGN_YAW_GAIN = env_float("RACE_STONE_POST_ALIGN_YAW_GAIN", 0.68, 0.10, 1.10)
STONE_POST_ALIGN_YAW_LIMIT = env_float("RACE_STONE_POST_ALIGN_YAW_LIMIT", 0.145, 0.030, 0.24)
STONE_LANE_HARD_Y = env_float("RACE_STONE_LANE_HARD_Y", 0.13, 0.06, 0.28)
STONE_LANE_HARD_VX = env_float("RACE_STONE_LANE_HARD_VX", SAFE_VX, 0.010, 0.280)
STONE_LANE_HARD_VY_GAIN = env_float("RACE_STONE_LANE_HARD_VY_GAIN", 0.28, 0.04, 0.45)
STONE_LANE_HARD_VY_LIMIT = env_float("RACE_STONE_LANE_HARD_VY_LIMIT", 0.060, 0.015, 0.085)
STONE_LANE_HARD_YAW_GAIN = env_float("RACE_STONE_LANE_HARD_YAW_GAIN", 0.90, 0.10, 1.40)
STONE_LANE_HARD_YAW_LIMIT = env_float("RACE_STONE_LANE_HARD_YAW_LIMIT", 0.210, 0.050, 0.32)
STAGE1_EXIT_RADIUS = 0.36
STAGE1_TAIL_CLEAR_X = env_float("RACE_STAGE1_TAIL_CLEAR_X", 2.58, 2.30, 2.72)
STAGE1_TAIL_CLEAR_SEC = env_float("RACE_STAGE1_TAIL_CLEAR_SEC", 0.45, 0.0, 3.00)
STAGE1_TAIL_CLEAR_VX = env_float("RACE_STAGE1_TAIL_CLEAR_VX", 0.180, 0.025, 0.320)
STAGE1_TAIL_CLEAR_YAW_LIMIT = env_float("RACE_STAGE1_TAIL_CLEAR_YAW_LIMIT", 0.0, 0.0, 0.28)
STAGE1_GAP_RADIUS = 0.32
STAGE1_GAP_BELOW_RADIUS = env_float("RACE_STAGE1_GAP_BELOW_RADIUS", 0.055, 0.04, 0.12)
STAGE1_RED_GRAY_LOCK_X_TOL = env_float("RACE_STAGE1_RED_GRAY_LOCK_X_TOL", 0.045, 0.035, 0.18)
STAGE1_RED_GRAY_LOCK_Y_TOL = env_float("RACE_STAGE1_RED_GRAY_LOCK_Y_TOL", 0.040, 0.030, 0.18)
STAGE1_STAGE2_TURN_X_TOL = env_float("RACE_STAGE1_STAGE2_TURN_X_TOL", 0.035, 0.020, 0.08)
STAGE1_STAGE2_TURN_Y_TOL = env_float("RACE_STAGE1_STAGE2_TURN_Y_TOL", 0.035, 0.020, 0.08)
STAGE1_GAP_INNER_RADIUS = env_float("RACE_STAGE1_GAP_INNER_RADIUS", 0.24, 0.12, 0.40)
STAGE1_GAP_CRAWL_VX = env_float("RACE_STAGE1_GAP_CRAWL_VX", 0.260, 0.010, 0.500)
STAGE1_GAP_YAW_LIMIT = env_float("RACE_STAGE1_GAP_YAW_LIMIT", 1.35, 0.040, 2.20)
STAGE1_GAP_TURN_SPEED_SCALE = env_float("RACE_STAGE1_GAP_TURN_SPEED_SCALE", 2.2, 0.5, 3.2)
STAGE1_GAP_X_TOL = env_float("RACE_STAGE1_GAP_X_TOL", 0.035, 0.025, 0.08)
STAGE1_GAP_X_ALIGN_TOL = env_float("RACE_STAGE1_GAP_X_ALIGN_TOL", 0.045, 0.015, 0.12)
STAGE1_GAP_X_ALIGN_VX = env_float("RACE_STAGE1_GAP_X_ALIGN_VX", 0.076, 0.012, 0.160)
STAGE1_GAP_X_ALIGN_YAW_TOL = env_float("RACE_STAGE1_GAP_X_ALIGN_YAW_TOL", 0.08, 0.03, 0.18)
STAGE1_GAP_TURN_Y_TOL = env_float("RACE_STAGE1_GAP_TURN_Y_TOL", 0.025, 0.015, 0.08)
STAGE1_GAP_TURN_VY_GAIN = env_float("RACE_STAGE1_GAP_TURN_VY_GAIN", 0.240, 0.0, 0.40)
STAGE1_GAP_TURN_VY_LIMIT = env_float("RACE_STAGE1_GAP_TURN_VY_LIMIT", 0.068, 0.0, 0.100)
STAGE1_GAP_X_SAFE_MAX = env_float("RACE_STAGE1_GAP_X_SAFE_MAX", 3.08, 3.02, 3.16)
STAGE1_GAP_RECOVER_X = env_float("RACE_STAGE1_GAP_RECOVER_X", 2.92, 2.86, 2.98)
STAGE1_GAP_RECOVER_Y_MAX = env_float("RACE_STAGE1_GAP_RECOVER_Y_MAX", 1.22, 0.90, 1.45)
STAGE1_GAP_RECOVER_WORLD_VX = env_float("RACE_STAGE1_GAP_RECOVER_WORLD_VX", 0.30, 0.06, 0.42)
STAGE1_GAP_RECOVER_WORLD_VY = env_float("RACE_STAGE1_GAP_RECOVER_WORLD_VY", 0.12, 0.0, 0.20)
STAGE1_GAP_HARD_WALL_X = env_float("RACE_STAGE1_GAP_HARD_WALL_X", 3.16, 3.05, 3.24)
STAGE1_GAP_TURN_X = env_float("RACE_STAGE1_GAP_TURN_X", STAGE1_GAP_X, 2.90, 3.04)
STAGE1_GAP_TURN_FORCE_X = env_float("RACE_STAGE1_GAP_TURN_FORCE_X", 3.03, 2.96, 3.12)
STAGE1_STONE_ALIGN_YAW = env_float("RACE_STAGE1_STONE_ALIGN_YAW", 0.0, -math.pi, math.pi)
# 石板路默认朝向世界坐标正 x 轴，后续缺口方向与石板路保持 90° 差值。
STAGE1_GAP_ALIGN_YAW = env_float(
    "RACE_STAGE1_GAP_ALIGN_YAW",
    STAGE1_STONE_ALIGN_YAW + math.pi / 2.0,
    -math.pi,
    math.pi,
)
STAGE1_GAP_ALIGN_YAW_TOL = env_float("RACE_STAGE1_GAP_ALIGN_YAW_TOL", 0.12, 0.04, 0.28)
STAGE1_GAP_STRAIGHT_VX = env_float("RACE_STAGE1_GAP_STRAIGHT_VX", SAFE_VX, 0.018, 0.420)
STAGE1_STONE_EXIT_CROSS_X = env_float("RACE_STAGE1_STONE_EXIT_CROSS_X", 2.18, 2.05, 2.55)
STAGE1_GAP_OVERRUN_X = env_float("RACE_STAGE1_GAP_OVERRUN_X", 3.08, 2.98, 3.16)
STAGE1_GAP_OVERRUN_STAGE2_Y = env_float("RACE_STAGE1_GAP_OVERRUN_STAGE2_Y", STAGE1_STAGE2_TURN_Y, 1.20, 1.38)
STAGE1_GAP_RECOVER_YAW_LIMIT = env_float("RACE_STAGE1_GAP_RECOVER_YAW_LIMIT", 0.800, 0.08, 0.90)
STAGE1_GAP_FAST_YAW_LIMIT = env_float("RACE_STAGE1_GAP_FAST_YAW_LIMIT", 1.60, 0.08, 2.20)
STAGE1_GATE_Y_MIN = env_float("RACE_STAGE1_GATE_Y_MIN", 0.72, 0.55, 1.05)
STAGE1_GAP_EARLY_TURN_Y = env_float("RACE_STAGE1_GAP_EARLY_TURN_Y", 0.58, 0.45, 0.86)
STAGE1_GAP_EARLY_TURN_X_TOL = env_float("RACE_STAGE1_GAP_EARLY_TURN_X_TOL", 0.22, 0.08, 0.38)
STAGE1_GAP_EXIT_Y = env_float("RACE_STAGE1_GAP_EXIT_Y", STAGE1_STAGE2_TURN_Y, 1.20, 1.45)
STAGE1_BODY_MID_CLEAR_Y = env_float("RACE_STAGE1_BODY_MID_CLEAR_Y", STAGE1_STAGE2_TURN_Y, 1.20, 1.45)
STAGE1_BODY_MID_CLEAR_X_TOL = env_float("RACE_STAGE1_BODY_MID_CLEAR_X_TOL", 0.045, 0.03, 0.08)
STAGE1_GAP_PASSED_X = env_float("RACE_STAGE1_GAP_PASSED_X", 2.96, 2.90, 3.04)
STAGE1_GAP_PASSED_Y = env_float("RACE_STAGE1_GAP_PASSED_Y", STAGE1_STAGE2_TURN_Y, 1.20, 1.45)
STAGE1_GAP_PASSED_X_MAX = env_float("RACE_STAGE1_GAP_PASSED_X_MAX", 3.02, 3.00, 3.06)
STAGE1_REAR_CLEAR_Y = env_float("RACE_STAGE1_REAR_CLEAR_Y", STAGE1_STAGE2_TURN_Y, 1.20, 1.45)
STAGE1_REAR_CLEAR_X_MIN = env_float("RACE_STAGE1_REAR_CLEAR_X_MIN", 2.86, 2.60, 3.10)
STAGE1_REAR_CLEAR_X_MAX = env_float("RACE_STAGE1_REAR_CLEAR_X_MAX", 3.02, 3.00, 3.08)
STAGE1_ENTRY_RADIUS = 0.36
STAGE1_TIMEOUT_FORCE_EXIT_X = 2.55
STAGE2_ENTRY_POINT = STAGE1_GAP_INNER_POINT
# 第二赛段出口在寻珠区左上角，越过虚线后才进入 S 弯；原点位在 S 弯中段，切段过晚。
STAGE2_EXIT_POINT = (-0.18, 4.58)
STAGE2_EXIT_PATH_POINTS = [
    ("第二赛段左上缺口前", (-0.14, 3.98), 0.28),
    ("第二赛段左上缺口内", (-0.20, 4.34), 0.34),
    ("第二赛段左上出口", STAGE2_EXIT_POINT, 0.42),
]
STAGE2_TO_STAGE3_GATE_X_MIN = env_float("RACE_STAGE2_TO_STAGE3_GATE_X_MIN", -0.56, -0.75, -0.30)
STAGE2_TO_STAGE3_GATE_X_MAX = env_float("RACE_STAGE2_TO_STAGE3_GATE_X_MAX", 0.18, -0.05, 0.45)
STAGE2_TO_STAGE3_GATE_Y = env_float("RACE_STAGE2_TO_STAGE3_GATE_Y", 4.46, 4.20, 4.72)
STAGE3_ENTRY_POINT = (-0.28, 4.72)
STAGE3_CURVE_ENTRY_POINT = (-0.18, 5.00)
STAGE3_LANE_LOCK_POINT = (0.38, 5.18)
# 第三赛段 STL 边界实测在 y≈4.58..6.67。门点必须落在两侧黄线中心，
# 不能继续追到 y=8.x，否则会从 S 弯踩线出界。
STAGE3_PATH_POINTS = [
    ("S弯入口中心", (-0.30, 4.72), 0.34),
    ("S弯左侧上弯中心", (-0.10, 4.96), 0.32),
    ("S弯中下段中心", (0.42, 5.18), 0.32),
    ("S弯右摆入口中心", (1.06, 5.38), 0.34),
    ("S弯右摆中线", (1.72, 5.62), 0.34),
    ("S弯右上回正中心", (2.30, 5.92), 0.34),
    ("S弯出口右下角中心", (2.86, 6.34), 0.36),
    ("第四赛道右下角入口", (3.02, 6.62), 0.40),
]
STAGE3_EXIT_POINT = STAGE3_PATH_POINTS[-1][1]
STAGE4_ENTRY_POINT = STAGE3_EXIT_POINT
STAGE4_ENTRY_PATH_POINTS = [
    ("第四赛段入口右下角", STAGE4_ENTRY_POINT, 0.42),
    ("第四赛段右侧通道", (3.02, 7.20), 0.38),
    ("第四赛段限高杆入口", (2.60, 8.05), 0.42),
]
STAGE4_LOW_BAR_1 = (-0.13, 9.60)
STAGE4_LOW_BAR_2 = (2.07, 10.58)
STAGE4_COKE_POINT = (-0.10, 11.10)
STAGE4_ORANGE_BALL_POINT = (0.95, 11.10)
STAGE4_FOOTBALL_POINT = (2.10, 10.80)
STAGE4_OBSTACLE_POINT = (0.97, 8.56)
STAGE5_BRIDGE_PREJUMP = (3.05, 8.05)
STAGE5_BRIDGE_ENTRY = (3.12, 7.66)
STAGE4_EXIT_PATH_POINTS = [
    ("第四赛段右侧回撤", (2.38, 10.20), 0.40),
    ("第四赛段右下桥前", STAGE5_BRIDGE_PREJUMP, 0.45),
]
# 独木桥模型 y≈7.66~12.16；赛题要求终点前 50cm 跳下。
STAGE5_BRIDGE_END_Y = 12.16
STAGE5_JUMP_BEFORE_END = 0.50
STAGE5_BRIDGE_EXIT_Y = STAGE5_BRIDGE_END_Y - STAGE5_JUMP_BEFORE_END
STAGE6_FOOTBALL_POINT = (0.40, 14.70)
STAGE6_FINISH_POINT = (3.05, 13.35)
MAP_GOAL_RADIUS = 0.45

HSV_YELLOW_LOWER = (16, 60, 80)
HSV_YELLOW_UPPER = (45, 255, 255)
HSV_ORANGE_LOWER = (0, 120, 120)
HSV_ORANGE_UPPER = (25, 255, 255)
HSV_RED_LOWER1 = (0, 100, 100)
HSV_RED_UPPER1 = (10, 255, 255)
HSV_RED_LOWER2 = (160, 100, 100)
HSV_RED_UPPER2 = (179, 255, 255)
HSV_WHITE_LOWER = (0, 0, 160)
HSV_WHITE_UPPER = (180, 40, 255)
HSV_DARK_LOWER = (0, 0, 0)
HSV_DARK_UPPER = (180, 255, 75)

IMG_WIDTH = 320
IMG_HEIGHT = 240

IMU_ROLL_LIMIT = 0.55
IMU_PITCH_LIMIT = 0.65
IMU_ROLL_PREFALL = 0.28
IMU_PITCH_PREFALL = 0.36
IMU_ANGVEL_XY_PREFALL = 1.35
IMU_ANGVEL_Z_PREFALL = 1.65
PREFALL_BRACE_HOLD = 0.22
MOBILE_BODY_HEIGHT = S2_BODY
STONE_BODY_HEIGHT = env_float("RACE_STONE_BODY_H", BODY_HEIGHT, 0.19, 0.30)
LOW_BODY_HEIGHT = 0.18
BRACE_BODY_HEIGHT = 0.20
MOBILE_PITCH_BIAS = -0.045
POST_STAGE_SPEED_SCALE = env_float("RACE_POST_STAGE_SPEED_SCALE", 2.0, 1.0, 3.0)
POST_STAGE_YAW_SCALE = env_float("RACE_POST_STAGE_YAW_SCALE", 2.0, 1.0, 3.0)
MIN_TRAVEL_STEP_HEIGHT = S2_STEP
MIN_TRAVEL_STEP_HEIGHT_REAR = env_float("RACE_S2_STEP_REAR", S2_STEP, 0.03, 0.14)
HIGH_TRAVEL_STEP_HEIGHT = STEP_FLAT
HIGH_TRAVEL_STEP_HEIGHT_REAR = STEP_FLAT_REAR
OBSTACLE_STEP_HEIGHT = 0.13
STAIR_SAFE_STEP_HEIGHT = env_float("RACE_STAIR_SAFE_STEP", 0.135, 0.10, 0.14)
STAIR_SAFE_BODY_HEIGHT = env_float("RACE_STAIR_SAFE_BODY_H", 0.195, 0.18, 0.22)
STAIR_SAFE_PITCH_BIAS = env_float("RACE_STAIR_SAFE_PITCH_BIAS", 0.035, -0.02, 0.08)
STAIR_SAFE_VY_LIMIT = env_float("RACE_STAIR_SAFE_VY_LIMIT", 0.060, 0.0, 0.12)
STAIR_SAFE_YAW_LIMIT = env_float("RACE_STAIR_SAFE_YAW_LIMIT", 0.24, 0.08, 0.42)
STONE_COMPLIANCE_ALPHA = env_float("RACE_STONE_COMPLIANCE_ALPHA", 0.22, 0.10, 0.55)
STONE_ROLL_LIMIT = env_float("RACE_STONE_ROLL_LIMIT", 0.095, 0.0, 0.14)
STONE_PITCH_LIMIT = env_float("RACE_STONE_PITCH_LIMIT", 0.125, 0.04, 0.14)
STONE_RPY_RATE_LIMIT = env_float("RACE_STONE_RPY_RATE", 0.032, 0.010, 0.070)
STONE_ROLL_KP = env_float("RACE_STONE_ROLL_KP", 0.40, 0.0, 0.80)
STONE_ROLL_KD = env_float("RACE_STONE_ROLL_KD", 0.065, 0.0, 0.090)
STONE_PITCH_KP = env_float("RACE_STONE_PITCH_KP", 0.36, 0.0, 0.70)
STONE_PITCH_KD = env_float("RACE_STONE_PITCH_KD", 0.058, 0.0, 0.080)
STONE_CLIMB_PITCH_BIAS = env_float("RACE_STONE_PITCH_BIAS", 0.050, -0.04, 0.09)
STONE_STEPUP_PITCH_BIAS = env_float("RACE_STONE_STEPUP_PITCH", 0.095, 0.03, 0.13)
STONE_FRONT_PAIR_PITCH_BIAS = env_float("RACE_STONE_FRONT_PAIR_PITCH", 0.105, 0.05, 0.14)
STONE_FRONT_PAIR_BODY_DROP = env_float("RACE_STONE_FRONT_PAIR_BODY_DROP", 0.010, 0.0, 0.025)
STONE_IMPACT_PITCH_GIVE = env_float("RACE_STONE_PITCH_GIVE", 0.030, 0.0, 0.055)
STONE_IMPACT_ROLL_GIVE = env_float("RACE_STONE_ROLL_GIVE", 0.012, 0.0, 0.035)
STONE_CONTACT_RATE_SLOWDOWN = env_float("RACE_STONE_CONTACT_RATE", 0.32, 0.20, 0.90)

STAGE_TIMEOUTS = {1: 900, 2: 900, 3: 900, 4: 900, 5: 900, 6: 900}

STAGE_NAMES = {
    0: "初始化",
    1: "第一赛段-石径探路",
    2: "第二赛段-荒野寻珠",
    3: "第三赛段-曲道冲锋",
    4: "第四赛段-深隧寻珍",
    5: "第五赛段-孤梁稳渡",
    6: "第六赛段-撷金建功",
}

ORANGE_BALL_TOTAL = 4

# 第二赛段直接追四个橙球固定坐标；不要先执行入口 guard 或折线门点。
STAGE2_FIXED_TARGETS = [
    {
        "name": "map_row4_col2_1area_ball1_2",
        "ball": (0.8, 1.34),
        "link": "hanging_ball::1area_ball1_2",
        "route": [],
        "route_radius": 0.20,
        "route_vx": 1.08,
        "strike": (0.8, 1.34),
        "coord_hit_radius": 0.22,
        "hit_travel_dist": 0.18,
        "hit_search_vx": 1.02,
        "hit_vx": 0.92,
        "pass_counts": True,
    },
    {
        "name": "map_row3_col3_1area_ball2_3",
        "ball": (2.0, 2.18),
        "link": "hanging_ball::1area_ball2_3",
        "route": [],
        "route_vx": 1.08,
        "route_radius": 0.20,
        "strike": (2.0, 2.18),
        "coord_hit_radius": 0.22,
        "hit_search_vx": 1.02,
        "hit_vx": 0.92,
    },
    {
        "name": "map_row2_col4_1area_ball3_4",
        "ball": (3.2, 3.02),
        "link": "hanging_ball::1area_ball3_4",
        "route": [],
        "strike": (3.2, 3.02),
        "coord_hit_radius": 0.20,
        "route_vx": 1.02,
        "hit_vx": 0.080,
        "hit_search_vx": 0.94,
    },
    {
        "name": "map_row1_col1_1area_ball4_1",
        "ball": (-0.4, 3.86),
        "link": "hanging_ball2::1area_ball4_1",
        "route": [],
        "strike": (-0.4, 3.86),
        "coord_hit_radius": 0.20,
        "route_vx": 1.08,
        "hit_search_vx": 1.02,
        "hit_vx": 0.075,
    },
]
STAGE2_BLUE_BALLS = [
    (-0.4, 1.34), (2.0, 1.34), (3.2, 1.34),
    (-0.4, 2.18), (0.8, 2.18), (3.2, 2.18),
    (-0.4, 3.02), (0.8, 3.02), (2.0, 3.02),
    (0.8, 3.86), (2.0, 3.86), (3.2, 3.86),
]
STAGE2_APPROACH_RADIUS = env_float("RACE_STAGE2_APPROACH_RADIUS", 0.30, 0.18, 0.55)
STAGE2_ALIGN_RADIUS = env_float("RACE_STAGE2_ALIGN_RADIUS", 0.40, 0.20, 0.70)
STAGE2_HIT_RADIUS = env_float("RACE_STAGE2_HIT_RADIUS", 0.30, 0.12, 0.40)
STAGE2_HEAD_OFFSET = env_float("RACE_STAGE2_HEAD_OFFSET", 0.38, 0.20, 0.60)
STAGE2_HEAD_HIT_RADIUS = env_float("RACE_STAGE2_HEAD_HIT_RADIUS", 0.15, 0.10, 0.36)
STAGE2_HEAD_YAW_TOL = env_float("RACE_STAGE2_HEAD_YAW_TOL", 0.20, 0.12, 0.65)
STAGE2_STRIKE_RADIUS = env_float("RACE_STAGE2_STRIKE_RADIUS", 0.24, 0.12, 0.42)
STAGE2_VISUAL_FUSE_DIST = env_float("RACE_STAGE2_VISUAL_FUSE_DIST", 1.10, 0.25, 1.50)
STAGE2_VISUAL_STEER_WEIGHT = env_float("RACE_STAGE2_VISUAL_STEER_WEIGHT", 0.45, 0.0, 0.80)
STAGE2_VISUAL_MAX_X = env_float("RACE_STAGE2_VISUAL_MAX_X", 0.42, 0.15, 0.80)
STAGE2_VISUAL_STEER_GATE = env_float("RACE_STAGE2_VISUAL_STEER_GATE", 0.34, 0.08, 0.70)
STAGE2_ORANGE_MIN_AREA = env_float("RACE_STAGE2_ORANGE_MIN_AREA", 80, 30, 220)
STAGE2_ORANGE_CENTER_X = env_float("RACE_STAGE2_ORANGE_CENTER_X", 0.24, 0.08, 0.45)
STAGE2_ORANGE_HIT_AREA = env_float("RACE_STAGE2_ORANGE_HIT_AREA", 220, 80, 500)
STAGE2_ROUTE_RADIUS = env_float("RACE_STAGE2_ROUTE_RADIUS", 0.30, 0.12, 0.45)
STAGE2_ROUTE_TIMEOUT = env_float("RACE_STAGE2_ROUTE_TIMEOUT", 0.9, 0.5, 5.0)
STAGE2_ROUTE_VX = env_float("RACE_STAGE2_ROUTE_VX", 1.080, 0.06, 1.30)
STAGE2_ALIGN_VX = env_float("RACE_STAGE2_ALIGN_VX", 1.020, 0.04, 1.25)
STAGE2_ALIGN_VISIBLE_VX = env_float("RACE_STAGE2_ALIGN_VISIBLE_VX", 0.940, 0.02, 1.15)
STAGE2_HIT_VX = env_float("RACE_STAGE2_HIT_VX", 1.180, 0.08, 1.35)
STAGE2_HIT_SEARCH_VX = env_float("RACE_STAGE2_HIT_SEARCH_VX", 1.020, 0.03, 1.20)
STAGE2_VISUAL_MEMORY_SEC = env_float("RACE_STAGE2_VISUAL_MEMORY_SEC", 1.7, 0.4, 3.0)
STAGE2_HIT_CONFIRM_TOF = env_float("RACE_STAGE2_HIT_CONFIRM_TOF", 0.34, 0.18, 0.55)
STAGE2_HIT_TIMEOUT = env_float("RACE_STAGE2_HIT_TIMEOUT", 2.6, 1.5, 7.0)
STAGE2_HIT_TRAVEL_DIST = env_float("RACE_STAGE2_HIT_TRAVEL_DIST", 0.38, 0.18, 0.70)
STAGE2_HIT_TRAVEL_MIN_SEC = env_float("RACE_STAGE2_HIT_TRAVEL_MIN_SEC", 0.25, 0.10, 1.60)
STAGE2_AIM_PASS_DIST = env_float("RACE_STAGE2_AIM_PASS_DIST", 0.04, 0.00, 0.32)
STAGE2_CONFIRM_NUDGE_VX = env_float("RACE_STAGE2_CONFIRM_NUDGE_VX", 0.0, 0.0, 0.60)
STAGE2_CONFIRM_NUDGE_MS = env_int("RACE_STAGE2_CONFIRM_NUDGE_MS", 0, 0, 500)
STAGE2_EXIT_VX = env_float("RACE_STAGE2_EXIT_VX", S2_VX, 0.06, 0.75)
STAGE2_VISUAL_HIT_MAX_DIST = env_float("RACE_STAGE2_VISUAL_HIT_MAX_DIST", 0.30, 0.22, 0.75)
STAGE2_BALL_SHAKE_DIST = env_float("RACE_STAGE2_BALL_SHAKE_DIST", 0.025, 0.015, 0.08)
STAGE2_INPLACE_TURN_YAW = env_float("RACE_STAGE2_INPLACE_TURN_YAW", 0.42, 0.20, 1.20)
STAGE2_INPLACE_TURN_DONE_YAW = env_float("RACE_STAGE2_INPLACE_TURN_DONE_YAW", 0.16, 0.06, 0.45)
STAGE2_INPLACE_TURN_RATE = env_float("RACE_STAGE2_INPLACE_TURN_RATE", 3.20, 0.16, 3.60)
STAGE2_FIRST_BALL_TURN_DONE_YAW = env_float("RACE_STAGE2_FIRST_BALL_TURN_DONE_YAW", 0.26, 0.08, 0.60)
STAGE2_FIRST_BALL_MIN_VX = env_float("RACE_STAGE2_FIRST_BALL_MIN_VX", 0.26, 0.05, 0.80)
STAGE2_DIRECT_MIN_VX = env_float("RACE_STAGE2_DIRECT_MIN_VX", 0.24, 0.05, 0.85)
STAGE2_DIRECT_YAW_GAIN = env_float("RACE_STAGE2_DIRECT_YAW_GAIN", 1.35, 0.20, 2.50)
STAGE2_DIRECT_YAW_LIMIT = env_float("RACE_STAGE2_DIRECT_YAW_LIMIT", 1.60, 0.30, 2.80)
STAGE2_STEER_LIMIT = env_float("RACE_STAGE2_STEER_LIMIT", 1.80, 0.50, 2.60)
STAGE2_STEER_GAIN = env_float("RACE_STAGE2_STEER_GAIN", 1.45, 0.40, 2.20)
STAGE2_FIRST_TARGET_ARC_VX = env_float("RACE_STAGE2_FIRST_TARGET_ARC_VX", 0.98, 0.20, 1.20)
STAGE2_FIRST_TARGET_ARC_YAW_LIMIT = env_float("RACE_STAGE2_FIRST_TARGET_ARC_YAW_LIMIT", 2.05, 0.60, 2.60)
STAGE2_COORD_VECTOR_VY_LIMIT = env_float("RACE_STAGE2_COORD_VECTOR_VY_LIMIT", 0.46, 0.10, 1.20)
STAGE2_COORD_VECTOR_YAW_LIMIT = env_float("RACE_STAGE2_COORD_VECTOR_YAW_LIMIT", 2.60, 0.80, 3.20)
STAGE2_COORD_TURN_START_YAW = env_float("RACE_STAGE2_COORD_TURN_START_YAW", 0.32, 0.10, 0.70)
STAGE2_COORD_TURN_DONE_YAW = env_float("RACE_STAGE2_COORD_TURN_DONE_YAW", 0.12, 0.04, 0.28)
STAGE2_COORD_MOVE_YAW_SLOW = env_float("RACE_STAGE2_COORD_MOVE_YAW_SLOW", 0.42, 0.15, 0.80)
STAGE2_COORD_FACE_YAW = env_float("RACE_STAGE2_COORD_FACE_YAW", 0.42, 0.15, 0.95)
STAGE2_ENTRY_FACE_DONE_YAW = env_float("RACE_STAGE2_ENTRY_FACE_DONE_YAW", 0.18, 0.06, 0.45)
STAGE2_FIRST_BALL_DRIVE_YAW_LIMIT = env_float("RACE_STAGE2_FIRST_BALL_DRIVE_YAW_LIMIT", 0.46, 0.18, 0.90)
STAGE2_FIRST_BALL_DRIVE_YAW_GAIN = env_float("RACE_STAGE2_FIRST_BALL_DRIVE_YAW_GAIN", 0.42, 0.15, 0.90)
STAGE2_FIRST_BALL_PROGRESS_EPS = env_float("RACE_STAGE2_FIRST_BALL_PROGRESS_EPS", 0.03, 0.005, 0.10)
STAGE2_FIRST_BALL_STALL_LIMIT = env_int("RACE_STAGE2_FIRST_BALL_STALL_LIMIT", 4, 2, 12)
STAGE2_FIRST_BALL_FORCE_VX = env_float("RACE_STAGE2_FIRST_BALL_FORCE_VX", 0.46, 0.10, 0.80)
STAGE2_FIRST_BALL_FORCE_VY = env_float("RACE_STAGE2_FIRST_BALL_FORCE_VY", 0.28, 0.08, 0.55)
STAGE2_COORD_NAV_MAX_VX = env_float("RACE_STAGE2_COORD_NAV_MAX_VX", S2_VX, 0.08, 1.20)
STAGE2_COORD_NAV_MAX_VY = env_float("RACE_STAGE2_COORD_NAV_MAX_VY", 0.08, 0.04, 0.90)
STAGE2_COORD_NAV_MIN_SPEED = env_float("RACE_STAGE2_COORD_NAV_MIN_SPEED", 0.08, 0.05, 0.50)
STAGE2_COORD_NAV_GAIN = env_float("RACE_STAGE2_COORD_NAV_GAIN", 0.28, 0.08, 0.90)
STAGE2_SIMPLE_COORD_MODE = env_int("RACE_STAGE2_SIMPLE_COORD", 1, 0, 1) > 0
STAGE2_SIMPLE_REACHED_RADIUS = env_float("RACE_STAGE2_SIMPLE_RADIUS", 0.32, 0.18, 0.55)
STAGE2_SIMPLE_FACE_YAW = env_float("RACE_STAGE2_SIMPLE_FACE_YAW", 0.52, 0.10, 0.95)
STAGE2_SIMPLE_TURN_DONE_YAW = env_float("RACE_STAGE2_SIMPLE_TURN_DONE_YAW", 0.30, 0.06, 0.55)
STAGE2_SIMPLE_TURN_START_YAW = env_float("RACE_STAGE2_SIMPLE_TURN_START_YAW", 0.85, 0.12, 3.14)
STAGE2_SIMPLE_DRIVE_VX = env_float("RACE_STAGE2_SIMPLE_DRIVE_VX", S2_VX, 0.04, 0.75)
STAGE2_SIMPLE_TURN_VX = env_float("RACE_STAGE2_SIMPLE_TURN_VX", min(S2_VX, 0.08), 0.0, 0.18)
STAGE2_FIRST_ARC_VX = env_float("RACE_STAGE2_FIRST_ARC_VX", S2_VX, 0.04, 0.55)
STAGE2_SIMPLE_DRIVE_VY = env_float("RACE_STAGE2_SIMPLE_DRIVE_VY", 0.08, 0.0, 0.35)
STAGE2_SIMPLE_MIN_WORLD_SPEED = env_float("RACE_STAGE2_SIMPLE_MIN_WORLD_SPEED", S2_VX, 0.06, 0.60)
STAGE2_SIMPLE_ARC_YAW = env_float("RACE_STAGE2_SIMPLE_ARC_YAW", 1.35, 0.45, 2.40)
STAGE2_SIMPLE_STEER_LIMIT = env_float("RACE_STAGE2_SIMPLE_STEER_LIMIT", 1.05, 0.25, 2.80)
STAGE2_SIMPLE_TURN_STEER_LIMIT = env_float("RACE_STAGE2_SIMPLE_TURN_STEER_LIMIT", 1.35, 0.35, 2.80)
STAGE2_SIMPLE_PROGRESS_EPS = env_float("RACE_STAGE2_SIMPLE_PROGRESS_EPS", 0.025, 0.005, 0.08)
STAGE2_SIMPLE_STALL_SEC = env_float("RACE_STAGE2_SIMPLE_STALL_SEC", 1.45, 0.50, 3.00)
STAGE2_SIMPLE_STALL_VX = env_float("RACE_STAGE2_SIMPLE_STALL_VX", S2_VX, 0.04, 0.60)
STAGE2_BLUE_AVOID_FORWARD = env_float("RACE_STAGE2_BLUE_AVOID_FORWARD", 0.82, 0.30, 1.30)
STAGE2_BLUE_AVOID_LATERAL = env_float("RACE_STAGE2_BLUE_AVOID_LATERAL", 0.30, 0.16, 0.55)
STAGE2_BLUE_AVOID_VY = env_float("RACE_STAGE2_BLUE_AVOID_VY", 0.06, 0.02, 0.35)
STAGE2_BLUE_AVOID_STEER = env_float("RACE_STAGE2_BLUE_AVOID_STEER", 0.18, 0.04, 0.80)
STAGE2_BOUND_X_MIN = env_float("RACE_STAGE2_BOUND_X_MIN", -0.56, -0.70, -0.20)
STAGE2_BOUND_X_MAX = env_float("RACE_STAGE2_BOUND_X_MAX", 3.02, 2.80, 3.18)
STAGE2_BOUND_Y_MIN = env_float("RACE_STAGE2_BOUND_Y_MIN", 0.72, 0.50, 1.10)
STAGE2_BOUND_Y_MAX = env_float("RACE_STAGE2_BOUND_Y_MAX", 4.42, 4.00, 4.70)
STAGE2_BOUND_MARGIN = env_float("RACE_STAGE2_BOUND_MARGIN", 0.18, 0.08, 0.34)
STAGE2_BOUND_CORRECT_GAIN = env_float("RACE_STAGE2_BOUND_GAIN", 1.60, 0.15, 2.40)
STAGE2_BOUND_CORRECT_LIMIT = env_float("RACE_STAGE2_BOUND_LIMIT", 0.18, 0.05, 0.40)
STAGE2_SIMPLE_RGB_ALIGN_DIST = env_float("RACE_STAGE2_SIMPLE_RGB_ALIGN_DIST", 1.25, 0.40, 1.80)
STAGE2_SIMPLE_RGB_STEER_GAIN = env_float("RACE_STAGE2_SIMPLE_RGB_STEER_GAIN", 0.72, 0.20, 1.30)
STAGE2_SIMPLE_RGB_WEIGHT = env_float("RACE_STAGE2_SIMPLE_RGB_WEIGHT", 0.45, 0.10, 0.80)
STAGE2_AXIS_TURN_YAW = env_float("RACE_STAGE2_AXIS_TURN_YAW", 1.05, 0.08, 1.40)
STAGE2_AXIS_STEER_LIMIT = env_float("RACE_STAGE2_AXIS_STEER_LIMIT", 0.34, 0.18, 1.80)
STAGE2_AXIS_DRIVE_STEER_LIMIT = env_float("RACE_STAGE2_AXIS_DRIVE_STEER_LIMIT", 0.0, 0.0, 0.20)
STAGE2_AXIS_TURN_MAX_SEC = env_float("RACE_STAGE2_AXIS_TURN_MAX_SEC", 3.20, 0.25, 5.00)
STAGE2_AXIS_DRIVE_PULSE_SEC = env_float("RACE_STAGE2_AXIS_DRIVE_PULSE_SEC", 0.45, 0.15, 1.20)
STAGE2_AXIS_FORCE_DRIVE_VX = env_float("RACE_STAGE2_AXIS_FORCE_DRIVE_VX", 0.24, 0.08, 0.50)
STAGE2_FINAL_APPROACH_DIST = env_float("RACE_STAGE2_FINAL_APPROACH_DIST", 0.48, 0.35, 1.20)
STAGE2_FINAL_APPROACH_YAW = env_float("RACE_STAGE2_FINAL_APPROACH_YAW", 0.24, 0.12, 0.70)
STAGE2_FINAL_APPROACH_VX = env_float("RACE_STAGE2_FINAL_APPROACH_VX", S2_VX, 0.04, 0.35)
STAGE2_FINAL_APPROACH_STEER_LIMIT = env_float("RACE_STAGE2_FINAL_APPROACH_STEER_LIMIT", 0.24, 0.08, 0.45)
STAGE2_HEAD_ARC_YAW = env_float("RACE_STAGE2_HEAD_ARC_YAW", 0.30, 0.12, 0.80)
STAGE2_HEAD_ARC_VY_SCALE = env_float("RACE_STAGE2_HEAD_ARC_VY_SCALE", 0.25, 0.0, 0.60)
STAGE2_HEAD_ARC_STEER_LIMIT = env_float("RACE_STAGE2_HEAD_ARC_STEER_LIMIT", 0.68, 0.20, 1.20)
STAGE2_COORD_ARC_TURN_YAW = env_float("RACE_STAGE2_COORD_ARC_TURN_YAW", 0.85, 0.35, 1.40)
STAGE2_COORD_ARC_VX_SCALE = env_float("RACE_STAGE2_COORD_ARC_VX_SCALE", 0.45, 0.20, 0.80)
STAGE2_FIRST_CURVE_TURN_ENABLED = env_int("RACE_STAGE2_FIRST_CURVE_TURN_ENABLED", 0, 0, 1) > 0
STAGE2_FIRST_CURVE_TURN_VX = env_float("RACE_STAGE2_FIRST_CURVE_TURN_VX", 0.450, 0.08, 0.52)
STAGE2_FIRST_CURVE_TURN_YAW_LIMIT = env_float("RACE_STAGE2_FIRST_CURVE_TURN_YAW_LIMIT", 0.72, 0.28, 0.95)
FSM_SCAN_TARGET = 0
FSM_COMPUTE_PATH = 1
FSM_ALIGN = 2
FSM_MOVE_TO_TARGET = 3
FSM_TASK_TRIGGER = 4
FSM_BACK_TO_CENTER = 5
FSM_NEXT_SEGMENT = 6
FSM_ROTATE_SEARCH = 7

FSM_NAMES = {
    0: "ScanTarget",
    1: "ComputePath",
    2: "Align",
    3: "MoveToTarget",
    4: "TaskTrigger",
    5: "BackToCenter",
    6: "NextSegment",
    7: "RotateSearch",
}


class SensorNode(Node):
    def __init__(self):
        super().__init__('race_sensor_node')
        qos_best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.tof_range = 99.0
        self.tof_time = 0.0

        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_angvel_x = 0.0
        self.imu_angvel_y = 0.0
        self.imu_angvel_z = 0.0
        self.imu_time = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_time = 0.0
        self.odom_got = False
        self.odom_frame = ""
        self.odom_miss_count = 0
        self.gazebo_pose_got = False
        self.gazebo_pose_x = 0.0
        self.gazebo_pose_y = 0.0
        self.gazebo_pose_yaw = 0.0
        self.gazebo_pose_vx = 0.0
        self.gazebo_pose_vy = 0.0
        self.gazebo_pose_time = 0.0
        self.gazebo_pose_link = ""

        self.rgb_image_raw = None
        self.rgb_image_time = 0.0
        self.link_positions = {}
        self.link_states_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_tof = self.create_subscription(
            LaserScan, '/tof_scan', self._tof_cb, qos_best)
        self.sub_imu = self.create_subscription(
            Imu, '/imu', self._imu_cb, qos_best)
        self.sub_rgb = self.create_subscription(
            Image, '/camera/rgb/image_raw', self._rgb_cb, qos_best)
        if LinkStates is not None:
            self.sub_link_states = self.create_subscription(
                LinkStates, '/gazebo/link_states', self._link_states_cb, qos_best)

    def update_odom(self):
        if (
            POSE_SOURCE != "tf" and
            self.gazebo_pose_got and
            time.time() - self.gazebo_pose_time <= GAZEBO_POSE_STALE_SEC
        ):
            self.odom_x = self.gazebo_pose_x
            self.odom_y = self.gazebo_pose_y
            self.odom_yaw = self.gazebo_pose_yaw
            self.odom_vx = self.gazebo_pose_vx
            self.odom_vy = self.gazebo_pose_vy
            self.odom_time = self.gazebo_pose_time
            self.odom_got = True
            self.odom_frame = f"gazebo:{self.gazebo_pose_link}"
            self.odom_miss_count = 0
            return True

        frames = []
        if self.odom_frame:
            frames.append(self.odom_frame)
        for frame in ODOM_FRAME_CANDIDATES:
            if frame not in frames:
                frames.append(frame)

        for frame in frames:
            try:
                t = self.tf_buffer.lookup_transform(frame, BASE_FRAME, rclpy.time.Time())
                now = time.time()
                new_x = t.transform.translation.x
                new_y = t.transform.translation.y
                if self.odom_got and self.odom_time > 0.0:
                    dt = now - self.odom_time
                    if 0.01 < dt < 0.30:
                        self.odom_vx = (new_x - self.odom_x) / dt
                        self.odom_vy = (new_y - self.odom_y) / dt
                self.odom_x = new_x
                self.odom_y = new_y
                q = t.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.odom_yaw = math.atan2(siny, cosy)
                self.odom_time = now
                self.odom_got = True
                self.odom_frame = frame
                self.odom_miss_count = 0
                return True
            except Exception:
                continue

        self.odom_miss_count += 1
        if time.time() - self.odom_time > 0.5:
            self.odom_got = False
        return False

    def _tof_cb(self, msg):
        if msg.ranges and len(msg.ranges) > 0:
            r = msg.ranges[0]
            if msg.range_min < r < msg.range_max:
                self.tof_range = r
            else:
                self.tof_range = 99.0
        self.tof_time = time.time()

    def _rgb_cb(self, msg):
        self.rgb_image_raw = msg
        self.rgb_image_time = time.time()

    def _link_states_cb(self, msg):
        now = time.time()
        positions = {}
        robot_pose = None
        robot_name = ""
        for name, pose in zip(msg.name, msg.pose):
            if "1area_ball" in name or "2area_ball" in name or "football" in name or "coke" in name:
                positions[name] = (pose.position.x, pose.position.y, pose.position.z)
            lower_name = name.lower()
            blocked_pose_link = (
                "hanging_ball" in lower_name or
                "football" in lower_name or
                "coke" in lower_name or
                lower_name.startswith("race::") or
                lower_name.startswith("ground_plane::")
            )
            if robot_pose is None and (
                name in GAZEBO_ROBOT_LINK_CANDIDATES or
                (
                    not blocked_pose_link and
                    (name.endswith("::base_link") or name.endswith("::body"))
                )
            ):
                robot_pose = pose
                robot_name = name
        if positions:
            self.link_positions = positions
            self.link_states_time = now
        if robot_pose is not None:
            new_x = robot_pose.position.x
            new_y = robot_pose.position.y
            if self.gazebo_pose_got and self.gazebo_pose_time > 0.0:
                dt = now - self.gazebo_pose_time
                if 0.01 < dt < 0.30:
                    self.gazebo_pose_vx = (new_x - self.gazebo_pose_x) / dt
                    self.gazebo_pose_vy = (new_y - self.gazebo_pose_y) / dt
            self.gazebo_pose_x = new_x
            self.gazebo_pose_y = new_y
            q = robot_pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.gazebo_pose_yaw = math.atan2(siny, cosy)
            self.gazebo_pose_time = now
            self.gazebo_pose_got = True
            self.gazebo_pose_link = robot_name

    def _imu_cb(self, msg):
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.imu_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.imu_pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.imu_pitch = math.asin(sinp)
        self.imu_angvel_x = msg.angular_velocity.x
        self.imu_angvel_y = msg.angular_velocity.y
        self.imu_angvel_z = msg.angular_velocity.z
        self.imu_time = time.time()

    def is_fallen(self):
        return abs(self.imu_roll) > IMU_ROLL_LIMIT or abs(self.imu_pitch) > IMU_PITCH_LIMIT


def image_msg_to_cv(msg):
    if msg is None:
        return None
    if msg.encoding == 'rgb8':
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'bgr8':
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding == 'bgra8':
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return None


def front_down_roi_mask(h, w):
    """
    近似相机前方+下方四分之一球面视野：保留中前方和脚下宽视野，
    石板起伏导致黄线靠近镜头时也不会被固定水平带裁掉。
    """
    mask = np.zeros((h, w), dtype=np.uint8)
    y_start = int(h * 0.16)
    for y in range(y_start, h):
        t = (y - y_start) / max(1.0, h - y_start)
        half_width = int((0.30 + 0.70 * t) * w * 0.5)
        cx = w // 2
        x0 = max(0, cx - half_width)
        x1 = min(w, cx + half_width)
        mask[y, x0:x1] = 255

    ellipse = np.zeros_like(mask)
    cv2.ellipse(
        ellipse,
        (w // 2, int(h * 0.98)),
        (int(w * 0.72), int(h * 0.88)),
        0,
        190,
        350,
        255,
        -1,
    )
    lower_band = np.zeros_like(mask)
    lower_band[int(h * 0.58):h, :] = 255
    return cv2.bitwise_or(cv2.bitwise_or(mask, ellipse), lower_band)


def build_track_line_mask(bgr, include_white=True, include_edges=True):
    if bgr is None:
        return None

    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER)
    relaxed_yellow = cv2.inRange(hsv, (12, 35, 45), (50, 255, 255))
    line_mask = cv2.bitwise_or(yellow_mask, relaxed_yellow)
    if include_white:
        white_mask = cv2.inRange(hsv, HSV_WHITE_LOWER, HSV_WHITE_UPPER)
        line_mask = cv2.bitwise_or(line_mask, white_mask)

    roi_mask = front_down_roi_mask(h, w)
    line_mask = cv2.bitwise_and(line_mask, roi_mask)
    kernel = np.ones((3, 3), np.uint8)
    line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_CLOSE, kernel)
    line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_OPEN, kernel)

    if include_edges:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 35, 115)
        color_neighborhood = cv2.dilate(line_mask, kernel, iterations=1)
        edges = cv2.bitwise_and(edges, color_neighborhood)
        line_mask = cv2.bitwise_or(line_mask, edges)

    return line_mask


def detect_centerline(bgr):
    """
    多区域中线检测 — 前下方宽 ROI + 多条水平带投票
    石板路起伏时部分带失效不影响整体结果
    返回 (offset[-1,1], ok)
    """
    if bgr is None:
        return 0.0, False
    h, w = bgr.shape[:2]
    line_mask = build_track_line_mask(bgr, include_white=True, include_edges=True)

    zones = [
        (int(h * 0.18), int(h * 0.36), 0.8),
        (int(h * 0.36), int(h * 0.54), 1.0),
        (int(h * 0.54), int(h * 0.72), 1.4),
        (int(h * 0.72), h, 1.9),
        (int(h * 0.22), h, 1.1),
    ]

    offsets = []
    weights = []

    for y0, y1, zone_wt in zones:
        y0 = max(0, y0)
        y1 = min(h, y1)
        if y1 <= y0:
            continue
        roi = line_mask[y0:y1, :]
        m = cv2.moments(roi)
        area = m['m00']
        if area > 50:
            cx = m['m10'] / area
            off = (cx - w / 2.0) / (w / 2.0)
            offsets.append(off)
            weights.append(min(area * zone_wt, 900))

    if not offsets:
        line_mask2 = build_track_line_mask(bgr, include_white=True, include_edges=False)
        roi_full = line_mask2[int(h * 0.16):h, :]
        m2 = cv2.moments(roi_full)
        if m2['m00'] > 30:
            cx = m2['m10'] / m2['m00']
            return (cx - w / 2.0) / (w / 2.0), True
        return 0.0, False

    weighted_sum = sum(o * wt for o, wt in zip(offsets, weights))
    total_weight = sum(weights)
    if total_weight > 0:
        return weighted_sum / total_weight, True

    return 0.0, False


def detect_by_hsv(bgr, lower, upper, min_area=80):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0
    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < min_area:
        return None, area
    (cx, cy), _ = cv2.minEnclosingCircle(best)
    return (int(cx), int(cy)), area


def detect_orange_ball(bgr):
    return detect_by_hsv(bgr, HSV_ORANGE_LOWER, HSV_ORANGE_UPPER, min_area=40)


def detect_red_object(bgr):
    center1, a1 = detect_by_hsv(bgr, HSV_RED_LOWER1, HSV_RED_UPPER1, min_area=40)
    center2, a2 = detect_by_hsv(bgr, HSV_RED_LOWER2, HSV_RED_UPPER2, min_area=40)
    if a1 >= a2:
        return center1, a1
    return center2, a2


def detect_coke_bottle(bgr):
    """可乐瓶在官方 world 中是深色竖直圆柱，不能按红色外膜识别。"""
    if bgr is None:
        return None, 0.0
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_DARK_LOWER, HSV_DARK_UPPER)
    mask[:int(h * 0.15), :] = 0
    mask[int(h * 0.90):, :] = 0
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_center = None
    best_score = 0.0
    best_area = 0.0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 35:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw <= 0 or bh <= 0:
            continue
        aspect = bh / float(bw)
        if aspect < 1.15 or bh < 10:
            continue
        cx = x + bw / 2.0
        cy = y + bh / 2.0
        center_bias = 1.0 - min(1.0, abs(cx - w / 2.0) / (w / 2.0))
        score = area * (0.5 + center_bias) * min(aspect, 3.0)
        if score > best_score:
            best_score = score
            best_area = area
            best_center = (int(cx), int(cy))
    return best_center, best_area


def rear_step_for(front_step):
    return STONE_CLIMB_STEP_HEIGHT_REAR if front_step >= STONE_CLIMB_STEP_HEIGHT else STONE_APPROACH_STEP_HEIGHT_REAR


def max_step_pair(step_h, front_step, rear_step=None):
    if rear_step is None:
        rear_step = rear_step_for(front_step)
    if isinstance(step_h, (list, tuple)):
        current_front = step_h[0]
        current_rear = step_h[1] if len(step_h) > 1 else rear_step_for(current_front)
    else:
        current_front = step_h
        current_rear = rear_step_for(step_h)
    return (max(current_front, front_step), max(current_rear, rear_step))


def format_step_height(step_h):
    if isinstance(step_h, (list, tuple)):
        rear = step_h[1] if len(step_h) > 1 else step_h[0]
        return f"{step_h[0]:.3f}/{rear:.3f}"
    return f"{step_h:.3f}"


def detect_football(bgr):
    """官方仿真足球主体是白色球体，和橙色球应分开检测。"""
    if bgr is None:
        return None, 0.0
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_WHITE_LOWER, HSV_WHITE_UPPER)
    mask[:int(h * 0.10), :] = 0
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_center = None
    best_score = 0.0
    best_area = 0.0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 45:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        if radius < 4:
            continue
        circle_area = math.pi * radius * radius
        fill = area / circle_area if circle_area > 0 else 0.0
        x, y, bw, bh = cv2.boundingRect(cnt)
        aspect = bw / float(bh) if bh > 0 else 0.0
        if fill < 0.35 or aspect < 0.55 or aspect > 1.8:
            continue
        center_bias = 1.0 - min(1.0, abs(cx - w / 2.0) / (w / 2.0))
        score = area * (0.6 + center_bias)
        if score > best_score:
            best_score = score
            best_area = area
            best_center = (int(cx), int(cy))
    return best_center, best_area


def tof_blocked(sensor):
    return TOF_OBSTACLE_MIN < sensor.tof_range < TOF_OBSTACLE_MAX


def smooth_centerline(history, new_offset, window=8):
    history.append(new_offset)
    if len(history) > window:
        history.pop(0)
    return sum(history) / len(history)


def detect_yellow_line_bounds(bgr):
    """
    多区域左右黄线边界检测 — 前下方宽 ROI + 多带峰值融合
    下半幅全宽参与，近处黄线落入脚下盲区前仍能保持定位。
    返回: (left_ok, left_norm, right_ok, right_norm, center_norm, cm_offset)
    """
    if bgr is None:
        return False, 0.0, False, 0.0, 0.0, 0.0

    h, w = bgr.shape[:2]
    half_w = w // 2
    line_mask = build_track_line_mask(bgr, include_white=False, include_edges=True)

    zones = [
        (int(h * 0.18), int(h * 0.38), 0.8),
        (int(h * 0.38), int(h * 0.58), 1.1),
        (int(h * 0.58), int(h * 0.78), 1.6),
        (int(h * 0.78), h, 2.1),
        (int(h * 0.58), h, 1.8),
    ]

    all_left_norm = []
    all_right_norm = []
    left_weights = []
    right_weights = []

    for y0, y1, zone_wt in zones:
        y0, y1 = max(0, y0), min(h, y1)
        if y1 <= y0:
            continue
        roi = line_mask[y0:y1, :]

        left_roi = roi[:, :half_w]
        right_roi = roi[:, half_w:]

        left_col = np.sum(left_roi, axis=0)
        right_col = np.sum(right_roi, axis=0)

        left_max = float(np.max(left_col))
        right_max = float(np.max(right_col))

        if left_max > 45:
            left_idx = int(np.argmax(left_col))
            left_n = (half_w - left_idx) / half_w
            all_left_norm.append(left_n)
            left_weights.append(min(left_max * zone_wt, 900))

        if right_max > 45:
            right_idx = int(np.argmax(right_col))
            right_n = -(right_idx / half_w)
            all_right_norm.append(right_n)
            right_weights.append(min(right_max * zone_wt, 900))

    left_ok = len(all_left_norm) > 0
    right_ok = len(all_right_norm) > 0

    l_norm = (sum(n * w for n, w in zip(all_left_norm, left_weights)) /
              sum(left_weights)) if left_weights else 0.0
    r_norm = (sum(n * w for n, w in zip(all_right_norm, right_weights)) /
              sum(right_weights)) if right_weights else 0.0

    if not left_ok and not right_ok:
        offset, ok = detect_centerline(bgr)
        if ok:
            cm_per_norm = 25.0
            return True, offset, True, offset, offset, offset * cm_per_norm
        return False, 0.0, False, 0.0, 0.0, 0.0

    if left_ok and right_ok:
        center_norm = (l_norm + r_norm) / 2.0
    elif left_ok:
        center_norm = l_norm - 0.15
    else:
        center_norm = r_norm + 0.15

    cm_per_norm = 25.0
    cm_offset = center_norm * cm_per_norm

    return left_ok, l_norm, right_ok, r_norm, center_norm, cm_offset


def detect_stone_pattern(bgr):
    """
    检测石板纹理，找到前方最近的石板中心
    通过检测石板间缝隙（暗色水平线）来划分石板
    返回: (found, target_y_norm, target_x_norm) 目标石板在图像中的位置
    """
    if bgr is None:
        return False, 0.0, 0.0

    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    roi = gray[int(h * 0.25):int(h * 0.85), int(w * 0.15):int(w * 0.85)]
    if roi.size == 0:
        return False, 0.0, 0.0

    edges = cv2.Canny(roi, 30, 100)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=40,
                            minLineLength=30, maxLineGap=10)

    horizontal_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = abs(math.atan2(y2 - y1, x2 - x1))
            if angle < 0.35 or angle > 2.79:
                horizontal_lines.append((y1 + y2) / 2.0)

    if not horizontal_lines:
        return False, 0.0, 0.0

    horizontal_lines.sort()
    roi_h = roi.shape[0]
    mid_line = roi_h * 0.45
    target_y = None
    for line_y in horizontal_lines:
        if line_y > mid_line:
            target_y = line_y + roi_h * 0.08
            break

    if target_y is None and horizontal_lines:
        target_y = horizontal_lines[-1] + roi_h * 0.06

    if target_y is None:
        return False, 0.0, 0.0

    real_y = int(round(target_y + int(h * 0.25)))
    target_y_norm = 1.0 - (real_y / h) * 2.0

    search_line = full_mask = cv2.bitwise_or(
        cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), HSV_YELLOW_LOWER, HSV_YELLOW_UPPER),
        cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), HSV_WHITE_LOWER, HSV_WHITE_UPPER))
    strip = search_line[max(0, real_y - 5):min(h, real_y + 5), :]
    if strip.size > 0:
        strip_m = cv2.moments(strip)
        if strip_m['m00'] > 100:
            cx_strip = strip_m['m10'] / strip_m['m00']
        else:
            cx_strip = w / 2
    else:
        cx_strip = w / 2

    target_x_norm = (cx_strip - w / 2) / (w / 2)

    return True, target_y_norm, target_x_norm


def compute_local_target(bgr, stage, object_center=None, object_area=0, odom_data=None):
    """
    统一局部目标点生成器，根据赛段类型返回归一化目标坐标
    返回: dict {
      'target_x': float,   # [-1,1], 负=左
      'target_y': float,   # [-1,1], 正=下方/前方
      'type': str,         # 'centerline', 'stone', 'curve', 'object', 'none'
      'conf': float,       # 0-1 置信度
      'cm_offset': float,  # 估算真实偏移(cm)
    }
    """
    result = {'target_x': 0.0, 'target_y': -0.3, 'type': 'none', 'conf': 0.0, 'cm_offset': 0.0}

    if stage == 1:
        stone_found, stone_ty, stone_tx = detect_stone_pattern(bgr)
        if stone_found:
            result['type'] = 'stone'
            result['target_y'] = stone_ty
            result['target_x'] = stone_tx
            result['conf'] = 0.7
        left_ok, ln, right_ok, rn, cn, cm = detect_yellow_line_bounds(bgr)
        if stone_found:
            result['target_x'] = stone_tx * 0.6 + cn * 0.4
            result['cm_offset'] = result['target_x'] * 25.0
        else:
            result['type'] = 'centerline' if (left_ok or right_ok) else 'none'
            result['target_x'] = cn
            result['cm_offset'] = cm
            result['conf'] = 0.5 if (left_ok or right_ok) else 0.1

    elif stage in (3,):
        if bgr is not None:
            h, w = bgr.shape[:2]
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            yellow_mask = cv2.inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER)
            white_mask = cv2.inRange(hsv, HSV_WHITE_LOWER, HSV_WHITE_UPPER)
            line_mask = cv2.bitwise_or(yellow_mask, white_mask)

            sample_ys = [int(h * 0.3), int(h * 0.45), int(h * 0.6), int(h * 0.75)]
            centers = []
            for sy in sample_ys:
                strip = line_mask[max(0, sy - 3):min(h, sy + 3), :]
                strip_m = cv2.moments(strip)
                if strip_m['m00'] > 100:
                    cx = strip_m['m10'] / strip_m['m00']
                    centers.append((sy, (cx - w / 2) / (w / 2)))
            if len(centers) >= 2:
                ys = np.array([c[0] for c in centers], dtype=np.float64)
                xs = np.array([c[1] for c in centers], dtype=np.float64)
                coeffs = np.polyfit(ys, xs, min(2, len(centers) - 1))
                lookahead_y = h * 0.5
                pred_x = np.polyval(coeffs, lookahead_y)
                result['type'] = 'curve'
                result['target_x'] = float(np.clip(pred_x, -0.8, 0.8))
                result['target_y'] = 0.0
                result['conf'] = min(0.9, len(centers) * 0.3)
                result['cm_offset'] = result['target_x'] * 25.0
            else:
                _, _, _, _, cn, cm = detect_yellow_line_bounds(bgr)
                result['type'] = 'centerline'
                result['target_x'] = cn
                result['cm_offset'] = cm
                result['conf'] = 0.4

    elif stage in (2, 4, 6):
        if object_center is not None and object_area > 80:
            h, w = bgr.shape[:2] if bgr is not None else (240, 320)
            result['type'] = 'object'
            result['target_x'] = (object_center[0] - w / 2) / (w / 2)
            result['target_y'] = (h / 2 - object_center[1]) / (h / 2)
            result['conf'] = min(0.95, object_area / 600.0)
            result['cm_offset'] = result['target_x'] * 30.0
        else:
            _, _, _, _, cn, cm = detect_yellow_line_bounds(bgr)
            result['type'] = 'centerline'
            result['target_x'] = cn
            result['cm_offset'] = cm
            result['conf'] = 0.4

    else:
        _, _, _, _, cn, cm = detect_yellow_line_bounds(bgr)
        result['type'] = 'centerline'
        result['target_x'] = cn
        result['cm_offset'] = cm
        result['conf'] = 0.5

    return result


class RaceController:
    def __init__(self):
        self.lc_cmd = lcm.LCM(LCM_CMD_URL)
        self.lc_resp = lcm.LCM(LCM_RESP_URL)
        self.lc_resp.subscribe("robot_control_response", self._resp_handler)

        self.cmd_msg = robot_control_cmd_lcmt()
        self.life_count = 0
        self.response_mode = 0
        self.response_gait_id = 0
        self.response_bar = 0
        self.response_time = 0.0
        self.response_switch_status = 0
        self.response_contact = 0
        self.response_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.delay_cnt = 0
        self.command_ready = False
        self.refresh_life_on_heartbeat = False
        self.last_cmd_update_time = 0.0
        self.last_cmd_mode = -1
        self.last_cmd_gait = -1
        self.last_cmd_duration = 0

        self.stage = 0
        self.stage_start_time = 0.0
        self.total_start_time = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vyaw = 0.0
        self.gait_id = GAIT_TROT_SLOW

        self.fall_count = 0
        self.prefall_hold_count = 0
        self.prefall_until = 0.0
        self.stage_phase = 0
        self.phase_start = 0.0
        self.stuck_timer = 0.0
        self.prev_steer = 0.0
        self.orange_hit_count = 0
        self.hit_cooldown = 0.0

        self.centerline_history = []
        self.centerline_smooth_window = 8
        self.last_target_steer = 0.0
        self.last_target_time = 0.0
        self.target_lost_hold = 0.5
        self.last_line_target = None
        self.last_line_target_time = 0.0
        self.stage1_stone_hits = 0
        self.stage1_reached_rockroad = False
        self.stage1_exit_step = 0
        self.stage1_tail_clear_start = 0.0
        self.stage1_tail_clear_done = False
        self.stage1_edge_jump_state = 0
        self.stage1_edge_prejump_time = 0.0
        self.stage1_edge_jump_time = 0.0
        self.stage1_edge_jump_stand_until = 0.0
        self.stage1_edge_jump_resume_time = 0.0
        self.stage1_edge_jump_resume_sent = False
        self.stage1_edge_jump_handoff_until = 0.0
        self.stage1_edge_jump_issued = False
        self.stage1_edge_jump_start_x = None
        self.stage1_edge_jump_force_reset_sent = False
        self.stage1_edge_jump_force_reset_time = 0.0
        self.stage5_jump_time = 0.0
        self.stage5_jump_resume_sent = False
        self.stage5_jump_handoff_until = 0.0
        self.stage5_entry_jump_done = False
        self.stage5_entry_jump_time = 0.0
        self.stage5_entry_jump_resume_sent = False
        self.stage5_entry_jump_handoff_until = 0.0
        self.stage5_entry_jump_force_reset_sent = False
        self.stage5_entry_jump_force_reset_time = 0.0
        self.stage5_jump_force_reset_sent = False
        self.stage5_jump_force_reset_time = 0.0
        self.stone_roll_cmd = 0.0
        self.stone_pitch_cmd = STONE_CLIMB_PITCH_BIAS
        self.stone_contact_relief_until = 0.0
        self.stone_contact_relief_start = 0.0
        self.stone_pair_settle_until = 0.0
        self.stone_front_pair_wait_until = 0.0
        self.stone_front_pair_wait_start = 0.0
        self.stone_front_pair_pulse_start = 0.0
        self.stone_front_pair_step_until = 0.0
        self.stone_front_pair_step_side = 0
        self.stone_front_pair_step_sent = 0.0
        self.stone_recontact_until = 0.0
        self.stone_recontact_start = 0.0
        self.stone_recontact_side = 1.0
        self.stage3_entry_step = 0
        self.stage3_path_idx = 0
        self.stage4_entry_idx = 0
        self.stage4_exit_idx = 0

        self.current_target = None
        self.target_history = []
        self.local_target_point = {'target_x': 0.0, 'target_y': -0.3, 'type': 'none', 'conf': 0.0, 'cm_offset': 0.0}

        self.lost_count = 0
        self.was_fallen = False
        self.relocating = False
        self.relocate_sweep_dir = 0.35
        self.relocate_best_centerline = 0.0
        self.relocate_best_strength = 0.0

        self.waypoints = []
        self.waypoint_idx = 0
        self.stage_goal_x = 0.0
        self.stage_goal_y = 0.0
        self.grid_cells = []
        self.grid_cell_idx = 0
        self.grid_initialized = False
        self.stage2_targets = []
        self.stage2_target_idx = 0
        self.stage2_route_idx = 0
        self.stage2_exit_idx = 0
        self.stage2_target_visual_time = 0.0
        self.stage2_target_visual_area = 0.0
        self.stage2_target_visible_once = False
        self.stage2_entry_clear_done = False
        self.stage2_entry_faced_first_ball = False
        self.stage2_first_ball_last_dist = 99.0
        self.stage2_first_ball_stall_count = 0
        self.stage2_first_ball_force_mode = False
        self.stage2_ball_start_pos = {}
        self.stage2_hit_start_pos = None
        self.stage2_hit_target_idx = -1
        self.stage2_hit_target_point = None
        self.stage2_target_turn_ready_idx = -1
        self.stage2_axis_turn_start = 0.0
        self.stage2_axis_force_drive_until = 0.0
        self.stage2_axis_turning_target_idx = -1
        self.stage2_post_hit_brake_until = 0.0
        self.stage2_simple_last_target_idx = -1
        self.stage2_simple_last_dist = 99.0
        self.stage2_simple_last_progress_time = 0.0
        self.last_log_times = {}
        self.speech_once_keys = set()
        self.tts_cmd = shutil.which("spd-say") or shutil.which("espeak-ng") or shutil.which("espeak")
        self.odom_ready_logged = False
        self.odom_missing_logged_at = 0.0
        self.control_response_warned_at = 0.0
        self.control_response_ready_logged = False

        self.running = True
        self._setup_signal()

        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)

    def _setup_signal(self):
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    def _sig_handler(self, signum, frame):
        self.running = False

    def _resp_handler(self, channel, data):
        try:
            resp = robot_control_response_lcmt.decode(data)
            with self.response_lock:
                self.response_mode = resp.mode
                self.response_gait_id = resp.gait_id
                self.response_bar = resp.order_process_bar
                self.response_switch_status = resp.switch_status
                self.response_contact = int(getattr(resp, "contact", 0))
                self.response_time = time.time()
        except Exception:
            pass

    def _bump_life_locked(self):
        self.life_count = (self.life_count + 1) % 127
        self.cmd_msg.life_count = self.life_count

    def _send_loop(self):
        while self.running:
            try:
                with self.send_lock:
                    if self.delay_cnt > 20 and self.command_ready:
                        now = time.time()
                        if (self.refresh_life_on_heartbeat and
                                now - self.last_cmd_update_time >= 0.08):
                            self._bump_life_locked()
                            self.last_cmd_update_time = now
                        self.lc_cmd.publish("robot_control_cmd", self.cmd_msg.encode())
                        self.delay_cnt = 0
                    self.delay_cnt += 1
            except Exception as exc:
                self.log_throttle("lcm_publish_error", 2.0, f"LCM命令发布异常: {exc}")
            time.sleep(0.005)

    def _recv_loop(self):
        while self.running:
            try:
                self.lc_resp.handle_timeout(5)
            except Exception:
                pass
            time.sleep(0.002)

    def start_threads(self):
        self.send_thread.start()
        self.recv_thread.start()

    def log(self, msg):
        elapsed = time.time() - self.total_start_time
        m, s = divmod(int(elapsed), 60)
        stage_name = STAGE_NAMES.get(self.stage, f"赛段{self.stage}")
        print(f"[{m:02d}:{s:02d}|{stage_name}] {msg}", flush=True)

    def log_throttle(self, key, interval, msg):
        now = time.time()
        if now - self.last_log_times.get(key, 0.0) >= interval:
            self.last_log_times[key] = now
            self.log(msg)

    def speak_once(self, key, text):
        if key in self.speech_once_keys:
            return
        self.speech_once_keys.add(key)
        self.log(f"语音播报: {text}")
        if not self.tts_cmd:
            self.log_throttle("tts_missing", 30.0, "未找到 espeak/espeak-ng，当前仅输出播报日志")
            return
        try:
            subprocess.Popen(
                [self.tts_cmd, text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            self.log_throttle("tts_failed", 30.0, f"语音播报命令启动失败: {exc}")

    def _check_odom_health(self):
        if self._using_gazebo_pose():
            if not self.odom_ready_logged:
                self.log(
                    f"真实位姿已就绪: /gazebo/link_states {sensor_node.gazebo_pose_link} "
                    f"pos=({sensor_node.gazebo_pose_x:.2f},{sensor_node.gazebo_pose_y:.2f})"
                )
                self.odom_ready_logged = True
            return True

        if sensor_node.odom_got:
            if not self.odom_ready_logged:
                self.log(f"里程计已就绪: {sensor_node.odom_frame}->{BASE_FRAME} "
                         f"pos=({sensor_node.odom_x:.2f},{sensor_node.odom_y:.2f})")
                self.odom_ready_logged = True
            return True

        now = time.time()
        if now - self.odom_missing_logged_at > 5.0:
            frames = ",".join(ODOM_FRAME_CANDIDATES)
            self.log(
                f"等待定位: pose_source={POSE_SOURCE} gazebo_links={GAZEBO_ROBOT_LINK_CANDIDATES} "
                f"tf_candidates=[{frames}], base={BASE_FRAME}"
            )
            self.odom_missing_logged_at = now
        return False

    def _check_control_response_health(self):
        now = time.time()
        with self.response_lock:
            resp_time = self.response_time
            mode = self.response_mode
            gait = self.response_gait_id
            bar = self.response_bar
            switch_status = self.response_switch_status

        if resp_time > 0.0:
            if not self.control_response_ready_logged:
                self.log(f"运控响应已就绪: mode={mode}, gait={gait}, bar={bar}, switch={switch_status}")
                self.control_response_ready_logged = True
            stale_sec = now - resp_time
            if stale_sec > CONTROL_RESPONSE_STALE_FATAL and self.command_ready:
                self.log_throttle(
                    "control_response_fatal",
                    1.0,
                    f"控制链路中断: {stale_sec:.1f}s未收到robot_control_response "
                    f"(last response mode={mode}, gait={gait}, bar={bar}, switch={switch_status})"
                )
                return False
            elif stale_sec > CONTROL_RESPONSE_STALE_WARN:
                self.log_throttle(
                    "control_response_stale", 2.0,
                    f"超过{stale_sec:.1f}s未收到robot_control_response，检查cyberdog_control/LCM"
                )
            return True

        with self.send_lock:
            command_ready = self.command_ready
            last_mode = self.last_cmd_mode
            last_gait = self.last_cmd_gait

        if command_ready and now - self.total_start_time > 3.0 and now - self.control_response_warned_at > 2.0:
            self.control_response_warned_at = now
            self.log(
                "尚未收到robot_control_response，命令已发布但运控未回包 "
                f"(last mode={last_mode}, gait={last_gait})"
            )
        if command_ready and now - self.total_start_time > CONTROL_RESPONSE_STARTUP_FATAL:
            self.log_throttle(
                "control_response_startup_fatal",
                1.0,
                "控制链路启动失败: 已发布命令但一直没有robot_control_response"
            )
        return False

    def send_cmd(self, mode, gait_id=0, vx=0.0, vy=0.0, vyaw=0.0,
                 step_h=0.06, duration=0, rpy=None, pos=None,
                 acc_des=None, ctrl_point=None, foot_pose=None,
                 contact=0, value=0, raw_step_height=None, clamp_pos=True):
        def finite_or(value, fallback=0.0):
            try:
                value = float(value)
            except Exception:
                return fallback
            return value if math.isfinite(value) else fallback

        encoded = None
        with self.send_lock:
            vx = finite_or(vx)
            vy = finite_or(vy)
            vyaw = finite_or(vyaw)
            if isinstance(step_h, (list, tuple)):
                step_h = [
                    max(0.0, finite_or(step_h[0])),
                    max(0.0, finite_or(step_h[1] if len(step_h) > 1 else step_h[0])),
                ]
            else:
                step_h = max(0.0, finite_or(step_h))
            if rpy:
                rpy = [finite_or(rpy[0]), finite_or(rpy[1]), finite_or(rpy[2])]
            if pos:
                pos = [finite_or(pos[0]), finite_or(pos[1]), finite_or(pos[2], MOBILE_BODY_HEIGHT)]
                if clamp_pos:
                    pos[2] = max(0.12, min(0.30, pos[2]))
            if acc_des:
                acc_des = [finite_or(v) for v in acc_des[:6]]
            if ctrl_point:
                ctrl_point = [finite_or(v) for v in ctrl_point[:3]]
            if foot_pose:
                foot_pose = [finite_or(v) for v in foot_pose[:6]]
            self._bump_life_locked()
            self.cmd_msg.mode = mode
            self.cmd_msg.gait_id = gait_id
            self.cmd_msg.vel_des = [vx, vy, vyaw]
            self.cmd_msg.duration = duration
            if mode == MODE_LOCOMOTION:
                if raw_step_height is not None:
                    self.cmd_msg.step_height = [finite_or(raw_step_height[0]), finite_or(raw_step_height[1])]
                else:
                    if isinstance(step_h, list):
                        self.cmd_msg.step_height = step_h
                    else:
                        sh = step_h
                        self.cmd_msg.step_height = [sh, sh]
            else:
                self.cmd_msg.step_height = [0.0, 0.0]
            self.cmd_msg.rpy_des = rpy if rpy else [0.0, 0.0, 0.0]
            self.cmd_msg.pos_des = pos if pos else [0.0, 0.0, 0.0]
            self.cmd_msg.acc_des = acc_des if acc_des else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.cmd_msg.ctrl_point = ctrl_point if ctrl_point else [0.0, 0.0, 0.0]
            self.cmd_msg.foot_pose = foot_pose if foot_pose else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.cmd_msg.contact = int(contact)
            self.cmd_msg.value = int(value)
            self.command_ready = True
            self.refresh_life_on_heartbeat = (
                duration == 0 and mode in (MODE_PURE_DAMPER, MODE_RECOVERY_STAND, MODE_LOCOMOTION)
            )
            self.last_cmd_update_time = time.time()
            self.last_cmd_mode = mode
            self.last_cmd_gait = gait_id
            self.last_cmd_duration = duration
            self.delay_cnt = 50
            encoded = self.cmd_msg.encode()
            try:
                self.lc_cmd.publish("robot_control_cmd", encoded)
                self.delay_cnt = 0
            except Exception as exc:
                self.log_throttle("lcm_publish_immediate_error", 1.0, f"LCM立即发布异常: {exc}")
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_vyaw = vyaw
        self.gait_id = gait_id

    def wait_finish(self, mode, timeout=10.0, after_time=None):
        t0 = time.time()
        while time.time() - t0 < timeout and self.running:
            with self.response_lock:
                bar = self.response_bar
                resp_mode = self.response_mode
                resp_gait = self.response_gait_id
                resp_time = self.response_time
            fresh_enough = after_time is None or resp_time >= after_time
            if fresh_enough and bar >= 95 and resp_mode == mode:
                return True
            if resp_time <= 0.0:
                self.log_throttle("wait_no_control_response", 2.0,
                                  f"等待mode={mode}完成，但尚未收到运控响应")
            else:
                self.log_throttle("wait_control_response", 2.0,
                                  f"等待mode={mode}完成: resp_mode={resp_mode}, gait={resp_gait}, bar={bar}, fresh={fresh_enough}")
            time.sleep(0.005)
        return False

    def _imu_stable_for_jump_recovery(self):
        now = time.time()
        if sensor_node.imu_time <= 0.0 or now - sensor_node.imu_time > 0.6:
            return False
        return (
            abs(sensor_node.imu_roll) < IMU_ROLL_PREFALL and
            abs(sensor_node.imu_pitch) < IMU_PITCH_PREFALL and
            abs(sensor_node.imu_angvel_x) < IMU_ANGVEL_XY_PREFALL and
            abs(sensor_node.imu_angvel_y) < IMU_ANGVEL_XY_PREFALL and
            abs(sensor_node.imu_angvel_z) < IMU_ANGVEL_Z_PREFALL
        )

    def _jump_recovery_ready(self, jump_start_time, stand_until=0.0, label="Jump3D"):
        now = time.time()
        elapsed = now - jump_start_time
        with self.response_lock:
            resp_mode = self.response_mode
            resp_bar = self.response_bar
            resp_time = self.response_time
            resp_gait = self.response_gait_id
            switch_status = self.response_switch_status

        response_ready = (
            resp_time > 0.0 and
            resp_mode == MODE_RECOVERY_STAND and
            (resp_bar >= 80 or (stand_until > 0.0 and now >= stand_until))
        )
        stable_ready = elapsed >= JUMP_RECOVERY_FALLBACK_SEC and self._imu_stable_for_jump_recovery()
        if stable_ready and not response_ready:
            self.log_throttle(
                f"{label}_stable_recovery_fallback",
                0.8,
                f"{label} 后运控回包未切到RecoveryStand，但IMU已稳定，放行Locomotion接管 "
                f"elapsed={elapsed:.2f} resp=({resp_mode},{resp_gait},{resp_bar},switch={switch_status}) "
                f"imu=({sensor_node.imu_roll:.2f},{sensor_node.imu_pitch:.2f})"
            )
        return response_ready or stable_ready, resp_mode, resp_bar, resp_time

    def _send_locomotion_handoff(self, gait=GAIT_TROT_SLOW, body_h=MOBILE_BODY_HEIGHT,
                                 step_h=None, pitch=0.0, vx=0.0):
        if step_h is None:
            step_h = (MIN_TRAVEL_STEP_HEIGHT, MIN_TRAVEL_STEP_HEIGHT_REAR)
        self.send_cmd(
            MODE_LOCOMOTION,
            gait,
            vx=vx,
            vy=0.0,
            vyaw=0.0,
            step_h=step_h,
            rpy=[0.0, pitch, 0.0],
            pos=[0.0, 0.0, body_h],
        )

    def stop_safe(self):
        self.log("安全停止 (PureDamper)...")
        self.send_cmd(MODE_PURE_DAMPER, 0, step_h=0.0)
        time.sleep(0.5)
        self.send_cmd(MODE_PURE_DAMPER, 0, step_h=0.0)
        time.sleep(0.5)
        self.send_cmd(MODE_PURE_DAMPER, 0, step_h=0.0)
        self.log("已停止")

    def _motion_risk(self, sensor):
        if sensor is None:
            return 0.0, False
        now = time.time()
        if sensor.imu_time <= 0.0 or now - sensor.imu_time > 0.5:
            return 0.0, False

        roll = abs(sensor.imu_roll)
        pitch = abs(sensor.imu_pitch)
        roll_rate = abs(sensor.imu_angvel_x)
        pitch_rate = abs(sensor.imu_angvel_y)
        yaw_rate = abs(sensor.imu_angvel_z)
        roll_risk = (roll - 0.08) / (IMU_ROLL_PREFALL - 0.08)
        pitch_risk = (pitch - 0.10) / (IMU_PITCH_PREFALL - 0.10)
        roll_rate_risk = (roll_rate - 0.55) / (IMU_ANGVEL_XY_PREFALL - 0.55)
        pitch_rate_risk = (pitch_rate - 0.55) / (IMU_ANGVEL_XY_PREFALL - 0.55)
        yaw_risk = (yaw_rate - 0.75) / (IMU_ANGVEL_Z_PREFALL - 0.75)
        risk = max(0.0, min(1.0, max(roll_risk, pitch_risk, roll_rate_risk, pitch_rate_risk, yaw_risk)))
        critical = (
            roll >= IMU_ROLL_PREFALL or
            pitch >= IMU_PITCH_PREFALL or
            roll_rate >= IMU_ANGVEL_XY_PREFALL or
            pitch_rate >= IMU_ANGVEL_XY_PREFALL or
            yaw_rate >= IMU_ANGVEL_Z_PREFALL
        )
        return risk, critical

    def _limit_locomotion_for_risk(self, vx, vy, vyaw, gait, step_h, sensor):
        if isinstance(step_h, (list, tuple)):
            step_h = (max(0.0, float(step_h[0])), max(0.0, float(step_h[1])))
        else:
            step_h = max(0.0, float(step_h))
        min_step = (MIN_TRAVEL_STEP_HEIGHT, MIN_TRAVEL_STEP_HEIGHT_REAR)
        if isinstance(step_h, tuple):
            return vx, vy, vyaw, gait, (max(step_h[0], min_step[0]), max(step_h[1], min_step[1]))
        return vx, vy, vyaw, gait, max(step_h, MIN_TRAVEL_STEP_HEIGHT)
        risk, critical = self._motion_risk(sensor)
        if risk <= 0.05:
            if isinstance(step_h, tuple):
                return vx, vy, vyaw, gait, (max(step_h[0], min_step[0]), max(step_h[1], min_step[1]))
            return vx, vy, vyaw, gait, max(step_h, MIN_TRAVEL_STEP_HEIGHT)

        if self.stage == 2:
            max_vx = 0.42 - risk * 0.16
            max_vy = 0.160 - risk * 0.060
            if abs(vx) < 0.03 and abs(vy) < 0.03:
                max_yaw = 1.65 - risk * 0.35
            else:
                max_yaw = 0.78 - risk * 0.22
        elif self.stage == 1:
            max_vx = 0.62 - risk * 0.32
            max_vy = 0.090 - risk * 0.050
            if abs(vx) < 0.03 and abs(vy) < 0.03:
                max_yaw = 1.65 - risk * 0.35
            else:
                max_yaw = 0.82 - risk * 0.42
        else:
            max_vx = 1.60 - risk * 0.45
            max_vy = 0.220 - risk * 0.070
            max_yaw = 2.20 - risk * 0.60
        if critical:
            max_vx, max_vy, max_yaw = 0.08, 0.020, 0.080
            gait = GAIT_TROT_SLOW
        elif risk > 0.35:
            gait = STAGE_RUN_GAIT if self.stage in (2, 3, 4) else GAIT_TROT_SLOW

        vx = max(-max_vx, min(max_vx, vx))
        vy = max(-max_vy, min(max_vy, vy))
        vyaw = max(-max_yaw, min(max_yaw, vyaw))
        self.log_throttle(
            "global_motion_risk_limit",
            0.8,
            f"全局风险降级 risk={risk:.2f} critical={critical} "
            f"cmd=({vx:.2f},{vy:.2f},{vyaw:.2f}) gait={gait}"
        )
        if isinstance(step_h, tuple):
            step_h = (max(step_h[0], min_step[0]), max(step_h[1], min_step[1]))
        else:
            step_h = max(step_h, MIN_TRAVEL_STEP_HEIGHT)
        return vx, vy, vyaw, gait, step_h

    def _prefall_brace(self, sensor):
        roll = sensor.imu_roll
        pitch = sensor.imu_pitch
        roll_rate = sensor.imu_angvel_x
        pitch_rate = sensor.imu_angvel_y
        yaw_rate = sensor.imu_angvel_z
        counter_vy = max(-0.055, min(0.055, -roll * 0.11 - roll_rate * 0.025))
        counter_yaw = max(-0.11, min(0.11, -yaw_rate * 0.045 - roll * 0.16))
        counter_rpy = [
            max(-0.14, min(0.14, -roll * 0.85 - roll_rate * 0.035)),
            max(-0.12, min(0.12, -pitch * 0.7 - pitch_rate * 0.03)),
            0.0,
        ]
        self.send_cmd(
            MODE_LOCOMOTION,
            GAIT_TROT_SLOW,
            vx=0.0,
            vy=counter_vy,
            vyaw=counter_yaw,
            step_h=(
                (STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR)
                if self.stage == 1 else
                (HIGH_TRAVEL_STEP_HEIGHT, HIGH_TRAVEL_STEP_HEIGHT_REAR)
            ),
            rpy=counter_rpy,
            pos=[0.0, 0.0, STONE_BODY_HEIGHT if self.stage == 1 else BRACE_BODY_HEIGHT],
        )

    def _mobile_pose(self, sensor, vx, vy, vyaw, body_h=None, pitch_bias=MOBILE_PITCH_BIAS):
        if body_h is None:
            body_h = MOBILE_BODY_HEIGHT
        roll = sensor.imu_roll if sensor is not None else 0.0
        pitch = sensor.imu_pitch if sensor is not None else 0.0
        roll_rate = sensor.imu_angvel_x if sensor is not None else 0.0
        pitch_rate = sensor.imu_angvel_y if sensor is not None else 0.0
        speed_load = min(1.0, abs(vx) / 0.25 + abs(vyaw) / 0.45)
        roll_comp = max(-0.13, min(0.13, -roll * 0.72 - roll_rate * 0.035))
        pitch_comp = max(-0.13, min(0.13, pitch_bias - pitch * 0.55 - pitch_rate * 0.028))
        height = body_h - 0.008 * speed_load
        height = max(0.17, min(0.30, height))
        return [roll_comp, pitch_comp, 0.0], [0.0, 0.0, height]

    def _step_pair_for_stage(self, step_h):
        if isinstance(step_h, (list, tuple)):
            return step_h
        if self.stage == 1:
            return (step_h, rear_step_for(step_h))
        if step_h >= HIGH_TRAVEL_STEP_HEIGHT:
            return (step_h, HIGH_TRAVEL_STEP_HEIGHT_REAR)
        return (step_h, MIN_TRAVEL_STEP_HEIGHT_REAR)

    def apply_locomotion(self, vx, vy, vyaw, gait=None, step_h=0.06, sensor=None,
                         boundary_check=True, target_point=None, body_h=None,
                         pitch_bias=MOBILE_PITCH_BIAS):
        if gait is None:
            gait = GAIT_TROT_SLOW
        step_h = self._step_pair_for_stage(step_h)

        final_vyaw = vyaw
        final_vy = vy
        final_vx = vx

        bgr = image_msg_to_cv(sensor.rgb_image_raw) if sensor is not None else None

        if target_point is not None and bgr is not None:
            tp = target_point
            target_steer = 0.0
            if tp['type'] == 'object' and tp['conf'] > 0.3:
                target_steer = tp['target_x'] * 0.7
                target_steer = max(-0.45, min(0.45, target_steer))
            elif tp['type'] in ('curve', 'stone') and tp['conf'] > 0.3:
                target_steer = tp['target_x'] * 0.5
                target_steer = max(-0.35, min(0.35, target_steer))
            elif tp['type'] == 'centerline' and tp['conf'] > 0.3:
                target_steer = -tp['target_x'] * CENTERLINE_KP

            if abs(target_steer) > 0.02 and abs(vyaw) < 0.05:
                if abs(vyaw) < 0.03:
                    final_vyaw = target_steer
                else:
                    final_vyaw = vyaw + target_steer * 0.4

            if boundary_check and tp['type'] != 'object' and abs(tp['cm_offset']) > 5.0:
                corr_strength = (abs(tp['cm_offset']) - 5.0) / 20.0
                corr_strength = max(0.0, min(1.0, corr_strength))
                final_vx = vx * (1.0 - corr_strength * 0.5)
                final_vyaw = -tp['target_x'] * CENTERLINE_KP * (1.0 + corr_strength)
                if corr_strength > 0.5:
                    self.log(f"黄线偏离>{abs(tp['cm_offset']):.0f}cm 强制回正 vx={final_vx:.2f}")

        elif sensor is not None:
            offset, ok = detect_centerline(bgr)

            if ok and abs(vyaw) < 0.05:
                smoothed_offset = smooth_centerline(
                    self.centerline_history, offset, self.centerline_smooth_window)
                d_offset = smoothed_offset - self.prev_steer
                pid = -smoothed_offset * CENTERLINE_KP - d_offset * CENTERLINE_KD
                pid = max(-0.5, min(0.5, pid))
                self.prev_steer = smoothed_offset

                if abs(vyaw) < 0.03:
                    final_vyaw = pid
                else:
                    final_vyaw = vyaw + pid * 0.4

                if boundary_check and abs(smoothed_offset) > 0.28:
                    correction_strength = (abs(smoothed_offset) - 0.28) / 0.72
                    correction_strength = max(0.0, min(1.0, correction_strength))
                    final_vx = vx * (1.0 - correction_strength * 0.6)
                    final_vyaw = pid * (1.0 + correction_strength * 0.8)
                    if correction_strength > 0.5:
                        self.log(f"黄线边界偏移过大 offset={smoothed_offset:.2f} 减速回正 vx={final_vx:.2f}")

        stage2_fixed_hit = self.stage == 2 and not boundary_check and not self.relocating
        if sensor is not None and tof_blocked(sensor) and not stage2_fixed_hit:
            if abs(vyaw) < 0.05 and abs(vy) < 0.02:
                final_vyaw = 0.4
                final_vy = 0.05
                self.log("TOF检测到障碍，微调偏航避让")

        if self.stage >= 2:
            final_vx *= POST_STAGE_SPEED_SCALE
            final_vy *= POST_STAGE_SPEED_SCALE
            final_vyaw *= POST_STAGE_YAW_SCALE

        final_vx, final_vy, final_vyaw, gait, step_h = self._limit_locomotion_for_risk(
            final_vx, final_vy, final_vyaw, gait, step_h, sensor
        )
        final_rpy, final_pos = self._mobile_pose(
            sensor, final_vx, final_vy, final_vyaw, body_h, pitch_bias=pitch_bias
        )
        if gait == GAIT_USER_WALK_WAVE:
            self._send_stone_user_gait(final_vx, final_vy, final_vyaw, step_h, final_rpy)
            return
        self.send_cmd(MODE_LOCOMOTION, gait,
                      vx=final_vx, vy=final_vy, vyaw=final_vyaw,
                      step_h=step_h, rpy=final_rpy, pos=final_pos)

    def apply_stair_safe_locomotion(self, vx, vy, vyaw, gait=None, step_h=STAIR_SAFE_STEP_HEIGHT,
                                    sensor=None, boundary_check=True, target_point=None,
                                    body_h=STAIR_SAFE_BODY_HEIGHT):
        safe_vy = max(-STAIR_SAFE_VY_LIMIT, min(STAIR_SAFE_VY_LIMIT, vy))
        safe_vyaw = max(-STAIR_SAFE_YAW_LIMIT, min(STAIR_SAFE_YAW_LIMIT, vyaw))
        safe_step = max_step_pair(step_h, STAIR_SAFE_STEP_HEIGHT, STAIR_SAFE_STEP_HEIGHT)
        return self.apply_locomotion(
            vx,
            safe_vy,
            safe_vyaw,
            gait=gait,
            step_h=safe_step,
            sensor=sensor,
            boundary_check=boundary_check,
            target_point=target_point,
            body_h=body_h,
            pitch_bias=STAIR_SAFE_PITCH_BIAS,
        )

    def _pack_user_gait_step_heights(self, fl, fr, rl, rr):
        def mm(value):
            return int(max(0.0, min(0.22, value)) * 1000.0)
        return [float(mm(fl) + mm(fr) * 1000), float(mm(rl) + mm(rr) * 1000)]

    def _front_contact_state(self):
        with self.response_lock:
            contact = self.response_contact
            resp_time = self.response_time
        if resp_time <= 0.0 or time.time() - resp_time > 0.35:
            return False, False, 0
        front_left = bool(contact & 0x01)
        front_right = bool(contact & 0x02)
        return front_left, front_right, contact

    def _send_stone_user_gait(self, vx, vy, vyaw, step_h, rpy, body_offset=0.015):
        if isinstance(step_h, (list, tuple)):
            front_step = step_h[0]
            rear_step = step_h[1] if len(step_h) > 1 else step_h[0]
        else:
            front_step = step_h
            rear_step = STONE_CLIMB_STEP_HEIGHT_REAR if step_h >= STONE_CLIMB_STEP_HEIGHT else STONE_APPROACH_STEP_HEIGHT_REAR
        packed_step = self._pack_user_gait_step_heights(front_step, front_step, rear_step, rear_step)
        landing_x = 0.055
        landing_y = 0.055
        self.send_cmd(
            MODE_LOCOMOTION,
            GAIT_USER_WALK_WAVE,
            vx=vx,
            vy=vy,
            vyaw=vyaw,
            rpy=rpy,
            pos=[0.0, 0.0, body_offset],
            # UserGait 读取每条腿落脚偏移：FL/RF/RL 来自 foot_pose，RR 来自 ctrl_point[0:2]。
            foot_pose=[landing_x, landing_y, landing_x, -landing_y, landing_x, landing_y],
            ctrl_point=[landing_x, -landing_y, 0.42],
            acc_des=[25.0, 25.0, 80.0, 8.0, 8.0, 8.0],
            contact=10,
            value=1,
            raw_step_height=packed_step,
            clamp_pos=False,
        )

    def _send_stone_front_pair_freeze(self, rpy, body_h, reason, contact_bits):
        self.send_cmd(
            MODE_POS_INTERP,
            0,
            duration=180,
            rpy=rpy,
            pos=[0.0, 0.0, max(0.200, body_h - STONE_FRONT_PAIR_BODY_DROP)],
        )
        self.log_throttle(
            "stage1_front_pair_freeze",
            0.25,
            f"单前脚上石板锁步保持：{reason} contact=0x{contact_bits:x} "
            f"rpy=({rpy[0]:.2f},{rpy[1]:.2f}) body_h={body_h:.3f}"
        )

    def _send_stone_front_pair_step(self, rpy, body_h, lift_left, reason, contact_bits):
        contact = 0x0E if lift_left else 0x0D
        side = 1 if lift_left else 2
        now = time.time()
        if self.stone_front_pair_step_side != side or now >= self.stone_front_pair_step_until:
            self.stone_front_pair_step_until = now + STONE_FRONT_PAIR_STEP_HOLD_SEC
            self.stone_front_pair_step_side = side
            self.stone_front_pair_step_sent = now
        self.send_cmd(
            MODE_POS_INTERP,
            0,
            duration=STONE_FRONT_PAIR_STEP_MS,
            rpy=rpy,
            pos=[0.0, 0.0, max(0.200, body_h - STONE_FRONT_PAIR_BODY_DROP)],
            foot_pose=[
                STONE_FRONT_PAIR_STEP_FORWARD,
                0.0,
                STONE_FRONT_PAIR_STEP_LIFT,
                0.0,
                0.0,
                0.0,
            ],
            contact=contact,
        )
        self.log_throttle(
            "stage1_front_pair_single_step",
            0.25,
            f"单前脚触石，暂停连续步态并只补{'左' if lift_left else '右'}前脚：{reason} "
            f"contact=0x{contact_bits:x}->pose_contact=0x{contact:x} "
            f"foot=({STONE_FRONT_PAIR_STEP_FORWARD:.3f},0,{STONE_FRONT_PAIR_STEP_LIFT:.3f})"
        )

    def apply_stabilized_locomotion(self, vx, vy, vyaw, gait=None, step_h=0.06,
                                    sensor=None, body_h=STONE_BODY_HEIGHT, stepup_active=False,
                                    suppress_min_progress=False, lane_center_y=STAGE1_CENTER_Y):
        if gait is None:
            gait = STAGE1_GAIT_NORMAL
        step_h = self._step_pair_for_stage(step_h)

        roll = sensor.imu_roll if sensor is not None else 0.0
        pitch = sensor.imu_pitch if sensor is not None else 0.0
        roll_rate = sensor.imu_angvel_x if sensor is not None else 0.0
        pitch_rate = sensor.imu_angvel_y if sensor is not None else 0.0
        pose_ready = self._has_global_pose()
        odom_vy = 0.0
        if sensor is not None:
            odom_vy = sensor.gazebo_pose_vy if self._using_gazebo_pose() else (
                sensor.odom_vy if sensor.odom_got else 0.0
            )

        contact_impulse = (
            abs(roll_rate) > STONE_CONTACT_RATE_SLOWDOWN or
            abs(pitch_rate) > STONE_CONTACT_RATE_SLOWDOWN or
            abs(odom_vy) > STONE_SIDE_SLIP_TRIGGER
        )
        now = time.time()
        if contact_impulse:
            if now >= self.stone_contact_relief_until:
                self.stone_contact_relief_start = now
            self.stone_contact_relief_until = max(
                self.stone_contact_relief_until,
                now + STONE_IMPACT_RELIEF_SEC,
            )
            if STONE_RECONTACT_TOTAL_SEC > 0.0 and now >= self.stone_recontact_until:
                self.stone_recontact_start = now
                self.stone_recontact_until = self.stone_recontact_start + STONE_RECONTACT_TOTAL_SEC
                self.stone_recontact_side *= -1.0
        relief_active = now < self.stone_contact_relief_until
        recontact_active = now < self.stone_recontact_until
        _, oy, oyaw = self._get_odom_pos()
        stone_yaw_err = (
            self._normalize_angle(STAGE1_STONE_ALIGN_YAW - oyaw)
            if sensor is not None and pose_ready else 0.0
        )
        front_left, front_right, front_contact_bits = self._front_contact_state()
        front_single_contact = front_left != front_right
        front_pair_contact = front_left and front_right
        if stepup_active and front_single_contact and STONE_FRONT_PAIR_WAIT_SEC > 0.0:
            if now >= self.stone_front_pair_wait_until:
                self.stone_front_pair_wait_start = now
                self.stone_front_pair_pulse_start = 0.0
            self.stone_front_pair_wait_until = max(
                self.stone_front_pair_wait_until,
                now + STONE_FRONT_PAIR_WAIT_SEC,
            )
        elif front_pair_contact:
            self.stone_front_pair_wait_until = 0.0
            self.stone_front_pair_wait_start = 0.0
            self.stone_front_pair_pulse_start = 0.0
            self.stone_front_pair_step_until = 0.0
            self.stone_front_pair_step_side = 0
        if stepup_active and (contact_impulse or abs(oy - STAGE1_CENTER_Y) > STONE_PAIR_SETTLE_Y):
            self.stone_pair_settle_until = max(self.stone_pair_settle_until, now + STONE_PAIR_SETTLE_SEC)
        pair_settle_active = now < self.stone_pair_settle_until
        front_pair_wait_active = (
            stepup_active and
            now < self.stone_front_pair_wait_until and
            not front_pair_contact
        )
        front_pair_wait_elapsed = max(0.0, now - self.stone_front_pair_wait_start)
        front_pair_hold_active = front_pair_wait_active and front_pair_wait_elapsed < STONE_FRONT_PAIR_HOLD_SEC
        lane_vy = 0.0
        if (sensor is not None and pose_ready and lane_center_y is not None and
                not pair_settle_active and not front_pair_wait_active):
            lane_vy = max(-STONE_LANE_VY_LIMIT, min(STONE_LANE_VY_LIMIT, -(oy - lane_center_y) * STONE_LANE_VY_GAIN))

        # 石板路柔顺姿态：保持连续对角步态，只在触石瞬间做小幅俯仰让角。
        # Locomotion 内部主要开放 pitch，roll 更多依靠横向速度/偏航修正完成。
        roll_give = STONE_IMPACT_ROLL_GIVE if roll_rate > 0.0 else -STONE_IMPACT_ROLL_GIVE
        pitch_give = STONE_IMPACT_PITCH_GIVE if pitch_rate > 0.0 else -STONE_IMPACT_PITCH_GIVE
        target_roll = -roll * STONE_ROLL_KP - roll_rate * STONE_ROLL_KD
        pitch_bias = STONE_STEPUP_PITCH_BIAS if stepup_active else STONE_CLIMB_PITCH_BIAS
        if front_pair_wait_active:
            pitch_bias = STONE_FRONT_PAIR_PITCH_BIAS
        target_pitch = pitch_bias - pitch * STONE_PITCH_KP - pitch_rate * STONE_PITCH_KD
        if contact_impulse or relief_active:
            target_roll += roll_give
            target_pitch += pitch_give
        target_roll = max(-STONE_ROLL_LIMIT, min(STONE_ROLL_LIMIT, target_roll))
        target_pitch = max(-STONE_PITCH_LIMIT, min(STONE_PITCH_LIMIT, target_pitch))

        desired_roll = self.stone_roll_cmd + (target_roll - self.stone_roll_cmd) * STONE_COMPLIANCE_ALPHA
        desired_pitch = self.stone_pitch_cmd + (target_pitch - self.stone_pitch_cmd) * STONE_COMPLIANCE_ALPHA
        roll_delta = max(-STONE_RPY_RATE_LIMIT, min(STONE_RPY_RATE_LIMIT, desired_roll - self.stone_roll_cmd))
        pitch_delta = max(-STONE_RPY_RATE_LIMIT, min(STONE_RPY_RATE_LIMIT, desired_pitch - self.stone_pitch_cmd))
        self.stone_roll_cmd += roll_delta
        self.stone_pitch_cmd += pitch_delta

        side_damp_vy = max(
            -STONE_SIDE_DAMP_VY_LIMIT,
            min(STONE_SIDE_DAMP_VY_LIMIT, -odom_vy * STONE_SIDE_DAMP_VY_GAIN - roll_rate * 0.014)
        )
        side_damp_yaw = max(
            -STONE_SIDE_DAMP_YAW_LIMIT,
            min(STONE_SIDE_DAMP_YAW_LIMIT, -odom_vy * STONE_SIDE_DAMP_YAW_GAIN - roll_rate * 0.024)
        )
        roll_vy_corr = max(-0.045, min(0.045, -roll * STONE_ROLL_VY_GAIN))
        roll_yaw_corr = max(-0.13, min(0.13, -roll * STONE_ROLL_YAW_GAIN - roll_rate * 0.024))
        final_vy = vy + lane_vy + side_damp_vy + roll_vy_corr
        final_vyaw = vyaw + side_damp_yaw + roll_yaw_corr
        final_rpy = [self.stone_roll_cmd, self.stone_pitch_cmd, 0.0]
        final_body_h = body_h
        if pair_settle_active:
            pair_center_vy = 0.0
            if lane_center_y is not None:
                pair_center_vy = max(
                    -STONE_PAIR_CENTER_VY_LIMIT,
                    min(STONE_PAIR_CENTER_VY_LIMIT, -(oy - lane_center_y) * STONE_PAIR_CENTER_VY_GAIN)
                )
            vx = min(vx, STONE_PAIR_SETTLE_VX)
            final_vy = max(-0.018, min(0.018, side_damp_vy + pair_center_vy))
            final_vyaw = max(-0.025, min(0.025, side_damp_yaw))
        if front_pair_wait_active:
            lift_left = front_right and not front_left
            lift_right = front_left and not front_right
            unload_vy = 0.0
            if sensor is not None and pose_ready:
                center_err = oy - STAGE1_CENTER_Y
                if abs(center_err) <= STONE_FRONT_PAIR_CENTER_DEADBAND:
                    center_err = 0.0
                unload_vy = max(
                    -STONE_FRONT_PAIR_UNLOAD_VY_LIMIT,
                    min(STONE_FRONT_PAIR_UNLOAD_VY_LIMIT, -center_err * STONE_FRONT_PAIR_UNLOAD_VY_GAIN)
                )
            pair_yaw_corr = 0.0
            if sensor is not None and pose_ready:
                pair_yaw_corr = max(
                    -STONE_FRONT_PAIR_UNLOAD_YAW_LIMIT,
                    min(
                        STONE_FRONT_PAIR_UNLOAD_YAW_LIMIT,
                        stone_yaw_err * STONE_FRONT_PAIR_YAW_GAIN
                    )
            )
            if front_pair_hold_active:
                gait = GAIT_TROT_SLOW
                step_h = max_step_pair(step_h, STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR)
            else:
                gait = GAIT_TROT_SLOW
            final_vy = max(
                -STONE_FRONT_PAIR_UNLOAD_VY_LIMIT,
                min(STONE_FRONT_PAIR_UNLOAD_VY_LIMIT, unload_vy + side_damp_vy * 0.10)
            )
            final_vyaw = max(
                -STONE_FRONT_PAIR_UNLOAD_YAW_LIMIT,
                min(STONE_FRONT_PAIR_UNLOAD_YAW_LIMIT, side_damp_yaw * 0.10 + pair_yaw_corr)
            )
            if not front_pair_hold_active:
                step_h = max_step_pair(step_h, STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR)
            vx = max(vx, min(STONE_FRONT_PAIR_STEPUP_VX, STONE_CRAWL_VX))
            final_body_h = max(0.200, body_h - STONE_FRONT_PAIR_BODY_DROP)
            self.log_throttle(
                "stage1_front_pair_stepup_crawl",
                0.25,
                f"单前脚触石，保持前进速度并加大抬腿上台阶 "
                f"elapsed={front_pair_wait_elapsed:.2f} contact=0x{front_contact_bits:x} "
                f"lift={'L' if lift_left else ('R' if lift_right else '-')} vx={vx:.3f}"
            )

        final_pos = [0.0, 0.0, final_body_h]
        if recontact_active:
            elapsed = now - self.stone_recontact_start
            gait = STAGE1_GAIT_RECONTACT
            vx = min(vx, STONE_RECONTACT_FORWARD_VX)
            final_vy = max(-0.035, min(0.035, final_vy + self.stone_recontact_side * STONE_RECONTACT_SIDE_VY))
            final_vyaw = max(-0.055, min(0.055, final_vyaw))
            step_h = max_step_pair(step_h, STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR)
            final_pos[2] = body_h
            self.log_throttle(
                "stage1_recontact_step",
                0.25,
                f"石板边缘重踏步 elapsed={elapsed:.2f} vx={vx:.2f} vy={final_vy:.2f} "
                f"rpy=({final_rpy[0]:.2f},{final_rpy[1]:.2f})"
            )

        if abs(roll) > 0.10 or abs(pitch) > 0.09 or contact_impulse or relief_active:
            compliance_load = max(abs(roll) / 0.28, abs(pitch) / 0.30)
            compliance_load = max(0.0, min(1.0, compliance_load))
            if (contact_impulse or relief_active) and not recontact_active:
                ramp = max(0.0, min(1.0, (now - self.stone_contact_relief_start) / STONE_IMPACT_RAMP_SEC))
                relief_target_vx = STONE_STEPUP_VX if stepup_active else STONE_BALANCE_CRAWL_VX
                vx = min(vx, STONE_IMPACT_RELIEF_VX + (relief_target_vx - STONE_IMPACT_RELIEF_VX) * ramp)
                final_vy = max(-0.038, min(0.038, final_vy))
                final_vyaw = max(-0.085, min(0.085, final_vyaw))
                compliance_load = max(compliance_load, 0.7)
            elif not recontact_active:
                vx = min(vx, STONE_BALANCE_CRAWL_VX)
            if not recontact_active:
                final_pos[2] = max(0.202, body_h - 0.004 - 0.006 * compliance_load)
            step_h = max_step_pair(step_h, STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR)
            self.log_throttle(
                "stage1_compliance_balance",
                0.5,
                f"石板路柔顺姿态 roll={roll:.2f} pitch={pitch:.2f} "
                f"rate=({roll_rate:.2f},{pitch_rate:.2f}) odom_vy={odom_vy:.2f} "
                f"rpy=({final_rpy[0]:.2f},{final_rpy[1]:.2f}) "
                f"relief={relief_active} stepup={stepup_active} pair={pair_settle_active} "
                f"front_wait={front_pair_wait_active} hold={front_pair_hold_active} contact=0x{front_contact_bits:x} "
                f"recontact={recontact_active} step={format_step_height(step_h)} "
                f"vx={vx:.2f} vy={final_vy:.2f} yaw={final_vyaw:.2f} body_h={final_pos[2]:.3f}"
            )
        else:
            step_h = max_step_pair(step_h, STONE_APPROACH_STEP_HEIGHT, STONE_APPROACH_STEP_HEIGHT_REAR)

        vx, final_vy, final_vyaw, gait, step_h = self._limit_locomotion_for_risk(
            vx, final_vy, final_vyaw, gait, step_h, sensor
        )
        if (not suppress_min_progress and
                vx >= 0.0 and abs(roll) < IMU_ROLL_PREFALL and abs(pitch) < IMU_PITCH_PREFALL
                and not relief_active):
            min_progress_vx = STONE_IMPACT_RELIEF_VX if relief_active else STAGE1_HIGH_STEP_MIN_VX
            vx = max(vx, min_progress_vx)
        if gait == GAIT_USER_WALK_WAVE:
            self._send_stone_user_gait(vx, final_vy, final_vyaw, step_h, final_rpy)
            return
        self.send_cmd(MODE_LOCOMOTION, gait,
                      vx=vx, vy=final_vy, vyaw=final_vyaw,
                      step_h=step_h, rpy=final_rpy, pos=final_pos)

    def is_fallen(self, sensor):
        return sensor.is_fallen()

    def is_stuck(self, sensor):
        if abs(self.cmd_vx) < 0.02 and abs(self.cmd_vyaw) < 0.02:
            self.stuck_timer = 0.0
            return False
        if abs(sensor.imu_angvel_z) < 0.005 and abs(self.cmd_vyaw) > 0.1:
            self.stuck_timer += 0.02
        else:
            self.stuck_timer = 0.0
        return self.stuck_timer > 4.0

    def recover_from_fall(self):
        self.fall_count += 1
        if self.fall_count > 5:
            self.log("跌倒次数过多，停止比赛")
            return False
        self.log("检测到跌倒，执行恢复...")
        self.send_cmd(MODE_PURE_DAMPER, 0, step_h=0.0)
        time.sleep(0.5)
        self.send_cmd(MODE_RECOVERY_STAND, 0, step_h=0.0)
        recovery_cmd_time = self.last_cmd_update_time
        if not self.wait_finish(MODE_RECOVERY_STAND, timeout=10.0, after_time=recovery_cmd_time):
            self.log("恢复站立未完成或运控未返回新的RecoveryStand完成状态，停止继续发步态命令")
            return False
        time.sleep(0.2)
        if self.is_fallen(sensor_node):
            self.log("运控返回恢复完成，但IMU仍判定跌倒，停止继续发步态命令")
            return False
        self.log("恢复完成，继续比赛")
        return True

    def _total_elapsed(self):
        return time.time() - self.total_start_time

    def _advance_stage(self, next_stage):
        self.log(f"──► 进入{STAGE_NAMES.get(next_stage, f'赛段{next_stage}')}")
        self.stage = next_stage
        self.stage_start_time = time.time()
        self.stage_phase = 0
        self.phase_start = time.time()
        self.prev_steer = 0.0
        self.stuck_timer = 0.0
        self.centerline_history.clear()
        self.last_target_steer = 0.0
        self.last_target_time = 0.0
        self.last_line_target = None
        self.last_line_target_time = 0.0
        self.stage1_stone_hits = 0
        self.stage1_reached_rockroad = False
        self.stage1_exit_step = 0
        self.stage1_tail_clear_start = 0.0
        self.stage1_tail_clear_done = False
        self.stage1_edge_jump_state = 0
        self.stage1_edge_prejump_time = 0.0
        self.stage1_edge_jump_time = 0.0
        self.stage1_edge_jump_stand_until = 0.0
        self.stage1_edge_jump_resume_time = 0.0
        self.stage1_edge_jump_resume_sent = False
        self.stage1_edge_jump_handoff_until = 0.0
        self.stage1_edge_jump_issued = False
        self.stage1_edge_jump_start_x = None
        self.stage1_edge_jump_force_reset_sent = False
        self.stage1_edge_jump_force_reset_time = 0.0
        self.stage5_jump_time = 0.0
        self.stage5_jump_resume_sent = False
        self.stage5_jump_handoff_until = 0.0
        self.stage5_entry_jump_done = False
        self.stage5_entry_jump_time = 0.0
        self.stage5_entry_jump_resume_sent = False
        self.stage5_entry_jump_handoff_until = 0.0
        self.stage5_entry_jump_force_reset_sent = False
        self.stage5_entry_jump_force_reset_time = 0.0
        self.stage5_jump_force_reset_sent = False
        self.stage5_jump_force_reset_time = 0.0
        self.stone_roll_cmd = 0.0
        self.stone_pitch_cmd = STONE_CLIMB_PITCH_BIAS
        self.stone_contact_relief_until = 0.0
        self.stone_contact_relief_start = 0.0
        self.stone_pair_settle_until = 0.0
        self.stone_front_pair_wait_until = 0.0
        self.stone_front_pair_wait_start = 0.0
        self.stone_front_pair_pulse_start = 0.0
        self.stone_front_pair_step_until = 0.0
        self.stone_front_pair_step_side = 0
        self.stone_front_pair_step_sent = 0.0
        self.stone_recontact_until = 0.0
        self.stone_recontact_start = 0.0
        self.stone_recontact_side = 1.0
        self.stage3_entry_step = 0
        self.stage3_path_idx = 0
        self.stage4_entry_idx = 0
        self.stage4_exit_idx = 0
        self.target_history.clear()
        self.local_target_point = {'target_x': 0.0, 'target_y': -0.3, 'type': 'none', 'conf': 0.0, 'cm_offset': 0.0}
        self.relocating = False
        self.was_fallen = False
        self.lost_count = 0
        self.relocate_best_strength = 0.0
        self.waypoints.clear()
        self.waypoint_idx = 0
        self.grid_cells.clear()
        self.grid_cell_idx = 0
        self.grid_initialized = False
        self.stage2_targets = []
        self.stage2_target_idx = 0
        self.stage2_route_idx = 0
        self.stage2_exit_idx = 0
        self.stage2_target_visual_time = 0.0
        self.stage2_target_visual_area = 0.0
        self.stage2_target_visible_once = False
        self.stage2_entry_clear_done = False
        self.stage2_entry_faced_first_ball = False
        self.stage2_first_ball_last_dist = 99.0
        self.stage2_first_ball_stall_count = 0
        self.stage2_first_ball_force_mode = False
        self.stage2_ball_start_pos = {}
        self.stage2_hit_start_pos = None
        self.stage2_hit_target_idx = -1
        self.stage2_hit_target_point = None
        self.stage2_target_turn_ready_idx = -1
        self.stage2_axis_turn_start = 0.0
        self.stage2_axis_force_drive_until = 0.0
        self.stage2_axis_turning_target_idx = -1
        self.stage2_post_hit_brake_until = 0.0
        self.stage2_simple_last_target_idx = -1
        self.stage2_simple_last_dist = 99.0
        self.stage2_simple_last_progress_time = 0.0
        self.last_log_times.clear()

    def _time_in_phase(self):
        return time.time() - self.phase_start

    def _time_in_stage(self):
        return time.time() - self.stage_start_time

    def _check_fall(self):
        return False
        if self.is_fallen(sensor_node):
            if not self.recover_from_fall():
                self.stop_safe()
                self.running = False
            self.was_fallen = True
            self.relocating = True
            self.relocate_sweep_dir = 0.35
            self.relocate_best_strength = 0.0
            self.lost_count = 0
            self.phase_start = time.time()
            return True
        now = time.time()
        if now < self.prefall_until:
            self._prefall_brace(sensor_node)
            return True
        risk, critical = self._motion_risk(sensor_node)
        if critical:
            if self.stage == 1:
                if now >= self.stone_contact_relief_until:
                    self.stone_contact_relief_start = now
                self.stone_contact_relief_until = max(
                    self.stone_contact_relief_until,
                    now + STONE_IMPACT_RELIEF_SEC,
                )
            self.prefall_hold_count += 1
            self.prefall_until = now + PREFALL_BRACE_HOLD
            self.log_throttle(
                "prefall_hold",
                0.25,
                f"预跌倒保护: roll={sensor_node.imu_roll:.2f} "
                f"pitch={sensor_node.imu_pitch:.2f} "
                f"rate=({sensor_node.imu_angvel_x:.2f},{sensor_node.imu_angvel_y:.2f},{sensor_node.imu_angvel_z:.2f})"
            )
            self._prefall_brace(sensor_node)
            self.prev_steer = 0.0
            return True
        self.prefall_hold_count = 0
        self.prefall_until = 0.0
        return False

    def _relocate_to_track(self):
        """
        跌倒后/黄线丢失后的赛道重定位
        旋转扫描 360° 寻找最强黄线信号，定向回到赛道中心
        返回 True 表示还在重定位中，False 表示已完成
        """
        bgr = image_msg_to_cv(sensor_node.rgb_image_raw)

        offset, ok = detect_centerline(bgr)
        if ok:
            line_strength = 1.0 - abs(offset)
            if line_strength > self.relocate_best_strength:
                self.relocate_best_strength = line_strength
                self.relocate_best_centerline = offset
                self.log(f"重定位: 找到黄线 offset={offset:.2f} strength={line_strength:.2f}")

            self.lost_count = max(0, self.lost_count - 2)
        else:
            self.lost_count += 1

        if self.lost_count > 6:
            self.relocate_best_strength = max(0, self.relocate_best_strength - 0.1)

        if self.relocate_best_strength > 0.6:
            self.log(f"重定位成功! offset={self.relocate_best_centerline:.2f} strength={self.relocate_best_strength:.2f}")
            self.was_fallen = False
            self.relocating = False
            self.relocate_best_strength = 0.0
            self.lost_count = 0
            self.prev_steer = 0.0
            self.centerline_history.clear()
            return False

        if self._time_in_phase() > 20.0:
            self.log("重定位超时，使用最佳估计方向继续")
            self.was_fallen = False
            self.relocating = False
            self.lost_count = 0
            return False

        self.apply_locomotion(0.0, 0.0, self.relocate_sweep_dir, gait=GAIT_TROT_SLOW,
                              step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node, boundary_check=False)
        self.relocate_sweep_dir *= 1.002
        return True

    def _check_stuck(self):
        if self.is_stuck(sensor_node) and self._time_in_phase() > 3.0:
            if self.stage == 1:
                self.log("检测到卡住，第一赛段禁止后退，改为低速向前调整方向...")
                self.apply_locomotion(0.04, 0.0, 0.22, sensor=sensor_node)
            else:
                self.log("检测到卡住，尝试调整方向...")
                self.apply_locomotion(-0.10, 0.0, 0.30, sensor=sensor_node)
            self.phase_start = time.time()
            self.stuck_timer = 0.0
            self.prev_steer = 0.0

    def _check_stage_timeout(self, stage_num):
        if self._time_in_stage() > STAGE_TIMEOUTS[stage_num]:
            self.log_throttle(
                f"stage{stage_num}_timeout_warn",
                2.0,
                f"赛段{stage_num}超过建议时间，继续按地图目标点执行，不直接跳段"
            )
        return False

    def _using_gazebo_pose(self):
        if POSE_SOURCE == "tf":
            return False
        if not sensor_node.gazebo_pose_got:
            return False
        return time.time() - sensor_node.gazebo_pose_time <= GAZEBO_POSE_STALE_SEC

    def _has_global_pose(self):
        return self._using_gazebo_pose() or sensor_node.odom_got

    def _get_odom_pos(self):
        if self._using_gazebo_pose():
            return sensor_node.gazebo_pose_x, sensor_node.gazebo_pose_y, sensor_node.gazebo_pose_yaw
        if sensor_node.odom_got:
            return sensor_node.odom_x, sensor_node.odom_y, sensor_node.odom_yaw
        return 0.0, 0.0, 0.0

    def _distance_to(self, tx, ty):
        ox, oy, oyaw = self._get_odom_pos()
        return math.hypot(tx - ox, ty - oy)

    def _map_goal_status(self, goal, radius=MAP_GOAL_RADIUS):
        if not self._has_global_pose():
            return False, 99.0
        dist = self._distance_to(goal[0], goal[1])
        return dist <= radius, dist

    def _steer_to_map_goal(self, goal, gain=1.0, limit=0.45):
        steer, dist = self._compute_steer_to_waypoint(goal[0], goal[1])
        steer = max(-limit, min(limit, steer * gain))
        return steer, dist

    def _advance_when_goal_reached(self, goal, next_stage, radius=MAP_GOAL_RADIUS, label="目标点"):
        reached, dist = self._map_goal_status(goal, radius)
        if reached:
            self.log(f"到达{label} {goal} dist={dist:.2f}，进入下一赛段")
            self._advance_stage(next_stage)
            return True
        return False

    def _compute_steer_to_waypoint(self, target_x, target_y):
        ox, oy, oyaw = self._get_odom_pos()
        dx = target_x - ox
        dy = target_y - oy
        theta_target = math.atan2(dy, dx)
        diff = self._normalize_angle(theta_target - oyaw)
        steer = max(-0.5, min(0.5, diff * 0.6))
        return steer, math.hypot(dx, dy)

    def _normalize_angle(self, diff):
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    def _path_goal_passed(self, prev_goal, goal, margin=0.08):
        if not sensor_node.odom_got:
            return False
        ox, oy, oyaw = self._get_odom_pos()
        vx = goal[0] - prev_goal[0]
        vy = goal[1] - prev_goal[1]
        seg_len = math.hypot(vx, vy)
        if seg_len < 1e-6:
            return False
        ux = vx / seg_len
        uy = vy / seg_len
        progress = (ox - prev_goal[0]) * ux + (oy - prev_goal[1]) * uy
        return progress >= seg_len - margin

    def _remember_line_target(self, lt):
        if lt.get('type') in ('stone', 'centerline', 'curve') and lt.get('conf', 0.0) > 0.3:
            self.last_line_target = dict(lt)
            self.last_line_target_time = time.time()

    def _line_target_with_hold(self, lt, hold_sec=STAGE1_LINE_HOLD_SEC):
        self._remember_line_target(lt)
        if lt.get('type') in ('stone', 'centerline', 'curve') and lt.get('conf', 0.0) > 0.3:
            return lt, False
        if self.last_line_target is None:
            return lt, False
        age = time.time() - self.last_line_target_time
        if age > hold_sec:
            return lt, False
        held = dict(self.last_line_target)
        held['conf'] = max(0.25, held.get('conf', 0.4) * (1.0 - age / hold_sec))
        held['held'] = True
        return held, True

    # ── 初始化 ──────────────────────────────────────────

    def _run_init(self):
        if self.stage_phase == 0:
            self.log("初始化: 直接RecoveryStand站立")
            self.send_cmd(MODE_RECOVERY_STAND, 0, step_h=0.0)
            self.stage_phase = 2
            self.last_init_cmd_time = time.time()
            self.phase_start = time.time()

        elif self.stage_phase == 2:
            with self.response_lock:
                bar = self.response_bar
                resp_mode = self.response_mode
                resp_gait = self.response_gait_id
                resp_time = self.response_time
            if resp_time <= 0.0:
                self.log_throttle("init_no_control_response", 2.0,
                                  "站立初始化等待运控响应，若Gazebo中仍趴着请看/tmp/control.log")
            else:
                self.log_throttle(
                    "init_recovery_progress", 1.0,
                    f"RecoveryStand进度: resp_mode={resp_mode}, gait={resp_gait}, bar={bar}"
                )
            if bar >= 95 and resp_mode == MODE_RECOVERY_STAND:
                self.log("站立完成，进入比赛")
                self._advance_stage(1)
            elif self._time_in_phase() > 12.0:
                self.log("RecoveryStand超时，重新发送站立命令")
                self.stage_phase = 0
                self.phase_start = time.time()

    # ── 第一赛段：石径探路 ──────────────────────────────

    def _run_stage1(self):
        jump_recovery_active = self.stage1_edge_jump_state in (1, 2)
        if not jump_recovery_active and self._check_fall():
            return
        stage_progress = self._time_in_stage()
        if self._time_in_stage() > STAGE_TIMEOUTS[1]:
            self.log_throttle(
                "stage1_timeout_no_skip",
                2.0,
                "第一赛段超过建议时间，继续按石板出口目标点执行"
            )

        if self.stage_phase == 0:
            self.log("初始化石板路出口门点序列")
            self.log(
                f"石板步态参数 gait={STAGE1_GAIT_HIGH_STEP} vx={STONE_CRAWL_VX:.3f}/{STONE_APPROACH_VX:.3f} "
                f"stepup_vx={STONE_STEPUP_VX:.3f} impact_vx={STONE_IMPACT_RELIEF_VX:.3f} "
                f"stable_vx={STONE_STABLE_CRUISE_VX:.3f} balance_vx={STONE_BALANCE_CRAWL_VX:.3f} "
                f"pair_vx={STONE_PAIR_SETTLE_VX:.3f} "
                f"tail_clear_x={STAGE1_TAIL_CLEAR_X:.2f} tail_vx={STAGE1_TAIL_CLEAR_VX:.3f} "
                f"edge_jump_after_x={STONE_EDGE_JUMP_AFTER_X:.2f} "
                f"edge_trigger_x={STONE_EDGE_JUMP_TRIGGER_X:.2f} "
                f"step={STONE_CLIMB_STEP_HEIGHT:.3f} "
                f"body_h={STONE_BODY_HEIGHT:.3f} pitch_bias={STONE_CLIMB_PITCH_BIAS:.3f} "
                f"lane_vy_gain={STONE_LANE_VY_GAIN:.3f}"
            )
            self.stage_goal_x = STAGE1_EXIT_X
            self.stage_goal_y = STAGE1_EXIT_Y
            self.stage1_stone_hits = 0
            self.stage1_reached_rockroad = False
            self.stage1_exit_step = 0
            self.stage1_tail_clear_start = 0.0
            self.stage1_tail_clear_done = False
            self.stage1_edge_jump_state = 0
            self.stage1_edge_prejump_time = 0.0
            self.stage1_edge_jump_time = 0.0
            self.stage1_edge_jump_stand_until = 0.0
            self.stage1_edge_jump_resume_time = 0.0
            self.stage1_edge_jump_resume_sent = False
            self.stage1_edge_jump_handoff_until = 0.0
            self.stage1_edge_jump_issued = False
            self.stage1_edge_jump_start_x = None
            self.stage_phase = 1
            self.phase_start = time.time()
            self.log("站立后直接触发 Jump3D 前向小跳上石板")
            self.send_cmd(MODE_JUMP3D, STONE_EDGE_JUMP_GAIT, duration=900)
            self.stage1_edge_jump_state = 1
            self.stage1_edge_jump_issued = True
            self.stage1_edge_jump_time = time.time()
            self.stage1_edge_jump_stand_until = self.stage1_edge_jump_time + STONE_EDGE_JUMP_STAND_SEC
            self.stage1_edge_jump_resume_sent = False
            self.stage1_edge_jump_force_reset_sent = False
            self.stage1_edge_jump_force_reset_time = 0.0
            self.stage1_reached_rockroad = True
            return

        roll = sensor_node.imu_roll
        pitch = sensor_node.imu_pitch
        angvel_z = abs(sensor_node.imu_angvel_z)

        roll_corr = -roll * 3.0
        roll_corr = max(-0.45, min(0.45, roll_corr))

        pitch_factor = 1.0 + abs(pitch) * 1.5
        pitch_factor = max(1.0, min(2.0, pitch_factor))

        ox, oy, oyaw = self._get_odom_pos()
        stone_yaw_err = (
            self._normalize_angle(STAGE1_STONE_ALIGN_YAW - oyaw)
            if sensor_node.odom_got else 0.0
        )
        if sensor_node.odom_got and self.stage1_edge_jump_start_x is None:
            self.stage1_edge_jump_start_x = ox
        stage1_edge_walked_x = 0.0
        if sensor_node.odom_got and self.stage1_edge_jump_start_x is not None:
            stage1_edge_walked_x = ox - self.stage1_edge_jump_start_x
        near_end = sensor_node.odom_got and ox > STAGE1_MIN_EXIT_X
        stage1_gap_overrun_clear = (
            sensor_node.odom_got and
            STAGE1_GAP_X <= ox <= STAGE1_GAP_PASSED_X_MAX and
            oy >= STAGE1_GAP_OVERRUN_STAGE2_Y and
            abs(ox - STAGE1_GAP_X) <= STAGE1_BODY_MID_CLEAR_X_TOL
        )
        stage1_at_stage2_turn_point = (
            sensor_node.odom_got and
            abs(ox - STAGE1_GAP_X) <= STAGE1_STAGE2_TURN_X_TOL and
            abs(oy - STAGE1_STAGE2_TURN_Y) <= STAGE1_STAGE2_TURN_Y_TOL
        )
        stage1_body_mid_clear = (
            sensor_node.odom_got and
            (self.stage1_reached_rockroad or ox > STAGE1_ODOM_ROCKROAD_X) and
            (
                (oy >= STAGE1_BODY_MID_CLEAR_Y and
                 abs(ox - STAGE1_GAP_X) <= STAGE1_BODY_MID_CLEAR_X_TOL and
                 ox <= STAGE1_GAP_PASSED_X_MAX) or
                (STAGE1_GAP_PASSED_X <= ox <= STAGE1_GAP_PASSED_X_MAX and
                 oy >= STAGE1_GAP_PASSED_Y) or
                stage1_gap_overrun_clear
            )
        )
        if stage1_body_mid_clear and stage1_at_stage2_turn_point:
            self.stage1_tail_clear_done = True
            if self.stage1_exit_step < 2:
                self.stage1_exit_step = 2
                self.phase_start = time.time()
                self.log(
                    f"身体中段已到缺口中心线附近，但尚未执行红灰点转向流程，先切到缺口转向，不提前进第二赛段 "
                    f"odom=({ox:.2f},{oy:.2f}) clear_y={STAGE1_BODY_MID_CLEAR_Y:.2f}"
                )
                return
            self.log(
                f"身体中段已过一二赛段缺口，绕过第一赛段门点机制，直接进入第二赛段追第一个橙球 "
                f"odom=({ox:.2f},{oy:.2f}) step={self.stage1_exit_step} "
                f"clear_y={STAGE1_BODY_MID_CLEAR_Y:.2f} passed=({STAGE1_GAP_PASSED_X:.2f}..{STAGE1_GAP_PASSED_X_MAX:.2f},{STAGE1_GAP_PASSED_Y:.2f}) "
                f"overrun_clear={stage1_gap_overrun_clear}"
            )
            self._advance_stage(2)
            return
        hard_edge_jump_distance_ready = (
            sensor_node.odom_got and
            self.stage1_edge_jump_state == 0 and
            not near_end and
            (
                ox >= STONE_EDGE_JUMP_TRIGGER_X or
                stage1_edge_walked_x >= STONE_EDGE_JUMP_AFTER_X
            )
        )
        hard_edge_jump_ready = hard_edge_jump_distance_ready
        if hard_edge_jump_ready:
            self.log(
                f"起立后不行走，直接触发 Jump3D 前向小跳 "
                f"gait={STONE_EDGE_JUMP_GAIT} odom=({ox:.2f},{oy:.2f}) "
                f"walked_x={stage1_edge_walked_x:.2f} jump_after={STONE_EDGE_JUMP_AFTER_X:.2f} "
                f"trigger_x={STONE_EDGE_JUMP_TRIGGER_X:.2f} yaw_err={stone_yaw_err:.2f} "
                f"roll={roll:.2f} pitch={pitch:.2f} yaw_rate={angvel_z:.2f}"
            )
            self.send_cmd(MODE_JUMP3D, STONE_EDGE_JUMP_GAIT, duration=900)
            self.stage1_edge_jump_state = 1
            self.stage1_edge_jump_issued = True
            self.stage1_edge_jump_time = time.time()
            self.stage1_edge_jump_stand_until = self.stage1_edge_jump_time + STONE_EDGE_JUMP_STAND_SEC
            self.stage1_edge_jump_resume_sent = False
            self.stage1_edge_jump_force_reset_sent = False
            self.stage1_edge_jump_force_reset_time = 0.0
            self.stage1_reached_rockroad = True
            return
        if near_end and not self.stage1_tail_clear_done and self.stage1_tail_clear_start <= 0.0:
            self.stage1_tail_clear_start = time.time()
        tail_clear_elapsed = (
            time.time() - self.stage1_tail_clear_start
            if self.stage1_tail_clear_start > 0.0 else 0.0
        )
        tail_clear_active = (
            near_end and
            not self.stage1_tail_clear_done and
            sensor_node.odom_got and
            (
                ox < STAGE1_TAIL_CLEAR_X or
                tail_clear_elapsed < STAGE1_TAIL_CLEAR_SEC
            )
        )
        if near_end and not tail_clear_active:
            self.stage1_tail_clear_done = True
        pre_jump_window = (
            sensor_node.odom_got and
            self.stage1_edge_jump_state == 0 and
            not near_end and
            (
                STONE_EDGE_X_MIN <= ox <= STONE_EDGE_JUMP_DONE_X or
                stage1_edge_walked_x >= STONE_EDGE_JUMP_AFTER_X
            )
        )
        stone_jump_completed = self.stage1_edge_jump_state >= 3
        stepup_zone = (not self.stage1_reached_rockroad) and (
            not sensor_node.odom_got or ox < STONE_STEPUP_END_X
        ) and not pre_jump_window and not stone_jump_completed
        exit_dist = 99.0
        exit_steer = 0.0

        bgr = image_msg_to_cv(sensor_node.rgb_image_raw)
        lt = compute_local_target(bgr, stage=1)
        lt, line_held = self._line_target_with_hold(lt)
        if not self.stage1_reached_rockroad and lt['type'] == 'centerline':
            lt = dict(lt)
            lt['type'] = 'none'
            lt['conf'] = 0.0
            line_held = False
        stone_count_allowed = (not sensor_node.odom_got) or ox < STAGE1_MIN_EXIT_X
        if stone_count_allowed and lt['type'] == 'stone' and lt['conf'] > 0.3:
            self.stage1_stone_hits = min(30, self.stage1_stone_hits + 1)
        elif self.stage1_stone_hits > 0:
            self.stage1_stone_hits -= 1
        if self.stage1_stone_hits >= STAGE1_MIN_STONE_HITS:
            self.stage1_reached_rockroad = True
        if sensor_node.odom_got and ox > STAGE1_ODOM_ROCKROAD_X and abs(oy - STAGE1_CENTER_Y) < 0.75:
            self.stage1_reached_rockroad = True

        self.local_target_point = lt

        stone_stable_cruise = (
            sensor_node.odom_got and
            not stepup_zone and
            not near_end and
            self.stage1_reached_rockroad and
            not (STONE_EDGE_TRAP_START_X <= ox <= STONE_EDGE_TRAP_END_X) and
            abs(roll) < 0.075 and
            abs(pitch) < 0.075 and
            angvel_z < 0.75
        )
        stone_edge_trap_guard = (
            sensor_node.odom_got and
            not stepup_zone and
            not near_end and
            self.stage1_reached_rockroad and
            STONE_EDGE_TRAP_START_X <= ox <= STONE_EDGE_TRAP_END_X
        )
        base_vx = STONE_STABLE_CRUISE_VX if stone_stable_cruise else STONE_APPROACH_VX
        base_vy = 0.0
        base_step = STONE_APPROACH_STEP_HEIGHT
        stage1_gait = STAGE1_GAIT_HIGH_STEP
        stage1_suppress_min_progress = False
        post_climb_align = False
        post_climb_yaw_cmd = 0.0
        post_climb_yaw_err = 0.0
        post_climb_y_err = 0.0
        hard_lane_keep = False
        entry_align = False
        entry_align_yaw_cmd = 0.0
        entry_align_y_err = 0.0
        entry_align_yaw_err = 0.0
        if stone_stable_cruise:
            self.log_throttle(
                "stage1_stable_cruise",
                0.8,
                f"石板稳定巡航提速 odom=({ox:.2f},{oy:.2f}) vx={base_vx:.3f} "
                f"roll={roll:.2f} pitch={pitch:.2f} yaw_rate={angvel_z:.2f}"
            )
        if stepup_zone:
            base_vx = STONE_STEPUP_VX
            base_step = STONE_CLIMB_STEP_HEIGHT
            stage1_gait = STONE_DEFAULT_GAIT
            self.log_throttle(
                "stage1_stepup_window",
                0.8,
                f"石板入口跨台阶窗口: odom=({ox:.2f},{oy:.2f}) "
                f"vx={base_vx:.3f} gait={stage1_gait} step={base_step:.2f}"
            )
        elif stone_edge_trap_guard:
            base_vx = min(base_vx, STONE_EDGE_TRAP_VX)
            base_step = STONE_CLIMB_STEP_HEIGHT
            stage1_gait = STAGE1_GAIT_HIGH_STEP
            stage1_suppress_min_progress = True
            self.log_throttle(
                "stage1_stone_edge_trap_guard",
                0.35,
                f"石板跨边卡脚风险窗口：前后脚可能同时压边，降速高抬脚通过 "
                f"odom=({ox:.2f},{oy:.2f}) x_window=({STONE_EDGE_TRAP_START_X:.2f},{STONE_EDGE_TRAP_END_X:.2f}) "
                f"vx={base_vx:.3f} step={base_step:.2f}"
            )

        if pre_jump_window:
            pre_jump_elapsed = 0.0
            if self.stage1_edge_prejump_time > 0.0:
                pre_jump_elapsed = time.time() - self.stage1_edge_prejump_time
            y_ready = abs(oy - STAGE1_CENTER_Y) <= STONE_EDGE_JUMP_Y_TOL
            yaw_err = stone_yaw_err
            yaw_ready = abs(yaw_err) <= STONE_EDGE_JUMP_YAW_TOL
            posture_ready = abs(roll) < STONE_EDGE_JUMP_ROLL_TOL and abs(pitch) < STONE_EDGE_JUMP_PITCH_TOL
            distance_ready = STONE_EDGE_X_MIN <= ox <= STONE_EDGE_X_MAX
            edge_jump_candidate = distance_ready and y_ready and yaw_ready and posture_ready
            trigger_x_ready = ox >= STONE_EDGE_JUMP_TRIGGER_X
            walked_x = (
                ox - self.stage1_edge_jump_start_x
                if self.stage1_edge_jump_start_x is not None else 0.0
            )
            walked_jump_ready = walked_x >= STONE_EDGE_JUMP_AFTER_X
            force_edge_jump = (
                STONE_EDGE_X_MIN <= ox <= STONE_EDGE_JUMP_DONE_X and
                abs(oy - STAGE1_CENTER_Y) <= STONE_EDGE_JUMP_Y_TOL * 1.35 and
                abs(stone_yaw_err) <= STONE_EDGE_JUMP_FORCE_YAW_TOL and
                posture_ready and
                angvel_z < 0.55 and
                (
                    walked_jump_ready or
                    trigger_x_ready or
                    pre_jump_elapsed >= STONE_EDGE_JUMP_ALIGN_SEC
                )
            )
            if edge_jump_candidate or force_edge_jump:
                self.log(
                    f"到达石板前沿预跳点，触发 Jump3D 石板方向小跳 "
                    f"gait={STONE_EDGE_JUMP_GAIT} odom=({ox:.2f},{oy:.2f}) yaw={oyaw:.2f} "
                    f"walked_x={walked_x:.2f} jump_after={STONE_EDGE_JUMP_AFTER_X:.2f} "
                    f"yaw_err={yaw_err:.2f} trigger_x={STONE_EDGE_JUMP_TRIGGER_X:.2f} "
                    f"force={force_edge_jump}"
                )
                self.send_cmd(MODE_JUMP3D, STONE_EDGE_JUMP_GAIT, duration=900)
                self.stage1_edge_jump_state = 1
                self.stage1_edge_jump_issued = True
                self.stage1_edge_jump_time = time.time()
                self.stage1_edge_jump_stand_until = self.stage1_edge_jump_time + STONE_EDGE_JUMP_STAND_SEC
                self.stage1_edge_jump_resume_sent = False
                self.stage1_edge_jump_force_reset_sent = False
                self.stage1_edge_jump_force_reset_time = 0.0
                self.stage1_reached_rockroad = True
                return
            if self.stage1_edge_prejump_time <= 0.0:
                self.stage1_edge_prejump_time = time.time()
            y_err = STAGE1_CENTER_Y - oy
            base_vx = STONE_EDGE_JUMP_ALIGN_VX if yaw_ready else 0.0
            base_vy = max(
                -STONE_EDGE_JUMP_ALIGN_VY_LIMIT,
                min(STONE_EDGE_JUMP_ALIGN_VY_LIMIT, y_err * 0.18)
            )
            final_vyaw = max(
                -STONE_EDGE_JUMP_ALIGN_YAW_LIMIT,
                min(STONE_EDGE_JUMP_ALIGN_YAW_LIMIT, yaw_err * 0.90)
            )
            self.log_throttle(
                "stage1_edge_jump_align",
                0.25,
                f"石板前沿预跳窗口，短暂对齐后必须起跳 odom=({ox:.2f},{oy:.2f}) "
                f"x_window=({STONE_EDGE_X_MIN:.2f},{STONE_EDGE_X_MAX:.2f}) "
                f"trigger_x={STONE_EDGE_JUMP_TRIGGER_X:.2f} stone_y={lt['target_y']:.2f} "
                f"walked_x={walked_x:.2f} jump_after={STONE_EDGE_JUMP_AFTER_X:.2f} "
                f"align_sec={STONE_EDGE_JUMP_ALIGN_SEC:.2f} elapsed={pre_jump_elapsed:.2f} "
                f"y_err={y_err:.2f} yaw_err={yaw_err:.2f} target_yaw={STAGE1_STONE_ALIGN_YAW:.2f} "
                f"yaw_ready={yaw_ready} "
                f"jump_gait={STONE_EDGE_JUMP_GAIT} "
                f"roll={roll:.2f} pitch={pitch:.2f}"
            )
            self.send_cmd(
                MODE_LOCOMOTION,
                STAGE1_GAIT_HIGH_STEP,
                vx=base_vx,
                vy=base_vy,
                vyaw=final_vyaw,
                step_h=(STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR),
                rpy=[0.0, min(STONE_CLIMB_PITCH_BIAS, 0.045), 0.0],
                pos=[0.0, 0.0, STONE_BODY_HEIGHT],
            )
            return
        if self.stage1_edge_jump_state == 1:
            now = time.time()
            jump_elapsed = now - self.stage1_edge_jump_time
            if sensor_node.odom_got and ox > STONE_EDGE_JUMP_DONE_X:
                self.stage1_edge_jump_state = 2
                self.stage1_edge_jump_resume_time = now
                if self.stage1_edge_jump_stand_until <= 0.0:
                    self.stage1_edge_jump_stand_until = now + STONE_EDGE_JUMP_STAND_SEC
                self.stage1_edge_jump_resume_sent = False
                self.stage1_edge_jump_force_reset_sent = False
                self.stage1_edge_jump_force_reset_time = 0.0
                self.stage1_reached_rockroad = True
                self.log(f"Jump3D 前向小跳位移完成，开始恢复接管 odom=({ox:.2f},{oy:.2f})")
            elif jump_elapsed < JUMP_RECOVERY_MIN_SEC:
                self.log_throttle(
                    "stage1_edge_jump_wait",
                    0.35,
                    f"Jump3D 前向小跳执行中，暂停普通行走命令 elapsed={jump_elapsed:.2f} odom=({ox:.2f},{oy:.2f})"
                )
                return
            else:
                self.stage1_edge_jump_state = 2
                self.stage1_edge_jump_resume_time = now
                if self.stage1_edge_jump_stand_until <= 0.0:
                    self.stage1_edge_jump_stand_until = now + STONE_EDGE_JUMP_STAND_SEC
                self.stage1_edge_jump_resume_sent = False
                self.stage1_edge_jump_force_reset_sent = False
                self.stage1_edge_jump_force_reset_time = 0.0
                self.log_throttle(
                    "stage1_edge_jump_timeout",
                    0.8,
                    f"Jump3D 前向小跳等待结束，开始恢复接管 odom=({ox:.2f},{oy:.2f})"
                )
        if self.stage1_edge_jump_state == 2:
            now = time.time()
            if not self.stage1_edge_jump_resume_sent:
                elapsed = now - self.stage1_edge_jump_time
                min_recover_sec = JUMP_RECOVERY_MIN_SEC
                if elapsed < min_recover_sec:
                    self.log_throttle(
                        "stage1_edge_jump_recover_min",
                        0.35,
                        f"石板入口前向小跳后短暂等待落地 elapsed={elapsed:.2f}"
                    )
                    return
                self.stage1_edge_jump_resume_sent = True
                self.stage1_edge_jump_handoff_until = now + max(JUMP_HANDOFF_SEC, 0.80)
                self.stage1_edge_jump_resume_time = now
                self.log("石板入口前向小跳落地后不收腿，直接正向Locomotion接管继续前进")
            if now < self.stage1_edge_jump_handoff_until:
                self.log_throttle(
                    "stage1_edge_jump_handoff",
                    0.25,
                    "石板入口小跳恢复后正向Locomotion接管，直接沿石板路前进"
                )
                self._send_locomotion_handoff(
                    gait=STAGE1_GAIT_HIGH_STEP,
                    body_h=STONE_BODY_HEIGHT,
                    step_h=STONE_CLIMB_STEP_HEIGHT,
                    pitch=min(STONE_CLIMB_PITCH_BIAS, 0.045),
                    vx=max(0.10, STONE_EDGE_JUMP_DRIVE_VX),
                )
                return
            drive_elapsed = now - self.stage1_edge_jump_resume_time
            drive_done = (
                sensor_node.odom_got and
                ox >= STONE_EDGE_JUMP_DRIVE_DONE_X and
                drive_elapsed >= STONE_EDGE_JUMP_DRIVE_SEC
            )
            if drive_elapsed < STONE_EDGE_JUMP_DRIVE_SEC and not drive_done:
                drive_yaw = 0.0
                drive_vy = 0.0
                drive_vx = max(0.10, STONE_EDGE_JUMP_DRIVE_VX)
                if sensor_node.odom_got:
                    drive_yaw_err = self._normalize_angle(STAGE1_STONE_ALIGN_YAW - oyaw)
                    drive_yaw = max(
                        -STONE_EDGE_JUMP_DRIVE_YAW_LIMIT,
                        min(STONE_EDGE_JUMP_DRIVE_YAW_LIMIT, drive_yaw_err * 0.45)
                    )
                    drive_vy = max(
                        -STONE_LANE_VY_LIMIT,
                        min(STONE_LANE_VY_LIMIT, (STAGE1_CENTER_Y - oy) * STONE_LANE_VY_GAIN)
                    )
                self.log_throttle(
                    "stage1_edge_jump_drive",
                    0.35,
                    f"Jump3D 后短推进上石板 elapsed={drive_elapsed:.2f}/{STONE_EDGE_JUMP_DRIVE_SEC:.2f} "
                    f"odom=({ox:.2f},{oy:.2f}) vx={drive_vx:.2f} "
                    f"vy={drive_vy:.2f} yaw={drive_yaw:.2f}"
                )
                self.send_cmd(
                    MODE_LOCOMOTION,
                    STAGE1_GAIT_HIGH_STEP,
                    vx=drive_vx,
                    vy=drive_vy,
                    vyaw=max(-STONE_EDGE_TRAP_YAW_LIMIT, min(STONE_EDGE_TRAP_YAW_LIMIT, drive_yaw)),
                    step_h=(STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR),
                    rpy=[0.0, min(STONE_CLIMB_PITCH_BIAS, 0.045), 0.0],
                    pos=[0.0, 0.0, STONE_BODY_HEIGHT],
                )
                return
            self.log_throttle(
                "stage1_edge_jump_drive_done",
                0.8,
                f"Jump3D 后推进完成，交回石板路普通行走 elapsed={drive_elapsed:.2f} "
                f"odom=({ox:.2f},{oy:.2f}) drive_done={drive_done}"
            )
            self.stage1_edge_jump_state = 3
        exit_goals = [
            ("石板末端", STAGE1_STONE_EXIT_POINT, STAGE1_EXIT_RADIUS),
            ("红灰线交叉转弯点", STAGE1_RED_GRAY_TURN_POINT, STAGE1_GAP_BELOW_RADIUS),
            ("一二赛段缺口内转正点", STAGE1_GAP_POINT, STAGE1_GAP_RADIUS),
            ("一二赛段缺口内侧分界线", STAGE1_GAP_INNER_POINT, STAGE1_ENTRY_RADIUS),
        ]
        exit_step = max(0, min(self.stage1_exit_step, len(exit_goals) - 1))
        exit_label, exit_goal, exit_radius = exit_goals[exit_step]
        exit_steer, exit_dist = self._compute_steer_to_waypoint(*exit_goal)
        gap_steer, gap_dist = self._compute_steer_to_waypoint(*STAGE1_GAP_POINT)
        goal_allowed = (
            sensor_node.odom_got and
            ox > STAGE1_GOAL_ENABLE_X and
            (self.stage1_reached_rockroad or stage_progress > 8.0)
        )
        goal_steer = exit_steer if goal_allowed else 0.0
        centerline_bias = 0.0
        if sensor_node.odom_got:
            centerline_bias = max(-0.16, min(0.16, -(oy - STAGE1_CENTER_Y) * 0.45))
        goal_term = max(-0.18, min(0.18, goal_steer * STAGE1_GOAL_STEER_WEIGHT + centerline_bias))
        lane_steer = 0.0
        if sensor_node.odom_got and not self.stage1_reached_rockroad:
            lane_x = max(STAGE1_ENTRY_X, min(STAGE1_MIN_EXIT_X, ox + 0.35))
            lane_steer, _ = self._compute_steer_to_waypoint(lane_x, STAGE1_CENTER_Y)
            lane_steer = max(-0.28, min(0.28, lane_steer * 0.9 + centerline_bias * 0.8))
        if sensor_node.odom_got and not near_end and not stepup_zone:
            post_climb_y_err = STAGE1_CENTER_Y - oy
            post_climb_yaw_err = stone_yaw_err
            post_climb_align = (
                STONE_POST_ALIGN_START_X <= ox <= STONE_POST_ALIGN_END_X and
                (abs(post_climb_y_err) > STONE_POST_ALIGN_Y_TOL or
                 abs(post_climb_yaw_err) > STONE_POST_ALIGN_YAW_TOL)
            )
            if post_climb_align:
                base_vx = min(base_vx, STONE_POST_ALIGN_VX)
                base_vy = max(
                    -STONE_POST_ALIGN_VY_LIMIT,
                    min(STONE_POST_ALIGN_VY_LIMIT, post_climb_y_err * STONE_POST_ALIGN_VY_GAIN)
                )
                post_climb_yaw_cmd = max(
                    -STONE_POST_ALIGN_YAW_LIMIT,
                    min(STONE_POST_ALIGN_YAW_LIMIT, post_climb_yaw_err * STONE_POST_ALIGN_YAW_GAIN)
                )
                self.log_throttle(
                    "stage1_post_climb_center_align",
                    0.5,
                    f"上石板后中线/朝向校正 odom=({ox:.2f},{oy:.2f}) "
                    f"y_err={post_climb_y_err:.2f} yaw_err={post_climb_yaw_err:.2f} "
                    f"vx={base_vx:.3f} vy={base_vy:.3f} yaw={post_climb_yaw_cmd:.3f}"
                )
            hard_lane_keep = (
                STONE_POST_ALIGN_START_X <= ox <= STAGE1_MIN_EXIT_X and
                (abs(post_climb_y_err) >= STONE_LANE_HARD_Y or
                 abs(post_climb_yaw_err) >= STONE_POST_ALIGN_YAW_TOL * 1.6)
            )
            if hard_lane_keep:
                base_vx = min(base_vx, STONE_LANE_HARD_VX)
                base_vy = max(
                    -STONE_LANE_HARD_VY_LIMIT,
                    min(STONE_LANE_HARD_VY_LIMIT, post_climb_y_err * STONE_LANE_HARD_VY_GAIN)
                )
                post_climb_yaw_cmd = max(
                    -STONE_LANE_HARD_YAW_LIMIT,
                    min(STONE_LANE_HARD_YAW_LIMIT, post_climb_yaw_err * STONE_LANE_HARD_YAW_GAIN)
                )
                self.log_throttle(
                    "stage1_hard_lane_keep",
                    0.25,
                    f"石板路硬车道保持：先回中线/对准x轴，防止跌出黄线 "
                    f"odom=({ox:.2f},{oy:.2f}) y_err={post_climb_y_err:.2f} "
                    f"yaw_err={post_climb_yaw_err:.2f} vx={base_vx:.3f} "
                    f"vy={base_vy:.3f} yaw={post_climb_yaw_cmd:.3f}"
                )
        front_left, front_right, front_contact_bits = self._front_contact_state()
        now = time.time()
        front_single_contact = front_left != front_right
        if sensor_node.odom_got and stepup_zone:
            entry_align_y_err = STAGE1_CENTER_Y - oy
            entry_align_yaw_err = stone_yaw_err
            entry_align = (
                ox < STONE_ENTRY_ALIGN_END_X and
                not front_single_contact and
                (abs(entry_align_y_err) > STONE_ENTRY_ALIGN_Y_TOL or
                 abs(entry_align_yaw_err) > STONE_ENTRY_ALIGN_YAW_TOL)
            )
            if entry_align:
                goal_term = 0.0
                lane_steer = 0.0
                base_vx = min(base_vx, STONE_ENTRY_ALIGN_VX)
                base_vy = max(
                    -STONE_ENTRY_ALIGN_VY_LIMIT,
                    min(STONE_ENTRY_ALIGN_VY_LIMIT, entry_align_y_err * STONE_ENTRY_ALIGN_VY_GAIN)
                )
                entry_align_yaw_cmd = max(
                    -STONE_ENTRY_ALIGN_YAW_LIMIT,
                    min(STONE_ENTRY_ALIGN_YAW_LIMIT, entry_align_yaw_err * STONE_ENTRY_ALIGN_YAW_GAIN)
                )
                self.log_throttle(
                    "stage1_entry_align_before_step",
                    0.4,
                    f"石板入口先对齐再上台阶 odom=({ox:.2f},{oy:.2f}) "
                    f"y_err={entry_align_y_err:.2f} yaw_err={entry_align_yaw_err:.2f} "
                    f"vx={base_vx:.3f} vy={base_vy:.3f} yaw={entry_align_yaw_cmd:.3f}"
                )
        if stepup_zone and front_single_contact and STONE_FRONT_PAIR_WAIT_SEC > 0.0:
            if now >= self.stone_front_pair_wait_until:
                self.stone_front_pair_wait_start = now
                self.stone_front_pair_pulse_start = 0.0
            self.stone_front_pair_wait_until = max(
                self.stone_front_pair_wait_until,
                now + STONE_FRONT_PAIR_WAIT_SEC,
            )
        front_pair_wait_hint = (
            stepup_zone and
            STONE_FRONT_PAIR_WAIT_SEC > 0.0 and
            (front_single_contact or now < self.stone_front_pair_wait_until) and
            not (front_left and front_right)
        )
        stepup_pair_hold = (
            stepup_zone and (
                front_pair_wait_hint or
                now < self.stone_pair_settle_until or
                (sensor_node.odom_got and abs(oy - STAGE1_CENTER_Y) > STONE_PAIR_SETTLE_Y) or
                abs(roll) > 0.10 or abs(pitch) > 0.08
            )
        )
        if stepup_pair_hold:
            goal_term = 0.0
            lane_steer = 0.0
            if front_pair_wait_hint:
                pair_wait_elapsed = 0.0
                if self.stone_front_pair_wait_start > 0.0:
                    pair_wait_elapsed = max(0.0, now - self.stone_front_pair_wait_start)
                if pair_wait_elapsed < STONE_FRONT_PAIR_HOLD_SEC:
                    stage1_gait = GAIT_TROT_SLOW
                else:
                    stage1_gait = GAIT_TROT_SLOW
            base_vx = max(base_vx, min(STONE_FRONT_PAIR_STEPUP_VX, STONE_CRAWL_VX))
            base_step = max(base_step, STONE_CLIMB_STEP_HEIGHT)
            self.log_throttle(
                "stage1_pair_hold_steer",
                0.5,
                f"石板入口单前脚补脚窗口：屏蔽横摆并加大抬腿，不降低前进速度 odom=({ox:.2f},{oy:.2f}) "
                f"roll={roll:.2f} pitch={pitch:.2f} contact=0x{front_contact_bits:x} vx={base_vx:.3f}"
            )

        if near_end:
            self.log_throttle(
                "stage1_exit_target", 1.0,
                f"石板路末端: 当前门点{exit_step + 1}/{len(exit_goals)} {exit_label}({exit_goal[0]:.2f},{exit_goal[1]:.2f}) "
                f"缺口目标({STAGE1_GAP_POINT[0]:.1f},{STAGE1_GAP_POINT[1]:.1f}) "
                f"dist={exit_dist:.2f} gap_dist={gap_dist:.2f} odom=({ox:.1f},{oy:.1f})"
            )
            if (sensor_node.odom_got and
                    oy < STAGE1_GAP_RECOVER_Y_MAX and
                    ox >= STAGE1_GAP_HARD_WALL_X):
                self._stage1_apply_gap_hard_wall(ox, oy, oyaw, "下石板后身体中心已到/越过右侧黄线安全上限")
                self._check_stuck()
                return
            at_red_gray_turn_point = (
                sensor_node.odom_got and
                abs(STAGE1_GAP_X - ox) <= STAGE1_RED_GRAY_LOCK_X_TOL and
                abs(STAGE1_GAP_BELOW_Y - oy) <= STAGE1_RED_GRAY_LOCK_Y_TOL
            )
            if at_red_gray_turn_point and self.stage1_exit_step < 2:
                self.stage1_tail_clear_done = True
                self.stage1_exit_step = 2
                self.phase_start = time.time()
                self.log(
                    f"身体中心已到红灰线交叉点锁定区，停止继续前冲，立刻转向缺口 "
                    f"odom=({ox:.2f},{oy:.2f}) target=({STAGE1_GAP_X:.2f},{STAGE1_GAP_BELOW_Y:.2f})"
                )
                return
            if tail_clear_active:
                clear_elapsed = time.time() - self.stage1_tail_clear_start
                world_x_err = STAGE1_GAP_X - ox
                world_y_err = STAGE1_GAP_BELOW_Y - oy
                world_vx = max(0.0, min(0.22, world_x_err * 0.95))
                world_vy = max(-0.10, min(0.10, world_y_err * 0.70))
                base_vx, base_vy = self._world_to_body_velocity(world_vx, world_vy, oyaw)
                base_vx = max(0.0, min(STAGE1_TAIL_CLEAR_VX, base_vx))
                base_vy = max(-0.09, min(0.09, base_vy))
                base_step = STONE_CLIMB_STEP_HEIGHT
                stage1_gait = STAGE1_GAIT_HIGH_STEP
                stage1_suppress_min_progress = True
                self.log_throttle(
                    "stage1_tail_clear",
                    0.4,
                    f"石板下台阶清尾：不再直冲右黄线，直接追红灰线交叉点 "
                    f"odom=({ox:.2f},{oy:.2f}) elapsed={clear_elapsed:.2f} "
                    f"target=({STAGE1_GAP_X:.2f},{STAGE1_GAP_BELOW_Y:.2f}) "
                    f"world_err=({world_x_err:.2f},{world_y_err:.2f}) "
                    f"cmd=({base_vx:.3f},{base_vy:.3f})"
                )
                self.apply_stabilized_locomotion(
                    base_vx, base_vy, 0.0,
                    gait=stage1_gait,
                    step_h=base_step,
                    sensor=sensor_node,
                    body_h=STONE_BODY_HEIGHT,
                    stepup_active=stepup_zone,
                    suppress_min_progress=True,
                )
                self._check_stuck()
                return
            if (sensor_node.odom_got and
                    self.stage1_reached_rockroad and
                    self.stage1_tail_clear_done and
                    self.stage1_exit_step >= 2 and
                    stage1_at_stage2_turn_point and
                    ox >= STAGE1_DIRECT_STAGE2_X and
                    ox <= STAGE1_GAP_PASSED_X_MAX and
                    abs(oy - STAGE1_DIRECT_STAGE2_Y) <= STAGE1_STAGE2_TURN_Y_TOL):
                self.log(
                    f"已完全下石板并到缺口附近，直接进入第二赛段追第一个橙色球，避免继续踩黄线 "
                    f"odom=({ox:.2f},{oy:.2f}) direct=({STAGE1_DIRECT_STAGE2_X:.2f},{STAGE1_DIRECT_STAGE2_Y:.2f})"
                )
                self._advance_stage(2)
                return
            rear_clear_stage1 = (
                sensor_node.odom_got and
                self.stage1_reached_rockroad and
                self.stage1_exit_step >= 2 and
                stage1_at_stage2_turn_point and
                STAGE1_REAR_CLEAR_X_MIN <= ox <= STAGE1_REAR_CLEAR_X_MAX and
                abs(oy - STAGE1_REAR_CLEAR_Y) <= STAGE1_STAGE2_TURN_Y_TOL and
                self.stage1_tail_clear_done
            )
            if rear_clear_stage1:
                self.log(
                    f"判定后腿已通过一二赛段黄线缺口，直接进入第二赛段 "
                    f"odom=({ox:.2f},{oy:.2f}) step={self.stage1_exit_step}"
                )
                self._advance_stage(2)
                return
            base_vx = STONE_CRAWL_VX
            base_step = STONE_CLIMB_STEP_HEIGHT
            stone_exit_crossed = (
                self.stage1_exit_step == 0 and
                sensor_node.odom_got and
                ox > STAGE1_STONE_EXIT_CROSS_X and
                self.stage1_reached_rockroad
            )
            if stone_exit_crossed and self.stage1_exit_step < 1:
                self.stage1_exit_step = 1
                self.phase_start = time.time()
                self.log(
                    f"已越过石板末端x线，先行至红灰线交叉转弯点，再转向一二赛段缺口 odom=({ox:.2f},{oy:.2f})"
                )
                return
            elif self.stage1_exit_step != 2 and exit_dist < exit_radius and self.stage1_exit_step < len(exit_goals) - 1:
                gap_turn_safe = True
                if self.stage1_exit_step == 1 and sensor_node.odom_got:
                    gap_turn_safe = (
                        abs(STAGE1_GAP_X - ox) <= STAGE1_GAP_X_TOL and
                        abs(STAGE1_GAP_BELOW_Y - oy) <= STAGE1_GAP_TURN_Y_TOL
                    )
                if self.stage1_exit_step == 1 and not gap_turn_safe:
                    self.log_throttle(
                        "stage1_gap_not_aligned",
                        0.5,
                        f"未到缺口入口，继续贴近缺口 "
                        f"odom=({ox:.2f},{oy:.2f}) "
                        f"x_err={STAGE1_GAP_X - ox:.2f} y_err={STAGE1_GAP_BELOW_Y - oy:.2f}"
                    )
                else:
                    self.stage1_exit_step += 1
                    self.phase_start = time.time()
                    next_label, next_goal, _ = exit_goals[self.stage1_exit_step]
                    self.log(f"第一赛段到达{exit_label}，切换到{next_label}门点 {next_goal}")
                    return

            if tail_clear_active:
                pass
            elif self.stage1_exit_step == 1:
                base_vx = STAGE1_GAP_CRAWL_VX
                stage1_gait = STAGE1_GAIT_HIGH_STEP
                gap_yaw_limit = STAGE1_GAP_YAW_LIMIT
                gap_x_err = STAGE1_GAP_X - ox if sensor_node.odom_got else 0.0
                x_err = (
                    STAGE1_GAP_TURN_X - ox
                    if sensor_node.odom_got and oy < STAGE1_GAP_EARLY_TURN_Y else
                    gap_x_err
                )
                y_err = STAGE1_GAP_BELOW_Y - oy if sensor_node.odom_got else 0.0
                yaw_to_gap = self._normalize_angle(STAGE1_GAP_ALIGN_YAW - oyaw) if sensor_node.odom_got else 0.0
                turn_stage_x_err = STAGE1_GAP_TURN_X - ox if sensor_node.odom_got else 0.0
                gap_turn_stage = (
                    sensor_node.odom_got and
                    abs(STAGE1_GAP_X - ox) <= STAGE1_GAP_X_TOL and
                    abs(STAGE1_GAP_BELOW_Y - oy) <= STAGE1_GAP_TURN_Y_TOL
                )
                gap_outer_overrun = (
                    sensor_node.odom_got and
                    oy < STAGE1_GAP_RECOVER_Y_MAX and
                    ox > STAGE1_GAP_X_SAFE_MAX
                )
                gap_x_aligned = sensor_node.odom_got and abs(x_err) <= STAGE1_GAP_X_ALIGN_TOL
                gap_y_aligned = sensor_node.odom_got and abs(y_err) <= STAGE1_GAP_TURN_Y_TOL
                gap_early_turn = (
                    sensor_node.odom_got and
                    abs(gap_x_err) <= STAGE1_GAP_EARLY_TURN_X_TOL and
                    oy >= STAGE1_GAP_EARLY_TURN_Y
                )
                base_vy = max(
                    -STAGE1_GAP_TURN_VY_LIMIT,
                    min(STAGE1_GAP_TURN_VY_LIMIT, y_err * STAGE1_GAP_TURN_VY_GAIN)
                )
                if gap_turn_stage:
                    self.stage1_exit_step = 2
                    self.phase_start = time.time()
                    self.log(
                        f"身体中心已到红灰线交叉转弯点，立刻转向一二赛段缺口 "
                        f"odom=({ox:.2f},{oy:.2f}) turn_x={STAGE1_GAP_TURN_X:.2f} "
                        f"gap_x={STAGE1_GAP_X:.2f} yaw_err={yaw_to_gap:.2f}"
                    )
                    return
                elif gap_outer_overrun:
                    self.stage1_tail_clear_done = True
                    self.stage1_exit_step = 2
                    self.phase_start = time.time()
                    self.log_throttle(
                        "stage1_gap_overrun_no_reverse",
                        0.25,
                        f"已到红灰线附近且略偏右，禁止倒车回拉，直接转向缺口 "
                        f"odom=({ox:.2f},{oy:.2f}) safe_x={STAGE1_GAP_X_SAFE_MAX:.2f}"
                    )
                    return
                else:
                    if sensor_node.odom_got:
                        world_x_err = STAGE1_GAP_X - ox
                        world_y_err = STAGE1_GAP_BELOW_Y - oy
                        world_vx = max(0.0, min(0.06, world_x_err * 0.85))
                        world_vy = max(-0.04, min(0.18, world_y_err * 0.70))
                        base_vx, base_vy = self._world_to_body_velocity(world_vx, world_vy, oyaw)
                        base_vx = max(0.0, min(STAGE1_GAP_CRAWL_VX, base_vx))
                        base_vy = max(-STAGE1_GAP_TURN_VY_LIMIT, min(STAGE1_GAP_TURN_VY_LIMIT, base_vy))
                        stage1_suppress_min_progress = True
                    else:
                        base_vx = STAGE1_GAP_CRAWL_VX
                    final_vyaw = max(-0.06, min(0.06, stone_yaw_err * 0.50))
                    self.log_throttle(
                        "stage1_gap_below",
                        0.5,
                        f"身体中心未到红灰线交叉转弯点，先精确走到转弯点再转缺口 odom=({ox:.2f},{oy:.2f}) "
                        f"target={exit_label} dist={exit_dist:.2f} x_err={x_err:.2f} gap_x_err={gap_x_err:.2f} y_err={y_err:.2f} "
                        f"vy={base_vy:.3f} yaw=0.00"
                    )
            elif self.stage1_exit_step == 2:
                stage1_suppress_min_progress = True
                yaw_err = self._normalize_angle(STAGE1_GAP_ALIGN_YAW - oyaw) if sensor_node.odom_got else 0.0
                x_err = STAGE1_GAP_X - ox if sensor_node.odom_got else 0.0
                turn_y_err = STAGE1_STAGE2_TURN_Y - oy if sensor_node.odom_got else 0.0
                if abs(yaw_err) > STAGE1_GAP_ALIGN_YAW_TOL:
                    base_vx = 0.0
                    base_vy = max(-0.018, min(0.018, -x_err * 0.08))
                    final_vyaw = max(-STAGE1_GAP_FAST_YAW_LIMIT,
                                      min(STAGE1_GAP_FAST_YAW_LIMIT,
                                          yaw_err * 1.15 * STAGE1_GAP_TURN_SPEED_SCALE))
                    self.log_throttle(
                        "stage1_gap_align_yaw",
                        0.4,
                        f"已到红灰线交叉点，先原地对准缺口，禁止未对准时继续顶右黄线 "
                        f"yaw_err={yaw_err:.2f} odom=({ox:.2f},{oy:.2f}) "
                        f"x_err={x_err:.2f} vy={base_vy:.2f} yaw={final_vyaw:.2f}"
                    )
                else:
                    world_x_err = STAGE1_GAP_X - ox if sensor_node.odom_got else 0.0
                    world_y_err = STAGE1_STAGE2_TURN_Y - oy if sensor_node.odom_got else 0.0
                    world_vx = max(-0.050, min(0.050, world_x_err * 0.55))
                    world_vy = max(-0.080, min(STAGE1_GAP_STRAIGHT_VX, world_y_err * 0.65))
                    base_vx, base_vy = self._world_to_body_velocity(world_vx, world_vy, oyaw)
                    base_vx = max(-0.080, min(STAGE1_GAP_STRAIGHT_VX, base_vx))
                    base_vy = max(-0.050, min(0.050, base_vy))
                    final_vyaw = max(-0.180, min(0.180, yaw_err * 0.22))
                    stage1_suppress_min_progress = True
                    self.log_throttle(
                        "stage1_stage2_turn_lock",
                        0.4,
                        f"已朝向缺口，锁定进入第二赛段固定转弯点 "
                        f"target=({STAGE1_GAP_X:.2f},{STAGE1_STAGE2_TURN_Y:.2f}) "
                        f"odom=({ox:.2f},{oy:.2f}) err=({world_x_err:.2f},{world_y_err:.2f}) "
                        f"cmd=({base_vx:.2f},{base_vy:.2f},{final_vyaw:.2f})"
                    )
                if (sensor_node.odom_got and
                        stage1_at_stage2_turn_point):
                    self.log(
                        f"base_link已精确到达进入第二赛段固定转弯点，立刻交给第二赛段坐标导航 "
                        f"odom=({ox:.2f},{oy:.2f}) target=({STAGE1_GAP_X:.2f},{STAGE1_STAGE2_TURN_Y:.2f}) "
                        f"err=({x_err:.2f},{turn_y_err:.2f})"
                    )
                    self._advance_stage(2)
                    return
            elif self.stage1_exit_step == 3:
                if (sensor_node.odom_got and
                        stage1_at_stage2_turn_point and
                        STAGE1_GAP_PASSED_X <= ox <= STAGE1_GAP_PASSED_X_MAX):
                    self.log(
                        f"身体中心已到达进入第二赛段固定转弯点，进入第二赛段坐标导航 "
                        f"odom=({ox:.2f},{oy:.2f}) target=({STAGE1_GAP_X:.2f},{STAGE1_STAGE2_TURN_Y:.2f})"
                    )
                    self._advance_stage(2)
                    return
                if sensor_node.odom_got and ox > STAGE1_GAP_X_SAFE_MAX:
                    stage1_suppress_min_progress = True
                    base_vx = min(STAGE1_GAP_STRAIGHT_VX, 0.16)
                    x_err = STAGE1_GAP_X - ox
                    yaw_err = self._normalize_angle(STAGE1_GAP_ALIGN_YAW - oyaw)
                    base_vy = max(-0.060, min(0.060, -x_err * 0.24))
                    final_vyaw = max(-0.220, min(0.220, yaw_err * 0.18 * STAGE1_GAP_TURN_SPEED_SCALE))
                    self.log_throttle(
                        "stage1_gap_exit_overrun_no_reverse",
                        0.25,
                        f"缺口内侧偏右，禁止倒车回拉，只小横移并继续进缺口 "
                        f"odom=({ox:.2f},{oy:.2f}) x_err={x_err:.2f} "
                        f"vx={base_vx:.3f} vy={base_vy:.3f} yaw={final_vyaw:.2f}"
                    )
                else:
                    base_vx = STAGE1_GAP_STRAIGHT_VX
                    x_err = STAGE1_GAP_X - ox if sensor_node.odom_got else 0.0
                    yaw_err = self._normalize_angle(STAGE1_GAP_ALIGN_YAW - oyaw) if sensor_node.odom_got else 0.0
                    base_vy = max(-0.040, min(0.040, -x_err * 0.22))
                    final_vyaw = max(-0.260, min(0.260, (x_err * 0.32 + yaw_err * 0.18) * STAGE1_GAP_TURN_SPEED_SCALE))
                    self.log_throttle(
                        "stage1_gap_exit_line",
                        0.5,
                        f"缺口内侧短直行，等待越过一二赛段分界线 odom=({ox:.2f},{oy:.2f}) "
                        f"x_err={x_err:.2f} vy={base_vy:.2f} exit_y={STAGE1_GAP_EXIT_Y:.2f}"
                    )
            else:
                final_vyaw = exit_steer * 0.65 * STAGE1_GAP_TURN_SPEED_SCALE + roll_corr * 0.22

            can_exit_stage1 = (
                self.stage1_exit_step >= len(exit_goals) - 1 and
                stage1_at_stage2_turn_point and
                gap_dist < 0.70 and
                STAGE1_REAR_CLEAR_X_MIN <= ox <= STAGE1_REAR_CLEAR_X_MAX and
                ox > STAGE1_MIN_EXIT_X and
                stage_progress > STAGE1_MIN_EXIT_TIME and
                self.stage1_reached_rockroad and
                self.stage1_tail_clear_done
            )
            if can_exit_stage1:
                self.log("已从黄线缺口进入第二赛段，准备直接转向第一个橙球")
                self._advance_stage(2)
                return
            if not self.stage1_reached_rockroad:
                self.log_throttle(
                    "stage1_exit_blocked",
                    1.0,
                    f"未确认经过石板路，禁止切到第二赛段 stone_hits={self.stage1_stone_hits} odom=({ox:.1f},{oy:.1f})"
                )

        else:
            if abs(roll) > 0.20 or angvel_z > 2.4:
                self.log_throttle(
                    "stage1_pre_fall",
                    1.0,
                    f"石板路预跌警告 roll={roll:.2f} angvel_z={angvel_z:.2f}"
                )
                base_vx = STONE_BALANCE_CRAWL_VX
                base_step = STONE_CLIMB_STEP_HEIGHT
                stage1_gait = STAGE1_GAIT_HIGH_STEP
            elif abs(roll) > 0.11:
                base_vx = min(base_vx, STONE_CRAWL_VX)
                base_step = STONE_CLIMB_STEP_HEIGHT

            base_vx = base_vx / pitch_factor

            if abs(roll) > 0.30:
                self.log_throttle("stage1_hard_tilt", 1.0, f"石板路严重倾斜 roll={roll:.2f} 紧急减速!")
                base_vx = STONE_IMPACT_RELIEF_VX
                base_step = STONE_CLIMB_STEP_HEIGHT
                stage1_gait = STAGE1_GAIT_HIGH_STEP

            final_vyaw = roll_corr * 0.40 + goal_term
            if lane_steer != 0.0:
                final_vyaw = lane_steer + roll_corr * 0.30
                base_vx = min(base_vx, STONE_STEPUP_VX if stepup_zone else STONE_APPROACH_VX)
                base_step = max(base_step, STONE_CLIMB_STEP_HEIGHT)
                self.log_throttle(
                    "stage1_force_rockroad_lane",
                    0.8,
                    f"未踩上石板，强制向石板中心线 y={STAGE1_CENTER_Y:.2f} 修正 "
                    f"odom=({ox:.2f},{oy:.2f}) steer={lane_steer:.2f}"
                )
            if stepup_pair_hold:
                final_vyaw = max(-0.055, min(0.055, roll_corr * 0.18))
            elif lt['type'] in ('stone', 'centerline') and lt['conf'] > 0.3:
                target_steer = lt['target_x'] * 0.35 if lt['type'] == 'stone' else -lt['target_x'] * CENTERLINE_KP
                target_steer = max(-0.22, min(0.22, target_steer))
                if lt['type'] == 'stone' and abs(lt['target_x']) > 0.35:
                    stage1_gait = STAGE1_GAIT_HIGH_STEP
                    base_step = max(base_step, STONE_CLIMB_STEP_HEIGHT)
                if lt['type'] == 'stone':
                    if lane_steer != 0.0:
                        final_vyaw = lane_steer * 0.75 + target_steer * 0.25 + roll_corr * 0.25
                    else:
                        final_vyaw = target_steer + roll_corr * 0.32 + goal_term
                else:
                    final_vyaw = target_steer + roll_corr * 0.36 + goal_term
            elif sensor_node.odom_got:
                self.log_throttle(
                    "stage1_line_hold_goal",
                    1.0,
                    f"石板路黄线短暂丢失，按出口/中线牵引行走 steer={goal_term:.2f} odom=({ox:.1f},{oy:.1f})"
                )
                base_vx = min(base_vx, STONE_BALANCE_CRAWL_VX)

            if line_held:
                base_vx = min(base_vx, STONE_STEPUP_VX if stepup_zone else STONE_CRAWL_VX)
                self.log_throttle(
                    "stage1_line_hold",
                    0.8,
                    f"沿用上一可靠黄线/石板目标 type={lt['type']} x={lt['target_x']:.2f} conf={lt['conf']:.2f}"
                )
            if entry_align:
                final_vyaw = entry_align_yaw_cmd + max(-0.018, min(0.018, entry_align_y_err * 0.08))
                base_vx = min(base_vx, STONE_ENTRY_ALIGN_VX)
                base_step = max(base_step, STONE_CLIMB_STEP_HEIGHT)
                stage1_gait = STAGE1_GAIT_HIGH_STEP
            if post_climb_align or hard_lane_keep:
                final_vyaw = post_climb_yaw_cmd + max(-0.025, min(0.025, post_climb_y_err * 0.08))
                base_vx = min(base_vx, STONE_LANE_HARD_VX if hard_lane_keep else STONE_POST_ALIGN_VX)
                base_step = max(base_step, STONE_CLIMB_STEP_HEIGHT)
                stage1_gait = STAGE1_GAIT_HIGH_STEP
            if stone_edge_trap_guard:
                final_vyaw = max(-STONE_EDGE_TRAP_YAW_LIMIT, min(STONE_EDGE_TRAP_YAW_LIMIT, final_vyaw))
                base_vx = min(base_vx, STONE_EDGE_TRAP_VX)
                base_step = max(base_step, STONE_CLIMB_STEP_HEIGHT)
                stage1_gait = STAGE1_GAIT_HIGH_STEP
                stage1_suppress_min_progress = True

        if (not stone_edge_trap_guard and
                not near_end and base_step >= OBSTACLE_STEP_HEIGHT and
                abs(roll) < 0.10 and abs(pitch) < 0.10 and angvel_z < 0.9):
            base_vx = max(base_vx, STAGE1_HIGH_STEP_MIN_VX)

        yaw_cmd_limit = STAGE1_GAP_FAST_YAW_LIMIT if near_end else 0.24
        final_vyaw = max(-yaw_cmd_limit, min(yaw_cmd_limit, final_vyaw))

        lane_center_y = STAGE1_CENTER_Y
        if near_end and self.stage1_exit_step >= 1:
            lane_center_y = None

        self.apply_stabilized_locomotion(base_vx, base_vy, final_vyaw, gait=stage1_gait,
                                         step_h=base_step, sensor=sensor_node,
                                         body_h=STONE_BODY_HEIGHT, stepup_active=stepup_zone,
                                         suppress_min_progress=stage1_suppress_min_progress,
                                         lane_center_y=lane_center_y)

        if not near_end and lt['type'] != 'stone' and lt['cm_offset'] > 8.0:
            if stepup_zone:
                self.log_throttle(
                    "stage1_offset_hold_stepup",
                    1.0,
                    f"石板入口偏移>{lt['cm_offset']:.0f}cm，先保持跨台阶，不执行强回正"
                )
            else:
                self.log_throttle("stage1_offset_recover", 1.0, f"石板路偏移>{lt['cm_offset']:.0f}cm 强制回正")
                self.apply_locomotion(0.025, 0.0, max(-0.18, min(0.18, roll_corr * 0.8)),
                                      gait=STAGE1_GAIT_HIGH_STEP, step_h=STONE_CLIMB_STEP_HEIGHT,
                                      sensor=sensor_node, boundary_check=True)

        self._check_stuck()
        if stage_progress > 75:
            self.log_throttle(
                "stage1_timeout_hold",
                3.0,
                "石板路超过75s仍未到出口条件，继续石板路保守行走，不按时间强制切换"
            )

    def _init_stage2_fixed_plan(self):
        if self.grid_initialized:
            return
        self.stage2_targets = [dict(item) for item in STAGE2_FIXED_TARGETS]
        self.stage2_target_idx = 0
        self.stage2_route_idx = 0
        self.stage2_exit_idx = 0
        self.orange_hit_count = 0
        self.stage2_target_turn_ready_idx = -1
        self.stage2_entry_faced_first_ball = False
        self.stage2_first_ball_last_dist = 99.0
        self.stage2_first_ball_stall_count = 0
        self.stage2_first_ball_force_mode = False
        self.stage2_axis_turn_start = 0.0
        self.stage2_axis_force_drive_until = 0.0
        self.stage2_axis_turning_target_idx = -1
        self.stage2_post_hit_brake_until = 0.0
        self.stage2_simple_last_target_idx = -1
        self.stage2_simple_last_dist = 99.0
        self.stage2_simple_last_progress_time = 0.0
        self.waypoints.clear()
        self.waypoint_idx = 0
        self.grid_cells.clear()
        self.grid_cell_idx = 0
        self.relocating = False
        self.stage_phase = FSM_MOVE_TO_TARGET
        self._stage2_reset_target_visual()
        self.grid_initialized = True
        desc = ", ".join(
            f"{i + 1}:{t['name']}@({t['ball'][0]:.1f},{t['ball'][1]:.2f})"
            for i, t in enumerate(self.stage2_targets)
        )
        self.log(f"第二赛段启用固定橙球坐标计划: {desc}")

    def _stage2_current_target(self):
        if self.stage2_target_idx >= len(self.stage2_targets):
            return None
        return self.stage2_targets[self.stage2_target_idx]

    def _stage2_aim_point(self, target):
        return target.get("strike") or target["ball"]

    def _stage2_nav_point(self, target):
        return self._stage2_aim_point(target), "strike"

    def _stage2_force_fixed_aim(self, target):
        return True

    def _stage2_heading_error_to_point(self, x, y):
        if not self._has_global_pose():
            return 0.0
        ox, oy, oyaw = self._get_odom_pos()
        theta_target = math.atan2(y - oy, x - ox)
        return self._normalize_angle(theta_target - oyaw)

    def _stage2_turn_in_place_if_needed(self, x, y, label, force=False):
        if not self._has_global_pose():
            return False
        yaw_err = self._stage2_heading_error_to_point(x, y)
        threshold = STAGE2_INPLACE_TURN_DONE_YAW if force else STAGE2_INPLACE_TURN_YAW
        if abs(yaw_err) <= threshold:
            return False
        turn_rate = max(
            -STAGE2_INPLACE_TURN_RATE,
            min(STAGE2_INPLACE_TURN_RATE, yaw_err * 1.05)
        )
        self.log_throttle(
            "stage2_inplace_turn",
            0.35,
            f"第二赛段先原地转向{label}，避免带前进速度扫到蓝球 "
            f"yaw_err={yaw_err:.2f} turn={turn_rate:.2f}"
        )
        self.apply_locomotion(0.0, 0.0, turn_rate, gait=GAIT_TROT_SLOW,
                              step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                              boundary_check=False)
        return True

    def _stage2_steer_to_point(self, x, y, orange_center=None, orange_area=0,
                               visual_weight=STAGE2_VISUAL_STEER_WEIGHT):
        if self._has_global_pose():
            ox, oy, oyaw = self._get_odom_pos()
            dx = x - ox
            dy = y - oy
            yaw_err = self._normalize_angle(math.atan2(dy, dx) - oyaw)
            fixed_steer = max(
                -STAGE2_STEER_LIMIT,
                min(STAGE2_STEER_LIMIT, yaw_err * STAGE2_STEER_GAIN)
            )
            dist = math.hypot(dx, dy)
        else:
            fixed_steer, dist = 0.0, 99.0

        steer = fixed_steer
        # 固定坐标负责绕开蓝球；视觉只在接近当前固定球且方向一致时做末端微调。
        visual_ok = False
        if orange_center is not None and orange_area > STAGE2_ORANGE_MIN_AREA and dist < STAGE2_VISUAL_FUSE_DIST:
            visual_x = (orange_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0)
            visual_steer = visual_x * 0.55
            visual_ok = (
                abs(visual_x) <= STAGE2_VISUAL_MAX_X and
                abs(visual_steer - fixed_steer) <= STAGE2_VISUAL_STEER_GATE
            )
        if visual_ok:
            steer = fixed_steer * (1.0 - visual_weight) + visual_steer * visual_weight
        return max(-STAGE2_STEER_LIMIT, min(STAGE2_STEER_LIMIT, steer)), dist

    def _stage2_orange_visible(self, orange_center, orange_area):
        return orange_center is not None and orange_area >= STAGE2_ORANGE_MIN_AREA

    def _stage2_orange_centered(self, orange_center, orange_area):
        if not self._stage2_orange_visible(orange_center, orange_area):
            return False
        visual_x = (orange_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0)
        return abs(visual_x) < STAGE2_ORANGE_CENTER_X

    def _stage2_reset_target_visual(self):
        self.stage2_target_visual_time = 0.0
        self.stage2_target_visual_area = 0.0
        self.stage2_target_visible_once = False

    def _stage2_update_target_visual(self, visible, orange_area):
        if not visible:
            return
        self.stage2_target_visual_time = time.time()
        self.stage2_target_visual_area = max(self.stage2_target_visual_area, orange_area)
        self.stage2_target_visible_once = True

    def _stage2_recent_visual_lock(self, now):
        return (
            self.stage2_target_visible_once and
            now - self.stage2_target_visual_time <= STAGE2_VISUAL_MEMORY_SEC
        )

    def _stage2_blue_clearance(self, x, y):
        if not STAGE2_BLUE_BALLS:
            return 99.0
        return min(math.hypot(x - bx, y - by) for bx, by in STAGE2_BLUE_BALLS)

    def _stage2_blue_avoid_adjust(self, move_vx, move_vy, steer):
        if not self._has_global_pose():
            return move_vx, move_vy, steer, None
        ox, oy, oyaw = self._get_odom_pos()
        nearest = None
        for bx, by in STAGE2_BLUE_BALLS:
            dx = bx - ox
            dy = by - oy
            forward = math.cos(oyaw) * dx + math.sin(oyaw) * dy
            lateral = -math.sin(oyaw) * dx + math.cos(oyaw) * dy
            if forward <= 0.05 or forward > STAGE2_BLUE_AVOID_FORWARD:
                continue
            if abs(lateral) > STAGE2_BLUE_AVOID_LATERAL:
                continue
            score = forward + abs(lateral) * 0.4
            if nearest is None or score < nearest[0]:
                nearest = (score, bx, by, forward, lateral)
        if nearest is None:
            return move_vx, move_vy, steer, None

        _, bx, by, forward, lateral = nearest
        side = -1.0 if lateral >= 0.0 else 1.0
        avoid_vy = side * STAGE2_BLUE_AVOID_VY
        avoid_steer = side * STAGE2_BLUE_AVOID_STEER
        move_vx = min(move_vx, max(STAGE2_COORD_NAV_MIN_SPEED, move_vx * 0.78))
        move_vy = max(
            -STAGE2_SIMPLE_DRIVE_VY,
            min(STAGE2_SIMPLE_DRIVE_VY, move_vy + avoid_vy)
        )
        steer = max(
            -STAGE2_SIMPLE_STEER_LIMIT,
            min(STAGE2_SIMPLE_STEER_LIMIT, steer + avoid_steer)
        )
        return move_vx, move_vy, steer, (bx, by, forward, lateral)

    def _stage2_world_to_body_velocity(self, world_vx, world_vy, oyaw):
        move_vx = math.cos(oyaw) * world_vx + math.sin(oyaw) * world_vy
        move_vy = -math.sin(oyaw) * world_vx + math.cos(oyaw) * world_vy
        return move_vx, move_vy

    def _world_to_body_velocity(self, world_vx, world_vy, oyaw):
        move_vx = math.cos(oyaw) * world_vx + math.sin(oyaw) * world_vy
        move_vy = -math.sin(oyaw) * world_vx + math.cos(oyaw) * world_vy
        return move_vx, move_vy

    def _stage1_apply_gap_hard_wall(self, ox, oy, oyaw, reason):
        world_x_err = STAGE1_GAP_RECOVER_X - ox
        world_y_err = STAGE1_GAP_BELOW_Y - oy
        world_vx = 0.0
        world_vy = max(-STAGE1_GAP_RECOVER_WORLD_VY, min(STAGE1_GAP_RECOVER_WORLD_VY, world_y_err * 0.55))
        move_vx, move_vy = self._world_to_body_velocity(world_vx, world_vy, oyaw)
        move_vx = max(0.0, min(0.06, move_vx))
        move_vy = max(-0.16, min(0.16, move_vy))
        yaw_err = self._normalize_angle(STAGE1_GAP_ALIGN_YAW - oyaw)
        vyaw = max(-0.45, min(0.45, yaw_err * 0.32))
        self.log_throttle(
            "stage1_gap_hard_wall",
            0.18,
            f"第一赛段右黄线硬墙：{reason}，只横向修正，不倒车 "
            f"odom=({ox:.2f},{oy:.2f}) wall_x={STAGE1_GAP_HARD_WALL_X:.2f} "
            f"recover=({STAGE1_GAP_RECOVER_X:.2f},{STAGE1_GAP_BELOW_Y:.2f}) "
            f"world_v=({world_vx:.2f},{world_vy:.2f}) cmd=({move_vx:.2f},{move_vy:.2f},{vyaw:.2f})"
        )
        self.send_cmd(
            MODE_LOCOMOTION,
            STAGE1_GAIT_HIGH_STEP,
            vx=move_vx,
            vy=move_vy,
            vyaw=vyaw,
            step_h=(STONE_CLIMB_STEP_HEIGHT, STONE_CLIMB_STEP_HEIGHT_REAR),
            rpy=[0.0, STONE_CLIMB_PITCH_BIAS, 0.0],
            pos=[0.0, 0.0, STONE_BODY_HEIGHT],
        )

    def _stage2_apply_boundary_guard(self, world_vx, world_vy):
        if not self._has_global_pose():
            return world_vx, world_vy, False
        ox, oy, _ = self._get_odom_pos()
        corrected = False
        if ox < STAGE2_BOUND_X_MIN + STAGE2_BOUND_MARGIN:
            correction = min(
                STAGE2_BOUND_CORRECT_LIMIT,
                (STAGE2_BOUND_X_MIN + STAGE2_BOUND_MARGIN - ox) * STAGE2_BOUND_CORRECT_GAIN
            )
            world_vx = max(world_vx, correction)
            corrected = True
        elif ox > STAGE2_BOUND_X_MAX - STAGE2_BOUND_MARGIN:
            correction = min(
                STAGE2_BOUND_CORRECT_LIMIT,
                (ox - (STAGE2_BOUND_X_MAX - STAGE2_BOUND_MARGIN)) * STAGE2_BOUND_CORRECT_GAIN
            )
            world_vx = min(world_vx, -correction)
            corrected = True
        if oy < STAGE2_BOUND_Y_MIN + STAGE2_BOUND_MARGIN:
            correction = min(
                STAGE2_BOUND_CORRECT_LIMIT,
                (STAGE2_BOUND_Y_MIN + STAGE2_BOUND_MARGIN - oy) * STAGE2_BOUND_CORRECT_GAIN
            )
            world_vy = max(world_vy, correction)
            corrected = True
        elif oy > STAGE2_BOUND_Y_MAX - STAGE2_BOUND_MARGIN:
            correction = min(
                STAGE2_BOUND_CORRECT_LIMIT,
                (oy - (STAGE2_BOUND_Y_MAX - STAGE2_BOUND_MARGIN)) * STAGE2_BOUND_CORRECT_GAIN
            )
            world_vy = min(world_vy, -correction)
            corrected = True
        return world_vx, world_vy, corrected

    def _stage2_ball_position(self, target):
        link_name = target.get("link")
        if not link_name:
            return None
        if time.time() - sensor_node.link_states_time > 0.6:
            return None
        return sensor_node.link_positions.get(link_name)

    def _stage2_ball_shaken(self, target):
        link_name = target.get("link")
        pos = self._stage2_ball_position(target)
        if not link_name or pos is None:
            return False, 0.0
        start = self.stage2_ball_start_pos.get(link_name)
        if start is None:
            self.stage2_ball_start_pos[link_name] = pos
            return False, 0.0
        moved = math.hypot(pos[0] - start[0], pos[1] - start[1])
        return moved >= STAGE2_BALL_SHAKE_DIST, moved

    def _stage2_head_contact_status(self, x, y):
        if not self._has_global_pose():
            return 99.0, 99.0, False
        ox, oy, oyaw = self._get_odom_pos()
        head_x = ox + math.cos(oyaw) * STAGE2_HEAD_OFFSET
        head_y = oy + math.sin(oyaw) * STAGE2_HEAD_OFFSET
        head_dist = math.hypot(x - head_x, y - head_y)
        yaw_err = self._stage2_heading_error_to_point(x, y)
        head_ok = head_dist <= STAGE2_HEAD_HIT_RADIUS and abs(yaw_err) <= STAGE2_HEAD_YAW_TOL
        return head_dist, yaw_err, head_ok

    def _stage2_head_error_to_point(self, x, y):
        if not self._has_global_pose():
            return x, y
        ox, oy, oyaw = self._get_odom_pos()
        head_x = ox + math.cos(oyaw) * STAGE2_HEAD_OFFSET
        head_y = oy + math.sin(oyaw) * STAGE2_HEAD_OFFSET
        return x - head_x, y - head_y

    def _stage2_bump_target(self, target, reason):
        self.log(f"撞击固定橙球 {target['name']}，原因={reason}")
        self.hit_cooldown = time.time()
        hit_count = int(target.get("count", 1))
        self.orange_hit_count = max(self.orange_hit_count, self.orange_hit_count + hit_count)
        self.orange_hit_count = min(self.orange_hit_count, ORANGE_BALL_TOTAL)
        self.log(f"橙色小球进度: {self.orange_hit_count}/{ORANGE_BALL_TOTAL}")
        self.stage_phase = FSM_TASK_TRIGGER
        self.phase_start = time.time()
        self.stage2_hit_start_pos = None
        self.stage2_hit_target_idx = -1
        self.stage2_hit_target_point = None

    def _stage2_target_reached_reason(self, target, aim_x, aim_y, dist,
                                      orange_center=None, orange_area=0):
        head_dist, head_yaw_err, head_ok = self._stage2_head_contact_status(aim_x, aim_y)
        coord_hit_radius = target.get("coord_hit_radius", STAGE2_HEAD_HIT_RADIUS)
        head_ok = head_dist <= coord_hit_radius and abs(head_yaw_err) <= STAGE2_HEAD_YAW_TOL
        ball_pos = self._stage2_ball_position(target)
        shaken, moved = self._stage2_ball_shaken(target)
        if shaken:
            return f"橙球位移确认 moved={moved:.3f}"

        if head_ok and ball_pos is None:
            return (
                f"头部撞击点对准橙球坐标 head_dist={head_dist:.2f} "
                f"<= {coord_hit_radius:.2f} yaw_err={head_yaw_err:.2f}"
            )

        if (
            ball_pos is None and
            head_dist <= STAGE2_VISUAL_HIT_MAX_DIST and
            orange_area >= STAGE2_ORANGE_HIT_AREA and
            self._stage2_orange_centered(orange_center, orange_area)
        ):
            return (
                f"坐标附近RGB确认橙球 area={orange_area:.0f} "
                f"head_dist={head_dist:.2f} <= {STAGE2_VISUAL_HIT_MAX_DIST:.2f}"
            )

        return None

    def _stage2_advance_target(self, target, reason):
        self.orange_hit_count = min(
            ORANGE_BALL_TOTAL,
            self.orange_hit_count + int(target.get("count", 1))
        )
        self.log(
            f"完成固定橙球 {self.stage2_target_idx + 1}/{len(self.stage2_targets)} "
            f"{target['name']}，原因={reason}，切下一个坐标"
        )
        self.stage2_target_idx += 1
        self.stage2_route_idx = 0
        self.stage2_target_turn_ready_idx = -1
        self.stage2_first_ball_last_dist = 99.0
        self.stage2_first_ball_stall_count = 0
        self.stage2_first_ball_force_mode = False
        self.stage2_axis_turn_start = 0.0
        self.stage2_axis_force_drive_until = 0.0
        self.stage2_axis_turning_target_idx = -1
        self.stage2_post_hit_brake_until = time.time() + 0.45
        self.stage2_simple_last_target_idx = -1
        self.stage2_simple_last_dist = 99.0
        self.stage2_simple_last_progress_time = 0.0
        self._stage2_reset_target_visual()
        self.stage_phase = FSM_MOVE_TO_TARGET
        self.phase_start = time.time()
        if self.stage2_target_idx >= len(self.stage2_targets):
            self.stage2_exit_idx = 0
            self.log(f"所有{ORANGE_BALL_TOTAL}个固定橙球坐标已完成，转向第二赛段左上出口")

    def _run_stage2_exit_to_stage3(self):
        ox, oy, _ = self._get_odom_pos()
        if (sensor_node.odom_got and
                STAGE2_TO_STAGE3_GATE_X_MIN <= ox <= STAGE2_TO_STAGE3_GATE_X_MAX and
                oy >= STAGE2_TO_STAGE3_GATE_Y):
            self.log(
                f"已进入第二三赛段左上缺口，直接切第三赛段S弯中心线 "
                f"odom=({ox:.2f},{oy:.2f})"
            )
            self._advance_stage(3)
            return

        exit_idx = max(0, min(self.stage2_exit_idx, len(STAGE2_EXIT_PATH_POINTS) - 1))
        label, goal, radius = STAGE2_EXIT_PATH_POINTS[exit_idx]
        steer, dist = self._steer_to_map_goal(goal, gain=1.10, limit=0.75)
        if dist <= radius:
            self.log(f"第二赛段到达{label}，dist={dist:.2f}")
            self.stage2_exit_idx += 1
            if self.stage2_exit_idx >= len(STAGE2_EXIT_PATH_POINTS):
                self.log("四个橙球完成并到达第二赛段左上出口，进入第三赛段缺口")
                self._advance_stage(3)
            return

        move_vx = min(STAGE2_EXIT_VX, max(0.12, dist * 0.45))
        self.log_throttle(
            "stage2_exit_to_stage3",
            0.35,
            f"第二赛段四球完成后沿左上缺口门点出赛段 {label} "
            f"goal=({goal[0]:.2f},{goal[1]:.2f}) dist={dist:.2f} steer={steer:.2f} vx={move_vx:.2f}"
        )
        self.apply_locomotion(move_vx, 0.0, steer, gait=STAGE_RUN_GAIT,
                              step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                              boundary_check=False)

    # ── 第二赛段：荒野寻珠 ──────────────────────────────

    def _run_stage2(self):
        if self._check_fall():
            return
        self._check_stage_timeout(2)

        ox, oy, oyaw = self._get_odom_pos()

        self._init_stage2_fixed_plan()

        target = self._stage2_current_target()
        if target is None:
            self._run_stage2_exit_to_stage3()
            return

        if not self._has_global_pose():
            self.log_throttle("stage2_no_pose", 2.0, "第二赛段等待全局位姿，暂不追固定坐标")
            self.apply_locomotion(0.06, 0.0, 0.0, gait=GAIT_TROT_SLOW,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node, boundary_check=False)
            return

        now = time.time()
        if now < self.stage2_post_hit_brake_until:
            self.log_throttle(
                "stage2_post_hit_brake",
                0.15,
                "橙球命中后短暂阻尼，压掉惯性后再切下一个目标/左上缺口"
            )
            self.apply_locomotion(0.0, 0.0, 0.0, gait=GAIT_TROT_SLOW,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                                  boundary_check=False)
            return

        if STAGE2_SIMPLE_COORD_MODE:
            aim_x, aim_y = self._stage2_aim_point(target)
            dx = aim_x - ox
            dy = aim_y - oy
            base_dist = math.hypot(dx, dy)
            yaw_err = self._normalize_angle(math.atan2(dy, dx) - oyaw)
            head_x = ox + math.cos(oyaw) * STAGE2_HEAD_OFFSET
            head_y = oy + math.sin(oyaw) * STAGE2_HEAD_OFFSET
            head_dist = math.hypot(aim_x - head_x, aim_y - head_y)
            radius = max(target.get("coord_hit_radius", STAGE2_SIMPLE_REACHED_RADIUS), 0.30)
            target_idx = self.stage2_target_idx
            shaken, moved = self._stage2_ball_shaken(target)
            if head_dist <= radius or shaken:
                reason = (
                    f"橙球位移确认 moved={moved:.3f}"
                    if shaken else
                    f"头部到达橙球坐标 head_dist={head_dist:.2f} <= {radius:.2f}"
                )
                self._stage2_advance_target(target, reason)
                return

            if self.stage2_simple_last_target_idx != target_idx:
                self.stage2_simple_last_target_idx = target_idx
                self.stage2_simple_last_dist = head_dist
                self.stage2_simple_last_progress_time = now
            elif head_dist < self.stage2_simple_last_dist - STAGE2_SIMPLE_PROGRESS_EPS:
                self.stage2_simple_last_dist = head_dist
                self.stage2_simple_last_progress_time = now
            stalled = now - self.stage2_simple_last_progress_time >= STAGE2_SIMPLE_STALL_SEC

            turn_only = abs(yaw_err) > STAGE2_SIMPLE_TURN_START_YAW
            if turn_only:
                steer = max(
                    -STAGE2_SIMPLE_TURN_STEER_LIMIT,
                    min(STAGE2_SIMPLE_TURN_STEER_LIMIT, yaw_err * STAGE2_DIRECT_YAW_GAIN)
                )
                boundary_text = ""
                turn_vy = 0.0
                if ox > STAGE2_BOUND_X_MAX - STAGE2_BOUND_MARGIN:
                    _, turn_vy = self._stage2_world_to_body_velocity(
                        -STAGE2_SIMPLE_DRIVE_VY, 0.0, oyaw
                    )
                    turn_vy = max(-STAGE2_SIMPLE_DRIVE_VY, min(STAGE2_SIMPLE_DRIVE_VY, turn_vy))
                    boundary_text = (
                        f" right_boundary=True bound_x={STAGE2_BOUND_X_MAX:.2f}"
                    )
                self.log_throttle(
                    "stage2_simple_face_target",
                    0.18,
                    f"第二赛段先把头部转向橙球坐标，再前进 "
                    f"target={target_idx + 1}/{len(self.stage2_targets)} "
                    f"goal=({aim_x:.2f},{aim_y:.2f}) odom=({ox:.2f},{oy:.2f}) "
                    f"head_dist={head_dist:.2f} yaw_err={yaw_err:.2f} steer={steer:.2f}"
                    f"{boundary_text}"
                )
                self.apply_locomotion(0.0, turn_vy, steer, gait=STAGE_RUN_GAIT,
                                      step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                                      boundary_check=False)
                return

            speed = min(
                STAGE2_SIMPLE_DRIVE_VX,
                max(STAGE2_SIMPLE_MIN_WORLD_SPEED, base_dist * 0.42)
            )
            if stalled:
                speed = max(speed, STAGE2_SIMPLE_STALL_VX)
            if base_dist > 1e-3:
                world_vx = dx / base_dist * speed
                world_vy = dy / base_dist * speed
            else:
                world_vx = 0.0
                world_vy = 0.0
            world_vx, world_vy, boundary_corrected = self._stage2_apply_boundary_guard(world_vx, world_vy)
            move_vx, move_vy = self._stage2_world_to_body_velocity(world_vx, world_vy, oyaw)
            move_vx = max(0.0, min(STAGE2_COORD_NAV_MAX_VX, move_vx))
            if 0.0 < move_vx < STAGE2_COORD_NAV_MIN_SPEED:
                move_vx = min(STAGE2_COORD_NAV_MAX_VX, STAGE2_COORD_NAV_MIN_SPEED)
            move_vy = max(-STAGE2_COORD_NAV_MAX_VY, min(STAGE2_COORD_NAV_MAX_VY, move_vy))
            steer = max(
                -STAGE2_SIMPLE_STEER_LIMIT,
                min(STAGE2_SIMPLE_STEER_LIMIT, yaw_err * 0.32)
            )
            rgb_text = ""
            if head_dist <= STAGE2_SIMPLE_RGB_ALIGN_DIST:
                bgr = image_msg_to_cv(sensor_node.rgb_image_raw)
                orange_center, orange_area = detect_orange_ball(bgr) if bgr is not None else (None, 0)
                if orange_center is not None and orange_area >= STAGE2_ORANGE_MIN_AREA:
                    visual_x = (orange_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0)
                    visual_steer = max(
                        -STAGE2_SIMPLE_STEER_LIMIT,
                        min(STAGE2_SIMPLE_STEER_LIMIT, visual_x * STAGE2_SIMPLE_RGB_STEER_GAIN)
                    )
                    if abs(visual_x) <= STAGE2_VISUAL_MAX_X:
                        steer = (
                            steer * (1.0 - STAGE2_SIMPLE_RGB_WEIGHT) +
                            visual_steer * STAGE2_SIMPLE_RGB_WEIGHT
                        )
                        steer = max(-STAGE2_SIMPLE_STEER_LIMIT, min(STAGE2_SIMPLE_STEER_LIMIT, steer))
                        rgb_text = (
                            f" rgb_align=True rgb_x={visual_x:.2f} "
                            f"rgb_area={orange_area:.0f}"
                        )
                    else:
                        rgb_text = (
                            f" rgb_align=False rgb_x={visual_x:.2f} "
                            f"rgb_area={orange_area:.0f}"
                        )
            blue_avoid = None
            avoid_text = ""
            if blue_avoid is not None:
                bx, by, forward, lateral = blue_avoid
                avoid_text = (
                    f" blue_avoid=({bx:.2f},{by:.2f}) "
                    f"front={forward:.2f} lateral={lateral:.2f}"
                )
            self.log_throttle(
                "stage2_simple_coord",
                0.20,
                f"第二赛段直线坐标追球 target={target_idx + 1}/{len(self.stage2_targets)} "
                f"goal=({aim_x:.2f},{aim_y:.2f}) odom=({ox:.2f},{oy:.2f}) "
                f"head_dist={head_dist:.2f} base_dist={base_dist:.2f} yaw_err={yaw_err:.2f} "
                f"stalled={stalled} boundary={boundary_corrected} "
                f"world_v=({world_vx:.2f},{world_vy:.2f}) vx={move_vx:.2f} vy={move_vy:.2f} steer={steer:.2f}"
                f"{rgb_text}{avoid_text}"
            )
            self.apply_locomotion(move_vx, move_vy, steer, gait=STAGE_RUN_GAIT,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                                  boundary_check=False)
            return

        bgr = image_msg_to_cv(sensor_node.rgb_image_raw)
        orange_center, orange_area = detect_orange_ball(bgr) if bgr is not None else (None, 0)
        orange_visible = self._stage2_orange_visible(orange_center, orange_area)
        self._stage2_update_target_visual(orange_visible, orange_area)

        aim_x, aim_y = self._stage2_aim_point(target)
        nav_point, nav_label = self._stage2_nav_point(target)
        nav_x, nav_y = nav_point
        if self.stage2_target_idx == 0:
            self.log_throttle(
                "stage2_first_orange_coord",
                0.8,
                f"第二赛段首目标锁定第一个橙球坐标 ({aim_x:.2f},{aim_y:.2f})，"
                f"当前 odom=({ox:.2f},{oy:.2f})"
            )
        _, dist = self._stage2_steer_to_point(aim_x, aim_y)
        reached_reason = self._stage2_target_reached_reason(
            target, aim_x, aim_y, dist, orange_center, orange_area
        )
        if reached_reason:
            self._stage2_advance_target(target, reached_reason)
            return

        target_idx = self.stage2_target_idx
        if nav_label == "strike":
            dx, dy = self._stage2_head_error_to_point(nav_x, nav_y)
        else:
            dx = nav_x - ox
            dy = nav_y - oy
        yaw_err = self._stage2_heading_error_to_point(nav_x, nav_y)
        world_vx = dx * STAGE2_COORD_NAV_GAIN
        world_vy = dy * STAGE2_COORD_NAV_GAIN
        move_vx_goal = math.cos(oyaw) * world_vx + math.sin(oyaw) * world_vy
        move_vy_goal = -math.sin(oyaw) * world_vx + math.cos(oyaw) * world_vy
        move_vx_goal = max(0.0, min(STAGE2_COORD_NAV_MAX_VX, move_vx_goal))
        move_vy_goal = max(-STAGE2_COORD_NAV_MAX_VY, min(STAGE2_COORD_NAV_MAX_VY, move_vy_goal))
        if 0.0 <= move_vx_goal < STAGE2_COORD_NAV_MIN_SPEED:
            move_vx_goal = min(STAGE2_COORD_NAV_MAX_VX, STAGE2_COORD_NAV_MIN_SPEED)
        visual_lock = (
            nav_label == "strike" and
            orange_visible and
            dist <= STAGE2_VISUAL_FUSE_DIST and
            abs(yaw_err) <= STAGE2_COORD_VECTOR_YAW_LIMIT
        )
        visual_x = 0.0
        visual_steer = 0.0
        near_strike = nav_label == "strike" and dist <= STAGE2_FINAL_APPROACH_DIST
        if visual_lock:
            visual_x = (orange_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0)
            visual_steer = max(
                -STAGE2_AXIS_STEER_LIMIT,
                min(STAGE2_AXIS_STEER_LIMIT, visual_x * 0.55)
            )
        force_drive = now < self.stage2_axis_force_drive_until
        turn_elapsed = 0.0
        if (
            abs(yaw_err) > STAGE2_AXIS_TURN_YAW and
            not force_drive and
            not near_strike
        ):
            if self.stage2_axis_turning_target_idx != target_idx:
                self.stage2_axis_turning_target_idx = target_idx
                self.stage2_axis_turn_start = now
            elif self.stage2_axis_turn_start <= 0.0:
                self.stage2_axis_turn_start = now
            turn_elapsed = now - self.stage2_axis_turn_start
            if turn_elapsed >= STAGE2_AXIS_TURN_MAX_SEC:
                self.stage2_axis_force_drive_until = now + STAGE2_AXIS_DRIVE_PULSE_SEC
                force_drive = True
        else:
            self.stage2_axis_turn_start = 0.0
            self.stage2_axis_turning_target_idx = -1

        if (
            abs(yaw_err) > STAGE2_AXIS_TURN_YAW and
            not force_drive and
            not near_strike
        ):
            move_vx = 0.0
            move_vy = 0.0
            steer = max(
                -STAGE2_AXIS_STEER_LIMIT,
                min(STAGE2_AXIS_STEER_LIMIT, yaw_err * STAGE2_DIRECT_YAW_GAIN)
            )
            motion_mode = "aim_strike" if near_strike else "turn"
        else:
            final_approach = (
                nav_label == "strike" and
                dist <= STAGE2_FINAL_APPROACH_DIST and
                abs(yaw_err) <= STAGE2_FINAL_APPROACH_YAW
            )
            if final_approach:
                target_hit_vx = target.get("hit_vx", STAGE2_FINAL_APPROACH_VX)
                move_vx = min(STAGE2_FINAL_APPROACH_VX, target_hit_vx, max(0.045, dist * 0.18))
                move_vy = 0.0
                steer = max(
                    -STAGE2_FINAL_APPROACH_STEER_LIMIT,
                    min(STAGE2_FINAL_APPROACH_STEER_LIMIT, yaw_err * 0.55)
                )
                motion_mode = "final_strike"
                self.stage2_axis_turn_start = 0.0
                self.stage2_axis_turning_target_idx = -1
            elif nav_label == "strike" and abs(yaw_err) > STAGE2_HEAD_ARC_YAW:
                move_vx = max(
                    STAGE2_COORD_NAV_MIN_SPEED,
                    min(STAGE2_COORD_NAV_MAX_VX, abs(move_vx_goal))
                )
                move_vy = max(
                    -STAGE2_COORD_NAV_MAX_VY * STAGE2_HEAD_ARC_VY_SCALE,
                    min(STAGE2_COORD_NAV_MAX_VY * STAGE2_HEAD_ARC_VY_SCALE, move_vy_goal)
                )
                steer = max(
                    -STAGE2_HEAD_ARC_STEER_LIMIT,
                    min(STAGE2_HEAD_ARC_STEER_LIMIT, yaw_err * STAGE2_DIRECT_YAW_GAIN)
                )
                motion_mode = "head_arc"
                self.stage2_axis_turn_start = 0.0
                self.stage2_axis_turning_target_idx = -1
            elif force_drive:
                move_vx = move_vx_goal
                move_vy = move_vy_goal
                motion_mode = "coord_recover"
            elif visual_lock and abs(visual_x) > STAGE2_ORANGE_CENTER_X:
                move_vx = move_vx_goal
                move_vy = move_vy_goal
                steer = max(
                    -STAGE2_AXIS_DRIVE_STEER_LIMIT,
                    min(STAGE2_AXIS_DRIVE_STEER_LIMIT, visual_steer)
                )
                motion_mode = "coord_visual"
                self.stage2_axis_turn_start = 0.0
                self.stage2_axis_turning_target_idx = -1
            else:
                move_vx = move_vx_goal
                move_vy = move_vy_goal
                motion_mode = "coord_vector"
            if motion_mode in ("coord_vector", "coord_recover"):
                steer = 0.0
        self.log_throttle(
            "stage2_direct_coord",
            0.25,
            f"第二赛段纯坐标导航 target={target_idx + 1}/{len(self.stage2_targets)} "
            f"{target['name']} aim=({aim_x:.2f},{aim_y:.2f}) "
            f"odom=({ox:.2f},{oy:.2f}) d=({dx:.2f},{dy:.2f}) yaw={oyaw:.2f} "
            f"dist={dist:.2f} yaw_err={yaw_err:.2f} mode={motion_mode} "
            f"force_drive={force_drive} turn_elapsed={turn_elapsed:.2f} "
            f"rgb_visible={orange_visible} rgb_area={orange_area:.0f} rgb_x={visual_x:.2f} "
            f"goal_v=({move_vx_goal:.2f},{move_vy_goal:.2f}) "
            f"vx={move_vx:.2f} vy={move_vy:.2f} steer={steer:.2f}"
        )
        self.apply_locomotion(move_vx, move_vy, steer, gait=STAGE_RUN_GAIT,
                              step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node, boundary_check=False)
        return

    # ── 第三赛段：曲道冲锋 ──────────────────────────────

    def _run_stage3(self):
        if self._check_fall():
            return
        self._check_stage_timeout(3)

        bgr = image_msg_to_cv(sensor_node.rgb_image_raw)
        lt = compute_local_target(bgr, stage=3)

        self.local_target_point = lt

        ox, oy, oyaw = self._get_odom_pos()

        if self.stage_phase == 0:
            self.stage3_path_idx = 0
            self.stage_phase = 1
            self.phase_start = time.time()
            self.log("第三赛段启用密集S弯中线门点")
            return

        if self.stage_phase == 1:
            idx = max(0, min(self.stage3_path_idx, len(STAGE3_PATH_POINTS) - 1))
            label, goal, radius = STAGE3_PATH_POINTS[idx]
            prev_goal = STAGE2_EXIT_POINT if idx == 0 else STAGE3_PATH_POINTS[idx - 1][1]
            entry_steer, entry_dist = self._steer_to_map_goal(goal, gain=1.15, limit=0.48)
            while idx < len(STAGE3_PATH_POINTS) - 1 and sensor_node.odom_got and oy > goal[1] + 0.18:
                self.stage3_path_idx += 1
                idx = self.stage3_path_idx
                label, goal, radius = STAGE3_PATH_POINTS[idx]
                entry_steer, entry_dist = self._steer_to_map_goal(goal, gain=1.15, limit=0.48)
                self.log(f"第三赛段已越过旧门点，推进到 {idx + 1}/{len(STAGE3_PATH_POINTS)} {label}")
            if lt['type'] in ('curve', 'centerline') and lt['conf'] > 0.35:
                line_steer = lt['target_x'] * 0.45 if lt['type'] == 'curve' else -lt['target_x'] * CENTERLINE_KP
                line_steer = max(-0.30, min(0.30, line_steer))
                entry_steer = max(-0.50, min(0.50, entry_steer * 0.45 + line_steer * 0.55))

            self.log_throttle(
                "stage3_path_gate",
                1.0,
                f"第三赛段门点{idx + 1}/{len(STAGE3_PATH_POINTS)} {label} goal=({goal[0]:.2f},{goal[1]:.2f}) "
                f"dist={entry_dist:.2f} odom=({ox:.2f},{oy:.2f})"
            )
            vx = 0.420
            if lt['type'] == 'curve' and abs(lt['target_x']) > 0.32:
                vx = 0.320
            if abs(lt.get('cm_offset', 0.0)) > 7.0:
                vx = 0.260
                entry_steer = max(-0.55, min(0.55, -lt['target_x'] * CENTERLINE_KP * 1.6))
                self.log_throttle(
                    "stage3_center_recover",
                    0.35,
                    f"第三赛段偏离黄线中心 {lt['cm_offset']:.1f}cm，减速回中线"
                )
            self.apply_locomotion(vx, 0.0, entry_steer, gait=STAGE_RUN_GAIT,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                                  boundary_check=True, target_point=lt)
            self._check_stuck()
            if sensor_node.odom_got and entry_dist < radius:
                if idx < len(STAGE3_PATH_POINTS) - 1:
                    self.stage3_path_idx += 1
                    self.log(f"第三赛段到达{label}，继续门点 {self.stage3_path_idx + 1}/{len(STAGE3_PATH_POINTS)}")
                else:
                    self.log("已按S弯中线抵达第三四赛段衔接处")
                    self._advance_stage(4)
                self.phase_start = time.time()
            elif self._time_in_phase() > 60.0:
                self.log_throttle(
                    "stage3_path_slow",
                    3.0,
                    f"第三赛段仍在寻找{label}，不越黄线兜底切段 dist={entry_dist:.2f}"
                )
            return

        map_steer, map_dist = self._steer_to_map_goal(STAGE3_EXIT_POINT, gain=1.05, limit=0.44)

        base_vx = 0.720
        base_step = HIGH_TRAVEL_STEP_HEIGHT

        if lt['type'] == 'curve' and lt['conf'] > 0.5:
            steer = lt['target_x'] * 0.70
            steer = max(-0.52, min(0.52, steer))
            if sensor_node.odom_got:
                steer = max(-0.54, min(0.54, steer * 0.70 + map_steer * 0.30))
            if abs(lt['target_x']) > 0.35:
                base_vx = 0.560
                base_step = MIN_TRAVEL_STEP_HEIGHT
                self.log(f"S弯大曲率 target_x={lt['target_x']:.2f}, 减速到{base_vx}")

            self.apply_locomotion(base_vx, 0.0, steer, gait=STAGE_RUN_GAIT,
                                  step_h=base_step, sensor=sensor_node,
                                  boundary_check=True, target_point=lt)

        elif lt['type'] == 'centerline' and lt['conf'] > 0.3:
            pid_steer = -lt['target_x'] * CENTERLINE_KP
            pid_steer = max(-0.42, min(0.42, pid_steer))
            if sensor_node.odom_got:
                pid_steer = max(-0.48, min(0.48, pid_steer * 0.65 + map_steer * 0.35))
            self.apply_locomotion(base_vx * 0.92, 0.0, pid_steer, gait=STAGE_RUN_GAIT,
                                  step_h=base_step, sensor=sensor_node,
                                  boundary_check=True, target_point=lt)
        else:
            self.apply_locomotion(base_vx * 0.92, 0.0, map_steer, gait=STAGE_RUN_GAIT,
                                  step_h=base_step, sensor=sensor_node,
                                  boundary_check=True, target_point=lt)

        if lt['cm_offset'] > 8.0:
            self.log(f"S弯偏移>{lt['cm_offset']:.0f}cm 减速校正")
            self.apply_locomotion(0.34, 0.0, -lt['target_x'] * CENTERLINE_KP * 1.7,
                                  gait=STAGE_RUN_GAIT, step_h=MIN_TRAVEL_STEP_HEIGHT,
                                  sensor=sensor_node, boundary_check=True)

        self._check_stuck()
        if sensor_node.odom_got and map_dist < 0.55:
            self.log(f"已通过曲道区域，抵达第三赛段出口 dist={map_dist:.2f}")
            self._advance_stage(4)
        elif self._time_in_stage() > 45:
            self.log_throttle(
                "stage3_no_timeout_skip",
                3.0,
                f"第三赛段行走较久，继续沿S弯出口执行，不按时间跳到第四赛段 dist={map_dist:.2f}"
            )

    # ── 第四赛段：深隧寻珍 ──────────────────────────────

    def _run_stage4(self):
        if self._check_fall():
            return
        self._check_stage_timeout(4)

        bgr = image_msg_to_cv(sensor_node.rgb_image_raw)
        red_center, red_area = detect_red_object(bgr) if bgr is not None else (None, 0)
        coke_center, coke_area = detect_coke_bottle(bgr) if bgr is not None else (None, 0)
        orange_center, orange_area = detect_orange_ball(bgr) if bgr is not None else (None, 0)
        football_center, football_area = detect_football(bgr) if bgr is not None else (None, 0)

        if self.stage_phase == 0:
            idx = max(0, min(self.stage4_entry_idx, len(STAGE4_ENTRY_PATH_POINTS) - 1))
            label, goal, radius = STAGE4_ENTRY_PATH_POINTS[idx]
            steer, dist = self._steer_to_map_goal(goal, gain=1.08, limit=0.46)
            self.log_throttle(
                "stage4_entry_path",
                0.8,
                f"第四赛段入口折线 {idx + 1}/{len(STAGE4_ENTRY_PATH_POINTS)} {label} "
                f"goal=({goal[0]:.2f},{goal[1]:.2f}) dist={dist:.2f}"
            )
            self.apply_locomotion(0.68, 0.0, steer, gait=STAGE_RUN_GAIT,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                                  boundary_check=True)
            self._check_stuck()
            if sensor_node.odom_got and dist < radius:
                if idx < len(STAGE4_ENTRY_PATH_POINTS) - 1:
                    self.stage4_entry_idx += 1
                    self.phase_start = time.time()
                    self.log(f"第四赛段到达入口门点 {label}")
                else:
                    self.log("已按S弯出口进入第四赛段入口，开始低身通过限高杆")
                    self.stage_phase = 1
                    self.phase_start = time.time()
            elif self._time_in_phase() > 35.0:
                self.log_throttle(
                    "stage4_entry_slow",
                    3.0,
                    f"第四赛段入口折线较慢，继续找入口门点 dist={dist:.2f}"
                )
            return

        if self.stage_phase == 1:
            self.log_throttle("stage4_lower_height", 1.0, "阶段1: 进入竖向通道，平滑降低身体")
            self.send_cmd(MODE_POS_INTERP, 0, duration=1200,
                          pos=[0.0, 0.0, LOW_BODY_HEIGHT], rpy=[0.0, 0.0, 0.0])
            if self._time_in_phase() > 0.7:
                self.stage_phase = 2
                self.phase_start = time.time()
            return

        if self.stage_phase == 2:
            self.speak_once("stage4_low_bar", "限高杆")
            self.log_throttle("stage4_low_bar", 1.0, "阶段2: 限高杆-低姿态通过")
            steer, dist = self._steer_to_map_goal(STAGE4_LOW_BAR_1, gain=0.86, limit=0.34)
            if dist < 0.65:
                steer, dist = self._steer_to_map_goal(STAGE4_LOW_BAR_2, gain=0.86, limit=0.34)
            roll = sensor_node.imu_roll
            pitch = sensor_node.imu_pitch
            nav_vx = 0.500
            if abs(roll) > 0.12 or abs(pitch) > 0.14:
                nav_vx = 0.360
                steer = max(-0.22, min(0.22, steer))
            self.apply_locomotion(nav_vx, 0.0, steer, gait=STAGE_RUN_GAIT,
                                  step_h=0.05, sensor=sensor_node,
                                  body_h=LOW_BODY_HEIGHT)
            if sensor_node.odom_got and dist < 0.65:
                self.stage_phase = 3
                self.phase_start = time.time()
            elif self._time_in_phase() > 35.0:
                self.log_throttle(
                    "stage4_low_bar_slow",
                    3.0,
                    f"限高杆通过较慢，继续低速通过，不按时间跳过 dist={dist:.2f}"
                )
            return

        if self.stage_phase == 3:
            self.log_throttle("stage4_recover_height", 1.0, "阶段3: 分段恢复正常高度")
            recover_h = 0.20 if self._time_in_phase() < 0.8 else MOBILE_BODY_HEIGHT
            self.send_cmd(MODE_POS_INTERP, 0, duration=1000,
                          pos=[0.0, 0.0, recover_h], rpy=[0.0, 0.0, 0.0])
            if self._time_in_phase() > 0.9:
                self.stage_phase = 4
                self.phase_start = time.time()
            return

        if self.stage_phase == 4:
            target_center, target_area = (coke_center, coke_area) if coke_area >= red_area else (red_center, red_area)
            self.log_throttle("stage4_coke", 1.0, f"阶段4: 搜索可乐瓶 area={target_area:.0f}")
            if target_center is not None and target_area > 80:
                self.speak_once("stage4_coke", "可乐瓶")
            steer, dist = self._steer_to_map_goal(STAGE4_COKE_POINT, gain=1.08, limit=0.46)
            if target_center and target_area > 80:
                visual_steer = (target_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0) * 0.4
                steer = max(-0.52, min(0.52, steer * 0.55 + visual_steer * 0.45))
            self.apply_locomotion(0.68, 0.0, steer, gait=STAGE_RUN_GAIT,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node)
            self._check_stuck()
            if (target_area > 220 and sensor_node.tof_range < 0.30) or (sensor_node.odom_got and dist < 0.25):
                self.log(f"TOF确认可乐瓶碰撞! tof={sensor_node.tof_range:.2f}m")
                self.send_cmd(MODE_LOCOMOTION, STAGE_RUN_GAIT,
                              vx=1.20, vy=0.0, vyaw=0.0, step_h=HIGH_TRAVEL_STEP_HEIGHT, duration=420)
                self.stage_phase = 5
                self.phase_start = time.time()
                return
            if self._time_in_phase() > 10.0:
                self.log("未找到可乐瓶，继续下一目标")
                self.stage_phase = 5
                self.phase_start = time.time()
            return

        if self.stage_phase == 5:
            self.log_throttle("stage4_orange", 1.0, f"阶段5: 搜索橙色小球 area={orange_area:.0f}")
            if orange_center is not None and orange_area > 80:
                self.speak_once("stage4_orange_ball", "橙色球")
            steer, dist = self._steer_to_map_goal(STAGE4_ORANGE_BALL_POINT, gain=1.08, limit=0.46)
            if orange_center and orange_area > 80:
                visual_steer = (orange_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0) * 0.4
                steer = max(-0.52, min(0.52, steer * 0.55 + visual_steer * 0.45))
            self.apply_locomotion(0.68, 0.0, steer, gait=STAGE_RUN_GAIT,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node)
            self._check_stuck()
            if (orange_area > 300 and sensor_node.tof_range < 0.28) or (sensor_node.odom_got and dist < 0.25):
                self.log(f"TOF确认橙色小球碰撞! tof={sensor_node.tof_range:.2f}m")
                self.send_cmd(MODE_LOCOMOTION, STAGE_RUN_GAIT,
                              vx=1.20, vy=0.0, vyaw=0.0, step_h=HIGH_TRAVEL_STEP_HEIGHT, duration=420)
                self.stage_phase = 6
                self.phase_start = time.time()
                return
            if self._time_in_phase() > 8.0:
                self.stage_phase = 6
                self.phase_start = time.time()
            return

        if self.stage_phase == 6:
            self.log_throttle("stage4_football", 1.0, f"阶段6: 搜索足球 area={football_area:.0f}")
            if football_center is not None and football_area > 80:
                self.speak_once("stage4_football", "足球")
            steer, dist = self._steer_to_map_goal(STAGE4_FOOTBALL_POINT, gain=1.08, limit=0.46)
            if football_center and football_area > 60:
                visual_steer = (football_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0) * 0.4
                steer = max(-0.52, min(0.52, steer * 0.55 + visual_steer * 0.45))
            self.apply_locomotion(0.70, 0.0, steer, gait=STAGE_RUN_GAIT,
                                  step_h=HIGH_TRAVEL_STEP_HEIGHT, sensor=sensor_node)
            self._check_stuck()
            if (football_area > 180 and sensor_node.tof_range < 0.30) or (sensor_node.odom_got and dist < 0.25):
                self.log(f"TOF确认足球碰撞! tof={sensor_node.tof_range:.2f}m")
                self.send_cmd(MODE_LOCOMOTION, STAGE_RUN_GAIT,
                              vx=1.20, vy=0.0, vyaw=0.0, step_h=0.10, duration=420)
                self.stage_phase = 7
                self.phase_start = time.time()
                return
            if self._time_in_phase() > 8.0:
                self.stage_phase = 7
                self.phase_start = time.time()
            return

        if self.stage_phase == 7:
            exit_idx = max(0, min(self.stage4_exit_idx, len(STAGE4_EXIT_PATH_POINTS) - 1))
            exit_label, exit_goal, exit_radius = STAGE4_EXIT_PATH_POINTS[exit_idx]
            self.log_throttle(
                "stage4_exit",
                1.0,
                f"阶段7: 沿第四赛段右下角折线回独木桥前 {exit_idx + 1}/{len(STAGE4_EXIT_PATH_POINTS)} "
                f"{exit_label}"
            )
            if tof_blocked(sensor_node):
                self.speak_once("stage4_unpassable_obstacle", "无法跨越障碍")
            steer, dist = self._steer_to_map_goal(exit_goal, gain=1.08, limit=0.50)
            self.apply_stair_safe_locomotion(0.70, 0.0, steer, gait=STAGE_RUN_GAIT,
                                             sensor=sensor_node)
            self._check_stuck()
            if sensor_node.odom_got and dist < exit_radius:
                if exit_idx < len(STAGE4_EXIT_PATH_POINTS) - 1:
                    self.stage4_exit_idx += 1
                    self.phase_start = time.time()
                    self.log(f"第四赛段到达出口门点 {exit_label}")
                else:
                    self.log(f"已到达独木桥桥前起跳位置 dist={dist:.2f}")
                    self._advance_stage(5)
            elif self._time_in_phase() > 50.0:
                self.log_throttle(
                    "stage4_exit_slow",
                    3.0,
                    f"第四赛段出口导航较慢，继续寻找桥前起跳点 dist={dist:.2f}"
                )

    # ── 第五赛段：孤梁稳渡 ──────────────────────────────

    def _run_stage5(self):
        if self._check_fall():
            return
        self._check_stage_timeout(5)

        if self.stage_phase == 0:
            self.log("阶段0: 接近独木桥，降低身体")
            self.send_cmd(MODE_POS_INTERP, 0, duration=900,
                          pos=[0.0, 0.0, LOW_BODY_HEIGHT], rpy=[0.0, 0.0, 0.0])
            if self._time_in_phase() > 1.1:
                self.stage_phase = 1
                self.phase_start = time.time()
            return

        if self.stage_phase == 1:
            self.log("阶段1: 上桥 - 视觉识别桥边缘 + 微速前进")
            steer, dist = self._steer_to_map_goal(STAGE5_BRIDGE_PREJUMP, gain=1.05, limit=0.34)
            entry_steer, entry_dist = self._steer_to_map_goal(STAGE5_BRIDGE_ENTRY, gain=1.05, limit=0.34)
            if not self.stage5_entry_jump_done and dist < 0.55:
                self.log(f"独木桥入口前执行 Jump3D 前向小跳上桥 dist={dist:.2f}")
                self.send_cmd(MODE_JUMP3D, JUMP_POS_X30, duration=900)
                self.stage5_entry_jump_done = True
                self.stage5_entry_jump_time = time.time()
                self.stage5_entry_jump_resume_sent = False
                self.stage5_entry_jump_handoff_until = 0.0
                self.stage5_entry_jump_force_reset_sent = False
                self.stage5_entry_jump_force_reset_time = 0.0
                return
            if self.stage5_entry_jump_done and not self.stage5_entry_jump_resume_sent:
                now = time.time()
                elapsed = now - self.stage5_entry_jump_time
                if elapsed < JUMP_RECOVERY_MIN_SEC:
                    self.log_throttle(
                        "stage5_entry_jump_wait",
                        0.35,
                        f"独木桥入口小跳执行中 elapsed={elapsed:.2f}"
                    )
                    return
                ready, resp_mode, resp_bar, _ = self._jump_recovery_ready(
                    self.stage5_entry_jump_time,
                    self.stage5_entry_jump_time + JUMP_RECOVERY_FALLBACK_SEC,
                    "stage5_entry_jump",
                )
                if elapsed > JUMP_RECOVERY_FORCE_RESET_SEC and not self.stage5_entry_jump_force_reset_sent:
                    self.stage5_entry_jump_force_reset_sent = True
                    self.stage5_entry_jump_force_reset_time = now
                    self.log(
                        f"独木桥入口小跳后未恢复，PureDamper 强制退出 Jump3D "
                        f"resp_mode={resp_mode} bar={resp_bar}"
                    )
                if (self.stage5_entry_jump_force_reset_sent and
                        now - self.stage5_entry_jump_force_reset_time < 0.25):
                    self.send_cmd(MODE_PURE_DAMPER, 0, step_h=0.0)
                    return
                if not ready:
                    self.send_cmd(MODE_RECOVERY_STAND, 0, step_h=0.0)
                    self.log_throttle(
                        "stage5_entry_jump_recover",
                        0.35,
                        f"独木桥入口小跳后恢复站立 elapsed={elapsed:.2f} "
                        f"resp_mode={resp_mode} bar={resp_bar}"
                    )
                    return
                self.stage5_entry_jump_resume_sent = True
                self.stage5_entry_jump_handoff_until = now + JUMP_HANDOFF_SEC
                self.log("独木桥入口小跳恢复完成，低身继续上桥")
            if self.stage5_entry_jump_done and time.time() < self.stage5_entry_jump_handoff_until:
                self.log_throttle(
                    "stage5_entry_jump_handoff",
                    0.25,
                    "独木桥入口小跳后零速Locomotion接管，防止卡在Jump3D"
                )
                self._send_locomotion_handoff(
                    gait=GAIT_TROT_SLOW,
                    body_h=LOW_BODY_HEIGHT,
                    step_h=STAIR_SAFE_STEP_HEIGHT,
                    pitch=STAIR_SAFE_PITCH_BIAS,
                )
                return
            post_jump_steer = entry_steer if self.stage5_entry_jump_done else steer
            self.apply_stair_safe_locomotion(0.32, 0.0, post_jump_steer, gait=GAIT_TROT_SLOW,
                                             sensor=sensor_node)
            if (sensor_node.odom_got and entry_dist < 0.35) or self._time_in_phase() > 10.0:
                self.stage_phase = 2
                self.phase_start = time.time()
            return

        if self.stage_phase == 2:
            roll_correction = -sensor_node.imu_roll * 2.5
            roll_correction = max(-0.5, min(0.5, roll_correction))
            pitch = sensor_node.imu_pitch
            pitch_adj = -pitch * 0.01
            bridge_steer = 0.0
            if sensor_node.odom_got:
                ox, oy, _ = self._get_odom_pos()
                bridge_steer = max(-0.26, min(0.26, -(ox - STAGE5_BRIDGE_ENTRY[0]) * 0.45))
            self.apply_stair_safe_locomotion(0.220, pitch_adj, roll_correction + bridge_steer,
                                             gait=GAIT_TROT_SLOW, sensor=sensor_node)
            self._check_stuck()
            if abs(sensor_node.imu_roll) > 0.30:
                self.log(f"独木桥横滚过大 roll={sensor_node.imu_roll:.2f}, 暂停微调")
                self.apply_stair_safe_locomotion(0.12, 0.0, roll_correction,
                                                 gait=GAIT_TROT_SLOW, sensor=sensor_node)
            if sensor_node.odom_got and sensor_node.odom_y > STAGE5_BRIDGE_EXIT_Y:
                self.log("已到达桥末端，准备跳下")
                self.stage_phase = 3
                self.phase_start = time.time()
                return
            if self._time_in_phase() > 75:
                self.log_throttle(
                    "stage5_bridge_slow",
                    3.0,
                    "独木桥通过较慢，继续按桥面里程计走到跳下点"
                )
            return

        if self.stage_phase == 3:
            self.log("执行跳下动作 (Jump3D / JumpDownStair)")
            self.send_cmd(MODE_JUMP3D, JUMP_DOWN_STAIR, duration=1200)
            self.stage5_jump_time = time.time()
            self.stage5_jump_resume_sent = False
            self.stage5_jump_handoff_until = 0.0
            self.stage5_jump_force_reset_sent = False
            self.stage5_jump_force_reset_time = 0.0
            self.stage_phase = 4
            self.phase_start = time.time()
            return

        if self.stage_phase == 4:
            now = time.time()
            with self.response_lock:
                resp_mode = self.response_mode
                resp_bar = self.response_bar
                resp_time = self.response_time
            if not self.stage5_jump_resume_sent:
                ready, resp_mode, resp_bar, resp_time = self._jump_recovery_ready(
                    self.stage5_jump_time,
                    self.stage5_jump_time + JUMP_RECOVERY_FALLBACK_SEC,
                    "stage5_jump_down",
                )
                if not ready:
                    if (now - self.stage5_jump_time > JUMP_RECOVERY_FORCE_RESET_SEC and
                            not self.stage5_jump_force_reset_sent):
                        self.stage5_jump_force_reset_sent = True
                        self.stage5_jump_force_reset_time = now
                        self.log(
                            f"独木桥跳下后未恢复，PureDamper 强制退出 Jump3D "
                            f"resp_mode={resp_mode} bar={resp_bar}"
                        )
                    if (self.stage5_jump_force_reset_sent and
                            now - self.stage5_jump_force_reset_time < 0.25):
                        self.send_cmd(MODE_PURE_DAMPER, 0, step_h=0.0)
                        return
                    self.log_throttle(
                        "stage5_jump_wait",
                        0.35,
                        f"独木桥跳下后等待恢复站立 resp_mode={resp_mode} bar={resp_bar}"
                    )
                    self.send_cmd(MODE_RECOVERY_STAND, 0, step_h=0.0)
                    return
                self.stage5_jump_resume_sent = True
                self.stage5_jump_handoff_until = now + JUMP_HANDOFF_SEC
                self.phase_start = time.time()
                self.log("独木桥跳下后恢复站立完成，进入第六赛段前准备")
            if now < self.stage5_jump_handoff_until:
                self.log_throttle(
                    "stage5_jump_down_handoff",
                    0.25,
                    "独木桥跳下后零速Locomotion接管，确认可继续行走"
                )
                self._send_locomotion_handoff(
                    gait=GAIT_TROT_SLOW,
                    body_h=MOBILE_BODY_HEIGHT,
                    step_h=STAIR_SAFE_STEP_HEIGHT,
                    pitch=MOBILE_PITCH_BIAS,
                )
                return
            if self._time_in_phase() > 2.0:
                self._advance_stage(6)

    # ── 第六赛段：撷金建功 ──────────────────────────────

    def _run_stage6(self):
        if self._check_fall():
            return
        self._check_stage_timeout(6)

        if self.stage_phase == 0:
            self.log("阶段0: 恢复站立，视觉寻找足球")
            self.send_cmd(MODE_RECOVERY_STAND, 0, step_h=0.0)
            self.stage_phase = 1
            self.phase_start = time.time()
            return

        if self.stage_phase == 1:
            with self.response_lock:
                bar = self.response_bar
            if bar >= 95 or self._time_in_phase() > 1.5:
                self.stage_phase = 2
                self.phase_start = time.time()
            return

        if self.stage_phase == 2:
            self.log_throttle("stage6_football", 1.0, "阶段2: 视觉引导接近足球")
            bgr = image_msg_to_cv(sensor_node.rgb_image_raw)
            football_center, football_area = detect_football(bgr) if bgr is not None else (None, 0)
            if football_center is not None and football_area > 80:
                self.speak_once("stage6_football", "足球")
            steer, dist = self._steer_to_map_goal(STAGE6_FOOTBALL_POINT, gain=1.08, limit=0.50)
            if football_center and football_area > 60:
                visual_steer = (football_center[0] - IMG_WIDTH / 2.0) / (IMG_WIDTH / 2.0) * 0.4
                steer = max(-0.52, min(0.52, steer * 0.55 + visual_steer * 0.45))
            self.apply_locomotion(0.68, 0.0, steer, gait=GAIT_TROT_SLOW,
                                  step_h=HIGH_TRAVEL_STEP_HEIGHT, sensor=sensor_node)
            if football_area > 260 or (sensor_node.odom_got and dist < 0.35) or self._time_in_phase() > 4.0:
                self.stage_phase = 3
                self.phase_start = time.time()
            return

        if self.stage_phase == 3:
            self.log("阶段3: 踢球动作")
            self.send_cmd(MODE_LOCOMOTION, GAIT_TROT_SLOW, vx=1.20, vy=0.0,
                          vyaw=0.0, step_h=0.12, duration=520)
            self.stage_phase = 4
            self.phase_start = time.time()
            return

        if self.stage_phase == 4:
            if self._time_in_phase() > 0.8:
                self.stage_phase = 5
                self.phase_start = time.time()
            return

        if self.stage_phase == 5:
            self.log("阶段5: 地图目标点引导向终点移动")
            bgr = image_msg_to_cv(sensor_node.rgb_image_raw)
            lt = compute_local_target(bgr, stage=6)
            roll = sensor_node.imu_roll
            roll_corr = -roll * 1.5
            roll_corr = max(-0.25, min(0.25, roll_corr))
            nav_vx = 0.62
            if abs(roll) > 0.12:
                nav_vx = 0.44
            steer, dist = self._steer_to_map_goal(STAGE6_FINISH_POINT, gain=1.08, limit=0.50)
            self.apply_locomotion(nav_vx, 0.0, steer + roll_corr * 0.4, gait=GAIT_TROT_SLOW,
                                  step_h=MIN_TRAVEL_STEP_HEIGHT, sensor=sensor_node,
                                  boundary_check=True, target_point=lt)
            self._check_stuck()
            if sensor_node.odom_got and dist < 0.45:
                self.stage_phase = 6
                self.phase_start = time.time()
            elif self._time_in_phase() > 75.0:
                self.log_throttle(
                    "stage6_finish_slow",
                    3.0,
                    f"终点导航较慢，继续向终点执行 dist={dist:.2f}"
                )
            return

        if self.stage_phase == 6:
            self.log("阶段6: 到达终点，趴下")
            self.send_cmd(MODE_PURE_DAMPER, 0, step_h=0.0)
            self._advance_stage(7)

    # ── 完成 ────────────────────────────────────────────

    def _run_complete(self):
        total = time.time() - self.total_start_time
        m, s = divmod(int(total), 60)
        self.log("=" * 60)
        self.log(f"比赛全部完成！总用时: {m}分{s}秒")
        self.log(f"跌倒次数: {self.fall_count}")
        self.log(f"橙色小球撞击: {self.orange_hit_count}/{ORANGE_BALL_TOTAL}")
        self.log("机器狗已安全趴下")
        self.log("=" * 60)
        self.running = False

    def run(self):
        self.total_start_time = time.time()
        self.log("=" * 60)
        self.log("CyberDog 全自动竞赛控制器 启动 (无雷达方案)")
        self.log("=" * 60)

        self.stage = 0
        self.stage_start_time = time.time()
        self.stage_phase = 0
        self.phase_start = time.time()

        self.start_threads()

        try:
            while self.running:
                rclpy.spin_once(sensor_node, timeout_sec=0.002)
                sensor_node.update_odom()
                self._check_odom_health()
                control_ok = self._check_control_response_health()
                if not control_ok:
                    with self.send_lock:
                        command_ready = self.command_ready
                        last_mode = self.last_cmd_mode
                        last_gait = self.last_cmd_gait
                    with self.response_lock:
                        resp_time = self.response_time
                    if command_ready and (resp_time > 0.0 or self._total_elapsed() > CONTROL_RESPONSE_STARTUP_FATAL):
                        self.log(
                            "控制链路异常，停止比赛控制循环 "
                            f"(last cmd mode={last_mode}, gait={last_gait})"
                        )
                        break

                if self._total_elapsed() > TOTAL_TIME_LIMIT:
                    self.log("★★★ 总时间超过15分钟，比赛结束 ★★★")
                    break

                if 1 <= self.stage <= 6 and self._check_fall():
                    time.sleep(0.005)
                    continue

                if self.stage == 2 and self.relocating:
                    self.relocating = False
                    self.log("第二赛段固定坐标导航优先，清除重定位进程")
                if self.relocating:
                    if self._relocate_to_track():
                        time.sleep(0.02)
                        continue

                if self.stage == 0:
                    self._run_init()
                elif self.stage == 1:
                    self._run_stage1()
                elif self.stage == 2:
                    self._run_stage2()
                elif self.stage == 3:
                    self._run_stage3()
                elif self.stage == 4:
                    self._run_stage4()
                elif self.stage == 5:
                    self._run_stage5()
                elif self.stage == 6:
                    self._run_stage6()
                elif self.stage >= 7:
                    self._run_complete()
                    break

                time.sleep(0.02)

        except KeyboardInterrupt:
            self.log("收到中断信号")
        except Exception as e:
            self.log(f"异常: {e}")
            traceback.print_exc()

        self.stop_safe()
        self.running = False
        self.log("比赛程序退出")


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    sensor_node = SensorNode()

    controller = RaceController()
    controller.run()

    rclpy.shutdown()
