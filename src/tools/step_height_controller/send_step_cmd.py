#!/usr/bin/env python3
"""
CLI to send a single `robot_control_cmd` LCM message.

Example:
  python3 send_step_cmd.py --step 0.07 --vx 0.2 --gait 26

This script is lightweight (no ROS required). It uses the generated LCM
python type `robot_control_cmd_lcmt.py` under the repo.
"""
import os
import sys
import argparse

def add_lcm_types_to_path():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    lcm_py_path = os.path.join(repo_root, 'src', 'cyberdog_locomotion', 'common', 'lcm_type', 'lcm')
    if lcm_py_path not in sys.path:
        sys.path.insert(0, lcm_py_path)


def main():
    parser = argparse.ArgumentParser(description='Send robot_control_cmd via LCM (CLI)')
    parser.add_argument('--lcm', default='udpm://239.255.76.67:7671?ttl=255', help='LCM URL')
    parser.add_argument('--mode', type=int, default=11, help='mode (default locomotion=11)')
    parser.add_argument('--gait', type=int, default=26, help='gait_id (default 26)')
    parser.add_argument('--step', type=float, help='step height in meters (applies to front/rear)')
    parser.add_argument('--vx', type=float, default=0.0, help='forward velocity (m/s)')
    parser.add_argument('--vy', type=float, default=0.0, help='lateral velocity (m/s)')
    parser.add_argument('--vyaw', type=float, default=0.0, help='yaw rate (rad/s)')
    parser.add_argument('--duration', type=int, default=0, help='duration ms (0 continuous)')
    parser.add_argument('--once', action='store_true', help='publish once and exit (default)')
    parser.add_argument('--repeat', type=int, default=10, help='repeat publish count when not --once')
    args = parser.parse_args()

    add_lcm_types_to_path()
    try:
        import lcm
        from robot_control_cmd_lcmt import robot_control_cmd_lcmt
    except Exception as e:
        print('Missing dependency or cannot import lcm types:', e)
        print('Make sure `lcm` and the repo generated types are available.')
        sys.exit(1)

    lc = lcm.LCM(args.lcm)

    msg = robot_control_cmd_lcmt()
    msg.mode = int(args.mode)
    msg.gait_id = int(args.gait)
    msg.life_count = (msg.life_count + 1) & 0xFF
    msg.vel_des = [float(args.vx), float(args.vy), float(args.vyaw)]
    if args.step is not None:
        # the controller expects two-step heights packed in two floats
        msg.step_height = [float(args.step), float(args.step)]
    msg.duration = int(args.duration)

    def publish():
        lc.publish('robot_control_cmd', msg.encode())
        print(f'Published robot_control_cmd: mode={msg.mode} gait_id={msg.gait_id} step={msg.step_height if hasattr(msg, "step_height") else None} vel={msg.vel_des}')

    if args.once:
        publish()
    else:
        for i in range(args.repeat):
            publish()
            try:
                import time
                time.sleep(0.2)
            except KeyboardInterrupt:
                break


if __name__ == '__main__':
    main()
