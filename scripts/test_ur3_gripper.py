"""Minimal Robotiq 2F-85 socket test for a UR controller."""

from __future__ import annotations

import argparse
import time

from lerobot_robot_ur3.robotiq_gripper import RobotiqGripper


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test Robotiq gripper commands through the UR controller.")
    parser.add_argument("--ip", required=True, help="UR controller IP address.")
    parser.add_argument("--port", type=int, default=63352, help="Robotiq socket port.")
    parser.add_argument("--speed", type=int, default=255, help="Robotiq speed command, 0..255.")
    parser.add_argument("--force", type=int, default=255, help="Robotiq force command, 0..255.")
    parser.add_argument("--open", type=int, default=0, help="Open POS command, 0..255.")
    parser.add_argument("--close", type=int, default=255, help="Close POS command, 0..255.")
    parser.add_argument("--sleep", type=float, default=2.0, help="Seconds to wait between commands.")
    parser.add_argument("--reset", action="store_true", help="Force gripper reset before activation.")
    return parser.parse_args()


def dump_status(gripper: RobotiqGripper, label: str) -> None:
    values = {}
    for var in (
        RobotiqGripper.ACT,
        RobotiqGripper.STA,
        RobotiqGripper.GTO,
        RobotiqGripper.POS,
        RobotiqGripper.PRE,
        RobotiqGripper.OBJ,
        RobotiqGripper.FLT,
    ):
        try:
            values[var] = gripper._get_var(var)
        except Exception as exc:  # noqa: BLE001
            values[var] = f"ERR:{exc}"
    print(f"{label}: {values}")


def main() -> None:
    args = parse_args()
    gripper = RobotiqGripper()
    gripper.connect(args.ip, args.port)
    try:
        if args.reset:
            gripper._reset()
            dump_status(gripper, "after reset")
        gripper.activate(auto_calibrate=False)
        print(f"active={gripper.is_active()} pos={gripper.get_current_position()}")
        dump_status(gripper, "after activate")

        for name, position in (("open", args.open), ("close", args.close), ("open", args.open)):
            ok, requested = gripper.move(position, args.speed, args.force)
            print(f"{name}: cmd={position} ok={ok} requested={requested}")
            dump_status(gripper, f"{name} immediately")
            deadline = time.perf_counter() + args.sleep
            while time.perf_counter() < deadline:
                time.sleep(0.2)
                dump_status(gripper, f"{name} polling")
            print(f"{name}: actual_pos={gripper.get_current_position()}")
    finally:
        gripper.disconnect()


if __name__ == "__main__":
    main()
