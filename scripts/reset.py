"""Reset a physical UR3 with Robotiq 2F-85 to the configured start joints."""

from __future__ import annotations

import argparse
import logging
import socket

from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.utils import init_logging
from lerobot_robot_ur3 import UR3, UR3Config

RTDE_CONTROL_PORT = 30004
ROBOTIQ_PORT = 63352


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reset UR3 to its initial joint configuration.")
    parser.add_argument("--ip", required=True, help="UR controller IP address.")  # 158.132.172.214
    parser.add_argument("--id", default="ur3", help="LeRobot robot id.")
    parser.add_argument("--speed", type=float, default=0.3, help="moveJ speed for reset.")
    parser.add_argument("--acceleration", type=float, default=0.2, help="moveJ acceleration for reset.")
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or activate the Robotiq gripper.")
    parser.add_argument("--skip-port-check", action="store_true", help="Skip TCP port checks before connecting.")
    return parser.parse_args()


def _check_tcp_port(host: str, port: int, timeout_s: float = 2.0) -> None:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return
    except OSError as exc:
        raise RuntimeError(
            f"Cannot connect to {host}:{port}. Check the robot IP, network route, and that the UR controller "
            f"has the corresponding service enabled."
        ) from exc


def main() -> None:
    args = parse_args()
    init_logging()
    register_third_party_devices()

    if not args.skip_port_check:
        _check_tcp_port(args.ip, RTDE_CONTROL_PORT)
        if not args.no_gripper:
            _check_tcp_port(args.ip, ROBOTIQ_PORT)
    print("TCP port checks passed. Connecting to the robot...")

    config = UR3Config(
        ip=args.ip,
        id=args.id,
        reset_speed=args.speed,
        reset_acceleration=args.acceleration,
        with_gripper=not args.no_gripper,
    )
    robot = UR3(config)
    print("robot")
    try:
        robot.connect()
        logging.info("Moving UR3 to start joints: %s", [round(value, 4) for value in config.start_joints])
        robot.move_to_start_joints(wait=True)
        logging.info("UR3 reset complete.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()












