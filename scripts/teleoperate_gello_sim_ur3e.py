"""Direct teleoperation script for GELLO controlling the simulated UR3e."""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

from lerobot.processor import RobotObservation, make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging

from lerobot_camera_mujoco import MujocoCameraConfig
from lerobot_robot_sim_ur3e import SimUR3EConfig
from lerobot_teleoperator_gello import GelloConfig


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate MuJoCo UR3e simulation with a GELLO leader.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="GELLO Dynamixel serial port.")
    parser.add_argument("--fps", type=int, default=30, help="Control loop frequency.")
    parser.add_argument("--teleop-id", default="gello", help="LeRobot id for the GELLO teleoperator.")
    parser.add_argument("--robot-id", default="sim_ur3e", help="LeRobot id for the simulated robot.")
    parser.add_argument("--no-camera", action="store_true", help="Disable MuJoCo camera observations.")
    parser.add_argument("--camera", default="agentview", help="MuJoCo camera name or integer id.")
    parser.add_argument("--camera-key", default="agentview", help="Observation key for the camera.")
    parser.add_argument("--no-eye-in-hand", action="store_true", help="Disable the wrist eye-in-hand camera.")
    parser.add_argument("--eye-in-hand-camera", default="eye_in_hand", help="MuJoCo wrist camera name.")
    parser.add_argument("--eye-in-hand-key", default="eye_in_hand", help="Observation key for the wrist camera.")
    parser.add_argument("--width", type=int, default=640, help="Camera image width.")
    parser.add_argument("--height", type=int, default=480, help="Camera image height.")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=Path("calibration"),
        help="Root directory for LeRobot robot and teleoperator calibration files.",
    )
    parser.add_argument("--display-data", action="store_true", help="Log observations and actions to Rerun.")
    parser.add_argument("--collision-debug", action="store_true", help="Enable UR3 self-collision debug logging.")
    parser.add_argument("--no-viewer", action="store_true", help="Do not open the MuJoCo viewer window.")
    parser.add_argument("--command-substeps", type=int, default=6, help="MuJoCo substeps for ordinary arm commands.")
    parser.add_argument(
        "--gripper-command-substeps",
        type=int,
        default=120,
        help="MuJoCo substeps when the Robotiq gripper target changes.",
    )
    return parser.parse_args()


def _maybe_init_rerun(display_data: bool) -> bool:
    if not display_data:
        return False

    try:
        from lerobot.utils.visualization_utils import init_rerun
    except Exception as exc:  # noqa: BLE001 - optional viewer
        logging.warning("Rerun display requested but visualization utilities are unavailable: %s", exc)
        return False

    init_rerun("gello_sim_ur3e")
    return True


def _maybe_log_rerun(display_data: bool, obs: RobotObservation, action: dict[str, float]) -> None:
    if not display_data:
        return

    try:
        from lerobot.utils.visualization_utils import log_rerun_data
    except Exception as exc:  # noqa: BLE001 - optional viewer
        logging.warning("Could not log Rerun data: %s", exc)
        return

    log_rerun_data(observation=obs, action=action)


def main() -> None:
    args = _parse_args()
    init_logging()
    logging.info("Starting GELLO <-> simulated UR3e teleoperation")

    register_third_party_devices()

    cameras = {}
    if not args.no_camera:
        cameras[args.camera_key] = MujocoCameraConfig(
            camera=args.camera,
            width=args.width,
            height=args.height,
            fps=args.fps,
        )
        if not args.no_eye_in_hand:
            cameras[args.eye_in_hand_key] = MujocoCameraConfig(
                camera=args.eye_in_hand_camera,
                width=args.width,
                height=args.height,
                fps=args.fps,
            )

    robot_cfg = SimUR3EConfig(
        id=args.robot_id,
        calibration_dir=args.calibration_dir / "robots" / "sim_ur3e",
        cameras=cameras,
        collision_debug=args.collision_debug,
        show_viewer=not args.no_viewer,
        command_substeps=args.command_substeps,
        gripper_command_substeps=args.gripper_command_substeps,
    )
    teleop_cfg = GelloConfig(
        port=args.port,
        id=args.teleop_id,
        calibration_dir=args.calibration_dir / "teleoperators" / "gello",
    )

    robot = make_robot_from_config(robot_cfg)
    teleop = make_teleoperator_from_config(teleop_cfg)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    display_data = _maybe_init_rerun(args.display_data)
    loop_period_s = 1.0 / args.fps

    try:
        robot.connect()
        teleop.connect()

        while True:
            loop_start = time.perf_counter()

            try:
                obs = robot.get_observation()
            except DeviceNotConnectedError:
                logging.warning("Robot disconnected while reading observation")
                obs = {}

            obs = robot_observation_processor(obs)
            raw_action = teleop.get_action()
            teleop_action = teleop_action_processor((raw_action, obs))
            robot_action = robot_action_processor((teleop_action, obs))
            sent_action = robot.send_action(robot_action)

            _maybe_log_rerun(display_data, obs, sent_action)

            busy_wait(loop_period_s - (time.perf_counter() - loop_start))

    except KeyboardInterrupt:
        logging.info("Teleoperation interrupted by user")
    finally:
        if teleop.is_connected:
            teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
