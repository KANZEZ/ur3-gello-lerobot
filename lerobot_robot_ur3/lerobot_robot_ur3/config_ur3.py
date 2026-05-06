"""Configuration dataclass for the UR3 robot plugin."""

from dataclasses import dataclass, field

import numpy as np
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


DEFAULT_UR3_START_JOINTS = (
    np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
)


@RobotConfig.register_subclass("ur3")
@dataclass
class UR3Config(RobotConfig):
    ip: str
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    start_joints: tuple[float, ...] = DEFAULT_UR3_START_JOINTS
    reset_acceleration: float = 0.2
    reset_speed: float = 0.5

    servo_acceleration: float = 0.2
    servo_speed: float = 0.2
    servoj_t: float = 1.0 / 500
    servoj_lookahead: float = 0.2
    servoj_gain: int = 100

    with_gripper: bool = True
    gripper_port: int = 63352
    gripper_speed: int = 160
    gripper_force: int = 255
    gripper_auto_calibrate: bool = False

    def start_joints_array(self) -> np.ndarray:
        joints = np.asarray(self.start_joints, dtype=float)
        if joints.shape != (6,):
            raise ValueError(f"Expected 6 start joints for UR3, got shape {joints.shape}.")
        return joints
