# LeRobot Sim UR3e Robot

A [LeRobot](https://github.com/huggingface/lerobot) plugin for the simulated Universal Robots UR3e.

## Overview

This package implements the `SimUR3E` robot class, enabling control and observation of the MuJoCo UR3e scene bundled in this package.

## Features

- **MuJoCo Simulation**: Wraps the bundled UR3e + Robotiq 2F-85 scene.
- **LeRobot Interface**: Exposes `joint_0` through `joint_5` and normalized `gripper` action/observation features.
- **Optional Sim Cameras**: Uses the separate `lerobot_camera_mujoco` plugin through the standard `robot.cameras` config.

## Requirements

- MuJoCo simulation assets under `third_party/ur_description` and `third_party/mujoco_menagerie`.
- `mujoco`, `dm-control`, `pin`, and `lerobot-camera-mujoco`.

## Installation

```bash
uv pip install -e ./lerobot_robot_sim_ur3e
```

## Configuration

Use robot type `sim_ur3e`. MuJoCo cameras are configured exactly like other LeRobot cameras:

```bash
--robot.type=sim_ur3e \
--robot.cameras="{agentview: {type: mujoco, camera: agentview, width: 128, height: 128, fps: 60}}"
```
