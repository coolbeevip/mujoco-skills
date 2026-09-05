#!/usr/bin/env python3
"""Validate and replay a taught dual-PiPER trajectory without running IK."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

import numpy as np

from _piper_core import PickPlaceController, TrajectoryStage
from _trajectory_io import load_trajectory, resolve_scene_path, scene_sha256


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a taught JSON trajectory using recorded actuator targets."
    )
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--scene", type=Path)
    parser.add_argument(
        "--allow-scene-mismatch",
        action="store_true",
        help="Replay even when scene.xml differs from the teaching scene.",
    )
    parser.add_argument(
        "--initial-position-tolerance",
        type=float,
        default=0.015,
        help="Maximum allowed object-start displacement from teaching, in metres.",
    )
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    trajectory_path = args.trajectory.expanduser().resolve()
    document = load_trajectory(trajectory_path)
    scene_path = resolve_scene_path(document, trajectory_path, args.scene)
    if not scene_path.is_file():
        raise FileNotFoundError(f"Scene not found: {scene_path}")

    recorded_digest = str(document["scene"]["sha256"])
    current_digest = scene_sha256(scene_path)
    if recorded_digest != current_digest and not args.allow_scene_mismatch:
        raise RuntimeError(
            "Scene fingerprint differs from the teaching scene. "
            "Use --allow-scene-mismatch only after reviewing the change."
        )

    task = document["task"]
    arm_name = str(task["arm"])
    object_name = str(task["object"])
    target_xy = np.asarray(task["place_xy"], dtype=float)
    parameters = task["parameters"]
    stages = tuple(
        TrajectoryStage.from_dict(stage)
        for stage in document["trajectory"]["stages"]
    )
    if not stages:
        raise ValueError("Recorded trajectory contains no stages.")

    with PickPlaceController(
        scene_path,
        arm_name=arm_name,
        realtime=args.realtime,
        viewer=args.viewer,
    ) as controller:
        recorded_arm_actuators = tuple(document["actuators"]["arm"])
        if recorded_arm_actuators != controller.arm.actuator_names:
            raise RuntimeError(
                "Arm actuator order differs from the teaching trajectory."
            )
        if str(document["actuators"]["gripper"]) != (
            controller.arm.gripper_actuator_name
        ):
            raise RuntimeError(
                "Gripper actuator differs from the teaching trajectory."
            )
        recorded_home = np.asarray(
            document["trajectory"]["home_joints"],
            dtype=float,
        )
        if not np.allclose(recorded_home, controller.home_joints, atol=1e-6):
            raise RuntimeError("Home pose differs from the teaching trajectory.")
        recorded_timestep = float(document["trajectory"]["timestep"])
        if not np.isclose(recorded_timestep, controller.model.opt.timestep):
            raise RuntimeError(
                "MuJoCo timestep differs from the teaching trajectory."
            )

        object_id = controller.model.body(object_name).id
        current_initial_position = controller.data.xpos[object_id].copy()
        recorded_initial_position = np.asarray(
            document["teaching"]["selected_result"]["initial_object_position"],
            dtype=float,
        )
        initial_error = float(
            np.linalg.norm(current_initial_position - recorded_initial_position)
        )
        if initial_error > args.initial_position_tolerance:
            raise RuntimeError(
                f"Object start position differs by {initial_error:.4f} m; "
                "run teaching again for the new layout."
            )

        print(
            f"replaying {len(stages)} stages for {object_name} with {arm_name}; "
            "IK is not used"
        )
        result = controller.execute_trajectory(
            object_name,
            target_xy,
            stages,
            placement_tolerance=float(parameters["placement_tolerance"]),
            minimum_lift=float(parameters["minimum_lift"]),
        )

    print("result:")
    print(json.dumps(result.as_dict(), indent=2, ensure_ascii=False))
    return 0 if result.successful else 2


if __name__ == "__main__":
    raise SystemExit(main())
