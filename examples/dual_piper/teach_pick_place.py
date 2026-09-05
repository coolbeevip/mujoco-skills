#!/usr/bin/env python3
"""Teach, simulate, score, and save a collision-free pick/place trajectory."""

from __future__ import annotations

import argparse
import itertools
import json
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np

from _piper_core import (
    DEFAULT_TRANSIT_HEIGHT,
    CartesianPose,
    JointPose,
    PickPlaceController,
    PickPlacePlan,
    TrajectoryStage,
    default_scene_path,
    validate_scene_and_target,
)
from _trajectory_io import (
    SCHEMA_NAME,
    SCHEMA_VERSION,
    scene_sha256,
    write_trajectory,
)


@dataclass(frozen=True)
class AttemptConfig:
    transit_height: float
    grasp_z_offset: float
    approach_clearance: float

    def as_dict(self) -> dict[str, float]:
        return {
            "transit_height": self.transit_height,
            "grasp_z_offset": self.grasp_z_offset,
            "approach_clearance": self.approach_clearance,
        }


def unique(values: Iterable[float]) -> list[float]:
    result: list[float] = []
    for value in values:
        rounded = round(float(value), 6)
        if rounded not in result:
            result.append(rounded)
    return result


def candidate_configs(args: argparse.Namespace) -> Iterable[AttemptConfig]:
    transit_heights = unique(
        (
            args.transit_height,
            args.transit_height - 0.04,
            args.transit_height + 0.04,
        )
    )
    grasp_offsets = unique(
        (
            args.grasp_z_offset,
            args.grasp_z_offset - 0.003,
            args.grasp_z_offset + 0.003,
        )
    )
    approach_clearances = unique(
        (
            args.approach_clearance,
            args.approach_clearance + 0.01,
        )
    )
    for transit_height, grasp_offset, approach_clearance in itertools.product(
        transit_heights,
        grasp_offsets,
        approach_clearances,
    ):
        if transit_height < 0.88:
            continue
        yield AttemptConfig(
            transit_height=transit_height,
            grasp_z_offset=grasp_offset,
            approach_clearance=approach_clearance,
        )


def resolve_arm(scene_path: Path, object_name: str, requested: str) -> str:
    if requested != "auto":
        return requested
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return "arm0" if float(data.xpos[model.body(object_name).id, 0]) < 0 else "arm1"


def trajectory_joint_travel(
    home_joints: np.ndarray,
    stages: Iterable[TrajectoryStage],
) -> float:
    previous = np.asarray(home_joints, dtype=float)
    travel = 0.0
    for stage in stages:
        if stage.command != "move_arm":
            continue
        assert isinstance(stage.target, np.ndarray)
        travel += float(np.linalg.norm(stage.target - previous))
        previous = stage.target
    return travel


def attempt_score(
    placement_error_xy: float,
    joint_travel: float,
    duration_seconds: float,
) -> float:
    """Lower is better: accuracy first, then motion and simulated duration."""

    placement_error_cm = placement_error_xy * 100.0
    return placement_error_cm + 0.02 * joint_travel + 0.001 * duration_seconds


def serialize_poses(
    plan: PickPlacePlan,
    solved: dict[str, np.ndarray],
) -> dict[str, dict[str, object]]:
    payload: dict[str, dict[str, object]] = {}
    for pose in plan.named_poses():
        if isinstance(pose, JointPose):
            payload[pose.name] = {
                "type": "joint",
                "joints": solved[pose.name].tolist(),
            }
        elif isinstance(pose, CartesianPose):
            payload[pose.name] = {
                "type": "cartesian",
                "position": pose.position.tolist(),
                "rotation": pose.rotation.tolist(),
                "joints": solved[pose.name].tolist(),
            }
    return payload


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Search successful MuJoCo pick/place attempts and save the best "
            "actuator trajectory to JSON."
        )
    )
    parser.add_argument("--scene", type=Path, default=default_scene_path())
    parser.add_argument("--object", default="red_cube", dest="object_name")
    parser.add_argument(
        "--arm",
        choices=("auto", "arm0", "arm1"),
        default="auto",
    )
    parser.add_argument("--place-x", type=float, default=0.0)
    parser.add_argument("--place-y", type=float, default=0.10)
    parser.add_argument("--approach-clearance", type=float, default=0.085)
    parser.add_argument("--grasp-z-offset", type=float, default=0.0)
    parser.add_argument("--place-z-offset", type=float, default=0.015)
    parser.add_argument("--transit-height", type=float, default=DEFAULT_TRANSIT_HEIGHT)
    parser.add_argument("--move-steps", type=int, default=700)
    parser.add_argument("--descend-steps", type=int, default=450)
    parser.add_argument("--gripper-steps", type=int, default=350)
    parser.add_argument("--settle-steps", type=int, default=300)
    parser.add_argument("--open-value", type=float, default=0.035)
    parser.add_argument("--close-value", type=float, default=0.0)
    parser.add_argument("--placement-tolerance", type=float, default=0.06)
    parser.add_argument("--minimum-lift", type=float, default=0.04)
    parser.add_argument("--max-attempts", type=int, default=12)
    parser.add_argument("--successes-needed", type=int, default=3)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    if args.max_attempts <= 0 or args.successes_needed <= 0:
        raise ValueError("max-attempts and successes-needed must be positive.")

    scene_path = args.scene.expanduser().resolve()
    place_xy = np.array([args.place_x, args.place_y], dtype=float)
    validate_scene_and_target(scene_path, args.object_name, place_xy)
    arm_name = resolve_arm(scene_path, args.object_name, args.arm)
    output_path = (
        args.output.expanduser().resolve()
        if args.output is not None
        else scene_path.parent
        / "trajectories"
        / f"{args.object_name}_to_center.json"
    )

    attempts: list[dict[str, object]] = []
    successful_attempts: list[dict[str, object]] = []

    for attempt_index, config in enumerate(candidate_configs(args), start=1):
        if attempt_index > args.max_attempts:
            break
        print(
            f"attempt {attempt_index}: "
            f"transit={config.transit_height:.3f}, "
            f"grasp_z={config.grasp_z_offset:.3f}, "
            f"approach={config.approach_clearance:.3f}"
        )
        summary: dict[str, object] = {
            "attempt": attempt_index,
            "parameters": config.as_dict(),
        }
        try:
            with PickPlaceController(
                scene_path,
                arm_name=arm_name,
                realtime=args.realtime,
                viewer=args.viewer,
            ) as controller:
                plan = controller.build_plan(
                    args.object_name,
                    place_xy,
                    approach_clearance=config.approach_clearance,
                    grasp_z_offset=config.grasp_z_offset,
                    place_z_offset=args.place_z_offset,
                    transit_height=config.transit_height,
                )
                solved = controller.solve_plan(plan)
                stages = controller.build_trajectory(
                    plan,
                    solved,
                    move_steps=args.move_steps,
                    descend_steps=args.descend_steps,
                    gripper_steps=args.gripper_steps,
                    settle_steps=args.settle_steps,
                    open_value=args.open_value,
                    close_value=args.close_value,
                )
                result = controller.execute_trajectory(
                    args.object_name,
                    place_xy,
                    stages,
                    placement_tolerance=args.placement_tolerance,
                    minimum_lift=args.minimum_lift,
                )
                joint_travel = trajectory_joint_travel(
                    controller.home_joints,
                    stages,
                )
                duration_seconds = (
                    sum(stage.steps for stage in stages) * controller.model.opt.timestep
                )
                score = attempt_score(
                    result.placement_error_xy,
                    joint_travel,
                    duration_seconds,
                )
                summary.update(
                    {
                        "success": result.successful,
                        "score": score,
                        "joint_travel": joint_travel,
                        "duration_seconds": duration_seconds,
                        "result": result.as_dict(),
                    }
                )
                if result.successful:
                    successful_attempts.append(
                        {
                            "summary": summary,
                            "config": config,
                            "plan": plan,
                            "solved": solved,
                            "stages": stages,
                            "home_joints": controller.home_joints.copy(),
                            "actuator_names": controller.arm.actuator_names,
                            "gripper_actuator": controller.arm.gripper_actuator_name,
                            "model_dimensions": {
                                "nq": controller.model.nq,
                                "nv": controller.model.nv,
                                "nu": controller.model.nu,
                            },
                            "timestep": float(controller.model.opt.timestep),
                        }
                    )
                    print(
                        f"  success score={score:.4f}, "
                        f"placement_error={result.placement_error_xy:.4f} m"
                    )
                else:
                    print(f"  failed: {json.dumps(result.as_dict())}")
        except (RuntimeError, ValueError) as exc:
            summary.update({"success": False, "error": str(exc)})
            print(f"  failed: {exc}")
        attempts.append(summary)
        if len(successful_attempts) >= args.successes_needed:
            break

    if not successful_attempts:
        print("No successful trajectory was found; JSON was not written.")
        return 2

    best = min(
        successful_attempts,
        key=lambda item: float(item["summary"]["score"]),
    )
    selected_config = best["config"]
    selected_plan = best["plan"]
    selected_solved = best["solved"]
    selected_stages = best["stages"]
    scene_reference = os.path.relpath(scene_path, output_path.parent)
    document = {
        "schema": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scene": {
            "file": scene_reference,
            "sha256": scene_sha256(scene_path),
            "keyframe": "ready",
            "model": best["model_dimensions"],
        },
        "task": {
            "object": args.object_name,
            "arm": arm_name,
            "place_xy": place_xy.tolist(),
            "parameters": {
                **selected_config.as_dict(),
                "place_z_offset": args.place_z_offset,
                "placement_tolerance": args.placement_tolerance,
                "minimum_lift": args.minimum_lift,
            },
        },
        "actuators": {
            "arm": list(best["actuator_names"]),
            "gripper": best["gripper_actuator"],
        },
        "poses": serialize_poses(selected_plan, selected_solved),
        "trajectory": {
            "control_mode": "position_targets",
            "timestep": best["timestep"],
            "home_joints": best["home_joints"].tolist(),
            "stages": [stage.as_dict() for stage in selected_stages],
        },
        "teaching": {
            "attempts_run": len(attempts),
            "successful_attempts": len(successful_attempts),
            "selected_attempt": best["summary"]["attempt"],
            "score_definition": (
                "placement_error_cm + 0.02 * joint_travel_rad "
                "+ 0.001 * duration_seconds"
            ),
            "selected_score": best["summary"]["score"],
            "selected_result": best["summary"]["result"],
            "attempts": attempts,
        },
    }
    write_trajectory(output_path, document)
    print(f"saved best trajectory: {output_path}")
    print(json.dumps(document["teaching"]["selected_result"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
