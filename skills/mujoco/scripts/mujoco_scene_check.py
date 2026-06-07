#!/usr/bin/env python3
"""Compile and run passive physical sanity checks for MuJoCo scenes."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


MARKER_NAME_TOKENS = (
    "marker",
    "site",
    "target",
    "grasp",
    "debug",
    "waypoint",
    "frame",
    "center",
)

FIXED_ARM_TOKENS = (
    "arm",
    "panda",
    "franka",
    "ur3",
    "ur5",
    "ur10",
    "xarm",
    "kuka",
    "iiwa",
    "jaco",
    "kinova",
    "widow",
)

MOBILE_TOKENS = (
    "mobile",
    "quadruped",
    "humanoid",
    "drone",
    "quadrotor",
    "go1",
    "go2",
    "unitree",
)


@dataclass(frozen=True)
class Issue:
    severity: str
    code: str
    message: str


@dataclass(frozen=True)
class FreeBodySnapshot:
    body_id: int
    name: str
    xpos: np.ndarray
    xquat: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compile a MuJoCo XML scene and run conservative checks for "
            "initial physical plausibility."
        )
    )
    parser.add_argument("scene", help="Path to the MuJoCo XML scene file.")
    parser.add_argument(
        "--steps",
        type=int,
        default=300,
        help="Number of passive simulation steps to run. Default: 300.",
    )
    parser.add_argument(
        "--drop-threshold",
        type=float,
        default=0.05,
        help="Free-body z drop in meters that counts as an error. Default: 0.05.",
    )
    parser.add_argument(
        "--xy-threshold",
        type=float,
        default=0.10,
        help="Free-body horizontal drift in meters that counts as an error. Default: 0.10.",
    )
    parser.add_argument(
        "--tilt-threshold-deg",
        type=float,
        default=25.0,
        help="Free-body orientation change that counts as an error. Default: 25.",
    )
    parser.add_argument(
        "--penetration-threshold",
        type=float,
        default=1e-4,
        help="Initial negative contact distance in meters that counts as an error.",
    )
    parser.add_argument(
        "--marker-radius",
        type=float,
        default=0.035,
        help="Sphere radius under which collision geoms are checked as possible markers.",
    )
    parser.add_argument(
        "--warn-only",
        action="store_true",
        help="Print errors but exit with status 0.",
    )
    return parser.parse_args()


def resolve_scene_path(scene: str) -> Path:
    path = Path(scene).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    else:
        path = path.resolve()
    if not path.exists():
        raise FileNotFoundError(f"Scene file not found: {path}")
    if not path.is_file():
        raise IsADirectoryError(f"Scene path is not a file: {path}")
    return path


def object_name(mujoco: object, model: object, obj_type: object, obj_id: int) -> str:
    name = mujoco.mj_id2name(model, obj_type, obj_id)
    return name if name else f"<unnamed:{obj_id}>"


def body_name(mujoco: object, model: object, body_id: int) -> str:
    return object_name(mujoco, model, mujoco.mjtObj.mjOBJ_BODY, body_id)


def geom_name(mujoco: object, model: object, geom_id: int) -> str:
    return object_name(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def joint_name(mujoco: object, model: object, joint_id: int) -> str:
    return object_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)


def body_joint_ids(model: object, body_id: int) -> range:
    start = int(model.body_jntadr[body_id])
    count = int(model.body_jntnum[body_id])
    return range(start, start + count)


def body_has_free_joint(mujoco: object, model: object, body_id: int) -> bool:
    for joint_id in body_joint_ids(model, body_id):
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            return True
    return False


def free_body_joint_names(mujoco: object, model: object, body_id: int) -> list[str]:
    names = []
    for joint_id in body_joint_ids(model, body_id):
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            names.append(joint_name(mujoco, model, joint_id))
    return names


def free_body_ids(mujoco: object, model: object) -> list[int]:
    return [
        body_id
        for body_id in range(1, model.nbody)
        if body_has_free_joint(mujoco, model, body_id)
    ]


def is_descendant_body(model: object, body_id: int, ancestor_id: int) -> bool:
    current = body_id
    while current > 0:
        if current == ancestor_id:
            return True
        current = int(model.body_parentid[current])
    return ancestor_id == 0


def descendant_body_ids(model: object, ancestor_id: int) -> list[int]:
    return [
        body_id
        for body_id in range(1, model.nbody)
        if is_descendant_body(model, body_id, ancestor_id)
    ]


def actuated_descendant_joint_count(mujoco: object, model: object, body_id: int) -> int:
    count = 0
    joint_trn = int(mujoco.mjtTrn.mjTRN_JOINT)
    for actuator_id in range(model.nu):
        if int(model.actuator_trntype[actuator_id]) != joint_trn:
            continue
        joint_id = int(model.actuator_trnid[actuator_id][0])
        if joint_id < 0:
            continue
        joint_body_id = int(model.jnt_bodyid[joint_id])
        if is_descendant_body(model, joint_body_id, body_id):
            count += 1
    return count


def name_blob(*parts: str) -> str:
    return " ".join(part.lower() for part in parts if part)


def contains_any(text: str, tokens: Iterable[str]) -> bool:
    return any(token in text for token in tokens)


def check_initial_contacts(
    mujoco: object,
    model: object,
    data: object,
    threshold: float,
) -> list[Issue]:
    issues = []
    mujoco.mj_forward(model, data)
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        if float(contact.dist) >= -threshold:
            continue
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        issues.append(
            Issue(
                severity="error",
                code="INITIAL_PENETRATION",
                message=(
                    f"{geom_name(mujoco, model, geom1)} and "
                    f"{geom_name(mujoco, model, geom2)} start with contact "
                    f"distance {float(contact.dist):.6f} m."
                ),
            )
        )
    return issues


def capture_free_body_snapshots(
    mujoco: object,
    model: object,
    data: object,
) -> dict[int, FreeBodySnapshot]:
    mujoco.mj_forward(model, data)
    snapshots = {}
    for body_id in free_body_ids(mujoco, model):
        snapshots[body_id] = FreeBodySnapshot(
            body_id=body_id,
            name=body_name(mujoco, model, body_id),
            xpos=np.array(data.xpos[body_id], dtype=float).copy(),
            xquat=np.array(data.xquat[body_id], dtype=float).copy(),
        )
    return snapshots


def quat_angle_degrees(q1: np.ndarray, q2: np.ndarray) -> float:
    q1_norm = q1 / np.linalg.norm(q1)
    q2_norm = q2 / np.linalg.norm(q2)
    dot = abs(float(np.dot(q1_norm, q2_norm)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def check_passive_drift(
    mujoco: object,
    model: object,
    data: object,
    initial: dict[int, FreeBodySnapshot],
    drop_threshold: float,
    xy_threshold: float,
    tilt_threshold_deg: float,
) -> list[Issue]:
    issues = []
    mujoco.mj_forward(model, data)
    for body_id, snapshot in initial.items():
        current_pos = np.array(data.xpos[body_id], dtype=float)
        current_quat = np.array(data.xquat[body_id], dtype=float)
        delta = current_pos - snapshot.xpos
        xy_drift = float(np.linalg.norm(delta[:2]))
        z_delta = float(delta[2])
        tilt_delta = quat_angle_degrees(snapshot.xquat, current_quat)

        messages = []
        if z_delta < -drop_threshold:
            messages.append(f"z dropped {abs(z_delta):.3f} m")
        if xy_drift > xy_threshold:
            messages.append(f"xy drifted {xy_drift:.3f} m")
        if tilt_delta > tilt_threshold_deg:
            messages.append(f"rotated {tilt_delta:.1f} deg")
        if not messages:
            continue

        issues.append(
            Issue(
                severity="error",
                code="PASSIVE_FREE_BODY_DRIFT",
                message=(
                    f"Free body {snapshot.name} changed too much under zero control: "
                    + ", ".join(messages)
                    + ". Check support height, freejoint use, contacts, inertia, and initial pose."
                ),
            )
        )
    return issues


def check_possible_marker_geoms(
    mujoco: object,
    model: object,
    marker_radius: float,
) -> list[Issue]:
    issues = []
    sphere_type = int(mujoco.mjtGeom.mjGEOM_SPHERE)
    for geom_id in range(model.ngeom):
        if int(model.geom_type[geom_id]) != sphere_type:
            continue
        radius = float(model.geom_size[geom_id][0])
        if radius > marker_radius:
            continue

        contact_enabled = (
            int(model.geom_contype[geom_id]) != 0
            or int(model.geom_conaffinity[geom_id]) != 0
        )
        if not contact_enabled:
            continue

        owning_body_id = int(model.geom_bodyid[geom_id])
        blob = name_blob(
            geom_name(mujoco, model, geom_id),
            body_name(mujoco, model, owning_body_id),
        )
        marker_named = contains_any(blob, MARKER_NAME_TOKENS)

        sibling_count = sum(
            1
            for sibling_id in range(model.ngeom)
            if int(model.geom_bodyid[sibling_id]) == owning_body_id
        )
        if not marker_named and sibling_count <= 1:
            continue

        issues.append(
            Issue(
                severity="warning",
                code="POSSIBLE_PHYSICAL_MARKER",
                message=(
                    f"Small collision-enabled sphere geom "
                    f"{geom_name(mujoco, model, geom_id)} on body "
                    f"{body_name(mujoco, model, owning_body_id)} has radius "
                    f"{radius:.3f} m. If this is a grasp/target/debug marker, "
                    f"use a site or set contype/conaffinity to 0."
                ),
            )
        )
    return issues


def check_free_robot_roots(mujoco: object, model: object) -> list[Issue]:
    issues = []
    for body_id in free_body_ids(mujoco, model):
        body = body_name(mujoco, model, body_id)
        joint_names = ", ".join(free_body_joint_names(mujoco, model, body_id))
        subtree_names = [
            body_name(mujoco, model, descendant_id)
            for descendant_id in descendant_body_ids(model, body_id)
        ]
        blob = name_blob(body, joint_names, *subtree_names)
        if contains_any(blob, MOBILE_TOKENS):
            continue
        actuated_joint_count = actuated_descendant_joint_count(mujoco, model, body_id)
        looks_like_arm = contains_any(blob, FIXED_ARM_TOKENS)
        if not looks_like_arm and actuated_joint_count < 3:
            continue
        issues.append(
            Issue(
                severity="warning",
                code="POSSIBLE_FREE_ARM_BASE",
                message=(
                    f"Root body {body} has a free joint ({joint_names}) and "
                    f"{actuated_joint_count} actuated descendant joint(s). "
                    f"If this is a fixed-base manipulator, remove the freejoint "
                    f"or mount the base to a fixed support."
                ),
            )
        )
    return issues


def run_passive_steps(mujoco: object, model: object, data: object, steps: int) -> None:
    for _ in range(max(0, steps)):
        mujoco.mj_step(model, data)


def print_report(scene_path: Path, model: object, steps: int, issues: list[Issue]) -> None:
    print(f"Scene: {scene_path}")
    print(
        "Model: "
        f"{model.nbody} bodies, {model.ngeom} geoms, "
        f"{model.njnt} joints, {model.nu} actuators"
    )
    print(f"Passive simulation: {steps} step(s)")

    if not issues:
        print("PASS: scene compiled and no physical sanity issues were detected.")
        return

    print("Issues:")
    for issue in issues:
        print(f"- {issue.severity.upper()} {issue.code}: {issue.message}")


def main() -> int:
    args = parse_args()
    try:
        import mujoco

        scene_path = resolve_scene_path(args.scene)
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)

        issues: list[Issue] = []
        issues.extend(
            check_initial_contacts(
                mujoco,
                model,
                data,
                threshold=args.penetration_threshold,
            )
        )
        issues.extend(check_possible_marker_geoms(mujoco, model, args.marker_radius))
        issues.extend(check_free_robot_roots(mujoco, model))

        initial = capture_free_body_snapshots(mujoco, model, data)
        run_passive_steps(mujoco, model, data, args.steps)
        issues.extend(
            check_passive_drift(
                mujoco,
                model,
                data,
                initial=initial,
                drop_threshold=args.drop_threshold,
                xy_threshold=args.xy_threshold,
                tilt_threshold_deg=args.tilt_threshold_deg,
            )
        )

        print_report(scene_path, model, args.steps, issues)
        has_errors = any(issue.severity == "error" for issue in issues)
        return 1 if has_errors and not args.warn_only else 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
