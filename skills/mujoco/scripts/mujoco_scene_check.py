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

VISIBLE_REFERENCE_SITE_TOKENS = (
    "marker",
    "target",
    "grasp",
    "debug",
    "waypoint",
    "drop",
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

MANIPULATION_TOKENS = (
    "sort",
    "sorting",
    "pick",
    "place",
    "grasp",
    "bin",
    "cube",
    "block",
)

END_EFFECTOR_TOKENS = (
    "gripper",
    "finger",
    "claw",
    "hand",
    "eef",
    "end_effector",
    "tcp",
    "tool",
)

WORK_SURFACE_TOKENS = (
    "belt",
    "conveyor",
    "table",
    "surface",
    "platform",
    "tray",
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
        "--visible-site-radius",
        type=float,
        default=0.008,
        help="Named reference site radius above which visible sites are warned about.",
    )
    parser.add_argument(
        "--visible-site-alpha",
        type=float,
        default=0.35,
        help="Named reference site alpha above which visible sites are warned about.",
    )
    parser.add_argument(
        "--warn-only",
        action="store_true",
        help="Print errors but exit with status 0.",
    )
    parser.add_argument(
        "--key",
        help=(
            "Optional keyframe name, id, or 'first' to load before checks. "
            "Use this for ready/presentation poses."
        ),
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


def body_id_by_name(mujoco: object, model: object, name: str) -> int | None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    return body_id if body_id >= 0 else None


def model_name_blob(mujoco: object, model: object) -> str:
    names = []
    for body_id in range(1, model.nbody):
        names.append(body_name(mujoco, model, body_id))
    for geom_id in range(model.ngeom):
        names.append(geom_name(mujoco, model, geom_id))
    for site_id in range(model.nsite):
        names.append(site_name(mujoco, model, site_id))
    return name_blob(*names)


def geom_name(mujoco: object, model: object, geom_id: int) -> str:
    return object_name(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)


def joint_name(mujoco: object, model: object, joint_id: int) -> str:
    return object_name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)


def site_name(mujoco: object, model: object, site_id: int) -> str:
    return object_name(mujoco, model, mujoco.mjtObj.mjOBJ_SITE, site_id)


def resolve_keyframe_id(mujoco: object, model: object, selector: str | None) -> int | None:
    if selector is None:
        return None
    if model.nkey < 1:
        raise ValueError(f"Cannot load keyframe '{selector}': model has no keyframes.")

    stripped = selector.strip()
    if stripped == "first":
        return 0
    if stripped.lstrip("-").isdigit():
        key_id = int(stripped)
        if key_id < 0 or key_id >= model.nkey:
            raise IndexError(f"keyframe id {key_id} out of range [0, {model.nkey})")
        return key_id

    for key_id in range(model.nkey):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_id)
        if name == stripped:
            return key_id
    raise KeyError(f"Unknown keyframe name: {selector}")


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


def check_visible_reference_sites(
    mujoco: object,
    model: object,
    site_radius: float,
    site_alpha: float,
) -> list[Issue]:
    issues = []
    for site_id in range(model.nsite):
        name = site_name(mujoco, model, site_id)
        blob = name.lower()
        if not contains_any(blob, VISIBLE_REFERENCE_SITE_TOKENS):
            continue
        radius = float(np.max(model.site_size[site_id]))
        alpha = float(model.site_rgba[site_id][3])
        if radius <= site_radius or alpha <= site_alpha:
            continue
        issues.append(
            Issue(
                severity="warning",
                code="VISIBLE_REFERENCE_SITE",
                message=(
                    f"Reference site {name} has size {radius:.3f} m and alpha "
                    f"{alpha:.2f}. In viewer screenshots this can look like a "
                    f"physical ball; make it tiny/transparent or omit it if it "
                    f"is not needed for the requested task."
                ),
            )
        )
    return issues


def geom_half_height(model: object, geom_id: int) -> float:
    geom_type = int(model.geom_type[geom_id])
    size = model.geom_size[geom_id]
    try:
        import mujoco
    except Exception:
        return float(np.max(size))

    if geom_type == int(mujoco.mjtGeom.mjGEOM_PLANE):
        return 0.0
    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        return float(size[2])
    if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return float(size[0])
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        return float(size[1])
    if geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        return float(size[1] + size[0])
    return float(np.max(size))


def named_geom_ids_containing(mujoco: object, model: object, token: str) -> list[int]:
    matches = []
    needle = token.lower()
    for geom_id in range(model.ngeom):
        if needle in geom_name(mujoco, model, geom_id).lower():
            matches.append(geom_id)
    return matches


def named_site_ids_containing(mujoco: object, model: object, token: str) -> list[int]:
    matches = []
    needle = token.lower()
    for site_id in range(model.nsite):
        if needle in site_name(mujoco, model, site_id).lower():
            matches.append(site_id)
    return matches


def has_named_end_effector(mujoco: object, model: object) -> bool:
    blob = model_name_blob(mujoco, model)
    return contains_any(blob, END_EFFECTOR_TOKENS)


def scene_has_manipulation_semantics(mujoco: object, model: object) -> bool:
    return contains_any(model_name_blob(mujoco, model), MANIPULATION_TOKENS)


def check_end_effector_presence_for_manipulation(
    mujoco: object,
    model: object,
) -> list[Issue]:
    if not scene_has_manipulation_semantics(mujoco, model):
        return []
    if has_named_end_effector(mujoco, model):
        return []
    return [
        Issue(
            severity="error",
            code="MISSING_END_EFFECTOR_FOR_MANIPULATION",
            message=(
                "This scene appears to involve manipulation or sorting, but no "
                "named gripper, finger, hand, TCP, or end-effector was found. "
                "Many industrial-arm models do not include a gripper by default; "
                "do not claim a grasping/sorting setup until the tool is chosen "
                "or explicitly modeled."
            ),
        )
    ]


def work_surface_top_z(mujoco: object, model: object, data: object) -> float | None:
    candidates = []
    for token in WORK_SURFACE_TOKENS:
        candidates.extend(named_geom_ids_containing(mujoco, model, token))
    candidates = sorted(set(candidates))
    if not candidates:
        return None
    tops = [
        float(data.geom_xpos[geom_id][2]) + geom_half_height(model, geom_id)
        for geom_id in candidates
    ]
    return max(tops)


def check_end_effector_work_surface_clearance(
    mujoco: object,
    model: object,
    data: object,
    clearance: float = 0.02,
) -> list[Issue]:
    top_z = work_surface_top_z(mujoco, model, data)
    if top_z is None:
        return []

    issues = []
    min_allowed_z = top_z + clearance
    tool_site_ids = []
    tool_geom_ids = []
    for token in END_EFFECTOR_TOKENS:
        tool_site_ids.extend(named_site_ids_containing(mujoco, model, token))
        tool_geom_ids.extend(named_geom_ids_containing(mujoco, model, token))
    tool_site_ids = sorted(set(tool_site_ids))
    tool_geom_ids = sorted(set(tool_geom_ids))

    for site_id in tool_site_ids:
        site_z = float(data.site_xpos[site_id][2])
        if site_z >= min_allowed_z:
            continue
        issues.append(
            Issue(
                severity="error",
                code="END_EFFECTOR_BELOW_WORK_SURFACE",
                message=(
                    f"End-effector reference site {site_name(mujoco, model, site_id)} "
                    f"is at z={site_z:.3f} m, below the required clearance over "
                    f"the work surface top z={top_z:.3f} m. The ready pose or "
                    f"tool mounting is likely under or through the conveyor/table."
                ),
            )
        )

    for geom_id in tool_geom_ids:
        geom_z = float(data.geom_xpos[geom_id][2])
        half_height = geom_half_height(model, geom_id)
        bottom_z = geom_z - half_height
        if bottom_z >= top_z - 0.005:
            continue
        issues.append(
            Issue(
                severity="error",
                code="END_EFFECTOR_GEOM_BELOW_WORK_SURFACE",
                message=(
                    f"End-effector geom {geom_name(mujoco, model, geom_id)} has "
                    f"bottom z={bottom_z:.3f} m below work surface top z={top_z:.3f} m. "
                    f"Check the tool orientation, tool length, and ready pose."
                ),
            )
        )
    return issues


def named_body_ids_containing(mujoco: object, model: object, token: str) -> list[int]:
    matches = []
    needle = token.lower()
    for body_id in range(1, model.nbody):
        if needle in body_name(mujoco, model, body_id).lower():
            matches.append(body_id)
    return matches


def check_sorting_workcell_layout(
    mujoco: object,
    model: object,
    data: object,
) -> list[Issue]:
    conveyor_id = body_id_by_name(mujoco, model, "conveyor")
    base_id = body_id_by_name(mujoco, model, "base")
    bin_ids = named_body_ids_containing(mujoco, model, "bin")
    object_ids = [
        body_id
        for body_id in named_body_ids_containing(mujoco, model, "cube")
        if "geom" not in body_name(mujoco, model, body_id).lower()
    ]

    if conveyor_id is None or base_id is None or len(bin_ids) < 2 or len(object_ids) < 2:
        return []

    mujoco.mj_forward(model, data)
    conveyor_xy = np.array(data.xpos[conveyor_id][:2], dtype=float)
    base_xy = np.array(data.xpos[base_id][:2], dtype=float)
    base_center_offset = abs(float(base_xy[0] - conveyor_xy[0]))

    issues = []
    if base_center_offset > 0.25:
        issues.append(
            Issue(
                severity="error",
                code="SORTING_BASE_OFF_CENTER",
                message=(
                    f"Robot base {body_name(mujoco, model, base_id)} is "
                    f"{base_center_offset:.3f} m from the conveyor centerline in x. "
                    f"For conveyor sorting scenes, place the fixed arm near the "
                    f"active belt midline."
                ),
            )
        )

    for body_id in bin_ids + object_ids:
        body = body_name(mujoco, model, body_id)
        xy = np.array(data.xpos[body_id][:2], dtype=float)
        distance = float(np.linalg.norm(xy - base_xy))
        limit = 0.90 if "bin" in body.lower() else 0.80
        if distance <= limit:
            continue
        issues.append(
            Issue(
                severity="error",
                code="SORTING_TARGET_TOO_FAR",
                message=(
                    f"Sorting target body {body} is {distance:.3f} m from the "
                    f"robot base. Keep cubes and bins comfortably inside the "
                    f"UR5e work area instead of at the far edge of reach."
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
        key_id = resolve_keyframe_id(mujoco, model, args.key)
        if key_id is not None:
            mujoco.mj_resetDataKeyframe(model, data, key_id)
            mujoco.mj_forward(model, data)

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
        issues.extend(check_end_effector_presence_for_manipulation(mujoco, model))
        issues.extend(check_end_effector_work_surface_clearance(mujoco, model, data))
        issues.extend(
            check_visible_reference_sites(
                mujoco,
                model,
                site_radius=args.visible_site_radius,
                site_alpha=args.visible_site_alpha,
            )
        )
        issues.extend(check_free_robot_roots(mujoco, model))
        issues.extend(check_sorting_workcell_layout(mujoco, model, data))

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
