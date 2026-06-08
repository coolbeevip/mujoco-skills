#!/usr/bin/env python3
"""Run a minimal MuJoCo grasp-chain check for manipulation scenes."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


GRIPPER_TOKENS = (
    "gripper",
    "grip",
    "finger",
    "claw",
    "hand",
    "jaw",
)

OBJECT_TOKENS = (
    "cube",
    "block",
    "object",
    "target",
)


@dataclass(frozen=True)
class CheckResult:
    status: str
    code: str
    message: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Load a MuJoCo scene and verify that a named gripper actuator can "
            "make contact with a target object. Optional lift controls can also "
            "verify that the target rises after closing."
        )
    )
    parser.add_argument("scene", help="Path to the MuJoCo XML scene file.")
    parser.add_argument(
        "--key",
        help="Optional keyframe name, id, or 'first' to load before the check.",
    )
    parser.add_argument(
        "--object",
        dest="target",
        help=(
            "Target body or geom name. If omitted, names containing cube, block, "
            "object, or target are used."
        ),
    )
    parser.add_argument(
        "--gripper-actuator",
        action="append",
        help=(
            "Gripper actuator name or id. May be repeated. If omitted, actuator "
            "names containing gripper, grip, finger, claw, hand, or jaw are used."
        ),
    )
    parser.add_argument(
        "--open-value",
        type=float,
        help=(
            "Control value used to open the gripper. If omitted, the high end of "
            "the first gripper actuator ctrlrange is used."
        ),
    )
    parser.add_argument(
        "--close-value",
        type=float,
        help=(
            "Control value used to close the gripper. If omitted, the low end of "
            "the first gripper actuator ctrlrange is used."
        ),
    )
    parser.add_argument(
        "--close-steps",
        type=int,
        default=240,
        help="Simulation steps after applying close controls. Default: 240.",
    )
    parser.add_argument(
        "--open-steps",
        type=int,
        default=160,
        help="Simulation steps after applying open controls. Default: 160.",
    )
    parser.add_argument(
        "--lift",
        action="append",
        default=[],
        metavar="ACTUATOR=VALUE",
        help=(
            "Additional actuator control applied after closing to test lift. "
            "May be repeated."
        ),
    )
    parser.add_argument(
        "--lift-steps",
        type=int,
        default=240,
        help="Simulation steps after applying lift controls. Default: 240.",
    )
    parser.add_argument(
        "--min-lift-z",
        type=float,
        default=0.02,
        help="Minimum target z increase in meters for lift PASS. Default: 0.02.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print a JSON report instead of text.",
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


def actuator_name(mujoco: object, model: object, actuator_id: int) -> str:
    return object_name(mujoco, model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)


def contains_any(text: str, tokens: Iterable[str]) -> bool:
    lowered = text.lower()
    return any(token in lowered for token in tokens)


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


def named_body_id(mujoco: object, model: object, name: str) -> int | None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    return body_id if body_id >= 0 else None


def named_geom_id(mujoco: object, model: object, name: str) -> int | None:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    return geom_id if geom_id >= 0 else None


def descendant_body_ids(model: object, ancestor_id: int) -> set[int]:
    descendants = {ancestor_id}
    for body_id in range(1, model.nbody):
        current = body_id
        while current > 0:
            if current == ancestor_id:
                descendants.add(body_id)
                break
            current = int(model.body_parentid[current])
    return descendants


def resolve_target_body_id(
    mujoco: object,
    model: object,
    target: str | None,
) -> int:
    if target:
        body_id = named_body_id(mujoco, model, target)
        if body_id is not None:
            return body_id
        geom_id = named_geom_id(mujoco, model, target)
        if geom_id is not None:
            return int(model.geom_bodyid[geom_id])
        raise KeyError(f"Target object was not found as a body or geom: {target}")

    matches = [
        body_id
        for body_id in range(1, model.nbody)
        if contains_any(body_name(mujoco, model, body_id), OBJECT_TOKENS)
    ]
    if len(matches) == 1:
        return matches[0]
    if matches:
        names = ", ".join(body_name(mujoco, model, body_id) for body_id in matches)
        raise ValueError(
            "Multiple possible target objects found; pass --object explicitly: "
            + names
        )
    raise ValueError(
        "No target object found. Pass --object with a body or geom name."
    )


def resolve_actuator_id(mujoco: object, model: object, selector: str) -> int:
    stripped = selector.strip()
    if stripped.lstrip("-").isdigit():
        actuator_id = int(stripped)
        if 0 <= actuator_id < model.nu:
            return actuator_id
        raise IndexError(f"actuator id {actuator_id} out of range [0, {model.nu})")

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, stripped)
    if actuator_id >= 0:
        return actuator_id
    raise KeyError(f"Unknown actuator name: {selector}")


def resolve_gripper_actuator_ids(
    mujoco: object,
    model: object,
    selectors: list[str] | None,
) -> list[int]:
    if selectors:
        return [resolve_actuator_id(mujoco, model, selector) for selector in selectors]

    matches = [
        actuator_id
        for actuator_id in range(model.nu)
        if contains_any(actuator_name(mujoco, model, actuator_id), GRIPPER_TOKENS)
    ]
    if not matches:
        raise ValueError(
            "No gripper-like actuator found. Pass --gripper-actuator explicitly."
        )
    return matches


def resolve_control_values(
    model: object,
    actuator_ids: list[int],
    open_value: float | None,
    close_value: float | None,
) -> tuple[float, float]:
    first_id = actuator_ids[0]
    if open_value is None or close_value is None:
        if not bool(model.actuator_ctrllimited[first_id]):
            raise ValueError(
                "Open/close values were not provided and the first gripper actuator "
                "has no ctrlrange. Pass --open-value and --close-value explicitly."
            )
        low, high = [float(value) for value in model.actuator_ctrlrange[first_id]]
        if open_value is None:
            open_value = high
        if close_value is None:
            close_value = low
    return float(open_value), float(close_value)


def clamp_ctrl(model: object, actuator_id: int, value: float) -> float:
    if bool(model.actuator_ctrllimited[actuator_id]):
        low, high = model.actuator_ctrlrange[actuator_id]
        return float(max(low, min(high, value)))
    return float(value)


def apply_controls(model: object, data: object, assignments: dict[int, float]) -> None:
    for actuator_id, value in assignments.items():
        data.ctrl[actuator_id] = clamp_ctrl(model, actuator_id, value)


def parse_lift_assignments(
    mujoco: object,
    model: object,
    raw_assignments: list[str],
) -> dict[int, float]:
    assignments: dict[int, float] = {}
    for raw in raw_assignments:
        if "=" not in raw:
            raise ValueError(f"Invalid --lift assignment {raw!r}; use ACTUATOR=VALUE.")
        selector, value = raw.split("=", 1)
        actuator_id = resolve_actuator_id(mujoco, model, selector)
        assignments[actuator_id] = float(value)
    return assignments


def run_steps(mujoco: object, model: object, data: object, steps: int) -> None:
    for _ in range(max(0, steps)):
        mujoco.mj_step(model, data)


def target_geom_ids(model: object, target_body_id: int) -> set[int]:
    body_ids = descendant_body_ids(model, target_body_id)
    return {
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) in body_ids
        and geom_contact_enabled(model, geom_id)
    }


def geom_contact_enabled(model: object, geom_id: int) -> bool:
    return (
        int(model.geom_contype[geom_id]) != 0
        or int(model.geom_conaffinity[geom_id]) != 0
    )


def gripper_geom_ids(mujoco: object, model: object) -> set[int]:
    matches = set()
    for geom_id in range(model.ngeom):
        if not geom_contact_enabled(model, geom_id):
            continue
        owning_body = int(model.geom_bodyid[geom_id])
        blob = f"{geom_name(mujoco, model, geom_id)} {body_name(mujoco, model, owning_body)}"
        if contains_any(blob, GRIPPER_TOKENS):
            matches.add(geom_id)
    return matches


def contact_pairs(
    mujoco: object,
    model: object,
    data: object,
    left_geoms: set[int],
    right_geoms: set[int],
) -> list[dict[str, Any]]:
    pairs = []
    mujoco.mj_forward(model, data)
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        if (geom1 in left_geoms and geom2 in right_geoms) or (
            geom2 in left_geoms and geom1 in right_geoms
        ):
            pairs.append(
                {
                    "geom1": geom_name(mujoco, model, geom1),
                    "geom2": geom_name(mujoco, model, geom2),
                    "distance": float(contact.dist),
                }
            )
    return pairs


def body_has_free_joint(mujoco: object, model: object, body_id: int) -> bool:
    start = int(model.body_jntadr[body_id])
    count = int(model.body_jntnum[body_id])
    for joint_id in range(start, start + count):
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            return True
    return False


def run_grasp_check(args: argparse.Namespace) -> tuple[dict[str, Any], int]:
    import mujoco

    scene_path = resolve_scene_path(args.scene)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    key_id = resolve_keyframe_id(mujoco, model, args.key)
    if key_id is not None:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    target_body_id = resolve_target_body_id(mujoco, model, args.target)
    target_name = body_name(mujoco, model, target_body_id)
    target_geoms = target_geom_ids(model, target_body_id)
    gripper_geoms = gripper_geom_ids(mujoco, model)
    gripper_actuators = resolve_gripper_actuator_ids(
        mujoco,
        model,
        args.gripper_actuator,
    )
    open_value, close_value = resolve_control_values(
        model,
        gripper_actuators,
        args.open_value,
        args.close_value,
    )

    results: list[CheckResult] = []
    if target_geoms:
        results.append(
            CheckResult(
                "pass",
                "TARGET_GEOMS_FOUND",
                f"Target {target_name} has {len(target_geoms)} contact-enabled geom(s).",
            )
        )
    else:
        results.append(
            CheckResult(
                "fail",
                "TARGET_GEOMS_MISSING",
                f"Target {target_name} has no contact-enabled geoms.",
            )
        )

    if gripper_geoms:
        results.append(
            CheckResult(
                "pass",
                "GRIPPER_GEOMS_FOUND",
                f"Found {len(gripper_geoms)} contact-enabled gripper-like geom(s).",
            )
        )
    else:
        results.append(
            CheckResult(
                "fail",
                "GRIPPER_GEOMS_MISSING",
                "No gripper-like geoms were found by name.",
            )
        )

    if body_has_free_joint(mujoco, model, target_body_id):
        results.append(
            CheckResult(
                "pass",
                "TARGET_HAS_FREEJOINT",
                f"Target {target_name} can move independently.",
            )
        )
    else:
        results.append(
            CheckResult(
                "warning",
                "TARGET_NOT_FREE",
                f"Target {target_name} has no free joint; lift/release checks may be limited.",
            )
        )

    apply_controls(model, data, {actuator_id: open_value for actuator_id in gripper_actuators})
    run_steps(mujoco, model, data, max(1, min(args.open_steps, 80)))
    initial_z = float(data.xpos[target_body_id][2])

    apply_controls(model, data, {actuator_id: close_value for actuator_id in gripper_actuators})
    run_steps(mujoco, model, data, args.close_steps)
    close_contacts = contact_pairs(mujoco, model, data, target_geoms, gripper_geoms)
    closed_z = float(data.xpos[target_body_id][2])

    if close_contacts:
        results.append(
            CheckResult(
                "pass",
                "CLOSE_CONTACT",
                f"Close command produced {len(close_contacts)} target/gripper contact(s).",
            )
        )
    else:
        results.append(
            CheckResult(
                "fail",
                "NO_CLOSE_CONTACT",
                "Close command did not produce target/gripper contact.",
            )
        )

    lift_assignments = parse_lift_assignments(mujoco, model, args.lift)
    lifted_z = None
    lift_contacts: list[dict[str, Any]] = []
    if lift_assignments:
        apply_controls(model, data, lift_assignments)
        run_steps(mujoco, model, data, args.lift_steps)
        lifted_z = float(data.xpos[target_body_id][2])
        lift_contacts = contact_pairs(mujoco, model, data, target_geoms, gripper_geoms)
        lift_delta = lifted_z - closed_z
        if lift_delta >= args.min_lift_z:
            results.append(
                CheckResult(
                    "pass",
                    "LIFT_Z_INCREASED",
                    f"Target z increased {lift_delta:.4f} m after lift controls.",
                )
            )
        else:
            results.append(
                CheckResult(
                    "fail",
                    "LIFT_Z_NOT_INCREASED",
                    f"Target z increased only {lift_delta:.4f} m after lift controls.",
                )
            )
        if lift_contacts:
            results.append(
                CheckResult(
                    "pass",
                    "LIFT_CONTACT_MAINTAINED",
                    f"Target/gripper contact remained during lift ({len(lift_contacts)} contact(s)).",
                )
            )
        else:
            results.append(
                CheckResult(
                    "warning",
                    "LIFT_CONTACT_LOST",
                    "No target/gripper contact remained after lift controls.",
                )
            )
    else:
        results.append(
            CheckResult(
                "skip",
                "LIFT_NOT_REQUESTED",
                "No --lift controls were provided; lift/pick validation was not performed.",
            )
        )

    apply_controls(model, data, {actuator_id: open_value for actuator_id in gripper_actuators})
    run_steps(mujoco, model, data, args.open_steps)
    release_contacts = contact_pairs(mujoco, model, data, target_geoms, gripper_geoms)
    released_z = float(data.xpos[target_body_id][2])
    if release_contacts:
        results.append(
            CheckResult(
                "fail",
                "RELEASE_CONTACT_REMAINS",
                f"Open command left {len(release_contacts)} target/gripper contact(s).",
            )
        )
    else:
        results.append(
            CheckResult(
                "pass",
                "RELEASED",
                "Open command removed target/gripper contact.",
            )
        )

    actuator_report = [
        {
            "id": actuator_id,
            "name": actuator_name(mujoco, model, actuator_id),
            "ctrlrange": (
                [float(value) for value in model.actuator_ctrlrange[actuator_id]]
                if bool(model.actuator_ctrllimited[actuator_id])
                else None
            ),
        }
        for actuator_id in gripper_actuators
    ]
    report = {
        "scene": str(scene_path),
        "key": args.key,
        "target": {
            "body_id": target_body_id,
            "name": target_name,
            "initial_z": initial_z,
            "closed_z": closed_z,
            "lifted_z": lifted_z,
            "released_z": released_z,
        },
        "gripper_actuators": actuator_report,
        "open_value": open_value,
        "close_value": close_value,
        "contacts": {
            "close": close_contacts,
            "lift": lift_contacts,
            "release": release_contacts,
        },
        "results": [result.__dict__ for result in results],
    }
    has_failure = any(result.status == "fail" for result in results)
    return report, 1 if has_failure else 0


def print_text_report(report: dict[str, Any]) -> None:
    print(f"Scene: {report['scene']}")
    if report.get("key"):
        print(f"Keyframe: {report['key']}")
    target = report["target"]
    print(f"Target: {target['name']} (body {target['body_id']})")
    actuator_names = ", ".join(
        actuator["name"] for actuator in report["gripper_actuators"]
    )
    print(f"Gripper actuator(s): {actuator_names}")
    print(f"Open value: {report['open_value']}")
    print(f"Close value: {report['close_value']}")
    print("Results:")
    for result in report["results"]:
        print(f"- {result['status'].upper()} {result['code']}: {result['message']}")


def main() -> int:
    args = parse_args()
    try:
        report, exit_code = run_grasp_check(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        print_text_report(report)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
