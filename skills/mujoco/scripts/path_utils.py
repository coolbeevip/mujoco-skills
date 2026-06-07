#!/usr/bin/env python3
"""Path helpers for scene-grouped MuJoCo assets."""

from __future__ import annotations

from pathlib import Path


MUJOCO_ROOT = Path.home() / "Documents" / "mujoco"


def list_scene_groups() -> list[str]:
    if not MUJOCO_ROOT.exists():
        return []
    return sorted(
        entry.name
        for entry in MUJOCO_ROOT.iterdir()
        if entry.is_dir()
    )


def resolve_scene_model_path(model: str | None) -> Path:
    if model:
        path = Path(model).expanduser()
        if path.exists():
            return path.resolve()

        candidate = MUJOCO_ROOT / model
        if candidate.exists():
            return candidate.resolve()

        matches = sorted(MUJOCO_ROOT.rglob(model))
        if len(matches) == 1:
            return matches[0].resolve()
        if len(matches) > 1:
            raise FileNotFoundError(
                f"Multiple scene models match '{model}'. Use a more specific path."
            )
        raise FileNotFoundError(f"Model not found: {model}")

    xml_files = sorted(MUJOCO_ROOT.rglob("*.xml"), key=lambda p: p.stat().st_mtime)
    if not xml_files:
        raise FileNotFoundError(
            f"No .xml models found under {MUJOCO_ROOT}. "
            "Pass --model or create a model there first."
        )
    return xml_files[-1].resolve()


def infer_scene_group(model_path: str | Path) -> str | None:
    path = Path(model_path).expanduser().resolve()
    try:
        relative = path.relative_to(MUJOCO_ROOT.resolve())
    except ValueError:
        return None

    parts = relative.parts
    if len(parts) >= 2:
        return parts[0]
    return None


def scene_script_dir(model_path: str | Path) -> Path | None:
    group = infer_scene_group(model_path)
    if not group:
        return None
    return MUJOCO_ROOT / group / "scripts"
