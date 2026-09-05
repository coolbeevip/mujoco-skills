"""Internal versioned JSON I/O for taught dual-PiPER trajectories."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


SCHEMA_NAME = "dual_piper_pick_place_trajectory"
SCHEMA_VERSION = 1


def scene_sha256(scene_path: Path) -> str:
    digest = hashlib.sha256()
    with scene_path.open("rb") as scene_file:
        for chunk in iter(lambda: scene_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_trajectory(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def load_trajectory(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("Trajectory JSON root must be an object.")
    if document.get("schema") != SCHEMA_NAME:
        raise ValueError(
            f"Unsupported trajectory schema: {document.get('schema')!r}."
        )
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported trajectory schema version: "
            f"{document.get('schema_version')!r}."
        )
    for key in ("scene", "task", "actuators", "poses", "trajectory", "teaching"):
        if key not in document:
            raise ValueError(f"Trajectory JSON is missing {key!r}.")
    return document


def resolve_scene_path(
    document: dict[str, Any],
    trajectory_path: Path,
    override: Path | None,
) -> Path:
    if override is not None:
        return override.expanduser().resolve()
    recorded = Path(str(document["scene"]["file"])).expanduser()
    if recorded.is_absolute():
        return recorded.resolve()
    return (trajectory_path.parent / recorded).resolve()
