#!/usr/bin/env python3
"""Render MuJoCo scenes from standard inspection views."""

from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path

import numpy as np


STANDARD_VIEWS = {
    "top": (0.0, -90.0),
    "bottom": (0.0, 90.0),
    "front": (0.0, -12.0),
    "back": (180.0, -12.0),
    "left": (-90.0, -12.0),
    "right": (90.0, -12.0),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render standard MuJoCo inspection views to PNG files."
    )
    parser.add_argument("scene", help="Path to the MuJoCo XML scene file.")
    parser.add_argument(
        "--out-dir",
        default="/tmp/mujoco-render-views",
        help="Directory where rendered PNG files will be written.",
    )
    parser.add_argument(
        "--key",
        help="Optional keyframe name, id, or 'first' to load before rendering.",
    )
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1200)
    parser.add_argument(
        "--distance-scale",
        type=float,
        default=1.55,
        help="Multiplier applied to model.stat.extent for free-camera distance.",
    )
    parser.add_argument(
        "--views",
        nargs="+",
        choices=sorted(STANDARD_VIEWS),
        default=list(STANDARD_VIEWS),
        help="Views to render. Default: top bottom front back left right.",
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


def write_png(path: Path, image: np.ndarray) -> None:
    if image.dtype != np.uint8:
        raise TypeError("PNG image array must use uint8 pixels.")
    if image.ndim != 3 or image.shape[2] not in {3, 4}:
        raise ValueError("PNG image array must have RGB or RGBA shape.")

    height, width, channels = image.shape
    color_type = 2 if channels == 3 else 6
    raw_rows = b"".join(
        b"\x00" + image[row].tobytes()
        for row in range(height)
    )

    def chunk(kind: bytes, payload: bytes) -> bytes:
        checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", checksum)
        )

    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0),
        )
        + chunk(b"IDAT", zlib.compress(raw_rows, level=6))
        + chunk(b"IEND", b"")
    )
    path.write_bytes(png)


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


def render_views(args: argparse.Namespace) -> list[Path]:
    import mujoco

    scene_path = resolve_scene_path(args.scene)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)
    key_id = resolve_keyframe_id(mujoco, model, args.key)
    if key_id is not None:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    rendered_paths = []
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    try:
        for view_name in args.views:
            azimuth, elevation = STANDARD_VIEWS[view_name]
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = model.stat.center
            camera.distance = float(model.stat.extent) * args.distance_scale
            camera.azimuth = azimuth
            camera.elevation = elevation
            renderer.update_scene(data, camera=camera)
            image = renderer.render()
            out_path = out_dir / f"{scene_path.stem}_{view_name}.png"
            write_png(out_path, image)
            rendered_paths.append(out_path)
    finally:
        renderer.close()
    return rendered_paths


def main() -> int:
    args = parse_args()
    for path in render_views(args):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
