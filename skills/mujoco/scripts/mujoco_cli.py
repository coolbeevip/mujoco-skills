"""MuJoCo viewer control CLI.

This script talks to a running ``mujoco_viewer.py`` process over a local
UNIX socket, so you can inspect and control the robot from another terminal
without closing the viewer window.

Typical workflow:

1. Start the viewer service first:

   python mujoco_viewer.py /path/to/scene.xml

2. Open another terminal and send commands with either:
   - ``--scene /path/to/scene.xml`` to derive the socket path automatically
   - ``--socket /tmp/mujoco-viewer-xxxx.sock`` to connect directly

Common commands:

- Check whether the viewer is alive:

  python mujoco_cli.py --scene /path/to/scene.xml ping

- Show scene path, socket path, sim time, actuator count, and current ctrl:

  python mujoco_cli.py --scene /path/to/scene.xml info

- List all actuators, including id, name, trnid, and ctrlrange:

  python mujoco_cli.py --scene /path/to/scene.xml actuators

- Read the current control vector:

  python mujoco_cli.py --scene /path/to/scene.xml ctrl

- Set one actuator by numeric id:

  python mujoco_cli.py --scene /path/to/scene.xml set 0 0.5

- Set one actuator by actuator name:

  python mujoco_cli.py --scene /path/to/scene.xml set shoulder_pan 0.5

- Set multiple actuators in one synchronized update:

  python mujoco_cli.py --scene /path/to/scene.xml set-batch shoulder_pan 0.5 elbow_flex 1.2 wrist_roll -0.4

- Advance the simulation by one step:

  python mujoco_cli.py --scene /path/to/scene.xml step

- Advance the simulation by many steps:

  python mujoco_cli.py --scene /path/to/scene.xml step 200

- Reset the simulation state:

  python mujoco_cli.py --scene /path/to/scene.xml reset

- Use the socket path printed by the viewer instead of the scene path:

  python mujoco_cli.py --socket /tmp/mujoco-viewer-1234567890abcdef.sock info

Practical examples:

- Inspect all available actuators, then move the gripper:

  python mujoco_cli.py --scene /path/to/scene.xml actuators
  python mujoco_cli.py --scene /path/to/scene.xml set gripper 0.2

- Move a joint and let the simulation run for a short while:

  python mujoco_cli.py --scene /path/to/scene.xml set wrist_flex 0.4
  python mujoco_cli.py --scene /path/to/scene.xml step 120

- Reset after a test:

  python mujoco_cli.py --scene /path/to/scene.xml reset

Notes:

- ``mujoco_viewer.py`` must already be running for the same scene or socket.
- ``set`` writes ``data.ctrl[actuator]`` and respects MuJoCo ``ctrlrange``.
- ``set-batch`` applies all listed actuator updates before the next viewer sync, so coordinated motions look simultaneous.
- ``step`` advances the loaded simulation state inside the running viewer.
- The response is printed as formatted JSON so it is easy to inspect or pipe.
"""

import argparse
import json
import socket
from pathlib import Path
from typing import Any

from mujoco_viewer import control_socket_path, resolve_scene_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send control commands to a running mujoco_viewer.py process."
    )
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        "--scene",
        help="Path to the scene XML used by mujoco_viewer.py.",
    )
    target_group.add_argument(
        "--socket",
        help="Explicit UNIX socket path printed by mujoco_viewer.py.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("ping", help="Check whether the viewer control socket is alive.")
    subparsers.add_parser("info", help="Show viewer and simulation info.")
    subparsers.add_parser("actuators", help="List available actuators.")
    subparsers.add_parser("ctrl", help="Show current control values.")
    subparsers.add_parser("reset", help="Reset the simulation state.")

    step_parser = subparsers.add_parser("step", help="Advance the simulation by N steps.")
    step_parser.add_argument("count", type=int, nargs="?", default=1)

    set_parser = subparsers.add_parser("set", help="Set one actuator control value.")
    set_parser.add_argument("actuator", help="Actuator id or actuator name.")
    set_parser.add_argument("value", type=float, help="Target control value.")

    set_batch_parser = subparsers.add_parser(
        "set-batch",
        help="Set multiple actuator controls in one synchronized update.",
    )
    set_batch_parser.add_argument(
        "assignments",
        nargs="+",
        help="Alternating actuator and value tokens: actuator1 value1 actuator2 value2 ...",
    )

    return parser.parse_args()


def resolve_socket_target(args: argparse.Namespace) -> Path:
    if args.socket:
        return Path(args.socket).expanduser().resolve()
    scene_path = resolve_scene_path(args.scene)
    return control_socket_path(scene_path)


def build_request(args: argparse.Namespace) -> dict[str, Any]:
    if args.command in {"ping", "info", "actuators", "ctrl", "reset"}:
        return {"command": args.command}
    if args.command == "step":
        return {"command": "step", "count": args.count}
    if args.command == "set":
        return {
            "command": "set",
            "actuator": args.actuator,
            "value": args.value,
        }
    if args.command == "set-batch":
        if len(args.assignments) % 2 != 0:
            raise ValueError(
                "set-batch expects alternating actuator/value pairs: actuator1 value1 actuator2 value2 ..."
            )
        assignments = []
        tokens = args.assignments
        for index in range(0, len(tokens), 2):
            assignments.append(
                {
                    "actuator": tokens[index],
                    "value": float(tokens[index + 1]),
                }
            )
        return {
            "command": "set_batch",
            "assignments": assignments,
        }
    raise ValueError(f"Unsupported CLI command: {args.command}")


def send_request(socket_path: Path, payload: dict[str, Any]) -> dict[str, Any]:
    if not socket_path.exists():
        raise FileNotFoundError(
            f"Viewer control socket not found: {socket_path}. Start mujoco_viewer.py first."
        )

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.connect(str(socket_path))
        client.sendall((json.dumps(payload) + "\n").encode("utf-8"))

        chunks = []
        while True:
            chunk = client.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break

    if not chunks:
        raise RuntimeError("Viewer control socket closed without a response.")

    return json.loads(b"".join(chunks).decode("utf-8").strip())


def print_response(response: dict[str, Any]) -> None:
    if not response.get("ok"):
        raise RuntimeError(response.get("error", "Unknown viewer control error."))

    result = response.get("result")
    if isinstance(result, str):
        print(result)
        return
    print(json.dumps(result, indent=2, ensure_ascii=False))


def main() -> None:
    args = parse_args()
    socket_path = resolve_socket_target(args)
    payload = build_request(args)
    response = send_request(socket_path, payload)
    print_response(response)


if __name__ == "__main__":
    main()
