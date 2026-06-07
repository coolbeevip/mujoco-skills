import argparse
import hashlib
import importlib.util
import json
import os
import platform
import queue
import site
import socket
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


MJPYTHON_RELAUNCH_FLAG = "MUJOCO_VIEWER_UNDER_MJPYTHON"


def ensure_mjpython_for_macos() -> None:
    """MuJoCo GUI tools on macOS must run under mjpython."""
    if platform.system() != "Darwin":
        return

    if os.environ.get(MJPYTHON_RELAUNCH_FLAG) == "1":
        return

    mujoco_spec = importlib.util.find_spec("mujoco")
    if mujoco_spec is None or mujoco_spec.origin is None:
        raise RuntimeError(
            "Could not locate the installed mujoco package needed for mjpython."
        )

    real_python = Path(sys.executable).resolve()
    mjpython_script = Path(sys.executable).with_name("mjpython")
    if not mjpython_script.exists():
        raise RuntimeError(
            "MuJoCo GUI tools require the mjpython launcher script on macOS, "
            f"but it was not found at: {mjpython_script}"
        )

    env = os.environ.copy()
    env[MJPYTHON_RELAUNCH_FLAG] = "1"
    env["VIRTUAL_ENV"] = str(Path(sys.prefix))

    site_packages = []
    for path in site.getsitepackages():
        if Path(path).exists():
            site_packages.append(path)
    user_site = site.getusersitepackages()
    if Path(user_site).exists():
        site_packages.append(user_site)
    existing_pythonpath = env.get("PYTHONPATH")
    if existing_pythonpath:
        site_packages.append(existing_pythonpath)
    if site_packages:
        env["PYTHONPATH"] = os.pathsep.join(site_packages)

    subprocess.run(
        [str(real_python), str(mjpython_script), *sys.argv],
        check=True,
        env=env,
    )
    raise SystemExit(0)


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


def control_socket_path(scene_path: Path) -> Path:
    digest = hashlib.sha256(str(scene_path).encode("utf-8")).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / f"mujoco-viewer-{digest}.sock"


@dataclass
class ViewerCommand:
    payload: dict[str, Any]
    response_queue: "queue.Queue[dict[str, Any]]"


class ControlServer:
    def __init__(
        self,
        socket_path: Path,
        request_queue: "queue.Queue[ViewerCommand]",
    ) -> None:
        self.socket_path = socket_path
        self.request_queue = request_queue
        self._server_socket: socket.socket | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._remove_stale_socket()

        self._server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server_socket.bind(str(self.socket_path))
        self._server_socket.listen()
        self._server_socket.settimeout(0.5)

        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._server_socket is not None:
            self._server_socket.close()
            self._server_socket = None
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.socket_path.exists():
            self.socket_path.unlink()

    def _remove_stale_socket(self) -> None:
        if not self.socket_path.exists():
            return

        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(0.2)
                client.connect(str(self.socket_path))
        except OSError:
            self.socket_path.unlink()
            return

        raise RuntimeError(
            f"Viewer control socket is already in use: {self.socket_path}"
        )

    def _serve(self) -> None:
        assert self._server_socket is not None

        while not self._stop_event.is_set():
            try:
                connection, _ = self._server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with connection:
                response = self._handle_connection(connection)
                connection.sendall((json.dumps(response) + "\n").encode("utf-8"))

    def _handle_connection(self, connection: socket.socket) -> dict[str, Any]:
        try:
            payload = self._read_payload(connection)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}

        response_queue: "queue.Queue[dict[str, Any]]" = queue.Queue(maxsize=1)
        self.request_queue.put(ViewerCommand(payload=payload, response_queue=response_queue))
        return response_queue.get()

    def _read_payload(self, connection: socket.socket) -> dict[str, Any]:
        chunks = []
        while True:
            chunk = connection.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break

        if not chunks:
            raise ValueError("Empty request.")

        raw_request = b"".join(chunks).decode("utf-8").strip()
        try:
            payload = json.loads(raw_request)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid JSON request: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("Request payload must be a JSON object.")
        return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open the MuJoCo viewer and expose a local control socket."
    )
    parser.add_argument("scene", help="Path to the MuJoCo XML scene file.")
    parser.add_argument(
        "--key",
        help=(
            "Optional keyframe to load at startup and on reset. Use a keyframe "
            "name, numeric id, or 'first'."
        ),
    )
    parser.add_argument(
        "--socket",
        help="Optional explicit UNIX socket path for mujoco_cli.py to connect to.",
    )
    return parser.parse_args()


def clamp_ctrl(model: Any, actuator_id: int, value: float) -> float:
    if model.actuator_ctrllimited[actuator_id]:
        low, high = model.actuator_ctrlrange[actuator_id]
        return max(low, min(high, value))
    return value


def resolve_actuator_id(model: Any, selector: Any) -> int:
    import mujoco

    if isinstance(selector, int):
        actuator_id = selector
    elif isinstance(selector, str):
        stripped = selector.strip()
        if stripped.lstrip("-").isdigit():
            actuator_id = int(stripped)
        else:
            for actuator_id in range(model.nu):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
                if name == stripped:
                    return actuator_id
            raise KeyError(f"Unknown actuator name: {selector}")
    else:
        raise TypeError("Actuator selector must be an integer id or actuator name.")

    if actuator_id < 0 or actuator_id >= model.nu:
        raise IndexError(f"actuator_id {actuator_id} out of range [0, {model.nu})")
    return actuator_id


def actuator_info(model: Any, actuator_id: int) -> dict[str, Any]:
    import mujoco

    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
    ctrlrange = (
        [float(value) for value in model.actuator_ctrlrange[actuator_id]]
        if model.actuator_ctrllimited[actuator_id]
        else None
    )
    return {
        "id": actuator_id,
        "name": name,
        "trnid": model.actuator_trnid[actuator_id].tolist(),
        "ctrlrange": ctrlrange,
    }


def list_actuators(model: Any) -> list[dict[str, Any]]:
    return [actuator_info(model, actuator_id) for actuator_id in range(model.nu)]


def resolve_keyframe_id(model: Any, selector: str | None) -> int | None:
    if selector is None:
        return None
    if model.nkey < 1:
        raise ValueError(f"Cannot load keyframe '{selector}': model has no keyframes.")

    import mujoco

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


def keyframe_name(model: Any, key_id: int | None) -> str | None:
    if key_id is None:
        return None

    import mujoco

    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_id)
    return name if name else str(key_id)


def parse_batch_assignments(command: dict[str, Any]) -> list[tuple[Any, float]]:
    raw_assignments = command.get("assignments")
    if not isinstance(raw_assignments, list) or not raw_assignments:
        raise ValueError("set_batch requires a non-empty 'assignments' list.")

    parsed_assignments: list[tuple[Any, float]] = []
    for item in raw_assignments:
        if not isinstance(item, dict):
            raise TypeError("Each set_batch assignment must be an object.")
        if "actuator" not in item:
            raise ValueError("Each set_batch assignment requires an 'actuator' field.")
        if "value" not in item:
            raise ValueError("Each set_batch assignment requires a 'value' field.")
        parsed_assignments.append((item["actuator"], float(item["value"])))
    return parsed_assignments


def handle_command(
    command: dict[str, Any],
    scene_path: Path,
    socket_path: Path,
    model: Any,
    data: Any,
    initial_key_id: int | None = None,
    viewer_handle: Any | None = None,
) -> dict[str, Any]:
    import mujoco

    command_name = command.get("command")
    if not isinstance(command_name, str):
        raise ValueError("Request is missing string field 'command'.")

    if command_name == "ping":
        return {"ok": True, "result": {"message": "pong"}}

    if command_name == "info":
        return {
            "ok": True,
            "result": {
                "scene_path": str(scene_path),
                "socket_path": str(socket_path),
                "simulation_time": float(data.time),
                "actuator_count": model.nu,
                "loaded_keyframe": keyframe_name(model, initial_key_id),
                "ctrl": data.ctrl.tolist(),
            },
        }

    if command_name == "actuators":
        return {"ok": True, "result": {"actuators": list_actuators(model)}}

    if command_name == "ctrl":
        return {"ok": True, "result": {"ctrl": data.ctrl.tolist()}}

    if command_name == "set":
        actuator_id = resolve_actuator_id(model, command.get("actuator"))
        value = float(command["value"])
        clamped_value = float(clamp_ctrl(model, actuator_id, value))
        data.ctrl[actuator_id] = clamped_value
        if viewer_handle is not None:
            viewer_handle.sync()
        return {
            "ok": True,
            "result": {
                "actuator": actuator_info(model, actuator_id),
                "value": clamped_value,
                "ctrl": data.ctrl.tolist(),
            },
        }

    if command_name == "set_batch":
        applied = []
        for selector, requested_value in parse_batch_assignments(command):
            actuator_id = resolve_actuator_id(model, selector)
            clamped_value = float(clamp_ctrl(model, actuator_id, requested_value))
            data.ctrl[actuator_id] = clamped_value
            applied.append(
                {
                    "actuator": actuator_info(model, actuator_id),
                    "value": clamped_value,
                }
            )
        if viewer_handle is not None:
            viewer_handle.sync()
        return {
            "ok": True,
            "result": {
                "applied": applied,
                "ctrl": data.ctrl.tolist(),
            },
        }

    if command_name == "step":
        count = int(command.get("count", 1))
        if count < 1:
            raise ValueError("step count must be >= 1")
        for _ in range(count):
            mujoco.mj_step(model, data)
            if viewer_handle is not None:
                viewer_handle.sync()
                time.sleep(model.opt.timestep)
        return {
            "ok": True,
            "result": {
                "steps": count,
                "simulation_time": float(data.time),
                "ctrl": data.ctrl.tolist(),
            },
        }

    if command_name == "reset":
        if initial_key_id is not None:
            mujoco.mj_resetDataKeyframe(model, data, initial_key_id)
        else:
            mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        if viewer_handle is not None:
            viewer_handle.sync()
        return {
            "ok": True,
            "result": {
                "simulation_time": float(data.time),
                "ctrl": data.ctrl.tolist(),
            },
        }

    raise ValueError(f"Unsupported command: {command_name}")


def process_pending_commands(
    request_queue: "queue.Queue[ViewerCommand]",
    scene_path: Path,
    socket_path: Path,
    model: Any,
    data: Any,
    initial_key_id: int | None = None,
    viewer_handle: Any | None = None,
) -> None:
    while True:
        try:
            request = request_queue.get_nowait()
        except queue.Empty:
            return

        try:
            response = handle_command(
                request.payload,
                scene_path=scene_path,
                socket_path=socket_path,
                model=model,
                data=data,
                initial_key_id=initial_key_id,
                viewer_handle=viewer_handle,
            )
        except Exception as exc:
            response = {"ok": False, "error": str(exc)}
        request.response_queue.put(response)


def main() -> None:
    args = parse_args()
    scene_path = resolve_scene_path(args.scene)
    socket_path = (
        Path(args.socket).expanduser().resolve()
        if args.socket
        else control_socket_path(scene_path)
    )
    ensure_mjpython_for_macos()

    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    initial_key_id = resolve_keyframe_id(model, args.key)
    if initial_key_id is not None:
        mujoco.mj_resetDataKeyframe(model, data, initial_key_id)
        mujoco.mj_forward(model, data)
    request_queue: "queue.Queue[ViewerCommand]" = queue.Queue()
    control_server = ControlServer(socket_path=socket_path, request_queue=request_queue)
    control_server.start()

    print(f"Opening MuJoCo viewer for: {scene_path}")
    print(f"Control socket ready: {socket_path}")
    if initial_key_id is not None:
        print(f"Loaded keyframe: {keyframe_name(model, initial_key_id)}")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer_handle:
            while viewer_handle.is_running():
                process_pending_commands(
                    request_queue,
                    scene_path=scene_path,
                    socket_path=socket_path,
                    model=model,
                    data=data,
                    initial_key_id=initial_key_id,
                    viewer_handle=viewer_handle,
                )
                mujoco.mj_step(model, data)
                viewer_handle.sync()
                time.sleep(model.opt.timestep)
    finally:
        control_server.close()


if __name__ == "__main__":
    main()
