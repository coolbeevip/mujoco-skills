"""终端按键控制 MicroDuck；macOS 请使用 mjpython 启动。"""

import argparse
import copy
import hashlib
import json
import os
import select
import sys
import termios
import time
import tty

import mujoco

from assets import DEFAULT_CACHE
from runtime import Runtime
from sequence import Measurement, Monitor

COMMANDS = {
    "w": (0.3, 0, 0),
    "a": (0, 0, 1.2),
    "d": (0, 0, -1.2),
    "s": (0, 0, 0),
    " ": (0, 0, 0),
}


def command_for_key(key, current):
    return COMMANDS.get(key, current)


class TerminalInput:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.previous = None

    def __enter__(self):
        if not os.isatty(self.fd):
            raise ValueError(
                "keyboard requires a terminal; use sequence.py for headless verification"
            )
        self.previous = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def read(self):
        if select.select([self.fd], [], [], 0)[0]:
            value = os.read(self.fd, 64)
            if not value:
                raise EOFError("terminal input closed")
            return value.decode("ascii", errors="ignore")
        return ""

    def __exit__(self, *args):
        if self.previous is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.previous)


class Display:
    """单向显示副本，viewer 永远拿不到权威物理数据的写权限。"""

    def __init__(self, runtime):
        self.model = copy.copy(runtime.model)
        self.data = mujoco.MjData(self.model)
        self.copy_from(runtime)

    def copy_from(self, runtime):
        mujoco.mj_copyData(self.data, self.model, runtime.data)


def run(cache=DEFAULT_CACHE):
    import mujoco.viewer

    runtime = Runtime(cache, seed=0)
    measurement = Measurement(runtime)
    monitor = Monitor(measurement.sample(runtime))
    display = Display(runtime)
    trace = hashlib.sha256()
    events = []
    command = (0, 0, 0)
    print(
        "在本终端输入小写按键（不是 MuJoCo 窗口）：w 前进，a 左转，d 右转，空格/s 停止，q 退出。",
        flush=True,
    )
    print(
        "指令保持到下一次按键；松键不会停止。未知键忽略。鼠标只调整视图，不影响物理。",
        flush=True,
    )
    with TerminalInput() as terminal:
        with mujoco.viewer.launch_passive(
            display.model, display.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            with viewer.lock():
                viewer.cam.distance = 0.6
                viewer.cam.elevation = -20
                viewer.cam.azimuth = 120
            reason = "window_closed"
            while viewer.is_running():
                start = time.monotonic()
                quit_requested = False
                for key in terminal.read():
                    if key in COMMANDS or key == "q":
                        command = command_for_key(key, command)
                        event = {
                            "key": key,
                            "control_step": runtime.ticks,
                            "command": command,
                            "position_m": runtime.snapshot()["position_m"],
                            "yaw_rad": monitor.yaw,
                        }
                        events.append(event)
                        print("INPUT " + json.dumps(event), flush=True)
                    if key == "q":
                        quit_requested = True
                        reason = "quit_key"
                        break
                if quit_requested:
                    break
                runtime.step(
                    command, observer=lambda r: monitor.update(measurement.sample(r))
                )
                trace.update(runtime.data.qpos.tobytes() + runtime.data.qvel.tobytes())
                display.copy_from(runtime)
                with viewer.lock():
                    viewer.cam.lookat[:] = runtime.data.xpos[runtime.root_id]
                viewer.sync()
                time.sleep(max(0, 0.02 - (time.monotonic() - start)))
    return {
        "status": "stopped",
        "reason": reason,
        "events": events,
        "final": runtime.snapshot(),
        "trace_sha256": trace.hexdigest(),
        "full_sequence_verified": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", default=DEFAULT_CACHE)
    args = parser.parse_args()
    try:
        print("RESULT " + json.dumps(run(args.cache), allow_nan=False), flush=True)
    except (Exception, KeyboardInterrupt) as error:
        print(
            f"keyboard stopped: {error or 'user interrupt'}. "
            "macOS: use mjpython keyboard.py; headless: python sequence.py",
            file=sys.stderr,
        )
        sys.exit(2)
