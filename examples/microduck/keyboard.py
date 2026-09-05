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
from behaviors import ACTION_KEYS, Behaviors, WALK_COMMANDS
from runtime import Runtime

# 按键表达的是「希望怎样运动」，不是直接指定腿部关节怎么转。
# 三个数依次为前向速度 m/s、左向速度 m/s、转向角速度 rad/s。
# 例如 w 表示希望以 0.3 m/s 向前走，具体关节动作交给 runtime.py 中的策略计算。
COMMANDS = WALK_COMMANDS


def command_for_key(key, current):
    # 每个有效键替换整条指令：例如 w 后按 a，就变成原地左转而非边走边转。
    # 未知键保持原指令；终端不提供松键事件，所以松开 w 后仍会持续前进，
    # 直到空格或 s 将指令改为零，或其他有效键改变运动目标。
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
        # cbreak 让按键立即可读，无需回车；保留 Ctrl+C 等终端信号。
        tty.setcbreak(self.fd)
        return self

    def read(self):
        # 零超时轮询：没有输入就返回，让物理控制循环继续运行。
        if select.select([self.fd], [], [], 0)[0]:
            value = os.read(self.fd, 64)
            if not value:
                raise EOFError("terminal input closed")
            return value.decode("ascii", errors="ignore")
        return ""

    def __exit__(self, *args):
        # with 块正常结束或抛出异常时，均恢复进入前的终端设置。
        if self.previous is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.previous)


class Display:
    """单向显示副本，viewer 永远拿不到权威物理数据的写权限。"""

    def __init__(self, runtime):
        self.model = copy.copy(runtime.model)
        self.data = mujoco.MjData(self.model)
        self.copy_from(runtime)

    def copy_from(self, runtime):
        # 数据只从物理核心流向显示副本；鼠标操作不会反向修改仿真状态。
        mujoco.mj_copyData(self.data, self.model, runtime.data)


def connect_pycharm(port):
    """可选连接本机调试服务器，在创建仿真和占用终端前暂停。"""
    if port is None:
        return
    if not 1 <= port <= 65535:
        raise ValueError("PyCharm debug port must be between 1 and 65535")
    try:
        import pydevd_pycharm
    except ImportError as error:
        raise RuntimeError(
            "PyCharm debugging requires pydevd-pycharm; install the version shown "
            "in your PyCharm Python Debug Server configuration into this virtualenv"
        ) from error
    print(f"Connecting to PyCharm Debug Server at 127.0.0.1:{port}...", flush=True)
    try:
        # 保留终端输出，并在连接后暂停，便于单步进入 Runtime 初始化。
        pydevd_pycharm.settrace(
            "127.0.0.1",
            port=port,
            stdout_to_server=False,
            stderr_to_server=False,
            suspend=True,
        )
    except Exception as error:
        raise RuntimeError(
            f"Cannot connect to PyCharm at 127.0.0.1:{port}; "
            "start the Python Debug Server and check its port"
        ) from error


def run(cache=DEFAULT_CACHE, *, mode="walk", ball=False):
    import mujoco.viewer

    runtime = Runtime(cache, seed=0, mode=mode, ball=ball)
    behaviors = Behaviors(runtime)
    display = Display(runtime)
    trace = hashlib.sha256()
    events = []
    print(
        "在本终端输入小写按键（不是 MuJoCo 窗口）：w 前进，a 左转，d 右转，空格/s 停止，q 退出。",
        flush=True,
    )
    print(
        "指令保持到下一次按键；松键不会停止。未知键忽略。鼠标只调整视图，不影响物理。",
        flush=True,
    )
    print(
        "walk：y 坐下/站起，g 俯身拾取动作，k/l 左/右踢球，r 单次翻滚。"
        if mode == "walk"
        else "roller：w 前进，a/d 相对航向误差 ±0.5 rad，c 蹲伏。",
        flush=True,
    )
    print(
        "先空格停止并等待站稳，再触发动作；动作期间空格清除运动目标，q 立即退出。",
        flush=True,
    )
    with TerminalInput() as terminal:
        # passive viewer 负责显示，物理时间由下面的 runtime.step 显式推进。
        with mujoco.viewer.launch_passive(
            display.model, display.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            with viewer.lock():
                viewer.cam.distance = 0.6
                viewer.cam.elevation = -20
                viewer.cam.azimuth = 120
            reason = "window_closed"
            # 主循环按「读按键 → 推进控制和物理 → 显示新状态」运行。
            # 没有新按键也会持续调用 runtime.step()，让策略不断调整关节以维持运动。
            while viewer.is_running():
                start = time.monotonic()
                quit_requested = False
                for key in terminal.read():
                    if key in COMMANDS or key in ACTION_KEYS or key == "q":
                        # w 在这里变成 (0.3, 0, 0)。这里只更新目标，机器人尚未运动。
                        response = "quit" if key == "q" else behaviors.handle(key)
                        event = {
                            "key": key,
                            "control_step": runtime.ticks,
                            "command": behaviors.command,
                            "response": response,
                            "behavior": behaviors.status(),
                            "position_m": runtime.snapshot()["position_m"],
                            "yaw_rad": behaviors.monitor.yaw,
                        }
                        events.append(event)
                        print("INPUT " + json.dumps(event), flush=True)
                    if key == "q":
                        quit_requested = True
                        reason = "quit_key"
                        break
                if quit_requested:
                    break
                # 普通移动使用 Runtime.step()；动作请求由 Behaviors 编码阶段信号，
                # 再调用 Runtime.step_policy()。两条路径每次都只推进 20 ms。
                previous_stage = (behaviors.active, behaviors.stage)
                behaviors.step()
                if (behaviors.active, behaviors.stage) != previous_stage:
                    print("BEHAVIOR " + json.dumps(behaviors.status()), flush=True)
                trace.update(runtime.data.qpos.tobytes() + runtime.data.qvel.tobytes())
                # 显示刚刚计算出的物理结果。窗口只接收副本，不参与策略推理。
                display.copy_from(runtime)
                with viewer.lock():
                    viewer.cam.lookat[:] = runtime.data.xpos[runtime.root_id]
                viewer.sync()
                # 只补足真实时间中的 20 ms 周期；计算超时也不额外补跑物理步。
                time.sleep(max(0, 0.02 - (time.monotonic() - start)))
    return {
        "status": "stopped",
        "reason": reason,
        "events": events,
        "final": runtime.snapshot(),
        "trace_sha256": trace.hexdigest(),
        "behavior": behaviors.status(),
        "full_sequence_verified": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", default=DEFAULT_CACHE)
    parser.add_argument("--mode", choices=("walk", "roller"), default="walk")
    parser.add_argument(
        "--ball", action="store_true", help="add a kick ball (walk mode only)"
    )
    parser.add_argument(
        "--pycharm-debug",
        type=int,
        nargs="?",
        const=5678,
        metavar="PORT",
        help="connect to a local PyCharm Debug Server and suspend (default port: 5678)",
    )
    args = parser.parse_args()
    try:
        connect_pycharm(args.pycharm_debug)
        print(
            "RESULT "
            + json.dumps(
                run(args.cache, mode=args.mode, ball=args.ball), allow_nan=False
            ),
            flush=True,
        )
    except (Exception, KeyboardInterrupt) as error:
        print(
            f"keyboard stopped: {error or 'user interrupt'}. "
            "macOS: use mjpython keyboard.py; headless: python sequence.py",
            file=sys.stderr,
        )
        sys.exit(2)
