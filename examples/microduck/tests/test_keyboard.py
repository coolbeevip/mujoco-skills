import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from keyboard import command_for_key, connect_pycharm, Display, TerminalInput
from runtime import Runtime
import numpy as np
import pytest


def test_debug_disabled_needs_no_optional_dependency(monkeypatch):
    monkeypatch.setitem(sys.modules, "pydevd_pycharm", None)
    connect_pycharm(None)


def test_debug_missing_dependency_reports_installation(monkeypatch):
    monkeypatch.setitem(sys.modules, "pydevd_pycharm", None)
    with pytest.raises(RuntimeError, match="install the version"):
        connect_pycharm(5678)


def test_debug_connects_locally_and_preserves_terminal(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock

    settrace = Mock()
    monkeypatch.setitem(
        sys.modules, "pydevd_pycharm", SimpleNamespace(settrace=settrace)
    )
    connect_pycharm(6789)
    settrace.assert_called_once_with(
        "127.0.0.1",
        port=6789,
        stdout_to_server=False,
        stderr_to_server=False,
        suspend=True,
    )


def test_debug_connection_failure_reports_server(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock

    monkeypatch.setitem(
        sys.modules,
        "pydevd_pycharm",
        SimpleNamespace(settrace=Mock(side_effect=ConnectionRefusedError("refused"))),
    )
    with pytest.raises(RuntimeError, match="start the Python Debug Server"):
        connect_pycharm(5678)


@pytest.mark.parametrize("port", [0, -1, 65536])
def test_debug_rejects_invalid_port(port):
    with pytest.raises(ValueError, match="between 1 and 65535"):
        connect_pycharm(port)


def test_keyboard_commands_replace_the_entire_previous_command():
    assert command_for_key("w", (0, 0, 1.2)) == (0.3, 0, 0)
    assert command_for_key("a", (0.3, 0, 0)) == (0, 0, 1.2)
    assert command_for_key("d", (0.3, 0, 0)) == (0, 0, -1.2)
    assert command_for_key(" ", (0.3, 0, 0)) == (0, 0, 0)
    assert command_for_key("s", (0, 0, -1.2)) == (0, 0, 0)


def test_unknown_key_keeps_current_command():
    assert command_for_key("x", (0.3, 0, 0)) == (0.3, 0, 0)
    assert command_for_key("A", (0.3, 0, 0)) == (0.3, 0, 0)


def test_display_mutation_cannot_change_simulation():
    runtime = Runtime()
    display = Display(runtime)
    for _ in range(10):
        runtime.step()
    before = runtime.snapshot()
    display.copy_from(runtime)
    np.testing.assert_array_equal(display.data.qpos, runtime.data.qpos)
    display.data.qpos[:] = 9
    display.data.ctrl[:] = 8
    display.model.opt.gravity[:] = 0
    assert runtime.snapshot() == before
    assert runtime.model.opt.gravity[2] == -9.81


def test_terminal_flags_restored_on_exception(monkeypatch):
    import os
    import pty
    import termios

    master, slave = pty.openpty()
    stream = os.fdopen(os.dup(slave), "r")
    monkeypatch.setattr(sys, "stdin", stream)
    before = termios.tcgetattr(slave)
    try:
        with pytest.raises(RuntimeError):
            with TerminalInput():
                assert termios.tcgetattr(slave) != before
                raise RuntimeError("simulated GUI failure")
        after = termios.tcgetattr(slave)
        # macOS 在重新启用 canonical 模式时设置内核 PENDIN 状态位。
        # 它不是用户终端配置；其余标志（含 ECHO/ICANON/ISIG）必须恢复。
        after[3] &= ~termios.PENDIN
        before[3] &= ~termios.PENDIN
        assert after == before
    finally:
        stream.close()
        os.close(master)
        os.close(slave)


def test_nonterminal_input_reports_headless_alternative(monkeypatch):
    with open(__file__) as stream:
        monkeypatch.setattr(sys, "stdin", stream)
        with pytest.raises(ValueError, match="sequence.py"):
            with TerminalInput():
                pass


def test_recorded_gui_trace_matches_headless():
    import hashlib
    import json
    import platform
    from sequence import Measurement, Monitor

    if platform.system() != "Darwin" or platform.machine() != "arm64":
        pytest.skip("recorded floating-point trajectory is specific to macOS arm64")
    evidence = json.loads(
        (Path(__file__).resolve().parents[1] / "keyboard-verification.json").read_text()
    )
    runtime = Runtime()
    measurement = Measurement(runtime)
    monitor = Monitor(measurement.sample(runtime))
    events = {event["control_step"]: event for event in evidence["events"]}
    trace = hashlib.sha256()
    command = (0, 0, 0)
    for tick in range(evidence["final"]["control_steps"]):
        if tick in events:
            command = command_for_key(events[tick]["key"], command)
        runtime.step(command, observer=lambda r: monitor.update(measurement.sample(r)))
        trace.update(runtime.data.qpos.tobytes() + runtime.data.qvel.tobytes())
    assert runtime.physics_steps == evidence["final"]["physics_steps"]
    assert trace.hexdigest() == evidence["trace_sha256"]
    np.testing.assert_array_equal(runtime.data.qpos, evidence["final"]["qpos"])
