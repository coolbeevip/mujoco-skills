"""用真实 viewer.close 事件验证窗口关闭路径；macOS 以 mjpython 在终端运行。"""

import json
import sys
import threading
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

import mujoco.viewer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from keyboard import run

launch = mujoco.viewer.launch_passive


@contextmanager
def closing_viewer(*args, **kwargs):
    with launch(*args, **kwargs) as viewer:
        timer = threading.Timer(1.0, viewer.close)
        timer.start()
        try:
            yield viewer
        finally:
            timer.cancel()
            timer.join()


if __name__ == "__main__":
    with patch("mujoco.viewer.launch_passive", closing_viewer):
        result = run()
    assert result["reason"] == "window_closed", result
    assert result["status"] == "stopped", result
    print(json.dumps(result, allow_nan=False))
