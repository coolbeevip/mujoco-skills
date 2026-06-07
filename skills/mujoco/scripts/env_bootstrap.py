#!/usr/bin/env python3
"""Environment checks and auto-install helpers for MuJoCo control scripts."""

from __future__ import annotations

import importlib.util
import shutil
import subprocess
import sys


REQUIRED_PACKAGES = [
    ("mujoco", "mujoco"),
]


def has_command(name: str) -> bool:
    return shutil.which(name) is not None


def has_module(module_name: str) -> bool:
    return importlib.util.find_spec(module_name) is not None


def install_with_pip(package_name: str) -> None:
    subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])


def ensure_environment(auto_install: bool = True) -> None:
    if not has_command("python3") and sys.executable == "":
        raise RuntimeError("Python 3 is not available.")

    missing = [package for module_name, package in REQUIRED_PACKAGES if not has_module(module_name)]
    if not missing:
        return

    if not auto_install:
        raise RuntimeError(
            "Missing required Python packages: "
            + ", ".join(missing)
            + ". Install them before running MuJoCo control scripts."
        )

    for package_name in missing:
        print(f"[bootstrap] Installing missing package: {package_name}")
        install_with_pip(package_name)
