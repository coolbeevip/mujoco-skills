# MuJoCo Robot Control

Read this file only when the task involves runtime execution, inspection, viewer use, actuators, gripper actions, or minimal control experiments.

The viewer/socket workflow below is for interactive operation. For training, demonstration collection, deterministic replay, or batch validation, read [training-readiness.md](training-readiness.md) and use a synchronous headless runtime that owns simulation stepping. These tasks do not require opening a viewer.

## Unified Entry Points

Prefer the combination of these two scripts:

- `scripts/mujoco_viewer.py`
- `scripts/mujoco_cli.py`

Supplementary helper:

- `scripts/env_bootstrap.py`

When the user only asks to list all scenes, do not enter the control workflow. Read first-level directory names under `~/Documents/mujoco`, or call:

```bash
python - <<'PY'
from path_utils import list_scene_groups
print("\n".join(list_scene_groups()))
PY
```

## Default Workflow

Proceed through interactive viewer control tasks in this order:

1. `bootstrap`
2. Connect to a live viewer for the same scene, or start `mujoco_viewer.py`
3. Use `mujoco_cli.py` to query actuators and current ctrl
4. Use `mujoco_cli.py` to send control commands

Verify the environment and live session before sending interactive controls. For an existing training runtime, preserve its state and execution interface.

## Check The Environment First

Before running viewer or control commands, confirm:

- Python 3 is available
- the `mujoco` Python package can be imported

Use the project's selected Python environment and check the actual import/version:

```bash
python -c "import sys, mujoco; print(sys.executable); print(mujoco.__version__)"
```

If dependencies are missing, install them in the project's environment within available permissions and respect its dependency constraints. Record the versions used; do not silently upgrade the environment during a reproducibility task. `env_bootstrap.py` currently exposes helper functions only; invoking the file directly does not run its checks.

## Treat Action Requests As Execution Requests

When the user says any of the following, do not stop at an explanation:

- open the gripper
- close the gripper
- lift the shoulder a little
- bend the elbow
- return the robot arm to zero
- set an actuator to a specific value

Default action for interactive operation:

1. Start or connect to `mujoco_viewer.py`.
2. Use `mujoco_cli.py actuators` / `info` to gather the minimum required structure information.
3. Once enough information is available, connect to the visual viewer and execute `set` + `step` by default.
4. Report the result after execution.

## Prefer Actuators; Do Not Guess From Joints

For users, "make the robot arm move" is an action request. In MuJoCo, it is usually a request to write actuator control values.

Default rules:

- Control by actuator name, not by guessed joint name.
- Start with small values; do not max out controls immediately.
- Test one axis before multiple axes.
- Test short durations before long durations.

## Viewer Reuse

Interactive robot operations should connect to the visual viewer by default. Learning and reproducibility tasks use the headless path described above.

- The viewer started by `mujoco_viewer.py` should be held by the Python script through `mujoco.viewer`, not by dropping the scene directly into `MuJoCo.app`.
- `mujoco_cli.py` should send commands through `CLI -> socket -> running viewer process -> data.ctrl`.
- Use a separate synchronous runtime for training, recorded rollouts, and repeatable validation; render optionally from that runtime.
- Do not open a new window repeatedly for repeated control requests.

- Before starting a viewer, check the same-scene socket with a bounded liveness query and confirm the loaded scene/keyframe. Socket-file existence alone does not prove a live service.
- Reuse a live compatible viewer instead of opening a new window. The current CLI has no built-in timeout, so bound diagnostics through the calling process.
- State in the result that the current viewer was reused.

## Common Commands

```bash
python scripts/mujoco_viewer.py ~/Documents/mujoco/so101/scene.xml
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml ping
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml info
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml actuators
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml set shoulder_lift 0.1
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml set-batch shoulder_lift 0.1 elbow_flex 0.6 wrist_roll -0.2
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml step 120
```

During inspection, focus on:

- whether actuators exist
- whether `ctrlrange` is reasonable
- whether joint topology matches expectations
- whether the end-effector `site` is defined

## Simple Action Sequences

If the goal is only a simple action, do not write a large new Python script each time. Prefer this flow:

1. Use `mujoco_cli.py actuators` and `info` to read the minimum structure information.
2. Generate a sequence of `set` + `step` commands.
3. Execute those commands on the running viewer.

If multiple joints or multiple robot arms need to move "at the same time", merge same-timestep control values into one `set-batch`. Do not send several consecutive `set` commands.

Prevalidate every selector and finite value before using `set-batch`: the current service can partially apply a batch before encountering an invalid selector. Its main loop also advances physics between requests, so `set` followed by `step` does not define an exact training transition. Use the training runtime for synchronized actions and fixed-step rollouts.

## Gripper Semantics

### Open The Gripper

Default order:

1. Prefer an actuator whose name is close to `gripper`, `grip`, or `finger`.
2. If the open/close direction is known, set the actuator directly to the open end.
3. Resolve open/close semantics from the robot's training contract, upstream model, or explicit actuator/transmission definitions.
4. If still unknown, use a small bounded probe from a clear pose and measure fingertip separation. Record the mapping; do not assume the larger end of `ctrlrange` means open. For multiple grippers, identify the requested arm and map each actuator separately.

### Close The Gripper

Default order:

1. Locate the gripper actuator.
2. Set its control value to the end opposite the open value.
3. Report the actual control value and result after execution.

Do not merely hand the actuator name to the user and make them try it themselves.

## Running Scenes

Do not send the scene directly to `Applications/MuJoCo.app`. By default, start the skill's Python script and let it call `mujoco.viewer` internally:

```bash
python scripts/mujoco_viewer.py /absolute/path/to/scene.xml
```

If the scene defines a ready or presentation keyframe, load it explicitly:

```bash
python scripts/mujoco_viewer.py /absolute/path/to/scene.xml --key ready
```

Do not use:

```bash
/Applications/MuJoCo.app/Contents/MacOS/simulate /absolute/path/to/scene.xml
```

## Handoff Requirements

- State the model path.
- State whether the operation was `inspect`, `run`, or `control`.
- If only structure inspection was performed and actuators were not actually driven, say so explicitly.
