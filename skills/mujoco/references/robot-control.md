# MuJoCo Robot Control

Read this file only when the task involves runtime execution, inspection, viewer use, actuators, gripper actions, or minimal control experiments.

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

Proceed through control tasks in this order:

1. `bootstrap`
2. Start `mujoco_viewer.py`
3. Use `mujoco_cli.py` to query actuators and current ctrl
4. Use `mujoco_cli.py` to send control commands

Do not skip the first two steps and jump straight into complex control.

## Check The Environment First

Before running viewer or control commands, confirm:

- Python 3 is available
- the `mujoco` Python package can be imported

The default behavior is not "wait for an error and react". Instead:

1. Check the environment first.
2. Automatically install missing packages.
3. Continue the control task after installation.

Explicit checks can call:

```bash
python scripts/env_bootstrap.py
```

If `mujoco` is missing, install it automatically and continue. Do not expose `ModuleNotFoundError: No module named 'mujoco'` to the user as the first response.

## Treat Action Requests As Execution Requests

When the user says any of the following, do not stop at an explanation:

- open the gripper
- close the gripper
- lift the shoulder a little
- bend the elbow
- return the robot arm to zero
- set an actuator to a specific value

Default action:

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

Robot operations should connect to the visual viewer by default. Do not default to offline headless execution.

- The viewer started by `mujoco_viewer.py` should be held by the Python script through `mujoco.viewer`, not by dropping the scene directly into `MuJoCo.app`.
- `mujoco_cli.py` should send commands through `CLI -> socket -> running viewer process -> data.ctrl`.
- Use a separate offline script or one-off Python snippet only when the user explicitly asks for no-window, batch, or offline execution.
- Do not open a new window repeatedly for repeated control requests.

- Before starting a viewer, check whether a viewer socket for the same scene already exists.
- If it exists, reuse it directly instead of opening a new window.
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

## Gripper Semantics

### Open The Gripper

Default order:

1. Prefer an actuator whose name is close to `gripper`, `grip`, or `finger`.
2. If the open/close direction is known, set the actuator directly to the open end.
3. If the direction is unknown, inspect `ctrlrange` first.
4. By default, interpret "open the gripper" as the larger end of the range; correct this if naming or previous results show the opposite.

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

Do not use:

```bash
/Applications/MuJoCo.app/Contents/MacOS/simulate /absolute/path/to/scene.xml
```

## Handoff Requirements

- State the model path.
- State whether the operation was `inspect`, `run`, or `control`.
- If only structure inspection was performed and actuators were not actually driven, say so explicitly.
