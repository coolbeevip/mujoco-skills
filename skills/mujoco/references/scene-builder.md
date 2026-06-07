# MuJoCo Scene Builder

Read this file only when the task involves modeling, model edits, scene construction, or scene visualization checks.

## Responsibilities

- Create or modify `MJCF` / `XML`
- Adjust `body` / `joint` / `geom` / `site` / `sensor` / `contact` / `equality` / `inertial`
- Add `scene.xml`, tables, obstacles, cameras, lights, and graspable objects for an existing robot
- Fix compile failures, hierarchy mistakes, inertial problems, contact penetration, and pose errors
- Open a specified scene directly for visual inspection

Not responsible for:

- actuator control
- minimal control experiments
- runtime actions such as opening or closing a gripper

Delegate those tasks to `MuJoCo Robot Control`.

## Default Workflow

1. Define the system boundary: are you editing the robot body itself, or only the top-level scene?
2. Reuse before rebuilding: when a real robot model exists, prefer upstream or official models before creating a placeholder from scratch.
3. Build the smallest compilable version first, then add details.
4. Save under `~/Documents/mujoco` with stable, readable file names.
5. Run XML, compile, and viewer checks before reporting back.

If the task has manipulation semantics such as grasping, pick-and-place, placing an object into a box, stacking, or picking, do not mistake "the model compiles" or "the robot moves" for "the task is executable". For these tasks, the end effector must also pass a grasp-chain check.

## Scene Listing

When the user asks to list all scenes:

- Read only `~/Documents/mujoco` by default.
- Treat only first-level directories under that path as scene names.
- Do not recursively list `scene.xml`, `assets`, `meshes`, or deeper files.

If a scripted listing is needed, prefer:

```bash
python - <<'PY'
from path_utils import list_scene_groups
print("\n".join(list_scene_groups()))
PY
```

## Reuse Priority

For real robots, use this default priority order:

1. MuJoCo official `Model Gallery`
2. `MuJoCo Menagerie`
3. Similar models already present in the current repository
4. From-scratch modeling only as the last resort

If the user actually wants to "put this robot into an operation scene", keep the robot body unchanged and modify only the top-level `scene.xml`.

Typical minimal edits:

- Adjust the base placement height or orientation
- Add a table, box, block, floor, or obstacle
- Add `camera`, `light`, `site`, or `keyframe`

Do not escalate "add a few scene objects" into "rewrite the robot body".

## Modeling Rules

- Inspect the body tree first; do not confuse independent rigid bodies with attached geometry.
- Prefer local coordinates when interpreting `body pos`, `joint pos`, `geom pos`, and `site pos`.
- Aim for compilability first; consider `default class`, abstraction reuse, and `include` splitting after that.
- Do not fake inertia or mass. If the model jitters, flips, or produces unrealistic torques, inspect inertia first.
- For contact issues, check `contype` / `conaffinity` / `solref` / `solimp` / `friction` / initial interpenetration together.
- Use only attribute names known to exist. If unsure, check official docs or current repository examples.

## Scene Visual Defaults

- By default, do not deliver high-brightness or near-white `skybox`, `haze`, checkerboard floor, or strong headlight combinations.
- Unless the user explicitly asks for a bright showroom style, prefer a low-saturation, darker blue-gray or neutral-gray background.
- Do not casually set `visual/rgba haze` close to `1 1 1 1`; do not make the top color of `texture type="skybox"` nearly pure white.
- The floor and background should help make the robot and objects legible, not dominate the scene with large bright areas that look washed out or glaring.

## Hard Constraints For Gripper Modeling

When the user wants grasping, gripping, or pick-and-place, check or model against the constraints below by default. Missing even one can produce a robot that moves but cannot lift the block.

### 1. Do Not Check Only The Actuator; Check The Full Grasp Chain

The model must include all of the following:

- A reasonable end-effector frame or grasp site
- At least two contact surfaces that can form a gripping constraint
- Explainable open/close semantics
- Collision geometry where the gripper and target object actually contact each other

Do not mistake "there is a `gripper` actuator" for "the gripper is usable".

### 2. Do Not Deliver A Fake Gripper With One-Sided Motion And No Opposing Contact Surface

If a model has only one moving jaw and no fixed opposing contact surface or opposite jaw, treat it as not capable of stable gripping by default.

For gripper-like end effectors, require at least one of the following:

- Two opposing jaws, with their opening/closing geometry kept consistent by a joint, equality constraint, or tendon
- One moving jaw plus a clear fixed jaw or palm contact surface, and verification that an object can actually be held between them

### 3. Do Not Use Visual Meshes As The Grasp Collision Basis

For grippers and fingertips:

- Keep meshes as visuals when useful.
- Prefer simple primitive collision geoms for grasp contact, such as `box` or `capsule`.
- Make fingertip contact surfaces explicit, stable, and easy to reason about. Do not depend entirely on complex STL meshes.

### 4. Define A Grasp Reference

Provide at least one of the following sites:

- `gripperframe`
- `grasp_center`
- left and right fingertip sites

If grasp planning is part of the task, prefer providing both:

- one central grasp site
- two fingertip sites

### 5. Define Open/Close Semantics

Do not provide only a joint range or `ctrlrange`.

The model must answer:

- Which end is open
- Which end is closed
- The approximate remaining gap between the fingers when closed

If the upstream model does not make this clear, add comments, sites, names, or README notes. Do not deliver an unclear gripper as a "graspable model".

### 6. Grasping Tasks Require A Minimal Grasp Validation

If the user's target is pick-and-place, do not validate only that:

- XML compiles
- the viewer opens
- the actuator can be driven

Add at least one minimal grasp check:

- whether the object actually leaves the table when the gripper closes
- whether the object's `z` keeps increasing after lift
- whether the object releases from the gripper after opening

If this step was not performed, explicitly state in the handoff: "The motion chain was verified, but the grasp chain was not verified."

## Default Breakdown For Pick-And-Place Tasks

When the user asks to "put the block into the box", do not treat it as a single action. Split it by default into:

1. Scene and target object check
2. End-effector grasp capability check
3. Approach pose / grasp pose / lift pose / place pose definition
4. Control or planning execution

If step 2 fails, fix the gripper model first instead of piling on action scripts or IK.

## Opening Scenes

When the task is "open the scene and take a look":

- If the user provides an explicit path, use that path directly.
- If the user provides only a file name, search the current workspace first, then fall back to `~/Documents/mujoco`.
- If the user provides no path, open the most recently modified XML by default.

To keep later control commands attached to the same viewer process, do not open the scene directly with `Applications/MuJoCo.app`. Prefer starting `mujoco.viewer` through the skill's bundled script:

```bash
python scripts/mujoco_viewer.py /absolute/path/to/scene.xml
```

Do not use:

```bash
/Applications/MuJoCo.app/Contents/MacOS/simulate /absolute/path/to/scene.xml
```

That window is not managed by the current skill's script process, and later commands cannot reliably attach to the `CLI -> socket -> viewer process -> data.ctrl` control chain.

## Handoff Requirements

- State the final file path.
- State whether you modified the robot body itself or only the top-level scene.
- If only static modeling was performed and runtime validation was not performed, say so explicitly.
