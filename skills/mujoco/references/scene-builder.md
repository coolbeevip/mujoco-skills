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
5. Run XML, compile, physical sanity, and viewer checks before reporting back.

If the task has manipulation semantics such as grasping, pick-and-place, placing an object into a box, stacking, or picking, do not mistake "the model compiles" or "the robot moves" for "the task is executable". For these tasks, the end effector must also pass a grasp-chain check.

## Clarify Before Building

Do not silently fill in scene requirements when user intent is unclear. Ask before creating or changing files if the missing information can change the robot model, task feasibility, object placement, physics, or validation result.

Ask a concise clarification question when any of these are unclear:

- Robot identity or variant, such as `UR5` vs `UR5e`, fixed arm vs mobile base, or real upstream model vs placeholder.
- Scene purpose, such as visual demo, grasping, sorting, navigation, obstacle crossing, or control experiment.
- Required task semantics, especially whether objects only need to appear in the scene or must be physically graspable, stackable, sortable, or movable.
- Object count, rough dimensions, placement relationships, support surfaces, or whether objects should be fixed or dynamic.
- Output path, when the user references an existing scene, asks for a project-specific location, or rejects the default `~/Documents/mujoco` location without giving an exact path.
- Validation expectation, such as compile-only, viewer check, physical sanity check, or grasp-chain validation.

When asking, keep the question narrow and decision-oriented. If several details are missing, ask for the smallest set needed to avoid a wrong scene. Do not ask about implementation details that are already determined by stable skill rules, such as using `site` for debug markers or keeping fixed industrial arms mounted unless the user explicitly asks otherwise.

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

## Initial Physical Plausibility

Do not deliver a scene only because it compiles. The first frame should look physically plausible before the user touches the controls.

### Robot Base Placement

- For fixed industrial or tabletop manipulators, keep the robot base fixed to the world or mounted to a fixed table/base body by default. Do not add a `freejoint` to an arm base unless the user explicitly asks for a floating or mobile robot.
- If a robot is imported from an upstream model, inspect the base frame and root body before placing it. Do not assume the robot's visual origin is at floor height.
- Set the root `body pos` and orientation so the base stands upright on the floor, table, or mounting plate. Avoid compensating for a wrong pose by moving child geoms.
- For mobile bases, quadrupeds, humanoids, and drones, a `freejoint` can be correct, but the initial pose still must be balanced and above the support surface.

### Object And Support Placement

- Place dynamic objects from their physical bottom surface, not from their center by guesswork. For a box geom with half-size `sx sy sz`, the object's center `z` should normally be `support_z + sz + small_clearance`.
- Do not leave bins, boxes, cubes, cups, or obstacles suspended in midair unless the user explicitly asks for a falling-object or aerial scenario.
- Fixed scene props such as tables, shelves, bins, and conveyor frames usually should not have `freejoint`.
- Graspable objects usually should have a `freejoint`, realistic mass/inertia, nonzero friction, and an initial pose resting on a support surface without visible penetration.
- Run a short passive simulation with zero control after construction. If a supposedly static scene object drops, rolls away, tips over, or the robot collapses, fix the initial pose, support geometry, joint setup, or inertial/contact parameters before handoff.

### Sorting Scene Layout

For robot sorting scenes with a conveyor, colored blocks, and bins:

- Place the robot base on the conveyor's working side near the conveyor midline, not at one end or far from the active belt area.
- Keep pick objects on the belt within the robot's reachable workspace and visually in front of the end effector.
- Place destination bins in the same reachable work area as the pick objects. Do not put them so far across or behind the conveyor that the arm cannot plausibly place objects into them.
- From the top view, the robot base, the belt center, the two blocks, and the two bins should form one coherent workcell. If this relationship is not obvious, adjust the layout before handoff.
- For fixed arms such as UR5e, use conservative reach planning. Keep pick and place targets comfortably inside the arm's nominal reach rather than at the far edge of the workspace.

### Sites, Markers, And Visual Helpers

- Use `site` for grasp centers, target points, fingertip references, debug markers, and visual annotations. Do not use a physical `geom` sphere as a marker on top of a block.
- For scene-presentation screenshots, do not leave grasp, drop, or target sites visibly rendered as obvious balls on top of task objects. Keep reference sites tiny, transparent, in a non-primary group, or omit them when they are not needed for the user's requested task.
- If a helper marker must be a `geom`, it must be visual-only with `contype="0"` and `conaffinity="0"`, and it should be named clearly as a visual marker.
- Do not attach small collision-enabled spheres to task objects unless they are physically part of the object requested by the user.

### Physical Sanity Check

Before reporting a newly built or materially modified scene, run the bundled sanity checker when MuJoCo Python is available:

```bash
python scripts/mujoco_scene_check.py /absolute/path/to/scene.xml
```

Treat failures as model issues to fix, not as viewer quirks. At minimum, investigate:

- initial interpenetration contacts
- dynamic free bodies that fall significantly from the initial pose
- root bodies with free joints that look like fixed-base manipulators
- tiny collision-enabled sphere geoms that look like marker mistakes

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
- When taking a screenshot for visual inspection, make the MuJoCo viewer window near-fullscreen before capturing so the robot, task objects, and workspace are all visible. If the first screenshot is cropped, zoom out, adjust the scene `statistic` extent/center or camera, and capture again.
- Prefer interface-based offscreen renders over operating-system screenshots when checking full scene layout. Render at least top, front, back, left, and right views; include bottom view when checking support and underside placement:

```bash
python scripts/mujoco_render_views.py /absolute/path/to/scene.xml --key ready --out-dir /tmp/mujoco-views
```

To keep later control commands attached to the same viewer process, do not open the scene directly with `Applications/MuJoCo.app`. Prefer starting `mujoco.viewer` through the skill's bundled script:

```bash
python scripts/mujoco_viewer.py /absolute/path/to/scene.xml
```

If the scene uses a keyframe for the intended presentation or ready pose, load it explicitly when opening the viewer:

```bash
python scripts/mujoco_viewer.py /absolute/path/to/scene.xml --key ready
```

Do not assume a `<keyframe>` entry is the initial viewer state. MuJoCo starts from `qpos0` unless the viewer or script loads the keyframe, so a robot can appear folded, collapsed, or away from the task even though a good keyframe exists.

Do not use:

```bash
/Applications/MuJoCo.app/Contents/MacOS/simulate /absolute/path/to/scene.xml
```

That window is not managed by the current skill's script process, and later commands cannot reliably attach to the `CLI -> socket -> viewer process -> data.ctrl` control chain.

## Handoff Requirements

- State the final file path.
- State whether you modified the robot body itself or only the top-level scene.
- State whether XML compile, physical sanity, viewer, and grasp-chain checks were performed.
- If only static modeling was performed and runtime validation was not performed, say so explicitly.
