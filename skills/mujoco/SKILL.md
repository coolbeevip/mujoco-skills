---
name: mujoco
description: "Unified workflow for MuJoCo modeling and runtime control. Use when the user mentions MuJoCo, MJCF, scene.xml, robot or robot arm scene setup, opening a viewer, actuator control, minimal control experiments, gripper open/close, or model structure inspection. After triggering, route first to MuJoCo Scene Builder or MuJoCo Robot Control through progressive disclosure."
---

# MuJoCo

This skill covers two MuJoCo task families. Do not load both detailed contexts at once.

First classify the user's task, then load only the relevant reference:

- **MuJoCo Scene Builder**: Create or modify `MJCF` / `XML`, build scenes, place robots/tables/objects, fix `body` / `joint` / `geom` / `site` / `contact` / `inertial` structure, compile/debug models, and open scenes for visual inspection.
  Read [references/scene-builder.md](references/scene-builder.md).
- **MuJoCo Robot Control**: Inspect model structure, start a viewer, execute actuator control, run minimal control experiments, open/close grippers, and distinguish model problems from control-chain problems.
  Read [references/robot-control.md](references/robot-control.md).

If a task includes both parts, use this fixed order:

1. Run Scene Builder first, and make the model or scene compile and open cleanly.
2. Run Robot Control second, start the viewer service, and send control commands.

## Shared Rules

- Do not invent MuJoCo tags, attribute names, or default behaviors. If syntax is uncertain, use official docs and existing models in the current workspace as the source of truth.
- Do not guess user intent for scene creation or model edits. If the scene goal, robot choice, task semantics, object layout, physical constraints, output path, or validation expectation is unclear, ask a concise clarification question before creating or changing files.
- The default output directory is `~/Documents/mujoco`. If the user does not provide an absolute path, prefer saving, searching, or resolving models there.
- When the user asks to "list all scenes", read only from `~/Documents/mujoco` by default, and return only first-level directory names as scene names. Do not recursively expand files.
- When the user only says "open MuJoCo" or "open a scene.xml", infer intent first:
  - If the goal is to check whether a scene opens normally, use Scene Builder.
  - If the follow-up involves robot operation, also load Robot Control.
- When reporting results, always state the final model path, the command or script entry point you actually used, and any risk that remains unverified.

## Official References

- MuJoCo modeling: `https://mujoco.readthedocs.io/en/stable/modeling.html`
- MuJoCo python: `https://mujoco.readthedocs.io/en/stable/python.html`
- MuJoCo api: `https://mujoco.readthedocs.io/en/stable/APIreference/index.html`

## Scripts And Resources

- Viewer service entry point: `scripts/mujoco_viewer.py`
- Control and query entry point: `scripts/mujoco_cli.py`
- Scene physical sanity checker: `scripts/mujoco_scene_check.py`
- Grasp-chain verifier: `scripts/mujoco_grasp_check.py`
- Offscreen multi-view renderer: `scripts/mujoco_render_views.py`
- Environment and path helpers: `scripts/env_bootstrap.py`, `scripts/path_utils.py`
- Do not preload the entire `scripts/` directory. Open a script only when you need to execute or modify it.
