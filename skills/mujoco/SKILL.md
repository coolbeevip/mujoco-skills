---
name: mujoco
description: "Build, validate, and control MuJoCo/MJCF robot scenes, including scenes intended for robot learning, reinforcement learning, and reproducible headless simulation. Use for MuJoCo scene creation, model inspection, viewer operation, actuator experiments, and training-environment preparation."
---

# MuJoCo

Create physically meaningful scenes that can be reused for robot learning. Preserve the user's requested task and distinguish scene preparation from implementing or running a training system.

First classify the user's task, then load only the relevant reference:

- **MuJoCo Scene Builder**: Create or modify `MJCF` / `XML`, build scenes, place robots/tables/objects, fix `body` / `joint` / `geom` / `site` / `contact` / `inertial` structure, compile/debug models, and open scenes for visual inspection.
  Read [references/scene-builder.md](references/scene-builder.md).
- **MuJoCo Robot Control**: Inspect model structure, start a viewer, execute actuator control, run minimal control experiments, open/close grippers, and distinguish model problems from control-chain problems.
  Read [references/robot-control.md](references/robot-control.md).
- **Training Preparation**: For every newly created robot scene, read the scene foundation and contract sections of [references/training-readiness.md](references/training-readiness.md). For explicit training, reinforcement learning, dataset generation, or environment implementation requests, also read its runtime and validation sections. Opening, listing, or inspecting an existing scene does not require creating training artifacts.

For combined tasks:

1. Establish the scene and training contract, then build and validate the model.
2. Use Robot Control for interactive operation, or a synchronous headless runtime for learning and reproducibility checks. A viewer is optional for training.
3. Report the highest readiness level actually demonstrated; a compilable scene is not a validated learning environment.

## Shared Rules

- Do not invent MuJoCo tags, attribute names, or default behaviors. If syntax is uncertain, use official docs and existing models in the current workspace as the source of truth.
- Clarify missing choices that change robot identity, task objective, control semantics, or physical boundaries. Use stated, reversible defaults for layout details and presentation; do not block scene preparation on a future RL algorithm or framework choice.
- New robot scenes should preserve real degrees of freedom, meaningful collisions, stable named interfaces, and reproducible initialization. Do not add hidden balance aids, object attachment, or scripted success to make a training scene appear functional. Respect an explicit preview-only request and label its limitations.
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
- Training contract, headless runtime requirements, and acceptance matrix: `references/training-readiness.md`
- Resolve bundled scripts relative to this skill directory, not the generated scene's working directory. The existing scene/grasp checkers provide partial diagnostics, not a complete training-readiness gate.
- Do not preload the entire `scripts/` directory. Open a script only when you need to execute or modify it.
