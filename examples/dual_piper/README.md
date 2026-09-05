## AgileX PiPER Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 2.3.4 or later.

### Overview

This package contains a simplified robot description (MJCF) of the [AgileX PiPER](https://global.agilex.ai/products/piper). It is derived from the publicly available [model](https://github.com/agilexrobotics/Piper_ros/tree/ros-noetic-no-aloha/src/piper_description/urdf).

The included scene places two PiPER arms around a tabletop with four cubes and
a shared placement area. It also includes four recorded trajectories, so you
can verify the scene with a complete pick-and-place run before teaching a new
trajectory.

## Quick start

From the repository root, create an isolated Python environment and install
the runtime dependencies:

```bash
cd examples/dual_piper
python3 -m venv .venv
source .venv/bin/activate
python -m pip install "mujoco>=2.3.4" numpy
```

Replay the recorded blue-cube trajectory headlessly:

```bash
python scripts/run_trajectory.py trajectories/blue_cube_to_center.json
```

A successful run exits with status `0` and finishes with a result containing:

```json
{
  "lifted": true,
  "placed": true,
  "released": true,
  "obstacle_collisions": []
}
```

On macOS, replay the same trajectory in the MuJoCo viewer:

```bash
mjpython scripts/run_trajectory.py \
  trajectories/blue_cube_to_center.json \
  --viewer \
  --realtime
```

The remaining sections explain the model and how to teach or replay other
trajectories. All commands below assume the current directory is
`examples/dual_piper`.

### Derivation steps

1.  Added `<mujoco> <compiler balanceinertia="true" discardvisual="false"/> </mujoco>` to the URDF's
   `<robot>` clause in order to preserve visual geometries.
2. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
3. Converted the the .objs to .xmls using [obj2mjcf](https://github.com/kevinzakka/obj2mjcf) and replaced the original stls with them (since each obj in mujoco can have 1 color).
4. Merged similar materials between the .objs
5. Created a `<default>` section to define common properties for joints, actuators, and geoms.
6. Added an equality constraint so that the right finger mimics the position of the left finger.
7. Manually designed box collision geoms for the gripper.
8. Added `exclude` clause to prevent collisions between `base_link` and `link1`.
9. Added position controlled actuators.
10. Added `impratio=10` and `cone=elliptic` for better noslip.
11. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

## License

This model is released under an [MIT License](LICENSE).

## Acknowledgement

This model was graciously contributed by [Omar Rayyan](https://orayyan.com/).

## Dual-arm pick and place

`scene.xml` contains two PiPER arms, four cube objects, a shared placement
area, two wrist cameras, and one top-down camera. The pick-and-place workflow
is split into two programs:

- `scripts/teach_pick_place.py` solves the task in simulation, retries parameter
  candidates, rejects collisions, and writes the best successful trajectory
  to JSON.
- `scripts/run_trajectory.py` validates the recorded scene and initial object
  position, then replays the actuator trajectory without running IK.

These are the only two command-line programs. Files whose names start with
an underscore in `scripts/` are internal implementation modules and should
not be run directly. The root directory contains only the MuJoCo model,
assets, documentation, and recorded trajectories.

The controller builds five named capabilities for every task:

- `home_pose`: safe start and finish configuration
- `pick_approach_pose`: pre-grasp pose above the selected object
- `pick_pose`: grasp pose at the selected object
- `place_approach_pose`: pre-place pose above the requested target
- `place_pose`: release pose at the requested target

It also generates `pick_clearance_pose` and `place_clearance_pose`. Horizontal
transport happens between these two points at a safe height. Candidate IK
solutions and their sampled joint-space segments are rejected when an arm
link intersects another cube, and runtime contacts are recorded in
`obstacle_collisions`.

### 1. Teach and record a trajectory

The teaching program starts each attempt from a fresh simulation. An attempt
is successful only when the cube is lifted, placed inside the target area,
released, and no obstacle contact is detected. After collecting the requested
number of successful attempts, it saves the lowest-score result based on
placement error, joint travel, and duration.

Teach the blue cube task:

```bash
python scripts/teach_pick_place.py \
  --object blue_cube \
  --arm arm0 \
  --place-x -0.08 \
  --place-y 0.10 \
  --output trajectories/blue_cube_to_center.json
```

The repository includes successful trajectories for all four cubes:

- `trajectories/red_cube_to_center.json`
- `trajectories/blue_cube_to_center.json`
- `trajectories/green_cube_to_center.json`
- `trajectories/yellow_cube_to_center.json`

### 2. Replay a recorded trajectory

Run headlessly:

```bash
python scripts/run_trajectory.py trajectories/blue_cube_to_center.json
```

Watch the exact same trajectory in a local MuJoCo viewer on macOS:

```bash
mjpython scripts/run_trajectory.py \
  trajectories/blue_cube_to_center.json \
  --viewer \
  --realtime
```

The JSON records the scene fingerprint, initial object position, actuator
order, named task poses, actuator targets for every stage, all teaching
attempts, and the selected result. Replay stops with an error when the scene,
actuator layout, timestep, or object starting position no longer matches the
recording. Re-run teaching after changing the scene or moving a cube.

For a new part, keep its free body name in MJCF and tune
`--grasp-z-offset`, `--place-z-offset`, `--approach-clearance`, and
`--transit-height` for its geometry and surrounding obstacles.
