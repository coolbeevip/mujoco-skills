# Preparing MuJoCo Scenes for Robot Learning

Read **Scene foundation**, **Training contract**, and **Handoff** when creating a robot scene. Read the entire file when implementing a training environment, collecting demonstrations, or validating readiness for learning. These requirements support future training without selecting a learning algorithm or promising convergence.

## Scope and readiness levels

- **Scene foundation**: portable physical model, named interfaces, explicit initialization, training contract, and model smoke checks. This is the default for new robot scenes unless the user explicitly requests a visual mockup.
- **Environment validated**: an implemented headless reset/step environment with action/observation, reward, episode, isolation, and reproducibility checks passing. Required when the user asks for a usable RL environment; a written contract alone does not meet this level.
- **Learning evaluated**: actual training and evaluation on recorded configurations and held-out seeds. Report measured performance, not guaranteed learnability or real-robot transfer.

For a scene intended for *future* training, deliver the foundation and identify the remaining environment work. Do not install a training stack or launch optimization merely to create a scene. When a usable environment is explicitly requested, implement and verify the runtime; do not stop at an XML and a specification.

## Scene foundation

### Preserve the intended physical problem

- Reuse traceable robot assets, retain license/provenance and upstream revision, and record intentional deviations. Package relative asset/include paths so the scene loads from any working directory.
- Keep visual and collision geometry separate where useful. Visual-only geoms must not accidentally contribute mass: use explicit body inertials or verified inertia inference settings. Primitive or convex collision approximations must retain task-relevant gaps, support surfaces, finger pads, and obstacle boundaries.
- Preserve floating bases for locomotion and flight. Do not weld a humanoid root, disable falls, switch torque motors to position servos, or increase friction/gains beyond a justified range to pass a preview check. Legitimate fixed mounts, mimic-joint constraints, and mechanisms remain valid.
- For grasping, use physical contact or an explicitly modeled tool. Do not teleport objects, overwrite object poses during a rollout, weld them to fingers, or introduce a hidden success controller. A kinematic/mocap controller is acceptable only when it is the declared action interface and its dynamics limitations are documented.
- Keep self-collision, inter-arm collision, robot/environment collision, and carried-object collision where relevant. Name and justify exclusions. Testing only other free objects is insufficient for general collision validation.
- Preserve realistic units, joint limits, actuator force/torque limits, damping, friction, mass, and positive physically valid inertias. Do not infer torque limits directly from `ctrlrange`: transmission and actuator type affect the mapping.
- Declare timestep, integrator, solver/contact settings, and control rate. Choose them against the actual contact and controller behavior, then measure stability; do not copy universal values from an unrelated robot.
- Make decorative geometry cheap and optional. Simulation state and success must not depend on viewer cameras, GUI input, wall-clock sleeps, or presentation-only aids.

### Initialization and interfaces

- Name task-relevant bodies, joints, actuators, sites, sensors, and cameras consistently. Resolve IDs from names after model compilation; do not publish fragile numeric IDs as the interface.
- Define a named initial keyframe or equivalent explicit initializer. Include compatible `qpos`, `qvel`, actuator activation, and `ctrl` where applicable. A position servo's neutral action normally means holding a valid target, not sending zero joint targets.
- Record world/base/TCP frames, length and angle units, quaternion ordering, and rotation conventions. MuJoCo free/ball joints have different position and velocity coordinate sizes; never assume `nq == nv` or index both with the same slices.
- Add meaningful TCP/fingertip sites for manipulation and named cameras/sensors for requested observations. Keep debug markers out of policy images. Ground-truth state used for reward/evaluation must be distinguishable from observations available to the deployed policy.
- Validate object support and collision clearance at initialization. Check nominal task targets and representative boundary targets with kinematics or a reference controller where available; a visually plausible layout alone does not prove reachability.
- For uncontrolled floating robots, falling can be expected. Check finite dynamics and plausible contacts, then record whether standing/hovering control was tested. Do not modify the physical problem to make passive stability pass.

### Foundation smoke check

Ship a small scene-specific headless check (or extend existing project checks) even when a full RL environment is deferred. It should load the scene from its own resolved path, resolve the contract's named interfaces, apply the declared initializer, call forward computation, and check finite state and initial support/clearance. Advance a fixed recorded number of steps using declared baseline controls and bounded per-channel probes appropriate to the robot. Reload/reset and repeat the same control sequence, comparing state within a recorded tolerance. Do not require a balance policy or a successful task rollout merely to establish this foundation level.

Run the check from outside the scene directory to verify asset portability. Fail on unresolved names, invalid initialization, non-finite dynamics, or non-reproducible nominal stepping; report task-specific expected motion separately. Record the command, versions, number of steps, tolerances, and results in the contract. This establishes a reusable simulation foundation; randomized reset, reward logic, and environment isolation remain later gates until implemented.

## Training contract

Write a compact, machine-readable `training_contract.json` beside a newly created scene, or extend the project's equivalent configuration instead of duplicating it. This is an agent/project convention, not a MuJoCo XML feature. Populate it from the compiled model and observed behavior; mark unresolved task-dependent choices explicitly. Use a version field and document changes that invalidate recordings or policies.

Include these fields or their project equivalents:

| Field | Required content |
| --- | --- |
| `schema_version`, `readiness` | Contract version, demonstrated level, outstanding checks |
| `model` | Relative scene path, dependency manifest and hashes covering XML includes, meshes and textures, upstream source/license, actual MuJoCo version |
| `physics` | Units, timestep, integrator, solver/contact configuration, intentional approximations and collision exclusions |
| `initialization` | Keyframe/initializer, baseline controls, movable objects, any settling controller and duration |
| `robot` | Named joint and actuator order, control types and units, limits, TCP and sensor/camera references, frame conventions |
| `action` | Intended policy interface, shape/order, bounds, mapping to controls, saturation/rate limits, neutral action, physics substeps per action |
| `observation` | Intended fields, shapes/dtypes/units/frames, bounds or normalization, noise/latency/history; separate policy observations from privileged state |
| `task` | Objective, success/failure conditions, tolerances and hold durations, reward terms when known, episode time limit |
| `randomization` | Baseline and bounded distributions, coupled physical parameters, sampling cadence, feasibility constraints, train/evaluation seeds |
| `validation` | Commands, tested seeds/rollouts, metrics, failures, untested capabilities and backend |

For future training with an unspecified task, record `task.status: "unresolved"` and the missing objective rather than inventing rewards or claiming an environment exists. Record a supported low-level control interface and observable signals; distinguish measured model facts from proposed future policy mappings. A concrete task is required before implementing reward and termination.

Do not freeze a policy observation/action API merely to make JSON complete. Explicitly identify a provisional interface and its assumptions. For heterogeneous actuators or multiple arms, define per-channel semantics; a single global scale or gripper range may be wrong.

## Headless environment runtime

### Ownership and stepping

- Own each environment's simulation state independently. Never route training actions through the bundled interactive viewer/socket service: its background stepping depends on request timing.
- Expose synchronous reset and step operations. An action advances exactly the configured number of physics substeps; all channels are validated before changing controls. Reject wrong shapes and non-finite values. State whether finite out-of-range actions are clipped or rejected.
- Distinguish physics timestep from policy timestep: `control_dt = physics_dt * frame_skip`. Specify whether targets are held or interpolated and whether a feedback controller updates every physics step. Scale time-dependent reward/cost terms consistently when changing rates.
- Rendering is optional and must not advance simulation or consume the physics/randomization RNG. State-based headless operation must not initialize a graphics context. Pixel policies may require an offscreen backend; validate it separately.
- Keep mutable model parameters private when randomizing mass, friction, or geometry. Separate `MjData` objects alone do not isolate mutations of a shared `MjModel`. Avoid global RNG and global controller state; callbacks, if used, must not couple environments.

### Seeded reset

1. Initialize or retain the environment-local RNG according to the reset API. In Gymnasium, use `super().reset(seed=seed)` and `self.np_random`; do not reseed to a constant on every episode.
2. Restore baseline model parameters before applying new randomization, avoiding cumulative drift. Refresh derived model constants through the supported API or recompile when the chosen parameter requires it.
3. Reset simulation time, positions, velocities, activation, controls, applied forces, mocap/equality state as applicable, controller integrators, history/noise/latency buffers, reward accumulators, episode counters, and success flags.
4. Apply the selected keyframe/initializer and sample bounded feasible starts. Normalize randomized quaternions. Preserve mass/inertia consistency and check joint limits, support, and task-specific collision constraints. Use bounded rejection sampling with a diagnostic on exhaustion.
5. Run forward computation before reading poses, contacts, or sensors. If settling is needed, use a fixed number of steps and a declared baseline controller; define the episode clock after settling.
6. Return observations and reproducibility metadata, including sampled parameters. Repeated equal seeds and actions should reproduce behavior within documented tolerance on the same software/backend configuration.

### Observations, rewards, and episode boundaries

- Return observation copies, not mutable aliases into simulation arrays. Keep shapes/order/dtypes stable across resets and randomizations, and ensure finite values remain inside declared spaces.
- Separate deployable sensing from privileged critic/reward signals. If action delay, filtering, or controller memory affects transitions, expose sufficient history/state or explicitly model partial observability.
- Define reward components and units, sparse success separately from shaping, and log component values. Verify that standing still, dropping the object, or touching a target without completing the task does not accidentally earn success.
- Measure success from physical state, not a trajectory stage name or high reward. For pick-and-place, check lift, sustained transport/hold, placement tolerance, release, and settled support as required by the task; include relevant collision failures.
- Distinguish task termination from an external rollout time limit. For Gymnasium implement `reset -> (obs, info)` and `step -> (obs, reward, terminated, truncated, info)`. An external time limit is truncation; an intrinsic finite-horizon task needs the corresponding state/time semantics. Do not collapse both flags into one `done`.
- Surface non-finite physics or solver instability as explicit failed validation/diagnostics. Do not silently reset mid-step, replace bad observations with zeros, or count invalid episodes as successful training data.
- When choosing Gymnasium, run its environment checker on the actual environment as well as task-specific checks. Do not require Gymnasium if the project already uses another runtime contract.

## Randomization, scale, and demonstration data

- Begin with a deterministic nominal task. Add bounded pose, dynamics, sensing, or appearance randomization incrementally, with an off switch and recorded sampled values. Vary physically coupled parameters together; size changes affect geometry, mass/inertia, support height, and reachability.
- Keep held-out evaluation seeds/configurations separate from demonstration search and training. Repeated success on an identical reset is replay evidence, not generalization evidence.
- For vectorized CPU simulation, verify instance isolation. For MJX or another accelerated backend, check the selected version's supported features, compile and run the actual model, compare task metrics with native MuJoCo, and measure throughput after warmup. A CPU XML compile does not establish accelerator compatibility. Avoid promising universal GPU compatibility or performance.
- Record trajectories through the same action/observation interface used for training. Include model dependency hashes, software/backend and controller versions, seed/randomization, control timing, initial state, actions, observations, rewards, terminal flags, success metrics, and camera configuration if used. Record raw controls as additional data when they differ from policy actions.
- Keep reset/IK planning state changes separate from rollout dynamics. Label synthetic or privileged demonstrations and check them through the physical runtime before treating them as executable examples.

## Validation gates

Create task-specific executable smoke checks beside generated training artifacts. Test observable behavior, including negative cases; a JSON field saying `validated` is not evidence. Start with modest recorded budgets, for example 10 reset seeds and 3 short bounded-action rollouts, increasing coverage for demonstrated risks. This is a smoke baseline, not a convergence claim.

| Gate | Evidence needed |
| --- | --- |
| Portable model | Compile from another working directory with packaged assets; resolve all declared names; verify dependency hashes and initialization |
| Physical integrity | Initial clearance/support, finite state under baseline and bounded controls, relevant contacts and exclusions; expected floating-base falls classified correctly |
| Action contract | Neutral, per-channel and boundary probes produce the declared semantics; invalid actions cannot partially mutate state; gripper direction measured or sourced |
| Reset reproducibility | Same seed/actions reproduce state and outputs; reset after a rollout matches a fresh instance; different seeds vary only declared quantities |
| Observation contract | Shapes/dtypes/spaces/finite values, copies remain unchanged after further steps, intended sensing and camera outputs |
| Episode logic | Deliberately constructed success, failure, time-limit, and near-miss cases produce correct metrics and flags; reward components match their definitions |
| Feasibility | Reference controller/trajectory or another concrete witness completes the nominal task, with measured lift/hold/place or task-equivalent criteria |
| Randomization | Sampled starts are feasible within declared bounds; exhaustion reports an error; no accumulated model changes across resets |
| Isolation and rendering | Reset/step one instance leaves another unchanged; enabling rendering does not change the physical rollout |
| Selected backend | Actual headless/backend run, software versions, warmup-separated throughput when scale is requested |

Use `mujoco_scene_check.py` for compile and heuristic physical diagnostics, not as a complete gate. Its name-based rules can misclassify tasks and passive drift can be expected in uncontrolled robots. Inspect findings against task semantics; do not hide them with `--warn-only` or distort the model to obtain PASS.

Use `mujoco_grasp_check.py` only with known object, gripper selectors and open/close values. It assumes the object is already approachable by the supplied controls; failure from a distant ready pose does not establish that the gripper model is broken. Without `--lift`, it does not test lifting. Lost lift contact is currently only a warning, and its exit code does not establish sustained grasp or placement success. Add task-level checks and inspect individual results.

If a feasibility witness cannot be found, deliver the model and diagnostics with `feasibility unverified`; do not claim the task is impossible from one failed controller, and do not claim an environment is fully validated when a required gate remains untested.

## Handoff

For scene preparation, deliver the model/assets, populated training contract (or project equivalent), reproducible model smoke command/results, and remaining task-dependent work. For an environment request, also deliver the runtime/configuration and executable validation checks. Reuse project structure rather than forcing new directories.

Report readiness level, exact commands, versions/backend, seeds and test budgets, observed metrics, and unresolved checks. Distinguish physical feasibility, API correctness, training performance, and sim-to-real validity. None implies the next without evidence.

## Authoritative references

Consult the installed version and current official documentation when implementing version-sensitive APIs:

- [MuJoCo modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [MuJoCo simulation and initialization](https://mujoco.readthedocs.io/en/stable/programming/simulation.html)
- [MuJoCo Python](https://mujoco.readthedocs.io/en/stable/python.html)
- [Gymnasium custom environments](https://gymnasium.farama.org/introduction/create_custom_env/)
- [Gymnasium time-limit semantics](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/)
- [MJX feature support and performance](https://mujoco.readthedocs.io/en/stable/mjx.html)
