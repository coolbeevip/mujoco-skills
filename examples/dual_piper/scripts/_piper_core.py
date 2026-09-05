"""Internal task-space controller for the dual PiPER scene.

The controller exposes five task capabilities:

* home_pose
* pick_approach_pose
* pick_pose
* place_approach_pose
* place_pose

Two additional collision-clearance waypoints, ``pick_clearance_pose`` and
``place_clearance_pose``, keep horizontal travel above tabletop obstacles.

This module is shared by the teaching and replay programs. It is deliberately
not a command-line entry point.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import mujoco
import numpy as np


ARM_JOINT_COUNT = 6
DEFAULT_HOME_KEY = "ready"
DEFAULT_PLACE_X = 0.0
DEFAULT_PLACE_Y = 0.10
DEFAULT_GRASP_Z_OFFSET = 0.0
DEFAULT_TRANSIT_HEIGHT = 0.98
TABLE_SURFACE_Z = 0.72
DEFAULT_APPROACH_AXIS = np.array([0.0, 0.956, -0.293])


@dataclass(frozen=True)
class JointPose:
    """A named arm configuration."""

    name: str
    joints: np.ndarray


@dataclass(frozen=True)
class CartesianPose:
    """A named end-effector pose in world coordinates."""

    name: str
    position: np.ndarray
    rotation: np.ndarray


@dataclass(frozen=True)
class PickPlacePlan:
    """Reusable pick/place poses plus two obstacle-clearance waypoints."""

    object_name: str
    home_pose: JointPose
    pick_clearance_pose: CartesianPose
    pick_approach_pose: CartesianPose
    pick_pose: CartesianPose
    place_clearance_pose: CartesianPose
    place_approach_pose: CartesianPose
    place_pose: CartesianPose

    def named_poses(self) -> tuple[JointPose | CartesianPose, ...]:
        return (
            self.home_pose,
            self.pick_clearance_pose,
            self.pick_approach_pose,
            self.pick_pose,
            self.place_clearance_pose,
            self.place_approach_pose,
            self.place_pose,
        )


@dataclass(frozen=True)
class ArmInterface:
    """Resolved MuJoCo names and indices for one PiPER arm."""

    name: str
    joint_names: tuple[str, ...]
    actuator_names: tuple[str, ...]
    gripper_actuator_name: str
    grasp_site_name: str
    qpos_indices: np.ndarray
    dof_indices: np.ndarray
    actuator_indices: np.ndarray
    gripper_actuator_index: int
    joint_ranges: np.ndarray
    grasp_site_id: int

    @classmethod
    def resolve(cls, model: mujoco.MjModel, arm: str) -> "ArmInterface":
        if arm not in {"arm0", "arm1"}:
            raise ValueError(f"Unknown arm {arm!r}; expected 'arm0' or 'arm1'.")

        joint_names = tuple(f"joint{index}_{arm}" for index in range(1, 7))
        actuator_names = joint_names
        gripper_actuator_name = f"gripper_{arm}"
        grasp_site_name = f"grasp_center_{arm}"

        joint_ids = np.array([model.joint(name).id for name in joint_names], dtype=int)
        actuator_ids = np.array(
            [model.actuator(name).id for name in actuator_names], dtype=int
        )
        gripper_id = model.actuator(gripper_actuator_name).id
        site_id = model.site(grasp_site_name).id

        return cls(
            name=arm,
            joint_names=joint_names,
            actuator_names=actuator_names,
            gripper_actuator_name=gripper_actuator_name,
            grasp_site_name=grasp_site_name,
            qpos_indices=model.jnt_qposadr[joint_ids].astype(int),
            dof_indices=model.jnt_dofadr[joint_ids].astype(int),
            actuator_indices=actuator_ids,
            gripper_actuator_index=gripper_id,
            joint_ranges=model.jnt_range[joint_ids].copy(),
            grasp_site_id=site_id,
        )


@dataclass(frozen=True)
class ExecutionResult:
    object_name: str
    arm: str
    initial_object_position: np.ndarray
    lifted_object_position: np.ndarray
    final_object_position: np.ndarray
    target_xy: np.ndarray
    max_object_z: float
    lift_height: float
    placement_error_xy: float
    lifted: bool
    placed: bool
    released: bool
    obstacle_collisions: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "object": self.object_name,
            "arm": self.arm,
            "initial_object_position": self.initial_object_position.tolist(),
            "lifted_object_position": self.lifted_object_position.tolist(),
            "final_object_position": self.final_object_position.tolist(),
            "target_xy": self.target_xy.tolist(),
            "max_object_z": self.max_object_z,
            "lift_height": self.lift_height,
            "placement_error_xy": self.placement_error_xy,
            "lifted": self.lifted,
            "placed": self.placed,
            "released": self.released,
            "obstacle_collisions": list(self.obstacle_collisions),
        }

    @property
    def successful(self) -> bool:
        return (
            self.lifted
            and self.placed
            and self.released
            and not self.obstacle_collisions
        )


@dataclass(frozen=True)
class TrajectoryStage:
    """One serializable actuator command in a taught trajectory."""

    name: str
    command: str
    steps: int
    target: np.ndarray | float | None = None

    def as_dict(self) -> dict[str, object]:
        target: list[float] | float | None
        if isinstance(self.target, np.ndarray):
            target = self.target.tolist()
        else:
            target = self.target
        return {
            "name": self.name,
            "command": self.command,
            "steps": self.steps,
            "target": target,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "TrajectoryStage":
        command = str(payload["command"])
        raw_target = payload.get("target")
        if command == "move_arm":
            target: np.ndarray | float | None = np.asarray(raw_target, dtype=float)
            if target.shape != (ARM_JOINT_COUNT,):
                raise ValueError(
                    f"move_arm stage requires {ARM_JOINT_COUNT} joint targets."
                )
        elif command == "set_gripper":
            target = float(raw_target)
        elif command == "hold":
            target = None
        else:
            raise ValueError(f"Unknown trajectory command: {command!r}")
        steps = int(payload["steps"])
        if steps < 0:
            raise ValueError("Trajectory stage steps must be non-negative.")
        return cls(
            name=str(payload["name"]),
            command=command,
            steps=steps,
            target=target,
        )


class DampedLeastSquaresIK:
    """Numerical site IK that is reusable across either PiPER arm.

    Position and gripper approach direction are constrained. Rotation around
    the approach axis is intentionally left free so the same solver can reach
    objects across the workspace without over-constraining wrist roll.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        arm: ArmInterface,
        *,
        damping: float = 2e-3,
        step_size: float = 0.65,
        max_update_norm: float = 0.12,
        position_tolerance: float = 1.5e-3,
        rotation_tolerance: float = 2.5e-2,
        max_iterations: int = 600,
    ) -> None:
        self.model = model
        self.arm = arm
        self.damping = damping
        self.step_size = step_size
        self.max_update_norm = max_update_norm
        self.position_tolerance = position_tolerance
        self.rotation_tolerance = rotation_tolerance
        self.max_iterations = max_iterations

    def solve(
        self,
        pose: CartesianPose,
        seed: np.ndarray,
        *,
        constrain_approach: bool = True,
        validity: Callable[[np.ndarray], bool] | None = None,
    ) -> np.ndarray:
        candidates = [
            np.asarray(seed, dtype=float),
            np.mean(self.arm.joint_ranges, axis=1),
        ]
        rng = np.random.default_rng(20260725)
        candidates.extend(
            rng.uniform(self.arm.joint_ranges[:, 0], self.arm.joint_ranges[:, 1])
            for _ in range(32)
        )

        last_error: RuntimeError | None = None
        for candidate in candidates:
            try:
                solution = self._solve_from_seed(
                    pose,
                    candidate,
                    constrain_approach=constrain_approach,
                )
                if validity is not None and not validity(solution):
                    last_error = RuntimeError(
                        f"IK solution for {pose.name} intersects an obstacle."
                    )
                    continue
                return solution
            except RuntimeError as exc:
                last_error = exc
        assert last_error is not None
        raise RuntimeError(
            f"IK failed for {pose.name} after {len(candidates)} deterministic seeds. "
            f"Last attempt: {last_error}"
        )

    def solve_position(self, position: np.ndarray, seed: np.ndarray) -> np.ndarray:
        """Find a reachable configuration and let the wrist choose its own roll."""

        seed_pose = CartesianPose(
            name="position_seed",
            position=np.asarray(position, dtype=float),
            rotation=np.eye(3),
        )
        return self.solve(seed_pose, seed, constrain_approach=False)

    def _solve_from_seed(
        self,
        pose: CartesianPose,
        seed: np.ndarray,
        *,
        constrain_approach: bool,
    ) -> np.ndarray:
        data = mujoco.MjData(self.model)
        data.qpos[self.arm.qpos_indices] = np.asarray(seed, dtype=float)
        mujoco.mj_forward(self.model, data)

        jacobian_position = np.zeros((3, self.model.nv))
        jacobian_rotation = np.zeros((3, self.model.nv))

        for _ in range(self.max_iterations):
            mujoco.mj_forward(self.model, data)
            current_position = data.site_xpos[self.arm.grasp_site_id].copy()
            current_rotation = data.site_xmat[self.arm.grasp_site_id].reshape(3, 3)

            position_error = pose.position - current_position
            rotation_error = (
                orientation_error(current_rotation, pose.rotation)
                if constrain_approach
                else np.zeros(3)
            )
            position_converged = (
                np.linalg.norm(position_error) <= self.position_tolerance
            )
            rotation_converged = (
                not constrain_approach
                or np.linalg.norm(rotation_error) <= self.rotation_tolerance
            )
            if position_converged and rotation_converged:
                return data.qpos[self.arm.qpos_indices].copy()

            jacobian_position.fill(0)
            jacobian_rotation.fill(0)
            mujoco.mj_jacSite(
                self.model,
                data,
                jacobian_position,
                jacobian_rotation,
                self.arm.grasp_site_id,
            )
            if constrain_approach:
                jacobian = np.vstack(
                    (
                        jacobian_position[:, self.arm.dof_indices],
                        jacobian_rotation[:, self.arm.dof_indices],
                    )
                )
                error = np.concatenate((position_error, rotation_error))
            else:
                # Zero angular velocity regularizes the redundant position-only
                # solve toward the seed orientation without imposing a target.
                jacobian = np.vstack(
                    (
                        jacobian_position[:, self.arm.dof_indices],
                        jacobian_rotation[:, self.arm.dof_indices],
                    )
                )
                error = np.concatenate((position_error, np.zeros(3)))
            normal_matrix = jacobian @ jacobian.T
            update = jacobian.T @ np.linalg.solve(
                normal_matrix + self.damping**2 * np.eye(len(error)),
                error,
            )
            update_norm = float(np.linalg.norm(update))
            if update_norm > self.max_update_norm:
                update *= self.max_update_norm / update_norm

            next_joints = (
                data.qpos[self.arm.qpos_indices] + self.step_size * update
            )
            data.qpos[self.arm.qpos_indices] = np.clip(
                next_joints,
                self.arm.joint_ranges[:, 0],
                self.arm.joint_ranges[:, 1],
            )

        mujoco.mj_forward(self.model, data)
        final_position_error = np.linalg.norm(
            pose.position - data.site_xpos[self.arm.grasp_site_id]
        )
        final_rotation_error = (
            np.linalg.norm(
                orientation_error(
                    data.site_xmat[self.arm.grasp_site_id].reshape(3, 3),
                    pose.rotation,
                )
            )
            if constrain_approach
            else 0.0
        )
        raise RuntimeError(
            f"IK failed for {pose.name}: position error={final_position_error:.4f} m, "
            f"rotation error={final_rotation_error:.4f} rad."
        )


class PickPlaceController:
    """Builds and executes reusable pick-and-place pose sequences."""

    def __init__(
        self,
        scene_path: Path,
        *,
        arm_name: str,
        home_key: str = DEFAULT_HOME_KEY,
        initial_settle_steps: int = 300,
        realtime: bool = False,
        viewer: bool = False,
    ) -> None:
        self.scene_path = scene_path
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.arm = ArmInterface.resolve(self.model, arm_name)
        self.home_key_id = self.model.key(home_key).id
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_key_id)
        mujoco.mj_forward(self.model, self.data)
        self.home_ctrl = self.data.ctrl.copy()
        self.home_joints = self.data.qpos[self.arm.qpos_indices].copy()
        # Free objects in the scene may intentionally start a few millimetres
        # above the table. Plan from their settled physical position instead
        # of the raw MJCF pose so grasp height transfers across part sizes.
        self.data.ctrl[:] = self.home_ctrl
        for _ in range(max(0, initial_settle_steps)):
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.realtime = realtime
        self.viewer_requested = viewer
        self.viewer_handle = None
        self.ik = DampedLeastSquaresIK(self.model, self.arm)
        self.max_object_z = -np.inf
        self.target_object_id = -1
        self.obstacle_collisions: set[str] = set()
        self.current_stage = "idle"
        self.arm_body_ids = {
            body_id
            for body_id in range(self.model.nbody)
            if (self.model.body(body_id).name or "").endswith(f"_{self.arm.name}")
        }
        self.free_body_ids = {
            int(self.model.jnt_bodyid[joint_id])
            for joint_id in range(self.model.njnt)
            if int(self.model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE)
        }
        self.planning_data = mujoco.MjData(self.model)
        self.planning_data.qpos[:] = self.data.qpos
        self.planning_data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.planning_data)

    def __enter__(self) -> "PickPlaceController":
        if self.viewer_requested:
            import mujoco.viewer

            self.viewer_handle = mujoco.viewer.launch_passive(self.model, self.data)
        return self

    def __exit__(self, *_: object) -> None:
        if self.viewer_handle is not None:
            self.viewer_handle.close()
            self.viewer_handle = None

    def build_plan(
        self,
        object_name: str,
        place_xy: np.ndarray,
        *,
        approach_clearance: float,
        grasp_z_offset: float,
        place_z_offset: float,
        transit_height: float = DEFAULT_TRANSIT_HEIGHT,
    ) -> PickPlacePlan:
        object_id = self.model.body(object_name).id
        object_position = self.data.xpos[object_id].copy()

        pick_position = object_position + np.array([0.0, 0.0, grasp_z_offset])
        place_position = np.array(
            [
                float(place_xy[0]),
                float(place_xy[1]),
                object_position[2] + place_z_offset,
            ]
        )
        # Rear-row objects need an arm-specific outward wrist yaw so links
        # 4/5 do not sweep through the neighboring front-row cube.
        if object_position[1] > 0.05:
            approach_axis = (
                np.array([0.5, 0.866, -0.293])
                if self.arm.name == "arm0"
                else np.array([0.259, 0.966, -0.293])
            )
        else:
            approach_axis = DEFAULT_APPROACH_AXIS
        shared_rotation = rotation_from_approach_axis(approach_axis)
        clearance_rotation = rotation_from_approach_axis(DEFAULT_APPROACH_AXIS)

        return PickPlacePlan(
            object_name=object_name,
            home_pose=JointPose("home_pose", self.home_joints.copy()),
            pick_clearance_pose=CartesianPose(
                "pick_clearance_pose",
                np.array([pick_position[0], pick_position[1], transit_height]),
                clearance_rotation,
            ),
            pick_approach_pose=CartesianPose(
                "pick_approach_pose",
                pick_position + np.array([0.0, 0.0, approach_clearance]),
                shared_rotation,
            ),
            pick_pose=CartesianPose(
                "pick_pose",
                pick_position,
                shared_rotation,
            ),
            place_clearance_pose=CartesianPose(
                "place_clearance_pose",
                np.array([place_position[0], place_position[1], transit_height]),
                clearance_rotation,
            ),
            place_approach_pose=CartesianPose(
                "place_approach_pose",
                place_position + np.array([0.0, 0.0, approach_clearance]),
                clearance_rotation,
            ),
            place_pose=CartesianPose(
                "place_pose",
                place_position,
                clearance_rotation,
            ),
        )

    def solve_plan(self, plan: PickPlacePlan) -> dict[str, np.ndarray]:
        target_object_id = self.model.body(plan.object_name).id

        def clear_segment(start: np.ndarray) -> Callable[[np.ndarray], bool]:
            return lambda end: self.joint_path_is_clear(
                start,
                end,
                target_object_id,
            )

        solved = {plan.home_pose.name: plan.home_pose.joints.copy()}
        solved[plan.pick_clearance_pose.name] = self.ik.solve(
            plan.pick_clearance_pose,
            solved[plan.home_pose.name],
            validity=clear_segment(solved[plan.home_pose.name]),
        )
        solved[plan.pick_approach_pose.name] = self.ik.solve(
            plan.pick_approach_pose,
            solved[plan.pick_clearance_pose.name],
            validity=clear_segment(solved[plan.pick_clearance_pose.name]),
        )
        solved[plan.pick_pose.name] = self.ik.solve(
            plan.pick_pose,
            solved[plan.pick_approach_pose.name],
            validity=clear_segment(solved[plan.pick_approach_pose.name]),
        )
        solved[plan.place_clearance_pose.name] = self.ik.solve(
            plan.place_clearance_pose,
            solved[plan.pick_clearance_pose.name],
            validity=clear_segment(solved[plan.pick_clearance_pose.name]),
        )
        solved[plan.place_approach_pose.name] = self.ik.solve(
            plan.place_approach_pose,
            solved[plan.place_clearance_pose.name],
            validity=clear_segment(solved[plan.place_clearance_pose.name]),
        )
        solved[plan.place_pose.name] = self.ik.solve(
            plan.place_pose,
            solved[plan.place_approach_pose.name],
            validity=clear_segment(solved[plan.place_approach_pose.name]),
        )
        return solved

    def joint_path_is_clear(
        self,
        start: np.ndarray,
        end: np.ndarray,
        target_object_id: int,
        *,
        samples: int = 80,
    ) -> bool:
        """Check a sampled arm segment against every non-target free object."""

        for fraction in np.linspace(0.0, 1.0, samples):
            joints = start + fraction * (end - start)
            if self.configuration_obstacle_contacts(joints, target_object_id):
                return False
        return True

    def configuration_obstacle_contacts(
        self,
        joints: np.ndarray,
        target_object_id: int,
    ) -> tuple[str, ...]:
        self.planning_data.qpos[self.arm.qpos_indices] = joints
        mujoco.mj_forward(self.model, self.planning_data)
        obstacle_body_ids = self.free_body_ids - {target_object_id}
        contacts: set[str] = set()
        for contact_id in range(self.planning_data.ncon):
            contact = self.planning_data.contact[contact_id]
            body1 = int(self.model.geom_bodyid[int(contact.geom1)])
            body2 = int(self.model.geom_bodyid[int(contact.geom2)])
            if body1 in self.arm_body_ids and body2 in obstacle_body_ids:
                contacts.add(
                    f"{self.model.body(body1).name}->{self.model.body(body2).name}"
                )
            elif body2 in self.arm_body_ids and body1 in obstacle_body_ids:
                contacts.add(
                    f"{self.model.body(body2).name}->{self.model.body(body1).name}"
                )
        return tuple(sorted(contacts))

    def execute(
        self,
        object_name: str,
        plan: PickPlacePlan,
        solved: dict[str, np.ndarray],
        *,
        move_steps: int,
        descend_steps: int,
        gripper_steps: int,
        settle_steps: int,
        open_value: float,
        close_value: float,
        placement_tolerance: float,
        minimum_lift: float,
    ) -> ExecutionResult:
        trajectory = self.build_trajectory(
            plan,
            solved,
            move_steps=move_steps,
            descend_steps=descend_steps,
            gripper_steps=gripper_steps,
            settle_steps=settle_steps,
            open_value=open_value,
            close_value=close_value,
        )
        return self.execute_trajectory(
            object_name,
            plan.place_pose.position[:2],
            trajectory,
            placement_tolerance=placement_tolerance,
            minimum_lift=minimum_lift,
        )

    def build_trajectory(
        self,
        plan: PickPlacePlan,
        solved: dict[str, np.ndarray],
        *,
        move_steps: int,
        descend_steps: int,
        gripper_steps: int,
        settle_steps: int,
        open_value: float,
        close_value: float,
    ) -> tuple[TrajectoryStage, ...]:
        """Convert a solved plan into actuator-level stages for JSON storage."""

        move = "move_arm"
        grip = "set_gripper"
        hold = "hold"
        return (
            TrajectoryStage("open_before_pick", grip, gripper_steps, open_value),
            TrajectoryStage(
                plan.home_pose.name,
                move,
                move_steps,
                solved[plan.home_pose.name].copy(),
            ),
            TrajectoryStage(
                plan.pick_clearance_pose.name,
                move,
                move_steps,
                solved[plan.pick_clearance_pose.name].copy(),
            ),
            TrajectoryStage(
                plan.pick_approach_pose.name,
                move,
                move_steps,
                solved[plan.pick_approach_pose.name].copy(),
            ),
            TrajectoryStage(
                plan.pick_pose.name,
                move,
                descend_steps,
                solved[plan.pick_pose.name].copy(),
            ),
            TrajectoryStage("close_for_pick", grip, gripper_steps, close_value),
            TrajectoryStage("grasp_settle", hold, gripper_steps // 2),
            TrajectoryStage(
                "lift_after_pick",
                move,
                descend_steps,
                solved[plan.pick_approach_pose.name].copy(),
            ),
            TrajectoryStage(
                "pick_clearance_after_lift",
                move,
                descend_steps,
                solved[plan.pick_clearance_pose.name].copy(),
            ),
            TrajectoryStage(
                plan.place_clearance_pose.name,
                move,
                move_steps,
                solved[plan.place_clearance_pose.name].copy(),
            ),
            TrajectoryStage(
                plan.place_approach_pose.name,
                move,
                move_steps,
                solved[plan.place_approach_pose.name].copy(),
            ),
            TrajectoryStage(
                plan.place_pose.name,
                move,
                descend_steps,
                solved[plan.place_pose.name].copy(),
            ),
            TrajectoryStage("open_for_release", grip, gripper_steps, open_value),
            TrajectoryStage("release_settle", hold, settle_steps),
            TrajectoryStage(
                "retreat_after_place",
                move,
                descend_steps,
                solved[plan.place_approach_pose.name].copy(),
            ),
            TrajectoryStage(
                "place_clearance_after_release",
                move,
                descend_steps,
                solved[plan.place_clearance_pose.name].copy(),
            ),
            TrajectoryStage(
                "return_home",
                move,
                move_steps,
                solved[plan.home_pose.name].copy(),
            ),
            TrajectoryStage("final_settle", hold, settle_steps),
        )

    def execute_trajectory(
        self,
        object_name: str,
        target_xy: np.ndarray,
        trajectory: Iterable[TrajectoryStage],
        *,
        placement_tolerance: float,
        minimum_lift: float,
    ) -> ExecutionResult:
        """Replay recorded actuator stages without solving IK."""

        object_id = self.model.body(object_name).id
        initial_object_position = self.data.xpos[object_id].copy()
        self.max_object_z = float(initial_object_position[2])
        self.target_object_id = object_id
        self.obstacle_collisions.clear()
        lifted_object_position = initial_object_position.copy()

        self.data.ctrl[:] = self.home_ctrl
        for stage in trajectory:
            if stage.command == "move_arm":
                assert isinstance(stage.target, np.ndarray)
                self.move_arm(stage.name, stage.target, stage.steps, object_id)
            elif stage.command == "set_gripper":
                assert isinstance(stage.target, float)
                self.set_gripper(
                    stage.target,
                    stage.steps,
                    object_id,
                    stage_name=stage.name,
                )
            elif stage.command == "hold":
                self.hold(stage.steps, object_id, stage_name=stage.name)
            else:
                raise ValueError(
                    f"Unknown trajectory command in stage {stage.name!r}: "
                    f"{stage.command!r}"
                )
            if stage.name == "lift_after_pick":
                lifted_object_position = self.data.xpos[object_id].copy()

        final_object_position = self.data.xpos[object_id].copy()
        target_xy = np.asarray(target_xy, dtype=float).copy()
        if target_xy.shape != (2,):
            raise ValueError("target_xy must contain exactly two values.")
        lift_height = float(self.max_object_z - initial_object_position[2])
        placement_error = float(
            np.linalg.norm(final_object_position[:2] - target_xy)
        )
        lifted = lift_height >= minimum_lift
        placed = placement_error <= placement_tolerance
        released = not self.object_touches_gripper(object_id)
        return ExecutionResult(
            object_name=object_name,
            arm=self.arm.name,
            initial_object_position=initial_object_position,
            lifted_object_position=lifted_object_position,
            final_object_position=final_object_position,
            target_xy=target_xy,
            max_object_z=float(self.max_object_z),
            lift_height=lift_height,
            placement_error_xy=placement_error,
            lifted=lifted,
            placed=placed,
            released=released,
            obstacle_collisions=tuple(sorted(self.obstacle_collisions)),
        )

    def move_arm(
        self,
        stage_name: str,
        target_joints: np.ndarray,
        steps: int,
        object_id: int,
    ) -> None:
        self.current_stage = stage_name
        start = self.data.ctrl[self.arm.actuator_indices].copy()
        target = np.asarray(target_joints, dtype=float)
        for step in range(1, max(1, steps) + 1):
            phase = smoothstep(step / max(1, steps))
            self.data.ctrl[self.arm.actuator_indices] = (
                start + phase * (target - start)
            )
            self.step(object_id)
        self.log_stage(stage_name, object_id)

    def set_gripper(
        self,
        value: float,
        steps: int,
        object_id: int,
        *,
        stage_name: str = "gripper",
    ) -> None:
        self.current_stage = stage_name
        start = float(self.data.ctrl[self.arm.gripper_actuator_index])
        for step in range(1, max(1, steps) + 1):
            phase = smoothstep(step / max(1, steps))
            self.data.ctrl[self.arm.gripper_actuator_index] = (
                start + phase * (value - start)
            )
            self.step(object_id)

    def hold(
        self,
        steps: int,
        object_id: int,
        *,
        stage_name: str = "hold",
    ) -> None:
        self.current_stage = stage_name
        for _ in range(max(0, steps)):
            self.step(object_id)

    def step(self, object_id: int) -> None:
        mujoco.mj_step(self.model, self.data)
        self.record_obstacle_contacts()
        self.max_object_z = max(
            self.max_object_z,
            float(self.data.xpos[object_id, 2]),
        )
        if self.viewer_handle is not None:
            self.viewer_handle.sync()
        if self.realtime:
            time.sleep(self.model.opt.timestep)

    def record_obstacle_contacts(self) -> None:
        """Record moving-arm or carried-object contacts with other free objects."""

        if self.target_object_id < 0:
            return
        moving_body_ids = self.arm_body_ids | {self.target_object_id}
        obstacle_body_ids = self.free_body_ids - {self.target_object_id}

        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]
            body1 = int(self.model.geom_bodyid[int(contact.geom1)])
            body2 = int(self.model.geom_bodyid[int(contact.geom2)])
            if (
                (body1 in moving_body_ids and body2 in obstacle_body_ids)
                or (body2 in moving_body_ids and body1 in obstacle_body_ids)
            ):
                moving = body1 if body1 in moving_body_ids else body2
                obstacle = body2 if moving == body1 else body1
                self.obstacle_collisions.add(
                    f"{self.current_stage}:"
                    f"{self.model.body(moving).name}->{self.model.body(obstacle).name}"
                )

    def object_touches_gripper(self, object_id: int) -> bool:
        object_geoms = {
            geom_id
            for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) == object_id
        }
        gripper_bodies = {
            self.model.body(f"left_finger_{self.arm.name}").id,
            self.model.body(f"right_finger_{self.arm.name}").id,
        }
        gripper_geoms = {
            geom_id
            for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) in gripper_bodies
        }
        mujoco.mj_forward(self.model, self.data)
        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]
            pair = {int(contact.geom1), int(contact.geom2)}
            if pair & object_geoms and pair & gripper_geoms:
                return True
        return False

    def log_stage(self, stage_name: str, object_id: int) -> None:
        site_position = self.data.site_xpos[self.arm.grasp_site_id]
        object_position = self.data.xpos[object_id]
        print(
            f"{stage_name}: "
            f"grasp_site={np.round(site_position, 4).tolist()} "
            f"object={np.round(object_position, 4).tolist()}"
        )


def orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Align the gripper approach axis while leaving wrist roll unconstrained."""

    return np.cross(current[:, 2], target[:, 2])


def rotation_from_approach_axis(axis: np.ndarray) -> np.ndarray:
    """Build a right-handed rotation whose local +Z follows ``axis``."""

    approach = np.asarray(axis, dtype=float)
    approach /= np.linalg.norm(approach)
    lateral = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(lateral, approach))) > 0.95:
        lateral = np.array([0.0, 0.0, 1.0])
    lateral -= np.dot(lateral, approach) * approach
    lateral /= np.linalg.norm(lateral)
    vertical = np.cross(approach, lateral)
    return np.column_stack((lateral, vertical, approach))


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def default_scene_path() -> Path:
    return Path(__file__).resolve().parent.parent / "scene.xml"


def validate_scene_and_target(
    scene_path: Path,
    object_name: str,
    place_xy: np.ndarray,
) -> None:
    if not scene_path.is_file():
        raise FileNotFoundError(f"Scene not found: {scene_path}")
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    model.body(object_name)
    if not (-0.65 <= place_xy[0] <= 0.65 and -0.40 <= place_xy[1] <= 0.40):
        raise ValueError(
            f"Placement target {place_xy.tolist()} is outside the tabletop workspace."
        )
