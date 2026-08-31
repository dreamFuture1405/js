"""Native MuJoCo model mirror used by the external planner process."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
from math import ceil
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class ModelSummary:
    """Serializable dimensions for one loaded MuJoCo model."""

    path: str
    model_name: str
    nq: int
    nv: int
    nu: int
    nbody: int
    ngeom: int
    njnt: int

    def to_dict(self) -> dict[str, str | int]:
        """Return a JSON-compatible summary."""

        return asdict(self)


@dataclass(frozen=True, slots=True)
class ContactPair:
    """One native MuJoCo contact mapped from geoms to bodies."""

    geom1: int
    geom2: int
    body1: int
    body2: int
    body1_name: str
    body2_name: str
    distance: float

    def to_dict(self) -> dict[str, str | int | float]:
        """Return a JSON-compatible contact record."""

        return asdict(self)


@dataclass(frozen=True, slots=True)
class IkResult:
    """Result of one damped-least-squares body-pose solve."""

    success: bool
    joints: list[float]
    position_error: float
    orientation_error: float
    iterations: int

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible IK result."""

        return asdict(self)


@dataclass(frozen=True, slots=True)
class PathValidation:
    """Collision and joint-limit result for an interpolated robot path."""

    valid: bool
    reason: str | None
    index: int
    path: list[list[float]]
    collisions: list[ContactPair]

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible path validation result."""

        result = asdict(self)
        result["collisions"] = [contact.to_dict() for contact in self.collisions]
        return result


@dataclass(frozen=True, slots=True)
class PlannedPath:
    """Direct or sampling-based robot path result."""

    success: bool
    method: str
    path: list[list[float]]
    reason: str | None
    iterations: int

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible planned path."""

        return asdict(self)


def _as_float_vector(
    values: Sequence[float],
    *,
    expected: int,
    label: str,
) -> NDArray[np.float64]:
    vector = np.asarray(values, dtype=np.float64)
    if vector.ndim != 1 or vector.shape[0] != expected:
        raise ValueError(f"{label} must contain {expected} values, received {vector.shape}")
    if not np.isfinite(vector).all():
        raise ValueError(f"{label} contains non-finite values")
    return vector


class ModelRuntime:
    """Own an independent native MuJoCo model and mutable planning data."""

    def __init__(self, model_path: Path, model: mujoco.MjModel) -> None:
        """Initialize a loaded model runtime.

        Args:
            model_path: Absolute XML path used to load the model.
            model: Native MuJoCo model.
        """

        self.model_path = model_path
        self.model = model
        self.data = mujoco.MjData(model)
        self._baseline_contact_keys: set[tuple[int, int]] = set()

    @classmethod
    def load(cls, model_path: Path) -> ModelRuntime:
        """Load an MJCF model and all relative assets from disk.

        Args:
            model_path: Path to the exported task XML.

        Returns:
            A native model runtime independent from browser WASM.

        Raises:
            FileNotFoundError: If the task XML does not exist.
            ValueError: If the path is not an XML file.
        """

        absolute_path = model_path.resolve()
        if not absolute_path.is_file():
            raise FileNotFoundError(f"Model XML does not exist: {absolute_path}")
        if absolute_path.suffix.lower() != ".xml":
            raise ValueError(f"Expected an XML model path: {absolute_path}")
        model = mujoco.MjModel.from_xml_path(str(absolute_path))
        runtime = cls(absolute_path, model)
        mujoco.mj_forward(runtime.model, runtime.data)
        runtime._capture_baseline_contacts()
        return runtime

    def summary(self) -> ModelSummary:
        """Describe the loaded native model."""

        model_name = bytes(self.model.names).split(b"\x00", 1)[0].decode("utf-8", errors="replace")
        return ModelSummary(
            path=str(self.model_path),
            model_name=model_name or self.model_path.stem,
            nq=int(self.model.nq),
            nv=int(self.model.nv),
            nu=int(self.model.nu),
            nbody=int(self.model.nbody),
            ngeom=int(self.model.ngeom),
            njnt=int(self.model.njnt),
        )

    def apply_state(
        self,
        *,
        qpos: Sequence[float],
        qvel: Sequence[float],
        ctrl: Sequence[float],
    ) -> None:
        """Mirror a browser simulation state and run forward kinematics.

        Args:
            qpos: Full MuJoCo generalized positions.
            qvel: Full MuJoCo generalized velocities.
            ctrl: Full actuator controls.
        """

        np.copyto(
            self.data.qpos,
            _as_float_vector(qpos, expected=int(self.model.nq), label="qpos"),
        )
        np.copyto(
            self.data.qvel,
            _as_float_vector(qvel, expected=int(self.model.nv), label="qvel"),
        )
        np.copyto(
            self.data.ctrl,
            _as_float_vector(ctrl, expected=int(self.model.nu), label="ctrl"),
        )
        mujoco.mj_forward(self.model, self.data)
        self._capture_baseline_contacts()

    def _capture_baseline_contacts(self) -> None:
        """Remember contact pairs that already exist in the mirrored live state."""

        self._baseline_contact_keys = {
            (
                min(contact.geom1, contact.geom2),
                max(contact.geom1, contact.geom2),
            )
            for contact in self.contacts()
        }

    def _scalar_joint_layout(
        self,
        joint_names: Sequence[str],
    ) -> tuple[list[int], list[int], list[int]]:
        """Resolve scalar hinge/slide joint ids and their qpos/dof addresses."""

        joint_ids: list[int] = []
        qpos_addresses: list[int] = []
        dof_addresses: list[int] = []
        scalar_types = {
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        }
        for name in joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                str(name),
            )
            if joint_id < 0:
                raise KeyError(f"Unknown joint: {name}")
            if int(self.model.jnt_type[joint_id]) not in scalar_types:
                raise ValueError(f"Robot joint must be hinge or slide: {name}")
            joint_ids.append(joint_id)
            qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            dof_addresses.append(int(self.model.jnt_dofadr[joint_id]))
        if not joint_ids:
            raise ValueError("At least one robot joint is required")
        return joint_ids, qpos_addresses, dof_addresses

    def _write_scalar_joints(
        self,
        joint_ids: Sequence[int],
        qpos_addresses: Sequence[int],
        joints: Sequence[float],
    ) -> None:
        """Write bounded scalar joint positions into planning data."""

        values = _as_float_vector(
            joints,
            expected=len(qpos_addresses),
            label="robot joints",
        )
        for index, (joint_id, address) in enumerate(zip(joint_ids, qpos_addresses, strict=True)):
            value = float(values[index])
            if bool(self.model.jnt_limited[joint_id]):
                minimum, maximum = self.model.jnt_range[joint_id]
                value = float(np.clip(value, minimum, maximum))
            self.data.qpos[address] = value

    def _robot_body_ids(self, joint_ids: Sequence[int]) -> set[int]:
        """Return bodies controlled by any selected joint, including descendants."""

        roots = {int(self.model.jnt_bodyid[joint_id]) for joint_id in joint_ids}
        robot_bodies: set[int] = set()
        for body_id in range(1, int(self.model.nbody)):
            ancestor = body_id
            while ancestor > 0:
                if ancestor in roots:
                    robot_bodies.add(body_id)
                    break
                ancestor = int(self.model.body_parentid[ancestor])
        return robot_bodies

    def solve_body_ik(
        self,
        *,
        body_name: str,
        joint_names: Sequence[str],
        target_position: Sequence[float],
        target_quaternion: Sequence[float] | None = None,
        seed: Sequence[float] | None = None,
        maximum_iterations: int = 240,
        position_tolerance: float = 0.003,
        orientation_tolerance: float = 0.03,
        damping: float = 0.04,
        maximum_joint_step: float = 0.16,
    ) -> IkResult:
        """Solve one body pose from the current mirrored state without mutating it.

        Quaternion input follows the external XYZW protocol.
        """

        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name,
        )
        if body_id < 0:
            raise KeyError(f"Unknown body: {body_name}")
        joint_ids, qpos_addresses, dof_addresses = self._scalar_joint_layout(joint_names)
        target = _as_float_vector(
            target_position,
            expected=3,
            label="target position",
        )
        target_matrix: NDArray[np.float64] | None = None
        if target_quaternion is not None:
            xyzw = _as_float_vector(
                target_quaternion,
                expected=4,
                label="target quaternion",
            )
            norm = float(np.linalg.norm(xyzw))
            if norm <= 1e-12:
                raise ValueError("target quaternion must have non-zero length")
            x, y, z, w = xyzw / norm
            target_matrix_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(
                target_matrix_flat,
                np.asarray([w, x, y, z], dtype=np.float64),
            )
            target_matrix = target_matrix_flat.reshape(3, 3)
        original_qpos = self.data.qpos.copy()
        initial = (
            [float(self.data.qpos[address]) for address in qpos_addresses] if seed is None else seed
        )
        self._write_scalar_joints(joint_ids, qpos_addresses, initial)
        mujoco.mj_forward(self.model, self.data)
        jacobian_position = np.zeros((3, int(self.model.nv)), dtype=np.float64)
        jacobian_rotation = np.zeros((3, int(self.model.nv)), dtype=np.float64)
        position_error = float("inf")
        orientation_error = 0.0
        completed_iterations = 0
        try:
            for iteration in range(max(1, int(maximum_iterations))):
                completed_iterations = iteration + 1
                position_delta = target - self.data.xpos[body_id]
                position_error = float(np.linalg.norm(position_delta))
                orientation_delta = np.zeros(3, dtype=np.float64)
                if target_matrix is not None:
                    current_matrix = self.data.xmat[body_id].reshape(3, 3)
                    orientation_delta = 0.5 * (
                        np.cross(current_matrix[:, 0], target_matrix[:, 0])
                        + np.cross(current_matrix[:, 1], target_matrix[:, 1])
                        + np.cross(current_matrix[:, 2], target_matrix[:, 2])
                    )
                    orientation_error = float(np.linalg.norm(orientation_delta))
                if position_error <= position_tolerance and (
                    target_matrix is None or orientation_error <= orientation_tolerance
                ):
                    break
                mujoco.mj_jacBody(
                    self.model,
                    self.data,
                    jacobian_position,
                    jacobian_rotation,
                    body_id,
                )
                columns = np.asarray(dof_addresses, dtype=np.int64)
                if target_matrix is None:
                    jacobian = jacobian_position[:, columns]
                    error = position_delta
                else:
                    jacobian = np.vstack(
                        (
                            jacobian_position[:, columns],
                            jacobian_rotation[:, columns],
                        )
                    )
                    error = np.concatenate((position_delta, orientation_delta))
                regularization = max(1e-6, float(damping)) ** 2
                normal = (
                    jacobian @ jacobian.T
                    + np.eye(jacobian.shape[0], dtype=np.float64) * regularization
                )
                delta = jacobian.T @ np.linalg.solve(normal, error)
                largest_step = float(np.max(np.abs(delta), initial=0.0))
                step_limit = max(0.001, float(maximum_joint_step))
                if largest_step > step_limit:
                    delta *= step_limit / largest_step
                current = [float(self.data.qpos[address]) for address in qpos_addresses]
                self._write_scalar_joints(
                    joint_ids,
                    qpos_addresses,
                    (np.asarray(current, dtype=np.float64) + delta).tolist(),
                )
                mujoco.mj_forward(self.model, self.data)
            solved = [float(self.data.qpos[address]) for address in qpos_addresses]
            success = position_error <= position_tolerance and (
                target_matrix is None or orientation_error <= orientation_tolerance
            )
            return IkResult(
                success=success,
                joints=solved,
                position_error=position_error,
                orientation_error=orientation_error,
                iterations=completed_iterations,
            )
        finally:
            np.copyto(self.data.qpos, original_qpos)
            mujoco.mj_forward(self.model, self.data)

    def robot_configuration_collisions(
        self,
        *,
        joint_names: Sequence[str],
        joints: Sequence[float],
        allowed_body_names: Sequence[str] = (),
    ) -> list[ContactPair]:
        """Return new robot contacts at one configuration and restore live state."""

        joint_ids, qpos_addresses, _ = self._scalar_joint_layout(joint_names)
        robot_body_ids = self._robot_body_ids(joint_ids)
        allowed = {str(name) for name in allowed_body_names}
        original_qpos = self.data.qpos.copy()
        try:
            self._write_scalar_joints(joint_ids, qpos_addresses, joints)
            mujoco.mj_forward(self.model, self.data)
            collisions: list[ContactPair] = []
            for contact in self.contacts():
                key = tuple(sorted((contact.geom1, contact.geom2)))
                if contact.distance > 0.001 or key in self._baseline_contact_keys:
                    continue
                first_robot = contact.body1 in robot_body_ids
                second_robot = contact.body2 in robot_body_ids
                if not first_robot and not second_robot:
                    continue
                environment_name = (
                    contact.body2_name
                    if first_robot and not second_robot
                    else contact.body1_name if second_robot and not first_robot else None
                )
                if environment_name is not None and environment_name in allowed:
                    continue
                collisions.append(contact)
            return collisions
        finally:
            np.copyto(self.data.qpos, original_qpos)
            mujoco.mj_forward(self.model, self.data)

    def validate_robot_path(
        self,
        *,
        joint_names: Sequence[str],
        start: Sequence[float],
        goal: Sequence[float],
        maximum_joint_step: float = 0.05,
        allowed_body_names: Sequence[str] = (),
    ) -> PathValidation:
        """Interpolate and collision-check every configuration on a direct path."""

        return self.validate_robot_path_samples(
            joint_names=joint_names,
            path=[list(start), list(goal)],
            maximum_joint_step=maximum_joint_step,
            allowed_body_names=allowed_body_names,
        )

    def validate_robot_path_samples(
        self,
        *,
        joint_names: Sequence[str],
        path: Sequence[Sequence[float]],
        maximum_joint_step: float = 0.05,
        allowed_body_names: Sequence[str] = (),
    ) -> PathValidation:
        """Densify and validate every edge in an arbitrary joint-space path."""

        joint_ids, _, _ = self._scalar_joint_layout(joint_names)
        if not path:
            raise ValueError("Path requires at least one configuration")
        nodes = [
            _as_float_vector(
                joints,
                expected=len(joint_ids),
                label=f"path node {index}",
            )
            for index, joints in enumerate(path)
        ]
        step = max(0.001, float(maximum_joint_step))
        dense: list[list[float]] = [[float(value) for value in nodes[0]]]
        for edge_index in range(1, len(nodes)):
            start_vector = nodes[edge_index - 1]
            goal_vector = nodes[edge_index]
            count = max(
                1,
                ceil(float(np.max(np.abs(goal_vector - start_vector))) / step),
            )
            dense.extend(
                [
                    [
                        float(value)
                        for value in start_vector + (goal_vector - start_vector) * (index / count)
                    ]
                    for index in range(1, count + 1)
                ]
            )
        for index, joints in enumerate(dense):
            for joint_index, joint_id in enumerate(joint_ids):
                if not bool(self.model.jnt_limited[joint_id]):
                    continue
                minimum, maximum = self.model.jnt_range[joint_id]
                if joints[joint_index] < minimum or joints[joint_index] > maximum:
                    return PathValidation(
                        valid=False,
                        reason="joint_limit",
                        index=index,
                        path=dense,
                        collisions=[],
                    )
            collisions = self.robot_configuration_collisions(
                joint_names=joint_names,
                joints=joints,
                allowed_body_names=allowed_body_names,
            )
            if collisions:
                return PathValidation(
                    valid=False,
                    reason="collision",
                    index=index,
                    path=dense,
                    collisions=collisions,
                )
        return PathValidation(
            valid=True,
            reason=None,
            index=-1,
            path=dense,
            collisions=[],
        )

    def plan_robot_path(
        self,
        *,
        joint_names: Sequence[str],
        start: Sequence[float],
        goal: Sequence[float],
        maximum_joint_step: float = 0.05,
        allowed_body_names: Sequence[str] = (),
        rrt_step: float = 0.18,
        maximum_iterations: int = 1200,
        goal_bias: float = 0.2,
        random_seed: int = 1,
    ) -> PlannedPath:
        """Plan a direct path, falling back to deterministic RRT-Connect."""

        joint_ids, _, _ = self._scalar_joint_layout(joint_names)
        start_vector = _as_float_vector(
            start,
            expected=len(joint_ids),
            label="path start",
        )
        goal_vector = _as_float_vector(
            goal,
            expected=len(joint_ids),
            label="path goal",
        )
        direct = self.validate_robot_path_samples(
            joint_names=joint_names,
            path=[start_vector.tolist(), goal_vector.tolist()],
            maximum_joint_step=maximum_joint_step,
            allowed_body_names=allowed_body_names,
        )
        if direct.valid:
            return PlannedPath(
                success=True,
                method="direct",
                path=direct.path,
                reason=None,
                iterations=0,
            )

        bounds: list[tuple[float, float]] = []
        for index, joint_id in enumerate(joint_ids):
            if bool(self.model.jnt_limited[joint_id]):
                minimum, maximum = self.model.jnt_range[joint_id]
                bounds.append((float(minimum), float(maximum)))
            else:
                center = float(start_vector[index])
                bounds.append((center - np.pi, center + np.pi))

        def configuration_valid(configuration: NDArray[np.float64]) -> bool:
            for index, (minimum, maximum) in enumerate(bounds):
                if configuration[index] < minimum or configuration[index] > maximum:
                    return False
            return not self.robot_configuration_collisions(
                joint_names=joint_names,
                joints=configuration.tolist(),
                allowed_body_names=allowed_body_names,
            )

        if not configuration_valid(start_vector) or not configuration_valid(goal_vector):
            return PlannedPath(
                success=False,
                method="rrt_connect",
                path=[],
                reason="endpoint_invalid",
                iterations=0,
            )

        rng = np.random.default_rng(int(random_seed))
        tree_a: list[NDArray[np.float64]] = [start_vector.copy()]
        parents_a = [-1]
        tree_b: list[NDArray[np.float64]] = [goal_vector.copy()]
        parents_b = [-1]
        tree_a_starts_at_start = True
        step = max(0.01, float(rrt_step))

        def nearest(
            tree: Sequence[NDArray[np.float64]],
            target: NDArray[np.float64],
        ) -> int:
            return min(
                range(len(tree)),
                key=lambda index: float(np.linalg.norm(tree[index] - target)),
            )

        def extend(
            tree: list[NDArray[np.float64]],
            parents: list[int],
            target: NDArray[np.float64],
        ) -> tuple[str, int]:
            nearest_index = nearest(tree, target)
            source = tree[nearest_index]
            delta = target - source
            distance = float(np.linalg.norm(delta))
            if distance <= 1e-12:
                return "reached", nearest_index
            candidate = target.copy() if distance <= step else source + delta / distance * step
            edge = self.validate_robot_path_samples(
                joint_names=joint_names,
                path=[source.tolist(), candidate.tolist()],
                maximum_joint_step=maximum_joint_step,
                allowed_body_names=allowed_body_names,
            )
            if not edge.valid:
                return "trapped", nearest_index
            tree.append(candidate)
            parents.append(nearest_index)
            return ("reached" if distance <= step else "advanced"), len(tree) - 1

        def trace(
            tree: Sequence[NDArray[np.float64]],
            parents: Sequence[int],
            index: int,
        ) -> list[NDArray[np.float64]]:
            output: list[NDArray[np.float64]] = []
            while index >= 0:
                output.append(tree[index])
                index = parents[index]
            output.reverse()
            return output

        iteration_limit = max(1, int(maximum_iterations))
        for iteration in range(1, iteration_limit + 1):
            if rng.random() < max(0.0, min(1.0, float(goal_bias))):
                sample = tree_b[0].copy()
            else:
                sample = np.asarray(
                    [rng.uniform(minimum, maximum) for minimum, maximum in bounds],
                    dtype=np.float64,
                )
            status_a, index_a = extend(tree_a, parents_a, sample)
            if status_a != "trapped":
                while True:
                    status_b, index_b = extend(
                        tree_b,
                        parents_b,
                        tree_a[index_a],
                    )
                    if status_b == "trapped":
                        break
                    if status_b == "reached":
                        path_a = trace(tree_a, parents_a, index_a)
                        path_b = trace(tree_b, parents_b, index_b)
                        sparse = (
                            path_a + list(reversed(path_b))[1:]
                            if tree_a_starts_at_start
                            else path_b + list(reversed(path_a))[1:]
                        )
                        validated = self.validate_robot_path_samples(
                            joint_names=joint_names,
                            path=[node.tolist() for node in sparse],
                            maximum_joint_step=maximum_joint_step,
                            allowed_body_names=allowed_body_names,
                        )
                        if validated.valid:
                            return PlannedPath(
                                success=True,
                                method="rrt_connect",
                                path=validated.path,
                                reason=None,
                                iterations=iteration,
                            )
                        break
            tree_a, tree_b = tree_b, tree_a
            parents_a, parents_b = parents_b, parents_a
            tree_a_starts_at_start = not tree_a_starts_at_start

        return PlannedPath(
            success=False,
            method="rrt_connect",
            path=[],
            reason=direct.reason or "rrt_connect_failed",
            iterations=iteration_limit,
        )

    def body_name(self, body_id: int) -> str:
        """Resolve a body id without leaking null names into the protocol."""

        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        return name if name is not None else f"body:{body_id}"

    def contacts(self) -> list[ContactPair]:
        """Read native contacts without touching browser WASM wrappers."""

        pairs: list[ContactPair] = []
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            body1 = int(self.model.geom_bodyid[geom1])
            body2 = int(self.model.geom_bodyid[geom2])
            pairs.append(
                ContactPair(
                    geom1=geom1,
                    geom2=geom2,
                    body1=body1,
                    body2=body2,
                    body1_name=self.body_name(body1),
                    body2_name=self.body_name(body2),
                    distance=float(contact.dist),
                )
            )
        return pairs

    def body_pose(self, body_name: str) -> dict[str, list[float]]:
        """Return one world-space body pose using protocol quaternion order XYZW."""

        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise KeyError(f"Unknown body: {body_name}")
        w, x, y, z = (float(value) for value in self.data.xquat[body_id])
        return {
            "position": [float(value) for value in self.data.xpos[body_id]],
            "quaternion": [x, y, z, w],
        }

    def manipulation_geometry(
        self,
        *,
        body_names: Sequence[str] = (),
        joint_names: Sequence[str] = (),
    ) -> dict[str, Any]:
        """Describe live body geoms and joint frames for semantic target generation."""

        bodies: dict[str, Any] = {}
        for body_name in body_names:
            body_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                body_name,
            )
            if body_id < 0:
                raise KeyError(f"Unknown body: {body_name}")
            geoms: list[dict[str, Any]] = []
            for geom_id in range(int(self.model.ngeom)):
                if int(self.model.geom_bodyid[geom_id]) != body_id:
                    continue
                matrix = self.data.geom_xmat[geom_id].reshape(3, 3)
                geoms.append(
                    {
                        "id": geom_id,
                        "name": mujoco.mj_id2name(
                            self.model,
                            mujoco.mjtObj.mjOBJ_GEOM,
                            geom_id,
                        ),
                        "type": int(self.model.geom_type[geom_id]),
                        "contype": int(self.model.geom_contype[geom_id]),
                        "conaffinity": int(self.model.geom_conaffinity[geom_id]),
                        "position": [float(value) for value in self.data.geom_xpos[geom_id]],
                        "axes": [
                            [float(value) for value in matrix[:, column]] for column in range(3)
                        ],
                        "size": [float(value) for value in self.model.geom_size[geom_id]],
                        "rbound": float(self.model.geom_rbound[geom_id]),
                    }
                )
            bodies[str(body_name)] = {
                **self.body_pose(str(body_name)),
                "id": body_id,
                "geoms": geoms,
            }

        joint_type_names = {
            int(mujoco.mjtJoint.mjJNT_FREE): "free",
            int(mujoco.mjtJoint.mjJNT_BALL): "ball",
            int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
            int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
        }
        joints: dict[str, Any] = {}
        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            if joint_id < 0:
                raise KeyError(f"Unknown joint: {joint_name}")
            joint_type = int(self.model.jnt_type[joint_id])
            qpos_address = int(self.model.jnt_qposadr[joint_id])
            qpos_count = (
                7
                if joint_type == int(mujoco.mjtJoint.mjJNT_FREE)
                else (4 if joint_type == int(mujoco.mjtJoint.mjJNT_BALL) else 1)
            )
            joints[str(joint_name)] = {
                "id": joint_id,
                "type": joint_type_names[joint_type],
                "bodyId": int(self.model.jnt_bodyid[joint_id]),
                "qposAddress": qpos_address,
                "dofAddress": int(self.model.jnt_dofadr[joint_id]),
                "qpos": [
                    float(value)
                    for value in self.data.qpos[qpos_address : qpos_address + qpos_count]
                ],
                "range": [float(value) for value in self.model.jnt_range[joint_id]],
                "anchor": [float(value) for value in self.data.xanchor[joint_id]],
                "axis": [float(value) for value in self.data.xaxis[joint_id]],
            }
        return {"bodies": bodies, "joints": joints}

    def mirror_report(self, body_names: Sequence[str] = ()) -> dict[str, Any]:
        """Return contacts and selected body poses for one mirrored snapshot."""

        return {
            "summary": self.summary().to_dict(),
            "contactCount": int(self.data.ncon),
            "contacts": [contact.to_dict() for contact in self.contacts()],
            "bodies": {name: self.body_pose(name) for name in body_names},
        }
