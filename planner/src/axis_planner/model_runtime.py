"""Native MuJoCo model mirror used by the external planner process."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
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
        """Return one world-space body pose after forward kinematics."""

        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise KeyError(f"Unknown body: {body_name}")
        return {
            "position": [float(value) for value in self.data.xpos[body_id]],
            "quaternion": [float(value) for value in self.data.xquat[body_id]],
        }

    def mirror_report(self, body_names: Sequence[str] = ()) -> dict[str, Any]:
        """Return contacts and selected body poses for one mirrored snapshot."""

        return {
            "summary": self.summary().to_dict(),
            "contactCount": int(self.data.ncon),
            "contacts": [contact.to_dict() for contact in self.contacts()],
            "bodies": {name: self.body_pose(name) for name in body_names},
        }
