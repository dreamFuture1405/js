"""Tests for the native MuJoCo model mirror."""

from pathlib import Path

import mujoco
import pytest

from axis_planner.model_runtime import ModelRuntime


@pytest.fixture
def model_path(tmp_path: Path) -> Path:
    """Create a compact free-body collision scene."""

    path = tmp_path / "scene.xml"
    path.write_text(
        """
<mujoco model="mirror_test">
  <option gravity="0 0 0"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 .1"/>
    <body name="item" pos="0 0 .2">
      <freejoint name="item_joint"/>
      <geom name="item_geom" type="box" size=".1 .1 .1"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    return path


def test_loads_model_and_reports_dimensions(model_path: Path) -> None:
    runtime = ModelRuntime.load(model_path)

    summary = runtime.summary()

    assert summary.nq == 7
    assert summary.nv == 6
    assert summary.nbody == 2
    assert summary.model_name


def test_rejects_invalid_state_dimensions(model_path: Path) -> None:
    runtime = ModelRuntime.load(model_path)

    with pytest.raises(ValueError, match="qpos"):
        runtime.apply_state(qpos=[0.0], qvel=[0.0] * 6, ctrl=[])


def test_mirrors_state_and_reports_native_contacts(model_path: Path) -> None:
    runtime = ModelRuntime.load(model_path)
    qpos = [0.0, 0.0, 0.05, 1.0, 0.0, 0.0, 0.0]

    runtime.apply_state(qpos=qpos, qvel=[0.0] * 6, ctrl=[])
    report = runtime.mirror_report(["item"])

    assert report["contactCount"] >= 1
    assert report["contacts"][0]["body2_name"] == "item"
    assert report["bodies"]["item"]["position"][2] == pytest.approx(0.05)
    assert report["bodies"]["item"]["quaternion"] == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_reports_unknown_body(model_path: Path) -> None:
    runtime = ModelRuntime.load(model_path)

    with pytest.raises(KeyError, match="Unknown body"):
        runtime.body_pose("missing")


def test_rejects_missing_or_non_xml_models(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        ModelRuntime.load(tmp_path / "missing.xml")
    text_path = tmp_path / "model.txt"
    text_path.write_text("<mujoco/>", encoding="utf-8")
    with pytest.raises(ValueError, match="XML"):
        ModelRuntime.load(text_path)


def test_body_name_falls_back_for_unnamed_world(model_path: Path) -> None:
    runtime = ModelRuntime.load(model_path)

    assert runtime.body_name(0) == "world"
    assert mujoco.mj_name2id(runtime.model, mujoco.mjtObj.mjOBJ_BODY, "item") == 1


def test_describes_live_body_geometry_and_joint_frames(model_path: Path) -> None:
    runtime = ModelRuntime.load(model_path)

    report = runtime.manipulation_geometry(
        body_names=["item"],
        joint_names=["item_joint"],
    )

    assert report["bodies"]["item"]["position"][2] == pytest.approx(0.2)
    assert report["bodies"]["item"]["geoms"][0]["contype"] == 1
    assert len(report["bodies"]["item"]["geoms"][0]["axes"]) == 3
    assert report["joints"]["item_joint"]["type"] == "free"
    assert len(report["joints"]["item_joint"]["anchor"]) == 3
