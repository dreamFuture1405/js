"""Tests for native IK and robot collision queries."""

from pathlib import Path

import pytest

from axis_planner.model_runtime import ModelRuntime


def _planar_arm(tmp_path: Path) -> Path:
    path = tmp_path / "arm.xml"
    path.write_text(
        """
<mujoco model="planar_arm">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="link1">
      <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
      <geom type="capsule" fromto="0 0 0 1 0 0" size=".03"/>
      <body name="link2" pos="1 0 0">
        <joint name="joint2" type="hinge" axis="0 0 1" range="-180 180"/>
        <geom type="capsule" fromto="0 0 0 1 0 0" size=".03"/>
        <body name="hand" pos="1 0 0">
          <geom type="sphere" size=".04"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    return path


def _sliding_robot(tmp_path: Path) -> Path:
    path = tmp_path / "collision.xml"
    path.write_text(
        """
<mujoco model="sliding_robot">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="robot">
      <joint name="slide" type="slide" axis="1 0 0" range="0 1"/>
      <geom name="robot_geom" type="sphere" size=".2"/>
    </body>
    <body name="obstacle" pos="1 0 0">
      <geom name="obstacle_geom" type="sphere" size=".2"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    return path


def _planar_slider_with_obstacle(tmp_path: Path) -> Path:
    path = tmp_path / "rrt.xml"
    path.write_text(
        """
<mujoco model="rrt_robot">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="robot">
      <joint name="x" type="slide" axis="1 0 0" range="0 1"/>
      <joint name="y" type="slide" axis="0 1 0" range="-1 1"/>
      <geom type="sphere" size=".1"/>
    </body>
    <body name="obstacle" pos=".5 0 0">
      <geom type="sphere" size=".24"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    return path


def test_solves_position_ik_from_the_live_seed(tmp_path: Path) -> None:
    runtime = ModelRuntime.load(_planar_arm(tmp_path))
    runtime.apply_state(qpos=[0.0, 0.0], qvel=[0.0, 0.0], ctrl=[])

    result = runtime.solve_body_ik(
        body_name="hand",
        joint_names=["joint1", "joint2"],
        target_position=[1.0, 1.0, 0.0],
        seed=[0.2, -0.2],
        maximum_iterations=300,
        position_tolerance=0.002,
    )

    assert result.success is True
    assert result.position_error < 0.002
    assert len(result.joints) == 2
    assert runtime.body_pose("hand")["position"] == pytest.approx([2.0, 0.0, 0.0])


def test_solves_position_and_xyzw_orientation_together(tmp_path: Path) -> None:
    runtime = ModelRuntime.load(_planar_arm(tmp_path))
    runtime.apply_state(qpos=[0.0, 0.0], qvel=[0.0, 0.0], ctrl=[])

    result = runtime.solve_body_ik(
        body_name="hand",
        joint_names=["joint1", "joint2"],
        target_position=[1.0, 1.0, 0.0],
        target_quaternion=[0.0, 0.0, 0.0, 1.0],
        seed=[0.3, -0.3],
        maximum_iterations=400,
        position_tolerance=0.003,
        orientation_tolerance=0.01,
    )

    assert result.success is True
    assert result.position_error < 0.003
    assert result.orientation_error < 0.01


def test_rejects_a_zero_length_target_quaternion(tmp_path: Path) -> None:
    runtime = ModelRuntime.load(_planar_arm(tmp_path))

    with pytest.raises(ValueError, match="non-zero"):
        runtime.solve_body_ik(
            body_name="hand",
            joint_names=["joint1", "joint2"],
            target_position=[1.0, 1.0, 0.0],
            target_quaternion=[0.0, 0.0, 0.0, 0.0],
        )


def test_reports_new_robot_collision_and_honors_allowed_target(tmp_path: Path) -> None:
    runtime = ModelRuntime.load(_sliding_robot(tmp_path))
    runtime.apply_state(qpos=[0.0], qvel=[0.0], ctrl=[])

    blocked = runtime.robot_configuration_collisions(
        joint_names=["slide"],
        joints=[0.7],
    )
    allowed = runtime.robot_configuration_collisions(
        joint_names=["slide"],
        joints=[0.7],
        allowed_body_names=["obstacle"],
    )

    assert len(blocked) >= 1
    assert {blocked[0].body1_name, blocked[0].body2_name} == {"robot", "obstacle"}
    assert allowed == []


def test_validates_every_interpolated_configuration_on_a_joint_path(
    tmp_path: Path,
) -> None:
    runtime = ModelRuntime.load(_sliding_robot(tmp_path))
    runtime.apply_state(qpos=[0.0], qvel=[0.0], ctrl=[])

    result = runtime.validate_robot_path(
        joint_names=["slide"],
        start=[0.0],
        goal=[1.0],
        maximum_joint_step=0.05,
    )

    assert result.valid is False
    assert result.reason == "collision"
    assert result.index > 0
    assert len(result.path) > 2


def test_accepts_a_clear_path_and_rejects_a_joint_limit_violation(
    tmp_path: Path,
) -> None:
    runtime = ModelRuntime.load(_sliding_robot(tmp_path))
    runtime.apply_state(qpos=[0.0], qvel=[0.0], ctrl=[])

    clear = runtime.validate_robot_path(
        joint_names=["slide"],
        start=[0.0],
        goal=[0.3],
        maximum_joint_step=0.05,
    )
    outside = runtime.validate_robot_path(
        joint_names=["slide"],
        start=[0.0],
        goal=[1.2],
        maximum_joint_step=0.05,
        allowed_body_names=["obstacle"],
    )

    assert clear.valid is True
    assert clear.reason is None
    assert outside.valid is False
    assert outside.reason == "joint_limit"


def test_rrt_connect_finds_a_deterministic_route_around_an_obstacle(
    tmp_path: Path,
) -> None:
    runtime = ModelRuntime.load(_planar_slider_with_obstacle(tmp_path))
    runtime.apply_state(qpos=[0.0, 0.0], qvel=[0.0, 0.0], ctrl=[])

    result = runtime.plan_robot_path(
        joint_names=["x", "y"],
        start=[0.0, 0.0],
        goal=[1.0, 0.0],
        maximum_joint_step=0.05,
        rrt_step=0.16,
        maximum_iterations=1500,
        random_seed=17,
    )

    assert result.success is True
    assert result.method == "rrt_connect"
    assert result.path[0] == pytest.approx([0.0, 0.0])
    assert result.path[-1] == pytest.approx([1.0, 0.0])
    assert any(abs(joints[1]) > 0.34 for joints in result.path)
    validated = runtime.validate_robot_path_samples(
        joint_names=["x", "y"],
        path=result.path,
        maximum_joint_step=0.05,
    )
    assert validated.valid is True


def test_rejects_unknown_or_non_scalar_robot_joints(tmp_path: Path) -> None:
    runtime = ModelRuntime.load(_planar_arm(tmp_path))

    with pytest.raises(KeyError, match="Unknown joint"):
        runtime.solve_body_ik(
            body_name="hand",
            joint_names=["missing"],
            target_position=[1.0, 1.0, 0.0],
        )
