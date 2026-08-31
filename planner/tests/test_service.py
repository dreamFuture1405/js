"""Tests for the JSON-lines planner protocol."""

import io
import json
from pathlib import Path

import pytest

from axis_planner.service import PlannerService, serve


def _model(tmp_path: Path) -> Path:
    path = tmp_path / "service.xml"
    path.write_text(
        """
<mujoco model="service_test">
  <worldbody>
    <body name="body">
      <freejoint/>
      <geom type="sphere" size=".1"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    return path


def _planning_model(tmp_path: Path) -> Path:
    path = tmp_path / "planning_service.xml"
    path.write_text(
        """
<mujoco model="planning_service">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="robot">
      <joint name="slide" type="slide" axis="1 0 0" range="0 1"/>
      <geom type="sphere" size=".2"/>
    </body>
    <body name="obstacle" pos="1 0 0">
      <geom type="sphere" size=".2"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    return path


def test_load_health_and_mirror_protocol(tmp_path: Path) -> None:
    service = PlannerService()
    model_path = _model(tmp_path)

    assert service.handle({"id": 1, "method": "health"})["result"]["ready"] is False
    loaded = service.handle({"id": 2, "method": "load_model", "params": {"path": str(model_path)}})
    mirrored = service.handle(
        {
            "id": 3,
            "method": "mirror_state",
            "params": {
                "simulation": {
                    "qpos": [0, 0, 0, 1, 0, 0, 0],
                    "qvel": [0] * 6,
                    "ctrl": [],
                },
                "bodyNames": ["body"],
            },
        }
    )

    assert loaded["ok"] is True
    assert mirrored["ok"] is True
    assert mirrored["result"]["bodies"]["body"]["position"] == [0.0, 0.0, 0.0]


def test_returns_structured_errors() -> None:
    service = PlannerService()

    response = service.handle({"id": 1, "method": "mirror_state", "params": {}})
    unknown = service.handle({"id": 2, "method": "unknown"})

    assert response["ok"] is False
    assert response["error"]["type"] in {"KeyError", "ValueError"}
    assert unknown["error"]["message"] == "Unknown planner method: unknown"


def test_exposes_ik_and_path_validation_protocol(tmp_path: Path) -> None:
    service = PlannerService()
    model_path = _planning_model(tmp_path)
    service.handle({"id": 1, "method": "load_model", "params": {"path": str(model_path)}})
    service.handle(
        {
            "id": 2,
            "method": "mirror_state",
            "params": {
                "simulation": {"qpos": [0.0], "qvel": [0.0], "ctrl": []},
            },
        }
    )

    ik = service.handle(
        {
            "id": 3,
            "method": "solve_ik",
            "params": {
                "bodyName": "robot",
                "jointNames": ["slide"],
                "targetPosition": [0.5, 0.0, 0.0],
            },
        }
    )
    path = service.handle(
        {
            "id": 4,
            "method": "validate_path",
            "params": {
                "jointNames": ["slide"],
                "start": [0.0],
                "goal": [1.0],
                "maximumJointStep": 0.05,
            },
        }
    )
    planned = service.handle(
        {
            "id": 5,
            "method": "plan_path",
            "params": {
                "jointNames": ["slide"],
                "start": [0.0],
                "goal": [0.3],
                "maximumJointStep": 0.05,
            },
        }
    )
    geometry = service.handle(
        {
            "id": 6,
            "method": "describe_geometry",
            "params": {
                "bodyNames": ["robot"],
                "jointNames": ["slide"],
            },
        }
    )
    configuration = service.handle(
        {
            "id": 7,
            "method": "check_configuration",
            "params": {
                "jointNames": ["slide"],
                "joints": [0.7],
            },
        }
    )

    assert ik["ok"] is True
    assert ik["result"]["success"] is True
    assert ik["result"]["joints"][0] == pytest.approx(0.5, abs=0.003)
    assert path["ok"] is True
    assert path["result"]["valid"] is False
    assert path["result"]["reason"] == "collision"
    assert planned["ok"] is True
    assert planned["result"]["success"] is True
    assert planned["result"]["method"] == "direct"
    assert geometry["ok"] is True
    assert geometry["result"]["joints"]["slide"]["type"] == "slide"
    assert len(geometry["result"]["bodies"]["robot"]["geoms"]) == 1
    assert configuration["ok"] is True
    assert len(configuration["result"]["collisions"]) >= 1


def test_serves_json_lines_and_survives_bad_input() -> None:
    input_stream = io.StringIO(
        "\n".join(
            [
                '{"id":1,"method":"health"}',
                "not-json",
                "",
            ]
        )
    )
    output_stream = io.StringIO()

    serve(input_stream, output_stream)

    responses = [json.loads(line) for line in output_stream.getvalue().splitlines()]
    assert responses[0]["ok"] is True
    assert responses[1]["ok"] is False
    assert responses[1]["error"]["type"] == "JSONDecodeError"
