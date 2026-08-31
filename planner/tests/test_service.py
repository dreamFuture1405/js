"""Tests for the JSON-lines planner protocol."""

import io
import json
from pathlib import Path

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
