"""JSON-lines protocol for the external native MuJoCo planner process."""

from __future__ import annotations

import json
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any, TextIO, cast

from axis_planner.model_runtime import ModelRuntime

JsonObject = dict[str, Any]


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{label} must be an object")
    return cast(Mapping[str, Any], value)


def _float_list(value: object, label: str) -> list[float]:
    if not isinstance(value, list):
        raise TypeError(f"{label} must be an array")
    return [float(item) for item in value]


def _string_list(value: object, label: str) -> list[str]:
    if value is None:
        return []
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise TypeError(f"{label} must be an array of strings")
    return list(value)


class PlannerService:
    """Dispatch typed protocol requests against one loaded model."""

    def __init__(self) -> None:
        """Create an unloaded service."""

        self.runtime: ModelRuntime | None = None

    def handle(self, request: Mapping[str, Any]) -> JsonObject:
        """Handle one JSON-compatible request.

        Args:
            request: Request containing ``id``, ``method`` and optional ``params``.

        Returns:
            Correlated success or error response.
        """

        request_id = request.get("id")
        method = str(request.get("method", ""))
        params = _mapping(request.get("params", {}), "params")
        try:
            if method == "health":
                result: Any = {
                    "ready": self.runtime is not None,
                    "model": self.runtime.summary().to_dict() if self.runtime else None,
                }
            elif method == "load_model":
                model_path = Path(str(params["path"]))
                self.runtime = ModelRuntime.load(model_path)
                result = self.runtime.summary().to_dict()
            elif method == "mirror_state":
                runtime = self._runtime()
                simulation = _mapping(params["simulation"], "simulation")
                runtime.apply_state(
                    qpos=_float_list(simulation["qpos"], "simulation.qpos"),
                    qvel=_float_list(simulation["qvel"], "simulation.qvel"),
                    ctrl=_float_list(simulation["ctrl"], "simulation.ctrl"),
                )
                result = runtime.mirror_report(_string_list(params.get("bodyNames"), "bodyNames"))
            else:
                raise ValueError(f"Unknown planner method: {method}")
            return {"id": request_id, "ok": True, "result": result}
        except (FileNotFoundError, KeyError, TypeError, ValueError) as error:
            return {
                "id": request_id,
                "ok": False,
                "error": {
                    "type": type(error).__name__,
                    "message": str(error),
                },
            }

    def _runtime(self) -> ModelRuntime:
        if self.runtime is None:
            raise ValueError("Planner model is not loaded")
        return self.runtime


def serve(input_stream: TextIO, output_stream: TextIO) -> None:
    """Serve newline-delimited JSON requests until EOF."""

    service = PlannerService()
    for line in input_stream:
        if not line.strip():
            continue
        try:
            request = _mapping(json.loads(line), "request")
            response = service.handle(request)
        except (json.JSONDecodeError, TypeError) as error:
            response = {
                "id": None,
                "ok": False,
                "error": {
                    "type": type(error).__name__,
                    "message": str(error),
                },
            }
        output_stream.write(json.dumps(response, separators=(",", ":")) + "\n")
        output_stream.flush()


def main() -> None:
    """Run the planner service over standard input/output."""

    serve(sys.stdin, sys.stdout)


if __name__ == "__main__":
    main()
