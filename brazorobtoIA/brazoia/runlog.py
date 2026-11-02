from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

from .execution import MotionProposal


class ExecutionLog:
    def __init__(self, root: Path):
        root.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime("prueba_%Y%m%d_%H%M%S_%f")
        self.path = root / name
        self.path.mkdir(exist_ok=False)
        self.metadata_path = self.path / "metadata.json"
        self.started = time.monotonic()
        self.data = {
            "schema": 1,
            "task": "cubo_rojo_a_recipiente_verde_lima",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "status": "running",
            "events": [],
        }
        self._write()

    def proposal(self, value: MotionProposal) -> None:
        self.data["events"].append(
            {
                "t": time.monotonic() - self.started,
                "kind": "proposal",
                "stage": value.stage,
                "target": {str(key): target for key, target in value.target.items()},
                "groups": [list(group.targets) for group in value.groups],
                "gripper_action": value.gripper_action,
                "recovery": value.recovery,
            }
        )
        self._write()

    def completed(self, stage: str) -> None:
        self.data["events"].append(
            {"t": time.monotonic() - self.started, "kind": "completed", "stage": stage}
        )
        self._write()

    def finish(
        self,
        result: str,
        automatic_verification: bool | None,
        notes: str = "",
    ) -> None:
        self.data.update(
            status="finished",
            result=result,
            automatic_verification=automatic_verification,
            notes=notes,
            duration_seconds=time.monotonic() - self.started,
        )
        self._write()

    def _write(self) -> None:
        temporary = self.metadata_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.data, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        temporary.replace(self.metadata_path)

