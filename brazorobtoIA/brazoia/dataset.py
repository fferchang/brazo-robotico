from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import cv2

from .capture import FrameSnapshot
from .config import KEYFRAME_STAGES
from .vision import SceneObservation, observation_features


def _atomic_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    temporary.replace(path)


class DemonstrationSession:
    """Guarda las poses configuradas, en orden y sin mover el robot."""

    def __init__(self, root: Path, configuration: dict[str, object]):
        root.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime("prueba_%Y%m%d_%H%M%S_%f")
        self.path = root / name
        self.path.mkdir(exist_ok=False)
        self.started = time.monotonic()
        self.data: dict[str, object] = {
            "schema": 4,
            "task": "cubo_rojo_a_recipiente_verde_lima",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "status": "incomplete",
            "result": None,
            "configuration": configuration,
            "stages": [],
        }
        self._write()

    @property
    def next_stage(self) -> str | None:
        index = len(self.data["stages"])
        return KEYFRAME_STAGES[index] if index < len(KEYFRAME_STAGES) else None

    @property
    def complete(self) -> bool:
        return len(self.data["stages"]) == len(KEYFRAME_STAGES)

    def capture(
        self,
        stage: str,
        snapshots: list[FrameSnapshot],
        positions: dict[int, float],
        observations: list[SceneObservation],
        *,
        torque_enabled: bool | None,
        gripper_state: str,
        warnings: list[str] | None = None,
        replace_last: bool = False,
    ) -> dict[str, object]:
        if self.data["status"] != "incomplete":
            raise ValueError("La demostración ya fue finalizada.")
        stages = self.data["stages"]
        if replace_last and not stages:
            raise ValueError("Todavía no hay una foto para reintentar.")
        expected = stages[-1]["stage"] if replace_last else self.next_stage
        if expected is None:
            raise ValueError("La demostración ya tiene todas las poses.")
        if stage != expected:
            raise ValueError(f"Se esperaba {expected}, no {stage}.")
        if len(snapshots) != 2 or any(item.frame is None for item in snapshots):
            raise ValueError("Se necesitan dos imágenes válidas.")
        if set(positions) != set(range(1, 7)):
            raise ValueError("Se necesitan los seis ángulos.")
        features = observation_features(observations)

        index = len(stages) - 1 if replace_last else len(stages)
        prefix = f"{index + 1:02d}_{stage}"
        if replace_last:
            # No sobrescribir las imágenes vigentes antes de confirmar ambos
            # archivos y el metadata. La toma anterior queda recuperable.
            prefix += f"_retoma_{uuid4().hex}"
        image_names = [f"{prefix}_frontal.png", f"{prefix}_lateral.png"]
        for image_name, snapshot in zip(image_names, snapshots):
            if not cv2.imwrite(str(self.path / image_name), snapshot.frame):
                raise RuntimeError(f"No se pudo guardar {image_name}.")

        record = {
            "index": index,
            "stage": stage,
            "captured_t": time.monotonic() - self.started,
            "images": image_names,
            "camera_receive_t": [item.timestamp for item in snapshots],
            "camera_sequence": [item.sequence for item in snapshots],
            "camera_indices": [item.camera_index for item in snapshots],
            "camera_rotation_degrees": [
                item.rotation_degrees for item in snapshots
            ],
            "camera_image_parameters": [
                {
                    "brightness_percent": item.brightness_percent,
                    "contrast_percent": item.contrast_percent,
                    "saturation_percent": item.saturation_percent,
                    "zoom_factor": item.zoom_factor,
                }
                for item in snapshots
            ],
            "joint_degrees_logical": {
                str(motor_id): round(float(positions[motor_id]), 3)
                for motor_id in range(1, 7)
            },
            "torque_enabled": torque_enabled,
            "gripper_state": gripper_state,
            "capture_warnings": list(warnings or []),
            "observations": [item.to_dict() for item in observations],
            "features": features.tolist(),
        }
        updated = dict(self.data)
        updated["stages"] = list(stages)
        if replace_last:
            updated["replaced_captures"] = [*self.data.get("replaced_captures", []), stages[-1]]
            updated["stages"][-1] = record
        else:
            updated["stages"].append(record)
        _atomic_json(self.path / "metadata.json", updated)
        self.data = updated
        return record

    def finish(self, result: str, notes: str = "") -> None:
        if result == "success" and not self.complete:
            raise ValueError(f"No se puede marcar éxito sin las {len(KEYFRAME_STAGES)} poses.")
        self.data.update(
            status="finished",
            result=result,
            notes=notes,
            duration_seconds=time.monotonic() - self.started,
        )
        self._write()

    def _write(self) -> None:
        _atomic_json(self.path / "metadata.json", self.data)


def load_successful_demonstrations(root: Path) -> list[dict[str, object]]:
    demonstrations: list[dict[str, object]] = []
    if not root.exists():
        return demonstrations
    for metadata_path in sorted(root.glob("*/metadata.json")):
        try:
            data = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        stages = data.get("stages", [])
        if (
            data.get("schema") == 4
            and data.get("status") == "finished"
            and data.get("result") == "success"
            and [item.get("stage") for item in stages] == list(KEYFRAME_STAGES)
        ):
            data["_path"] = str(metadata_path.parent)
            demonstrations.append(data)
    return demonstrations

