"""Registro de cada ejecución automática en `datos/ejecuciones/prueba_*/`.

Desde el esquema 2 guarda, además de las propuestas y las etapas, lo necesario
para entender después qué pasó sin el brazo delante: la foto de Inicio con la
que se planificó y dónde ubicó el cubo y el recipiente, una foto y los ángulos
al terminar cada etapa, los avisos del ejecutor (llegada con desvío, garra
abierta antes de 140°), los bloqueos y el detalle de la verificación visual.
"""
from __future__ import annotations

import json
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2

from .capture import FrameSnapshot
from .execution import MotionProposal
from .vision import ContainerCheck, SceneObservation

CAMERA_NAMES = ("frontal", "lateral")


def _joints(positions: dict[int, float] | None) -> dict[str, float] | None:
    if positions is None:
        return None
    return {str(motor): round(float(value), 3) for motor, value in sorted(positions.items())}


class ExecutionLog:
    def __init__(self, root: Path, configuration: dict[str, object] | None = None):
        root.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime("prueba_%Y%m%d_%H%M%S_%f")
        self.path = root / name
        self.path.mkdir(exist_ok=False)
        self.metadata_path = self.path / "metadata.json"
        self.started = time.monotonic()
        # Los avisos del ejecutor llegan desde su hilo; la app escribe desde el suyo.
        self._lock = threading.Lock()
        self.data = {
            "schema": 2,
            "task": "cubo_rojo_a_recipiente_verde_lima",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "status": "running",
            "configuration": configuration or {},
            "scene": None,
            "captures": [],
            "events": [],
            "verification": None,
        }
        self._write()

    def _now(self) -> float:
        return time.monotonic() - self.started

    def proposal(self, value: MotionProposal) -> None:
        self._event(
            {
                "kind": "proposal",
                "stage": value.stage,
                "target": {str(key): target for key, target in value.target.items()},
                "groups": [list(group.targets) for group in value.groups],
                "gripper_action": value.gripper_action,
                "recovery": value.recovery,
            }
        )

    def completed(
        self,
        stage: str,
        positions: dict[int, float] | None = None,
        gripper_state: str | None = None,
    ) -> None:
        """Etapa terminada, con los ángulos y el estado de la garra al llegar."""
        self._event(
            {
                "kind": "completed",
                "stage": stage,
                "joints": _joints(positions),
                "gripper_state": gripper_state,
            }
        )

    def note(self, kind: str, text: str, stage: str | None = None) -> None:
        """«aviso» del ejecutor o «bloqueo» de una etapa."""
        self._event({"kind": kind, "stage": stage, "text": text})

    def scene(self, summary: dict[str, object]) -> None:
        """Lo que la política sacó de la foto de Inicio (p. ej. dónde están cubo y recipiente)."""
        with self._lock:
            self.data["scene"] = summary
            self._write()

    def capture(
        self,
        label: str,
        snapshots: list[FrameSnapshot],
        observations: list[SceneObservation],
        positions: dict[int, float] | None = None,
        gripper_state: str | None = None,
    ) -> dict[str, object]:
        """Guarda las dos fotos y lo detectado. Nunca frena la tarea: si falta una
        imagen o no se puede escribir, lo anota y sigue."""
        with self._lock:
            index = len(self.data["captures"]) + 1
            record: dict[str, object] = {
                "index": index,
                "label": label,
                "t": self._now(),
                "images": [],
                "joints": _joints(positions),
                "gripper_state": gripper_state,
                "observations": [item.to_dict() for item in observations],
                "warnings": [],
            }
            if len(snapshots) != 2:
                record["warnings"].append(f"Se esperaban dos cámaras y hay {len(snapshots)}.")
            for camera, snapshot in zip(CAMERA_NAMES, snapshots):
                if snapshot.frame is None:
                    record["warnings"].append(f"Sin imagen de la cámara {camera}.")
                    continue
                name = f"{index:02d}_{label}_{camera}.png"
                try:
                    saved = cv2.imwrite(str(self.path / name), snapshot.frame)
                except cv2.error as exc:
                    saved = False
                    record["warnings"].append(f"{name}: {exc}")
                if saved:
                    record["images"].append(name)
                else:
                    record["warnings"].append(f"No se pudo guardar {name}.")
            self.data["captures"].append(record)
            self._write()
            return record

    def verification(self, passed: bool, checks: list[ContainerCheck], capture: int | None = None) -> None:
        with self._lock:
            self.data["verification"] = {
                "passed": passed,
                "capture": capture,
                "cameras": {
                    camera: check.to_dict() for camera, check in zip(CAMERA_NAMES, checks)
                },
            }
            self._write()

    def finish(
        self,
        result: str,
        automatic_verification: bool | None,
        notes: str = "",
    ) -> None:
        with self._lock:
            self.data.update(
                status="finished",
                result=result,
                automatic_verification=automatic_verification,
                notes=notes,
                duration_seconds=self._now(),
            )
            self._write()

    def _event(self, event: dict[str, object]) -> None:
        with self._lock:
            self.data["events"].append({"t": self._now(), **event})
            self._write()

    def _write(self) -> None:
        temporary = self.metadata_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.data, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        temporary.replace(self.metadata_path)
