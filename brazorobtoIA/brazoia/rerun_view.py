from __future__ import annotations

import json
from pathlib import Path

from .gripper_geometry import GripperGeometry, joint_angle


class RerunArmView:
    """Adaptador opcional; la aplicación sigue funcionando si Rerun falta."""

    def __init__(self, assets_root: Path, *, recording_name="brazo_robot_ia_final",
                 view_name="SO-ARM — estado y propuesta", clamp_joint_limits=True):
        self.assets_root = assets_root
        self.recording_name = recording_name
        self.view_name = view_name
        self.clamp_joint_limits = clamp_joint_limits
        self.available = False
        self.error = ""
        self.recording = None
        self.joints = {}
        self.calibration: dict[int, dict[str, object]] = {}
        self.frame = 0
        self.geometry = None
        self.clearance = None

    def open(self) -> None:
        if self.available:
            return
        try:
            import rerun as rr
            import rerun.blueprint as rrb

            urdf_path = self.assets_root / "modelo" / "so100.urdf"
            calibration_path = self.assets_root / "calibracion.json"
            raw = json.loads(calibration_path.read_text(encoding="utf-8"))
            self.calibration = {int(key): value for key, value in raw.items()}
            recording = rr.RecordingStream(self.recording_name)
            blueprint = rrb.Spatial3DView(
                name=self.view_name,
                overrides={
                    "so_arm100/collision_geometries": rrb.EntityBehavior(visible=False)
                },
            )
            recording.spawn(default_blueprint=blueprint, hide_welcome_screen=True)
            recording.log_file_from_path(urdf_path, static=True)
            tree = rr.urdf.UrdfTree.from_file_path(urdf_path)
            self.joints = {joint.name: joint for joint in tree.joints()}
            self.geometry = GripperGeometry(self.assets_root)
            z = self.geometry.floor_z
            recording.log("diagnostico_piso", rr.CoordinateFrame("base"), static=True)
            recording.log("diagnostico_piso/plano", rr.LineStrips3D(
                [[[-.5, -.5, z], [.5, -.5, z], [.5, .5, z], [-.5, .5, z], [-.5, -.5, z]],
                 [[-.5, 0, z], [.5, 0, z]], [[0, -.5, z], [0, .5, z]]],
                colors=[140, 140, 140]), rr.CoordinateFrame("base"), static=True)
            self.recording = recording
            self.available = True
            self.error = ""
        except Exception as exc:
            self.error = str(exc)
            self.available = False

    def log_pose(self, logical_positions: dict[int, float]) -> None:
        if not self.available or self.recording is None:
            return
        self.recording.set_time("estado", sequence=self.frame)
        for motor_id, logical_degrees in logical_positions.items():
            if motor_id not in self.calibration:
                continue
            config = self.calibration[motor_id]
            angle = joint_angle(config, motor_id, logical_degrees)
            joint = self.joints[str(config["joint"])]
            if self.clamp_joint_limits and joint.limit_lower is not None:
                angle = max(float(joint.limit_lower), angle)
            if self.clamp_joint_limits and joint.limit_upper is not None:
                angle = min(float(joint.limit_upper), angle)
            self.recording.log("joint_transforms", joint.compute_transform(angle, clamp=False))
        self._log_floor_clearance(logical_positions)
        self.frame += 1

    def _log_floor_clearance(self, positions):
        import rerun as rr

        self.clearance = None
        # Nunca completar una pose parcial con valores de cuadros anteriores.
        try:
            result = self.geometry.clearance(positions)
            shown = self.geometry.clearance(positions, clamp=self.clamp_joint_limits)
        except ValueError as exc:
            self.recording.log("diagnostico_piso/medicion", rr.Clear(recursive=True))
            self.recording.log("diagnostico_piso/estado", rr.TextDocument(str(exc)))
            return
        self.clearance = result
        clipped = result["clipped_motors"]
        height = result["height_mm"]
        color = [255, 160, 0] if clipped else [255, 50, 50] if height <= 5 else [70, 220, 120]
        text = f"Garra-piso: {height:.1f} mm (estimación 3D)"
        if clipped:
            if self.clamp_joint_limits:
                text += " · VISTA RECORTADA: " + ", ".join(f"M{i}" for i in clipped)
                text += f" · altura dibujada: {shown['height_mm']:.1f} mm"
            else:
                text += " · FUERA DEL RANGO URDF: " + ", ".join(f"M{i}" for i in clipped)
        text += " · diagnóstico, no habilita movimientos"
        point = shown["point"]
        self.recording.log("diagnostico_piso/medicion/punta", rr.Points3D(
            [point], radii=.003, colors=color, labels=[text]), rr.CoordinateFrame("base"))
        self.recording.log("diagnostico_piso/medicion/distancia", rr.LineStrips3D(
            [[point, [point[0], point[1], self.geometry.floor_z]]], colors=color), rr.CoordinateFrame("base"))
        self.recording.log("diagnostico_piso/estado", rr.TextDocument(text))

