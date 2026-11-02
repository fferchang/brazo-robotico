"""Distancia geométrica al piso del modelo de Rerun; diagnóstico, no autorización.

Unidades internas: metros. Se usan las mallas visuales de ambas mordazas y
el motor de la garra, incluyendo M5 y M6. No estima flexión ni error de encoders.
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def joint_angle(config, motor_id, logical_degrees):
    servo = float(logical_degrees) + (10.0 if motor_id == 6 else 0.0)
    return (float(config["urdf_zero_radians"])
            + math.radians(servo - float(config["servo_zero_degrees"])) * float(config["direction"])
            + math.radians(float(config.get("visual_offset_degrees", 0.0))))


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin_transform(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return result


def stl_vertices(path):
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if len(data) == 84 + count * 50:
        dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
        vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)["vertices"].reshape(-1, 3)
    else:
        vertices = np.asarray([list(map(float, line.split()[1:]))
                               for line in data.decode("ascii").splitlines()
                               if line.strip().startswith("vertex ")])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError(f"Malla inválida: {path}")
    return np.unique(vertices.astype(float), axis=0)


# Eje M2 sobre la mesa, ajustado con 12 toques de la garra el 30/10/2026
# (CALIBRACION.md). Antes se usaban los 100 mm medidos a mano.
SHOULDER_HEIGHT_MM = 121.9


class GripperGeometry:
    def __init__(self, assets_root: Path, *, shoulder_height_mm=SHOULDER_HEIGHT_MM):
        assets_root = Path(assets_root)
        if not math.isfinite(shoulder_height_mm) or shoulder_height_mm <= 0:
            raise ValueError("Altura del eje M2 inválida")
        self.shoulder_height_mm = float(shoulder_height_mm)
        self.calibration = {int(k): v for k, v in json.loads(
            (assets_root / "calibracion.json").read_text(encoding="utf-8")).items()}
        model_path = assets_root / "modelo" / "so100.urdf"
        root = ET.parse(model_path).getroot()
        self.joints = {}
        for element in root.findall("joint"):
            limit = element.find("limit")
            self.joints[element.get("name")] = {
                "parent": element.find("parent").get("link"),
                "child": element.find("child").get("link"),
                "origin": origin_transform(element.find("origin")),
                "axis": np.fromstring(element.find("axis").get("xyz"), sep=" "),
                "lower": float(limit.get("lower")), "upper": float(limit.get("upper")),
            }
        self.meshes = []
        for link in root.findall("link"):
            if link.get("name") not in ("gripper", "jaw"):
                continue
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                vertices = stl_vertices(model_path.parent / mesh.get("filename"))
                vertices *= np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                transform = origin_transform(visual.find("origin"))
                vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
                self.meshes.append((link.get("name"), mesh.get("filename"), vertices))
        if not self.meshes:
            raise ValueError("Faltan las mallas de la garra")
        # Cota de desplazamiento de cualquier vértice al girar cada eje.
        # Las longitudes de la cadena se suman: no depende de la pose.
        self.motion_radii_mm = {i: 0.0 for i in self.calibration}
        by_child = {joint["child"]: (name, joint) for name, joint in self.joints.items()}
        by_joint = {config["joint"]: i for i, config in self.calibration.items()}
        for link, _, vertices in self.meshes:
            radius = float(np.linalg.norm(vertices, axis=1).max())
            while link in by_child:
                name, joint = by_child[link]
                motor = by_joint[name]
                self.motion_radii_mm[motor] = max(self.motion_radii_mm[motor], radius * 1000)
                radius += float(np.linalg.norm(joint["origin"][:3, 3]))
                link = joint["parent"]
        # La mesa está `shoulder_height_mm` debajo del eje M2. El cero del URDF
        # no está en ese eje; el plano se ubica en su marco base.
        transforms, _ = self.transforms({1: 180, 2: 180, 3: 90, 4: 180, 5: 263, 6: 0})
        self.floor_z = float(transforms["upper_arm"][2, 3]) - shoulder_height_mm / 1000

    def motion_bound_mm(self, start, end):
        """Cota de variación de altura a lo largo de interpolación articular."""
        return sum(self.motion_radii_mm[i] * math.radians(abs(end[i] - start[i]))
                   * abs(float(self.calibration[i]["direction"])) for i in range(1, 7))

    def transforms(self, pose, *, clamp=False):
        if any(i not in pose or not math.isfinite(float(pose[i])) for i in range(1, 7)):
            raise ValueError("Se requieren los seis ángulos finitos para medir la garra")
        transforms = {"base": np.eye(4)}
        clipped = []
        pending = list(self.calibration.items())
        while pending:
            progress = False
            for motor_id, config in pending[:]:
                joint = self.joints[config["joint"]]
                if joint["parent"] not in transforms:
                    continue
                angle = joint_angle(config, motor_id, pose[motor_id])
                bounded = max(joint["lower"], min(joint["upper"], angle))
                if abs(angle - bounded) > 1e-9:
                    clipped.append(motor_id)
                turn = np.eye(4)
                turn[:3, :3] = rotation(joint["axis"], bounded if clamp else angle)
                transforms[joint["child"]] = transforms[joint["parent"]] @ joint["origin"] @ turn
                pending.remove((motor_id, config))
                progress = True
            if not progress:
                raise ValueError("Cadena URDF incompleta")
        return transforms, tuple(clipped)

    def clearance(self, pose, *, clamp=False):
        transforms, clipped = self.transforms(pose, clamp=clamp)
        candidates = []
        for link, name, vertices in self.meshes:
            transform = transforms[link]
            world = vertices @ transform[:3, :3].T + transform[:3, 3]
            point = world[np.argmin(world[:, 2])]
            candidates.append((float(point[2]), name, point))
        _, name, point = min(candidates, key=lambda item: item[0])
        return {"height_mm": float((point[2] - self.floor_z) * 1000),
                "point": point, "mesh": name, "clipped_motors": clipped}
