# 🦾 Robotic Arm with AI-Based Object Detection & Autonomous Sorting

**6-DOF robotic arm with computer vision for autonomous object detection and sorting.**  
Built at Instituto Industrial Luis A. Huergo, Buenos Aires · 2026

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-pre--alpha-orange.svg)]()
[![Based on](https://img.shields.io/badge/based%20on-SO--ARM101-lightgrey.svg)](https://github.com/TheRobotStudio/SO-ARM101)

---

## 📌 Overview

This project implements a low-cost 6 degree-of-freedom robotic arm that picks a
red cube and drops it into a lime-green container:

- **Object detection** with two fixed USB cameras (front + side) and OpenCV color segmentation
- **Learning from demonstration**: 12 key poses per demonstration, recorded with photos and joint angles
- **Supervised execution**: every move is proposed first and needs operator confirmation
- **Physics simulation** in MuJoCo with an inverse-kinematics expert that performs the full task virtually
- **Calibrated 3D model** and geometric safety rules shared by the PC planner and the ESP32 firmware

Total hardware cost: ~USD 350 vs USD 30,000–150,000 for equivalent industrial arms.

The current version lives in [`brazorobtoIA/`](brazorobtoIA/) (documentation in Spanish).
The other folders hold earlier prototypes.

---

## 🔗 Project Links

| Resource | Link |
|----------|------|
| 🌐 Landing Page | [fferchang.github.io/brazo-robotico](https://fferchang.github.io/brazo-robotico) |
| 📄 Technical Report | [View PDF](https://github.com/fferchang/brazo-robotico/blob/main/docs/informe_brazo_robotico.pdf) |
| 📸 Photo Gallery | [Google Photos](https://photos.app.goo.gl/9NATdUXzkGk8ssc47) |
| 🎥 Demo Videos | *(coming soon)* |

---

## 🧱 Based On

This project builds upon the open-source [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) design by [TheRobotStudio](https://github.com/TheRobotStudio), published under the Apache 2.0 license.

> 📦 **Bill of Materials, CAD files, and hardware assembly instructions** are available in the original [SO-ARM101 repository](https://github.com/TheRobotStudio/SO-ARM101). Our technical report (linked above) includes the specific components and pricing used in this build.

Key differences from the original:
- Python desktop app for teaching, training and supervised execution (`brazorobtoIA/`)
- MuJoCo simulation of the whole task, with an inverse-kinematics expert
- 3D model calibrated against the real arm (table touches, gripper closing)
- Geometric safety rules enforced twice: PC planner and ESP32 firmware

---

## ⚙️ Hardware

| Component | Specs |
|-----------|-------|
| Servomotors | 6× Feetech STS3215 (12V, 30kg·cm) |
| Driver | Waveshare Bus Servo Adapter (A) V1.1 |
| Microcontroller | ESP32 |
| Vision | 2× fixed USB webcams (front + side view) |
| Frame | 3D printed (PLA MAX, Creality Ender 3 V3 SE) |
| Base | Recycled 40×40cm pressed wood board |
| Power | External 12V supply |

---

## 🧠 Software Stack

| Layer | Technology |
|-------|-----------|
| Main app | Python 3 + Tkinter (`brazorobtoIA/app.py`) |
| Vision | OpenCV (HSV color segmentation, two cameras) |
| Learned policy | NumPy (per-stage regression from demonstrations) |
| Physics simulation | MuJoCo 3.17 + inverse-kinematics expert |
| 3D view / digital twin | Rerun 0.35 (live telemetry on the URDF model) |
| Firmware | Arduino IDE (ESP32): `brazorobtoIA/firmware/firmware_secuencias.ino` |
| Earlier prototypes | Electron app, web simulators, Python kinematics scripts |
| Dev tools | VS Code, OpenAI Codex, Claude Code, GitHub |

---

## 📁 Repository Structure

```
brazo-robotico/
├── brazorobtoIA/                    # ← Current version: Python app, firmware, simulation
│   ├── app.py                       # Teaching, training and supervised execution
│   ├── brazoia/                     # Vision, policy, planner, MuJoCo sim, calibration
│   ├── firmware/                    # ESP32 sketch with geometric safety rules
│   ├── assets/                      # URDF + meshes (SO-ARM100), calibration, references
│   ├── tests/                       # Automated tests (no hardware needed)
│   └── *.md                         # Guides: simulation, calibration, safety rules
├── app/                             # Earlier prototype: Electron app + WebSerial UI
├── firmware/                        # Earlier Arduino/ESP32 test sketches
├── kinematics/                      # Earlier Python geometric simulation
├── simulation/                      # Earlier web-based 3D simulator prototypes
├── docs/                            # Technical report
│   └── informe_brazo_robotico.pdf
├── index.html                       # Landing page (GitHub Pages)
└── README.md
```

> ⚠️ **Pre-alpha:** code is actively being tested. Demonstration photos are not
> uploaded (they can show people); virtual environments and trained models are excluded.

---

## 🚧 Development Status

| Module | Status |
|--------|--------|
| Mechanical assembly | ✅ 100% complete |
| Servo ID assignment & telemetry | ✅ Complete |
| Teaching app (12-pose demonstrations, two cameras) | ✅ Functional — 15 successful demonstrations |
| Learned policy | 🔄 Experimental, not validated (validation error 44°; more demonstrations needed) |
| MuJoCo simulation + inverse kinematics | ✅ 100/100 virtual pick & place episodes |
| 3D model calibration | ✅ Table and gripper (Oct 2026); M3/M4 zeros and M1 direction pending |
| Geometric safety rules (planner + firmware) | ✅ Retuned Nov 2026 — new firmware pending upload to the ESP32 |
| Gripper current protection | 🔄 Fixed in firmware (Nov 2026), host-tested; pending upload and hardware test (`brazorobtoIA/DIAGNOSTICO_GARRA.md`) |
| Camera calibration (sim ↔ real) | ⏳ Pending |
| YOLO fine-tuning | ⏳ Pending — the current version detects by color |
| Full sorting demo | ⏳ Target: Nov 2026 |

---

## 🔩 Joint Angle Limits

Fixed limits per motor (`brazorobtoIA/config.json`):

| Motor | Range |
|-------|-------|
| M1 | 60° – 300° |
| M2 | 90° – 270° |
| M3 | 90° – 276° |
| M4 | 75° – 278° (+ dynamic rules for folded poses) |
| M5 | 0° – 340° |
| M6 | 0° – 140° (gripper, with offset) |

The floor rule keeps the gripper outline (for any wrist roll and the current
gripper opening) more than 5 mm above the table. Details in
[`brazorobtoIA/REGLAS_GEOMETRIA.md`](brazorobtoIA/REGLAS_GEOMETRIA.md).

---

## 🚀 Getting Started

> Full setup instructions coming with alpha release.

### Prerequisites

- Windows with Python 3.12 or newer (developed with 3.12, also tested with 3.14)
- Arduino IDE 2.x with the ESP32 core and the SCServo library (firmware)
- Feetech STS3215 servos × 6
- Waveshare Bus Servo Adapter (A) V1.1
- External 12V power supply

### Quick Start *(pre-alpha)*

```bash
git clone https://github.com/fferchang/brazo-robotico.git
cd brazo-robotico/brazorobtoIA
```

Then, from the `brazorobtoIA` folder:

- `INSTALAR.bat` once, to create the virtual environment and install dependencies
- `INICIAR.bat` for the main app (cameras, ESP32, teaching and execution)
- `SIMULACION_MUJOCO.bat` for the virtual pick & place (no hardware needed)
- `CALIBRACION.bat` for the model-to-arm calibration

---

## 👥 Team

| Name | Role |
|------|------|
| Stefano Cavallaro | Development |
| Fernando Chang | Development |
| Federico Ottolini | Development |

**Tutors:** Gonzalo Rodriguez Goris · Fabián Vega  
**Institution:** Instituto Industrial Luis A. Huergo, Buenos Aires

---

## 📄 License

This project is licensed under the MIT License.  
Hardware design credits: [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM101) by TheRobotStudio (Apache 2.0).
