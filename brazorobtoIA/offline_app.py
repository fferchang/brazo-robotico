"""Ventana de prueba sin ESP32, cámaras en vivo ni comandos de motores."""
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path

from PIL import Image, ImageTk

from brazoia.config import KEYFRAME_STAGES, STAGE_LABELS
from brazoia.dataset import load_successful_demonstrations
from brazoia.offline import OfflineModel
from brazoia.rerun_view import RerunArmView

ROOT = Path(__file__).resolve().parent


class OfflineApp:
    def __init__(self, root, *, project_root=ROOT):
        self.root, self.project_root = root, Path(project_root)
        self.model = None
        self.result = None
        self.demo = None
        self.photos = []
        self.timer = None
        self.view = RerunArmView(self.project_root / "assets", recording_name="brazo_ia_OFFLINE",
                                 view_name="OFFLINE · PREDICCIÓN · SIN MOTORES")
        root.title("Brazo IA · PRUEBA OFFLINE · SIN MOTORES")
        root.geometry("1150x800")
        root.minsize(900, 680)
        root.protocol("WM_DELETE_WINDOW", self.close)
        ttk.Label(root, text="PRUEBA OFFLINE — NO CONECTA EL ESP32 NI MUEVE EL BRAZO",
                  foreground="#a12525", font=("Segoe UI", 14, "bold")).pack(pady=8)
        row = ttk.Frame(root, padding=6)
        row.pack(fill="x")
        ttk.Button(row, text="Cargar modelo (.npz)", command=self.choose_model).pack(side="left")
        self.model_text = tk.StringVar(value="Sin modelo")
        ttk.Label(row, textvariable=self.model_text, wraplength=850).pack(side="left", padx=10)
        row = ttk.Frame(root, padding=6)
        row.pack(fill="x")
        ttk.Label(row, text="Demostración:").pack(side="left")
        self.demo_combo = ttk.Combobox(row, state="readonly", width=58)
        self.demo_combo.pack(side="left", padx=6)
        self.demo_combo.bind("<<ComboboxSelected>>", self.select_demo)
        self.split_text = tk.StringVar()
        ttk.Label(row, textvariable=self.split_text).pack(side="left")
        row = ttk.Frame(root, padding=6)
        row.pack(fill="x")
        self.stage_combo = ttk.Combobox(row, state="readonly", width=32,
                                       values=[f"{i+1}. {STAGE_LABELS[s]}" for i, s in enumerate(KEYFRAME_STAGES)])
        self.stage_combo.pack(side="left")
        self.stage_combo.current(0)
        self.stage_combo.bind("<<ComboboxSelected>>", self.select_stage)
        for text, command in (("Anterior", lambda: self.step(-1)), ("Siguiente", lambda: self.step(1)),
                              ("Reproducir 12 etapas", self.play), ("Pausar", self.pause),
                              ("Ver predicción en Rerun", self.open_view)):
            ttk.Button(row, text=text, command=command).pack(side="left", padx=3)
        images = ttk.Frame(root)
        images.pack(fill="both", expand=True, padx=8)
        self.image_labels = []
        for name in ("FRONTAL · foto guardada", "LATERAL · foto guardada"):
            box = ttk.Frame(images)
            box.pack(side="left", fill="both", expand=True)
            ttk.Label(box, text=name).pack()
            label = ttk.Label(box, text="Sin imagen", anchor="center")
            label.pack(fill="both", expand=True)
            self.image_labels.append(label)
        self.table = ttk.Treeview(root, columns=("motor", "actual", "raw", "preview", "error"), show="headings", height=6)
        for key, label in zip(self.table["columns"], ("Motor", "Ángulo enseñado", "Predicción cruda", "Vista Rerun", "Error absoluto")):
            self.table.heading(key, text=label)
            self.table.column(key, width=165, anchor="center")
        self.table.pack(fill="x", padx=8, pady=5)
        self.status = tk.StringVar(value="Elegí un modelo y una demostración.")
        ttk.Label(root, textvariable=self.status, wraplength=1080).pack(anchor="w", padx=8)
        self.notes = tk.Text(root, height=4, wrap="word", state="disabled")
        self.notes.pack(fill="x", padx=8, pady=5)
        ttk.Label(root, text="M6 copia la pose grabada: no es una predicción ni una simulación de presión. "
                  "El URDF puede limitar visualmente articulaciones. No certifica colisiones ni éxito físico.",
                  wraplength=1080).pack(anchor="w", padx=8, pady=4)
        self.demos = load_successful_demonstrations(self.project_root / "datos" / "demostraciones")
        self.demo_combo["values"] = [Path(d["_path"]).name for d in self.demos]
        if self.demos:
            self.demo_combo.current(0)
            self.demo = self.demos[0]
        candidates = list((self.project_root / "modelos" / "no_validados").glob("*.npz"))
        active = self.project_root / "modelos" / "politica_etapas.npz"
        if active.is_file():
            candidates.append(active)
        if candidates:
            self.load_model(max(candidates, key=lambda path: path.stat().st_mtime))

    def choose_model(self):
        path = filedialog.askopenfilename(parent=self.root, initialdir=self.project_root / "modelos",
                                          filetypes=[("Modelo local", "*.npz")])
        if path:
            self.load_model(Path(path))

    def load_model(self, path):
        self.pause()
        try:
            model = OfflineModel(path)
        except Exception as exc:
            messagebox.showerror("Modelo offline", str(exc), parent=self.root)
            return
        self.model = model
        experimental = model.metadata.get("experimental_only") or model.metadata.get("validation_passed") is False
        self.model_text.set(f"{model.path.name} · {'EXPERIMENTAL NO VALIDADO' if experimental else 'solo visualización offline'}")
        validation = model.metadata.get("validation_sources", [])
        for i, demo in enumerate(self.demos):
            if Path(demo["_path"]).name in validation:
                self.demo_combo.current(i)
                self.demo = demo
                break
        self.refresh()

    def select_demo(self, _event=None):
        self.pause()
        index = self.demo_combo.current()
        if index >= 0:
            self.demo = self.demos[index]
            self.stage_combo.current(0)
            self.refresh()

    def select_stage(self, _event=None):
        self.pause()
        self.refresh()

    def step(self, delta):
        self.pause()
        self.stage_combo.current(max(0, min(11, self.stage_combo.current() + delta)))
        self.refresh()

    def refresh(self):
        self.result = None
        self.table.delete(*self.table.get_children())
        if self.model is None or self.demo is None:
            self.status.set("Falta un modelo o una demostración completa guardada.")
            return
        index = self.stage_combo.current()
        stage = self.demo["stages"][index]
        name = Path(self.demo["_path"]).name
        split = "VALIDACIÓN" if name in self.model.metadata.get("validation_sources", []) else (
            "ENTRENAMIENTO" if name in self.model.metadata.get("training_sources", []) else "Partición no identificada")
        self.split_text.set(split)
        self.photos = []
        for label, filename in zip(self.image_labels, stage["images"]):
            try:
                base = Path(self.demo["_path"]).resolve()
                path = (base / filename).resolve()
                if not path.is_relative_to(base):
                    raise ValueError("Imagen fuera de la demostración")
                with Image.open(path) as original:
                    frame = original.convert("RGB")
                    frame.thumbnail((500, 240))
                photo = ImageTk.PhotoImage(frame)
                self.photos.append(photo)
                label.configure(image=photo, text="")
            except Exception as exc:
                label.configure(image="", text=f"No se pudo abrir la foto: {exc}")
        try:
            self.result = self.model.evaluate(index, stage)
        except Exception as exc:
            self.pause()
            self.status.set(f"No se puede evaluar: {exc}")
            return
        r = self.result
        for i in range(1, 7):
            self.table.insert("", "end", values=(f"M{i}", f"{r['actual'][i-1]:.1f}°",
                f"{r['raw'][i-1]:.1f}°" if i < 6 else "No aprendido",
                f"{r['preview'][i]:.1f}°", f"{abs(r['raw'][i-1]-r['actual'][i-1]):.1f}°" if i < 6 else "—"))
        self.status.set(f"{index+1}/12 · {STAGE_LABELS[KEYFRAME_STAGES[index]]} · "
                        f"Error del regresor M1–M5: {r['mae']:.1f}° · {split} · NO ES EJECUCIÓN REAL")
        self.notes.configure(state="normal")
        self.notes.delete("1.0", "end")
        self.notes.insert("end", "\n".join(r["notes"]) or "Sin avisos de esta captura.")
        self.notes.configure(state="disabled")
        if self.view.available:
            self.view.log_pose(r["preview"])

    def open_view(self):
        if self.result is None:
            self.status.set("Cargá un modelo y una demostración para ver la predicción.")
            return
        self.view.open()
        if not self.view.available:
            self.status.set(f"Rerun no pudo abrirse: {self.view.error}")
            return
        self.view.log_pose(self.result["preview"])

    def play(self):
        self.pause()
        if self.model is None or self.demo is None:
            return
        self.stage_combo.current(0)
        self.refresh()
        if self.result is not None:
            self.timer = self.root.after(1500, self.advance)

    def advance(self):
        self.timer = None
        index = self.stage_combo.current()
        if index >= 11:
            return
        self.stage_combo.current(index + 1)
        self.refresh()
        if self.result is not None:
            self.timer = self.root.after(1500, self.advance)

    def pause(self):
        if self.timer is not None:
            self.root.after_cancel(self.timer)
            self.timer = None

    def close(self):
        self.pause()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    OfflineApp(root)
    root.mainloop()
