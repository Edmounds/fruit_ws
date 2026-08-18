#!/usr/bin/env python3
"""Edit fruit waypoint YAML files on top of the Nav2 PGM map."""

from __future__ import annotations

import argparse
import copy
import math
import os
import shutil
import sys
import tkinter as tk
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any

import yaml
from PIL import Image, ImageTk


WORKSPACE = Path(__file__).resolve().parent
DEFAULT_MAP = (
    WORKSPACE
    / "src"
    / "navigation"
    / "pb2025_sentry_nav"
    / "pb2025_nav_bringup"
    / "maps"
    / "field.yaml"
)
DEFAULT_WAYPOINTS_DIR = WORKSPACE / "src" / "fruit_waypoint_recorder" / "waypoints"
DEFAULT_BACKUP_DIR = WORKSPACE / "tmp" / "waypoint_editor_backups"
DEFAULT_DISPLAY = ":0"
VALID_TYPES = ("waypoint", "goal")
SHIFT_MASK = 0x0001


@dataclass
class MapInfo:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    image: Image.Image

    @property
    def width(self) -> int:
        return self.image.width

    @property
    def height(self) -> int:
        return self.image.height


@dataclass
class Point:
    name: str
    point_type: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = copy.deepcopy(self.extra)
        data["name"] = self.name
        data["position"] = {"x": self.x, "y": self.y, "z": self.z}
        data["orientation"] = {
            "x": self.qx,
            "y": self.qy,
            "z": self.qz,
            "w": self.qw,
        }
        data["yaw"] = round(self.yaw, 6)
        data["type"] = self.point_type
        return data


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float]:
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length == 0.0:
        raise ValueError("quaternion length must not be zero")
    return x / length, y / length, z / length, w / length


def circular_mean_angle(a: float, b: float) -> float:
    return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


def load_map(path: Path) -> MapInfo:
    path = path.expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("map YAML root must be a mapping")

    raw_image = document.get("image")
    if not isinstance(raw_image, str) or not raw_image:
        raise ValueError("map YAML must include image")

    resolution = float(document["resolution"])
    if resolution <= 0.0:
        raise ValueError("map resolution must be positive")

    origin = document.get("origin")
    if not isinstance(origin, list) or len(origin) < 2:
        raise ValueError("map origin must contain at least x and y")
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0

    image_path = Path(raw_image).expanduser()
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    image_path = image_path.resolve()
    image = Image.open(image_path).convert("RGB")
    return MapInfo(
        yaml_path=path,
        image_path=image_path,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=origin_yaw,
        image=image,
    )


def map_to_image(map_info: MapInfo, x: float, y: float) -> tuple[float, float]:
    dx = x - map_info.origin_x
    dy = y - map_info.origin_y
    cos_yaw = math.cos(map_info.origin_yaw)
    sin_yaw = math.sin(map_info.origin_yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    px = local_x / map_info.resolution
    py = map_info.height - (local_y / map_info.resolution)
    return px, py


def image_to_map(map_info: MapInfo, px: float, py: float) -> tuple[float, float]:
    local_x = px * map_info.resolution
    local_y = (map_info.height - py) * map_info.resolution
    cos_yaw = math.cos(map_info.origin_yaw)
    sin_yaw = math.sin(map_info.origin_yaw)
    x = map_info.origin_x + cos_yaw * local_x - sin_yaw * local_y
    y = map_info.origin_y + sin_yaw * local_x + cos_yaw * local_y
    return x, y


def load_waypoint_file(path: Path) -> tuple[dict[str, Any], list[Point]]:
    path = path.expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("waypoint YAML root must be a mapping")
    if document.get("schema_version") != 1:
        raise ValueError("waypoint YAML schema_version must be 1")
    if document.get("frame_id") != "map":
        raise ValueError("waypoint YAML frame_id must be map")
    raw_points = document.get("points")
    if not isinstance(raw_points, list):
        raise ValueError("waypoint YAML points must be a list")

    points: list[Point] = []
    seen: set[str] = set()
    for index, item in enumerate(raw_points, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"point {index} must be a mapping")
        name = str(item.get("name", "")).strip()
        if not name:
            raise ValueError(f"point {index} name must not be empty")
        if name in seen:
            raise ValueError(f"duplicate point name: {name}")

        point_type = str(item.get("type", "")).strip()
        if point_type not in VALID_TYPES:
            raise ValueError(f"point {name} type must be waypoint or goal")

        position = item.get("position")
        orientation = item.get("orientation")
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            raise ValueError(f"point {name} must include position and orientation")

        qx, qy, qz, qw = normalize_quaternion(
            float(orientation["x"]),
            float(orientation["y"]),
            float(orientation["z"]),
            float(orientation["w"]),
        )
        yaw = float(item.get("yaw", yaw_from_quaternion(qx, qy, qz, qw)))
        extra = copy.deepcopy(item)
        for key in ("name", "type", "position", "orientation", "yaw"):
            extra.pop(key, None)
        points.append(
            Point(
                name=name,
                point_type=point_type,
                x=float(position["x"]),
                y=float(position["y"]),
                z=float(position.get("z", 0.0)),
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
                yaw=yaw,
                extra=extra,
            )
        )
        seen.add(name)
    return document, points


def write_waypoint_file(
    path: Path,
    document: dict[str, Any],
    points: list[Point],
    backup_dir: Path,
) -> Path:
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    backup_path = backup_dir / f"{path.stem}_{stamp}{path.suffix}"
    if backup_path.exists():
        microsecond = datetime.now().microsecond
        backup_path = backup_dir / f"{path.stem}_{stamp}_{microsecond}{path.suffix}"
    shutil.copy2(path, backup_path)

    updated = copy.deepcopy(document)
    updated["points"] = [point.to_dict() for point in points]
    with path.open("w", encoding="utf-8") as output:
        yaml.safe_dump(updated, output, allow_unicode=True, sort_keys=False)
    return backup_path


def unique_inserted_name(points: list[Point], insert_index: int) -> str:
    names = {point.name for point in points}
    base = f"inserted_{insert_index + 1}"
    candidate = base
    suffix = 2
    while candidate in names:
        candidate = f"{base}_{suffix}"
        suffix += 1
    return candidate


def make_inserted_point(points: list[Point], before_index: int) -> Point:
    first = points[before_index]
    second = points[before_index + 1]
    yaw = circular_mean_angle(first.yaw, second.yaw)
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    return Point(
        name=unique_inserted_name(points, before_index + 1),
        point_type="waypoint",
        x=(first.x + second.x) * 0.5,
        y=(first.y + second.y) * 0.5,
        z=(first.z + second.z) * 0.5,
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        yaw=yaw,
    )


def make_appended_point(points: list[Point]) -> Point:
    last = points[-1]
    return Point(
        name=unique_inserted_name(points, len(points)),
        point_type="waypoint",
        x=last.x,
        y=last.y,
        z=last.z,
        qx=last.qx,
        qy=last.qy,
        qz=last.qz,
        qw=last.qw,
        yaw=last.yaw,
    )


def parse_float(value: str, label: str) -> float:
    try:
        return float(value)
    except ValueError as error:
        raise ValueError(f"{label} must be a number") from error


def configure_desktop_display(display: str) -> None:
    runtime_dir = Path(f"/run/user/{os.getuid()}")
    xauthority = runtime_dir / "gdm" / "Xauthority"
    os.environ["DISPLAY"] = display
    os.environ.setdefault("XDG_RUNTIME_DIR", str(runtime_dir))
    os.environ.setdefault(
        "DBUS_SESSION_BUS_ADDRESS",
        f"unix:path={runtime_dir}/bus",
    )
    if xauthority.is_file():
        os.environ["XAUTHORITY"] = str(xauthority)


class WaypointEditorApp:
    def __init__(
        self,
        root: tk.Tk,
        map_path: Path,
        waypoints_dir: Path,
        initial_file: Path | None,
    ) -> None:
        self.root = root
        self.root.title("Fruit Waypoint Editor")
        self.map_info = load_map(map_path)
        self.waypoints_dir = waypoints_dir.expanduser().resolve()
        self.backup_dir = DEFAULT_BACKUP_DIR
        self.document: dict[str, Any] = {}
        self.points: list[Point] = []
        self.current_file: Path | None = None
        self.selected_index: int | None = None
        self.scale = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.drag_mode: str | None = None
        self.pan_start: tuple[int, int, float, float] | None = None
        self.tk_map_image: ImageTk.PhotoImage | None = None
        self._loading_form = False
        self._form_orientation_source = "yaw"
        self._dirty = False

        self._build_ui()
        self._bind_events()
        self._refresh_file_list()
        self._load_initial_file(initial_file)
        self.root.after(100, self.fit_map)

    def _build_ui(self) -> None:
        self.root.geometry("1280x820")
        self.root.minsize(980, 640)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.grid(row=0, column=0, sticky="nsew")

        left = ttk.Frame(main)
        right = ttk.Frame(main, width=360)
        main.add(left, weight=4)
        main.add(right, weight=1)

        toolbar = ttk.Frame(left, padding=(6, 6, 6, 2))
        toolbar.grid(row=0, column=0, sticky="ew")
        ttk.Button(toolbar, text="Fit", command=self.fit_map).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="100%", command=self.zoom_actual).pack(
            side=tk.LEFT, padx=(4, 0)
        )
        ttk.Button(toolbar, text="+", width=3, command=lambda: self.zoom_by(1.25)).pack(
            side=tk.LEFT, padx=(12, 0)
        )
        ttk.Button(toolbar, text="-", width=3, command=lambda: self.zoom_by(0.8)).pack(
            side=tk.LEFT, padx=(4, 0)
        )
        self.map_label_var = tk.StringVar()
        ttk.Label(toolbar, textvariable=self.map_label_var).pack(
            side=tk.LEFT, padx=(16, 0)
        )

        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(left, bg="#181818", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew")

        file_frame = ttk.LabelFrame(right, text="Waypoint File", padding=8)
        file_frame.pack(fill=tk.X, padx=8, pady=(8, 6))
        self.file_var = tk.StringVar()
        self.file_combo = ttk.Combobox(
            file_frame,
            textvariable=self.file_var,
            state="readonly",
            values=(),
        )
        self.file_combo.pack(fill=tk.X)
        buttons = ttk.Frame(file_frame)
        buttons.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(buttons, text="Open", command=self.open_file_dialog).pack(
            side=tk.LEFT
        )
        ttk.Button(buttons, text="Reload", command=self.reload_current_file).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(buttons, text="Save", command=self.save_current_file).pack(
            side=tk.RIGHT
        )

        list_frame = ttk.LabelFrame(right, text="Points", padding=8)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)
        list_frame.rowconfigure(0, weight=1)
        list_frame.columnconfigure(0, weight=1)
        self.tree = ttk.Treeview(
            list_frame,
            columns=("idx", "name", "type", "x", "y"),
            show="headings",
            selectmode="browse",
            height=12,
        )
        for column, label, width in (
            ("idx", "#", 42),
            ("name", "name", 118),
            ("type", "type", 78),
            ("x", "x", 70),
            ("y", "y", 70),
        ):
            self.tree.heading(column, text=label)
            self.tree.column(column, width=width, anchor=tk.W, stretch=False)
        tree_scroll = ttk.Scrollbar(
            list_frame,
            orient=tk.VERTICAL,
            command=self.tree.yview,
        )
        self.tree.configure(yscrollcommand=tree_scroll.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        tree_scroll.grid(row=0, column=1, sticky="ns")

        edit_frame = ttk.LabelFrame(right, text="Selected Point", padding=8)
        edit_frame.pack(fill=tk.X, padx=8, pady=6)
        self.fields: dict[str, tk.StringVar] = {}
        self._add_text_field(edit_frame, "name", "Name", 0)
        self.type_var = tk.StringVar()
        self.fields["type"] = self.type_var
        ttk.Label(edit_frame, text="Type").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Combobox(
            edit_frame,
            textvariable=self.type_var,
            state="readonly",
            values=VALID_TYPES,
        ).grid(row=1, column=1, sticky="ew", pady=2)
        for row, (key, label) in enumerate(
            (
                ("x", "X"),
                ("y", "Y"),
                ("z", "Z"),
                ("qx", "QX"),
                ("qy", "QY"),
                ("qz", "QZ"),
                ("qw", "QW"),
                ("yaw", "Yaw"),
            ),
            start=2,
        ):
            self._add_text_field(edit_frame, key, label, row)
        edit_frame.columnconfigure(1, weight=1)
        form_buttons = ttk.Frame(edit_frame)
        form_buttons.grid(row=10, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(form_buttons, text="Apply", command=self.apply_form).pack(
            side=tk.LEFT
        )
        ttk.Button(form_buttons, text="Delete", command=self.delete_selected).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        insert_frame = ttk.LabelFrame(right, text="Insert", padding=8)
        insert_frame.pack(fill=tk.X, padx=8, pady=(6, 8))
        self.insert_var = tk.StringVar()
        self.insert_combo = ttk.Combobox(
            insert_frame,
            textvariable=self.insert_var,
            state="readonly",
            values=(),
        )
        self.insert_combo.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.insert_button = ttk.Button(
            insert_frame,
            text="Insert",
            command=self.insert_between,
        )
        self.insert_button.pack(side=tk.LEFT, padx=(6, 0))

        self.status_var = tk.StringVar()
        ttk.Label(self.root, textvariable=self.status_var, anchor=tk.W).grid(
            row=1,
            column=0,
            sticky="ew",
            padx=6,
            pady=(0, 4),
        )

    def _add_text_field(
        self,
        parent: ttk.Frame,
        key: str,
        label: str,
        row: int,
    ) -> None:
        variable = tk.StringVar()
        self.fields[key] = variable
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
        ttk.Entry(parent, textvariable=variable).grid(
            row=row,
            column=1,
            sticky="ew",
            pady=2,
        )

    def _bind_events(self) -> None:
        self.file_combo.bind("<<ComboboxSelected>>", self._on_file_combo)
        self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._on_left_press)
        self.canvas.bind("<B1-Motion>", self._on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_release)
        self.canvas.bind("<ButtonPress-2>", self._on_pan_press)
        self.canvas.bind("<B2-Motion>", self._on_pan_drag)
        self.canvas.bind("<ButtonRelease-2>", self._on_pan_release)
        self.canvas.bind("<ButtonPress-3>", self._on_pan_press)
        self.canvas.bind("<B3-Motion>", self._on_pan_drag)
        self.canvas.bind("<ButtonRelease-3>", self._on_pan_release)
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind(
            "<Button-4>",
            lambda event: self.zoom_at(1.1, event.x, event.y),
        )
        self.canvas.bind(
            "<Button-5>",
            lambda event: self.zoom_at(0.9, event.x, event.y),
        )
        self.root.bind("<Control-s>", lambda _event: self.save_current_file())
        self.root.bind("<Delete>", lambda _event: self.delete_selected())

        for key, variable in self.fields.items():
            variable.trace_add(
                "write",
                lambda *_args, field=key: self._on_field_change(field),
            )

    def _refresh_file_list(self) -> None:
        files = sorted(self.waypoints_dir.glob("*.yaml")) + sorted(
            self.waypoints_dir.glob("*.yml")
        )
        labels = [str(path) for path in files]
        self.file_combo.configure(values=labels)
        self.map_label_var.set(
            f"{self.map_info.yaml_path.name} | res={self.map_info.resolution:g} "
            f"| origin=({self.map_info.origin_x:g}, {self.map_info.origin_y:g}, "
            f"{self.map_info.origin_yaw:g})"
        )

    def _load_initial_file(self, initial_file: Path | None) -> None:
        if initial_file is not None:
            self.load_file(initial_file)
            return
        values = list(self.file_combo.cget("values"))
        if values:
            self.load_file(Path(values[0]))
        else:
            self._set_status(f"No waypoint YAML files found in {self.waypoints_dir}")
            self.redraw()

    def _on_file_combo(self, _event: tk.Event) -> None:
        selected = self.file_var.get()
        if selected:
            self.load_file(Path(selected))

    def open_file_dialog(self) -> None:
        path = filedialog.askopenfilename(
            initialdir=str(self.waypoints_dir),
            title="Open waypoint YAML",
            filetypes=(("YAML files", "*.yaml *.yml"), ("All files", "*.*")),
        )
        if path:
            self.load_file(Path(path))

    def reload_current_file(self) -> None:
        if self.current_file is None:
            self._set_status("No waypoint file selected")
            return
        self.load_file(self.current_file)

    def load_file(self, path: Path) -> None:
        if not self._confirm_discard_changes():
            return
        try:
            document, points = load_waypoint_file(path)
        except Exception as error:
            messagebox.showerror("Load waypoint file", str(error))
            self._set_status(str(error))
            return
        self.current_file = path.expanduser().resolve()
        self.document = document
        self.points = points
        self.selected_index = 0 if points else None
        self.file_var.set(str(self.current_file))
        values = list(self.file_combo.cget("values"))
        if str(self.current_file) not in values:
            values.append(str(self.current_file))
            self.file_combo.configure(values=values)
        self._dirty = False
        self._refresh_all()
        self._set_status(f"Loaded {self.current_file} ({len(self.points)} points)")

    def _confirm_discard_changes(self) -> bool:
        if not self._dirty:
            return True
        return messagebox.askyesno(
            "Unsaved waypoint changes",
            "Current waypoint edits are not saved. Discard them and continue?",
        )

    def save_current_file(self) -> None:
        if self.current_file is None:
            self._set_status("No waypoint file selected")
            return
        try:
            self.apply_form(update_status=False)
            self._validate_points()
            backup = write_waypoint_file(
                self.current_file,
                self.document,
                self.points,
                self.backup_dir,
            )
        except Exception as error:
            messagebox.showerror("Save waypoint file", str(error))
            self._set_status(str(error))
            return
        self._dirty = False
        self._set_status(f"Saved {self.current_file}; backup: {backup}")

    def _validate_points(self) -> None:
        names: set[str] = set()
        for index, point in enumerate(self.points, start=1):
            if not point.name.strip():
                raise ValueError(f"point {index} name must not be empty")
            if point.name in names:
                raise ValueError(f"duplicate point name: {point.name}")
            if point.point_type not in VALID_TYPES:
                raise ValueError(f"point {point.name} type must be waypoint or goal")
            normalize_quaternion(point.qx, point.qy, point.qz, point.qw)
            names.add(point.name)

    def _refresh_all(self) -> None:
        self._refresh_tree()
        self._refresh_insert_choices()
        self._load_selected_to_form()
        self.redraw()

    def _refresh_tree(self) -> None:
        selected_iid = (
            str(self.selected_index) if self.selected_index is not None else ""
        )
        for item in self.tree.get_children():
            self.tree.delete(item)
        for index, point in enumerate(self.points):
            self.tree.insert(
                "",
                tk.END,
                iid=str(index),
                values=(
                    index + 1,
                    point.name,
                    point.point_type,
                    f"{point.x:.3f}",
                    f"{point.y:.3f}",
                ),
            )
        if selected_iid in self.tree.get_children():
            self.tree.selection_set(selected_iid)
            self.tree.see(selected_iid)

    def _refresh_insert_choices(self) -> None:
        previous_choice = self.insert_var.get()
        values = [f"{index + 1}-{index + 2}" for index in range(len(self.points) - 1)]
        if self.points:
            values.append(f"{len(self.points)}-end")
        self.insert_combo.configure(values=values)
        if values:
            if previous_choice.endswith("-end"):
                self.insert_var.set(values[-1])
            elif previous_choice not in values:
                self.insert_var.set(values[0])
            self.insert_button.configure(state=tk.NORMAL)
            self.insert_combo.configure(state="readonly")
        else:
            self.insert_var.set("")
            self.insert_button.configure(state=tk.DISABLED)
            self.insert_combo.configure(state=tk.DISABLED)

    def _on_tree_select(self, _event: tk.Event) -> None:
        selection = self.tree.selection()
        if not selection:
            return
        self.selected_index = int(selection[0])
        self._load_selected_to_form()
        self.redraw()

    def _load_selected_to_form(self) -> None:
        self._loading_form = True
        try:
            if self.selected_index is None or self.selected_index >= len(self.points):
                for variable in self.fields.values():
                    variable.set("")
                return
            point = self.points[self.selected_index]
            values = {
                "name": point.name,
                "type": point.point_type,
                "x": f"{point.x:.9g}",
                "y": f"{point.y:.9g}",
                "z": f"{point.z:.9g}",
                "qx": f"{point.qx:.9g}",
                "qy": f"{point.qy:.9g}",
                "qz": f"{point.qz:.9g}",
                "qw": f"{point.qw:.9g}",
                "yaw": f"{point.yaw:.9g}",
            }
            for key, value in values.items():
                self.fields[key].set(value)
            self._form_orientation_source = "yaw"
        finally:
            self._loading_form = False

    def _on_field_change(self, field_name: str) -> None:
        if self._loading_form:
            return
        if field_name == "yaw":
            self._form_orientation_source = "yaw"
        elif field_name in {"qx", "qy", "qz", "qw"}:
            self._form_orientation_source = "quaternion"

    def apply_form(self, update_status: bool = True) -> None:
        if self.selected_index is None:
            if update_status:
                self._set_status("No point selected")
            return
        if self.selected_index >= len(self.points):
            raise ValueError("selected point no longer exists")

        name = self.fields["name"].get().strip()
        if not name:
            raise ValueError("point name must not be empty")
        for index, point in enumerate(self.points):
            if index != self.selected_index and point.name == name:
                raise ValueError(f"duplicate point name: {name}")

        point_type = self.fields["type"].get().strip()
        if point_type not in VALID_TYPES:
            raise ValueError("point type must be waypoint or goal")

        x = parse_float(self.fields["x"].get(), "x")
        y = parse_float(self.fields["y"].get(), "y")
        z = parse_float(self.fields["z"].get(), "z")
        qx = parse_float(self.fields["qx"].get(), "qx")
        qy = parse_float(self.fields["qy"].get(), "qy")
        qz = parse_float(self.fields["qz"].get(), "qz")
        qw = parse_float(self.fields["qw"].get(), "qw")
        yaw = parse_float(self.fields["yaw"].get(), "yaw")
        if self._form_orientation_source == "yaw":
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
        else:
            qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
            yaw = yaw_from_quaternion(qx, qy, qz, qw)

        point = self.points[self.selected_index]
        point.name = name
        point.point_type = point_type
        point.x = x
        point.y = y
        point.z = z
        point.qx = qx
        point.qy = qy
        point.qz = qz
        point.qw = qw
        point.yaw = yaw
        self._dirty = True
        self._refresh_all()
        if update_status:
            self._set_status(f"Applied point {self.selected_index + 1}: {name}")

    def delete_selected(self) -> None:
        if self.selected_index is None or not self.points:
            self._set_status("No point selected")
            return
        removed = self.points.pop(self.selected_index)
        if self.points:
            self.selected_index = min(self.selected_index, len(self.points) - 1)
        else:
            self.selected_index = None
        self._dirty = True
        self._refresh_all()
        self._set_status(f"Deleted {removed.name}; indexes updated")

    def insert_between(self) -> None:
        value = self.insert_var.get()
        if not value:
            self._set_status("No insert range selected")
            return
        try:
            before = int(value.split("-", 1)[0]) - 1
            append_to_end = value.endswith("-end")
            if before < 0 or before >= len(self.points):
                raise ValueError
            if not append_to_end and before + 1 >= len(self.points):
                raise ValueError
        except ValueError:
            self._set_status("Invalid insert range")
            return
        if append_to_end:
            new_point = make_appended_point(self.points)
            insert_at = len(self.points)
        else:
            new_point = make_inserted_point(self.points, before)
            insert_at = before + 1
        self.points.insert(insert_at, new_point)
        self.selected_index = insert_at
        self._dirty = True
        self._refresh_all()
        self._set_status(f"Inserted {new_point.name} at index {insert_at + 1}")

    def fit_map(self) -> None:
        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        if canvas_width <= 1 or canvas_height <= 1:
            return
        margin = 20.0
        self.scale = min(
            (canvas_width - margin * 2.0) / self.map_info.width,
            (canvas_height - margin * 2.0) / self.map_info.height,
        )
        self.scale = max(0.05, self.scale)
        self.offset_x = (canvas_width - self.map_info.width * self.scale) * 0.5
        self.offset_y = (canvas_height - self.map_info.height * self.scale) * 0.5
        self.redraw()

    def zoom_actual(self) -> None:
        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        center_x = canvas_width * 0.5
        center_y = canvas_height * 0.5
        image_x, image_y = self.screen_to_image(center_x, center_y)
        self.scale = 1.0
        self.offset_x = center_x - image_x * self.scale
        self.offset_y = center_y - image_y * self.scale
        self.redraw()

    def zoom_by(self, factor: float) -> None:
        self.zoom_at(
            factor,
            self.canvas.winfo_width() // 2,
            self.canvas.winfo_height() // 2,
        )

    def zoom_at(self, factor: float, screen_x: float, screen_y: float) -> None:
        old_image_x, old_image_y = self.screen_to_image(screen_x, screen_y)
        self.scale = min(12.0, max(0.05, self.scale * factor))
        self.offset_x = screen_x - old_image_x * self.scale
        self.offset_y = screen_y - old_image_y * self.scale
        self.redraw()

    def screen_to_image(self, screen_x: float, screen_y: float) -> tuple[float, float]:
        return (
            (screen_x - self.offset_x) / self.scale,
            (screen_y - self.offset_y) / self.scale,
        )

    def image_to_screen(self, image_x: float, image_y: float) -> tuple[float, float]:
        return (
            self.offset_x + image_x * self.scale,
            self.offset_y + image_y * self.scale,
        )

    def point_to_screen(self, point: Point) -> tuple[float, float]:
        image_x, image_y = map_to_image(self.map_info, point.x, point.y)
        return self.image_to_screen(image_x, image_y)

    def redraw(self) -> None:
        self.canvas.delete("all")
        if self.map_info.width <= 0 or self.map_info.height <= 0:
            return

        scaled_width = max(1, int(round(self.map_info.width * self.scale)))
        scaled_height = max(1, int(round(self.map_info.height * self.scale)))
        resized = self.map_info.image.resize(
            (scaled_width, scaled_height),
            Image.NEAREST,
        )
        self.tk_map_image = ImageTk.PhotoImage(resized)
        self.canvas.create_image(
            self.offset_x,
            self.offset_y,
            image=self.tk_map_image,
            anchor=tk.NW,
        )
        self.canvas.create_rectangle(
            self.offset_x,
            self.offset_y,
            self.offset_x + scaled_width,
            self.offset_y + scaled_height,
            outline="#606060",
        )
        self._draw_origin()
        self._draw_route_line()
        for index, point in enumerate(self.points):
            self._draw_point(index, point)

    def _draw_origin(self) -> None:
        origin_px, origin_py = map_to_image(
            self.map_info,
            self.map_info.origin_x,
            self.map_info.origin_y,
        )
        if not (
            0 <= origin_px <= self.map_info.width
            and 0 <= origin_py <= self.map_info.height
        ):
            return
        x, y = self.image_to_screen(origin_px, origin_py)
        size = 9
        self.canvas.create_line(x - size, y, x + size, y, fill="#ff4040", width=2)
        self.canvas.create_line(x, y - size, x, y + size, fill="#ff4040", width=2)
        self.canvas.create_text(
            x + 8,
            y - 8,
            text="origin",
            anchor=tk.SW,
            fill="#ff6060",
            font=("TkDefaultFont", 9, "bold"),
        )

    def _draw_route_line(self) -> None:
        if len(self.points) < 2:
            return
        coords: list[float] = []
        for point in self.points:
            x, y = self.point_to_screen(point)
            coords.extend((x, y))
        self.canvas.create_line(*coords, fill="#ffcc33", width=2)

    def _draw_point(self, index: int, point: Point) -> None:
        x, y = self.point_to_screen(point)
        selected = index == self.selected_index
        radius = 8 if selected else 6
        fill = "#2bd95f" if point.point_type == "goal" else "#20c8ff"
        outline = "#ffffff" if selected else "#101010"
        width = 3 if selected else 1

        self.canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            fill=fill,
            outline=outline,
            width=width,
        )
        arrow_length = 26 if selected else 20
        end_x = x + math.cos(point.yaw) * arrow_length
        end_y = y - math.sin(point.yaw) * arrow_length
        self.canvas.create_line(
            x,
            y,
            end_x,
            end_y,
            fill=fill,
            width=3 if selected else 2,
            arrow=tk.LAST,
            arrowshape=(9, 11, 4),
        )
        label = f"{index + 1}: {point.name} [{point.point_type}]"
        self.canvas.create_text(
            x + 10,
            y - 10,
            text=label,
            anchor=tk.SW,
            fill="#ffffff",
            font=("TkDefaultFont", 9, "bold" if selected else "normal"),
        )

    def _nearest_point_index(self, screen_x: float, screen_y: float) -> int | None:
        best_index: int | None = None
        best_distance = 16.0
        for index, point in enumerate(self.points):
            px, py = self.point_to_screen(point)
            distance = math.hypot(screen_x - px, screen_y - py)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def _on_left_press(self, event: tk.Event) -> None:
        index = self._nearest_point_index(event.x, event.y)
        if index is None:
            self.drag_mode = None
            return
        self.selected_index = index
        self._refresh_tree()
        self._load_selected_to_form()
        self.drag_mode = "yaw" if event.state & SHIFT_MASK else "move"
        self.redraw()

    def _on_left_drag(self, event: tk.Event) -> None:
        if self.selected_index is None or self.drag_mode is None:
            return
        point = self.points[self.selected_index]
        image_x, image_y = self.screen_to_image(event.x, event.y)
        if self.drag_mode == "yaw" or event.state & SHIFT_MASK:
            point_screen_x, point_screen_y = self.point_to_screen(point)
            point.yaw = math.atan2(point_screen_y - event.y, event.x - point_screen_x)
            point.qx, point.qy, point.qz, point.qw = quaternion_from_yaw(point.yaw)
        else:
            point.x, point.y = image_to_map(self.map_info, image_x, image_y)
        self._dirty = True
        self._refresh_tree()
        self._load_selected_to_form()
        self.redraw()

    def _on_left_release(self, _event: tk.Event) -> None:
        self.drag_mode = None

    def _on_pan_press(self, event: tk.Event) -> None:
        self.pan_start = (event.x, event.y, self.offset_x, self.offset_y)

    def _on_pan_drag(self, event: tk.Event) -> None:
        if self.pan_start is None:
            return
        start_x, start_y, start_offset_x, start_offset_y = self.pan_start
        self.offset_x = start_offset_x + event.x - start_x
        self.offset_y = start_offset_y + event.y - start_y
        self.redraw()

    def _on_pan_release(self, _event: tk.Event) -> None:
        self.pan_start = None

    def _on_mousewheel(self, event: tk.Event) -> None:
        factor = 1.1 if event.delta > 0 else 0.9
        self.zoom_at(factor, event.x, event.y)

    def _on_motion(self, event: tk.Event) -> None:
        image_x, image_y = self.screen_to_image(event.x, event.y)
        map_x, map_y = image_to_map(self.map_info, image_x, image_y)
        dirty = " | modified" if self._dirty else ""
        file_name = self.current_file.name if self.current_file else "no file"
        self.status_var.set(
            f"{file_name}{dirty} | map=({map_x:.3f}, {map_y:.3f}) "
            f"| {self.map_info.image_path.name}"
        )

    def _set_status(self, message: str) -> None:
        dirty = " | modified" if self._dirty else ""
        self.status_var.set(f"{message}{dirty}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Edit fruit waypoint YAML files on the Nav2 PGM map.",
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=DEFAULT_MAP,
        help=f"Nav2 map YAML path (default: {DEFAULT_MAP})",
    )
    parser.add_argument(
        "--waypoints-dir",
        type=Path,
        default=DEFAULT_WAYPOINTS_DIR,
        help=f"Waypoint YAML directory (default: {DEFAULT_WAYPOINTS_DIR})",
    )
    parser.add_argument(
        "--file",
        type=Path,
        default=None,
        help="Waypoint YAML file to open at startup.",
    )
    parser.add_argument(
        "--display",
        default=DEFAULT_DISPLAY,
        help=f"Desktop display for the GUI window (default: {DEFAULT_DISPLAY})",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        configure_desktop_display(args.display)
        root = tk.Tk()
        WaypointEditorApp(root, args.map, args.waypoints_dir, args.file)
        root.mainloop()
    except Exception as error:
        print(f"waypoint_editor: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
