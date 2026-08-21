#!/usr/bin/env python3
"""Small desktop control panel for the field mission scripts."""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable


WORKSPACE = Path(__file__).resolve().parent
NAVIGATION_SCRIPT = WORKSPACE / "start_navigation.sh"
MISSION_SCRIPT = WORKSPACE / "start_fruit_mission.sh"
KILL_SCRIPT = WORKSPACE / "kill_fruit_processes.sh"


class ToggleSwitch(tk.Canvas):
    def __init__(
        self,
        parent: tk.Misc,
        variable: tk.BooleanVar,
        command: Callable[[], None],
        background: str,
    ) -> None:
        super().__init__(
            parent,
            width=112,
            height=56,
            background=background,
            borderwidth=0,
            highlightthickness=3,
            highlightbackground=background,
            highlightcolor="#0b5d3b",
            cursor="hand2",
            takefocus=True,
        )
        self.variable = variable
        self.command = command
        self.enabled = True
        self.variable.trace_add("write", self._redraw)
        self.bind("<Button-1>", self._toggle)
        self.bind("<space>", self._toggle)
        self.bind("<Return>", self._toggle)
        self.bind("<FocusIn>", self._redraw)
        self.bind("<FocusOut>", self._redraw)
        self._redraw()

    def set_enabled(self, enabled: bool) -> None:
        if self.enabled == enabled:
            return
        self.enabled = enabled
        self.configure(cursor="hand2" if enabled else "arrow")
        self._redraw()

    def _toggle(self, _event: object = None) -> str:
        if self.enabled:
            self.variable.set(not self.variable.get())
            self.command()
        return "break"

    def _redraw(self, *_args: object) -> None:
        self.delete("all")
        if not self.enabled:
            track_color = "#75b798" if self.variable.get() else "#b5b8bc"
        elif self.variable.get():
            track_color = "#16834f"
        else:
            track_color = "#656b73"
        self.create_oval(8, 8, 48, 48, fill=track_color, outline=track_color)
        self.create_rectangle(28, 8, 84, 48, fill=track_color, outline=track_color)
        self.create_oval(64, 8, 104, 48, fill=track_color, outline=track_color)
        knob_left = 66 if self.variable.get() else 10
        knob_color = "#f8fafc" if self.enabled else "#e5e7eb"
        self.create_oval(
            knob_left,
            10,
            knob_left + 36,
            46,
            fill=knob_color,
            outline="#d1d5db",
            width=1,
        )


class MissionPanel:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("水果采摘任务控制")
        self.root.geometry("1100x760")
        self.root.minsize(880, 640)
        self.root.attributes("-zoomed", True)
        # A physical field must be selected explicitly before any launch action.
        self.map_var = tk.StringVar(value="")
        self.flow_var = tk.StringVar(value="full")
        self.odom_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="请先选择场地地图 A 或 B")
        self.output_queue: queue.Queue[str] = queue.Queue()
        self.processes: dict[str, subprocess.Popen[str]] = {}
        self.map_radios: list[ttk.Radiobutton] = []
        self.flow_box: ttk.Combobox | None = None
        self.odom_switch: ToggleSwitch | None = None
        self.navigation_button: ttk.Button | None = None
        self.mission_button: ttk.Button | None = None
        self._build_widgets()
        self._refresh_controls()
        self.root.after(200, self._drain_output)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def _build_widgets(self) -> None:
        self.control_font = tkfont.nametofont("TkDefaultFont").copy()
        self.control_font.configure(size=24)
        self.button_font = tkfont.nametofont("TkDefaultFont").copy()
        self.button_font.configure(size=27, weight="bold")
        self.log_font = tkfont.nametofont("TkFixedFont").copy()
        self.log_font.configure(size=14)
        style = ttk.Style(self.root)
        style.configure("Mission.TLabel", font=self.control_font)
        style.configure("Mission.TRadiobutton", font=self.control_font)
        style.configure("Mission.TCombobox", font=self.control_font, padding=6)
        style.configure("Mission.TButton", font=self.button_font, padding=(18, 22))
        style.configure(
            "MissionStatus.TLabel",
            font=self.control_font,
            foreground="#0b5d3b",
        )

        frame = ttk.Frame(self.root, padding=14)
        frame.pack(fill=tk.BOTH, expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=2, uniform="main_sections")
        frame.rowconfigure(1, weight=1, uniform="main_sections")

        control_panel = ttk.Frame(frame, padding=(12, 12, 12, 22))
        control_panel.grid(row=0, column=0, sticky="nsew")
        control_panel.columnconfigure(3, weight=1)

        ttk.Label(
            control_panel, text="场地地图", style="Mission.TLabel"
        ).grid(row=0, column=0, sticky="w", pady=10)
        for column, map_id in enumerate(("A", "B"), start=1):
            radio = ttk.Radiobutton(
                control_panel,
                text=map_id,
                value=map_id,
                variable=self.map_var,
                command=lambda selected=map_id: self._on_map_selected(selected),
                style="Mission.TRadiobutton",
            )
            radio.grid(row=0, column=column, sticky="w", padx=14)
            self.map_radios.append(radio)

        ttk.Label(
            control_panel, text="任务区域流程", style="Mission.TLabel"
        ).grid(row=1, column=0, sticky="w", pady=10)
        flows = {
            "full": "全程",
            "ac": "AC",
            "a": "A区",
            "b": "B区",
            "c": "C区",
        }
        flow_box = ttk.Combobox(
            control_panel,
            textvariable=self.flow_var,
            values=list(flows),
            state="readonly",
            width=14,
            style="Mission.TCombobox",
        )
        flow_box.grid(row=1, column=1, columnspan=2, sticky="w", padx=14)
        flow_box.bind("<<ComboboxSelected>>", lambda _event: self._update_status())
        self.flow_box = flow_box

        ttk.Label(
            control_panel,
            text="纯里程计模式",
            style="Mission.TLabel",
        ).grid(row=2, column=0, sticky="w", pady=12)
        switch_background = style.lookup("TFrame", "background") or "#d9d9d9"
        odom_switch = ToggleSwitch(
            control_panel,
            variable=self.odom_var,
            command=self._update_status,
            background=switch_background,
        )
        odom_switch.grid(
            row=2,
            column=1,
            sticky="w",
            padx=14,
            pady=12,
        )
        ttk.Label(
            control_panel,
            text="--odom，不加载地图/GICP",
            style="Mission.TLabel",
        ).grid(row=2, column=2, columnspan=2, sticky="w", padx=14, pady=12)
        self.odom_switch = odom_switch

        controls = ttk.Frame(control_panel)
        controls.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(14, 8))
        for column in range(3):
            controls.columnconfigure(column, weight=1, uniform="launch_buttons")
        self.navigation_button = ttk.Button(
            controls,
            text="启动定位/导航",
            command=self.start_navigation,
            style="Mission.TButton",
        )
        self.navigation_button.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self.mission_button = ttk.Button(
            controls,
            text="启动比赛流程",
            command=self.start_mission,
            style="Mission.TButton",
        )
        self.mission_button.grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(
            controls,
            text="停止全部进程",
            command=self.stop_stack,
            style="Mission.TButton",
        ).grid(row=0, column=2, sticky="ew", padx=(8, 0))

        ttk.Label(
            control_panel, text="当前状态", style="Mission.TLabel"
        ).grid(row=4, column=0, sticky="nw", pady=10)
        ttk.Label(
            control_panel,
            textvariable=self.status_var,
            style="MissionStatus.TLabel",
            wraplength=720,
        ).grid(row=4, column=1, columnspan=3, sticky="nw", padx=14, pady=10)

        log_panel = ttk.Frame(frame)
        log_panel.grid(row=1, column=0, sticky="nsew")
        log_panel.columnconfigure(0, weight=1)
        log_panel.rowconfigure(0, weight=1)
        self.output = tk.Text(
            log_panel,
            height=6,
            state=tk.DISABLED,
            wrap=tk.WORD,
            font=self.log_font,
        )
        self.output.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(
            log_panel, orient=tk.VERTICAL, command=self.output.yview
        )
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.output.configure(yscrollcommand=scrollbar.set)

    def _command_env(self) -> dict[str, str]:
        env = os.environ.copy()
        runtime_dir = f"/run/user/{os.getuid()}"
        env.update(
            {
                "DISPLAY": ":0",
                "XDG_RUNTIME_DIR": runtime_dir,
                "XAUTHORITY": f"{runtime_dir}/gdm/Xauthority",
                "DBUS_SESSION_BUS_ADDRESS": f"unix:path={runtime_dir}/bus",
            }
        )
        return env

    def _start(self, name: str, command: list[str]) -> None:
        process = self.processes.get(name)
        if process is not None and process.poll() is None:
            self._append(f"{name} 已在运行，PID={process.pid}")
            return
        self._append("启动: " + " ".join(command))
        try:
            process = subprocess.Popen(
                command,
                cwd=WORKSPACE,
                env=self._command_env(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as error:
            self.status_var.set(f"启动失败: {error}")
            self._append(str(error))
            return
        self.processes[name] = process
        mode = (
            "纯里程计 odom，等待稳定起点"
            if self.odom_var.get()
            else "地图/GICP"
        )
        self.status_var.set(f"{name} 运行中，模式 {mode}，PID={process.pid}")
        self._refresh_controls()
        threading.Thread(
            target=self._read_process, args=(name, process), daemon=True
        ).start()

    def _read_process(self, name: str, process: subprocess.Popen[str]) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                self.output_queue.put(f"[{name}] {line.rstrip()}")
        code = process.wait()
        self.output_queue.put(f"[{name}] 已退出，状态码={code}")

    def start_navigation(self) -> None:
        map_id = self.map_var.get().strip()
        if not map_id:
            self.status_var.set("请先选择场地地图 A 或 B")
            return
        if self._is_running("mission"):
            self.status_var.set("比赛流程正在运行，请先停止全部进程")
            return
        command = [str(NAVIGATION_SCRIPT), "--map", map_id]
        if self.odom_var.get():
            command.append("--odom")
        self._start(
            "navigation",
            command,
        )

    def start_mission(self) -> None:
        map_id = self.map_var.get().strip()
        flow = self.flow_var.get().strip()
        if not map_id:
            self.status_var.set("请先选择场地地图 A 或 B")
            return
        if not flow:
            self.status_var.set("请先选择任务区域流程")
            return
        command = [
            str(MISSION_SCRIPT),
            "--map",
            map_id,
            "--flow",
            flow,
        ]
        if self.odom_var.get():
            command.append("--odom")
        if self._is_running("navigation"):
            command.append("--reuse-navigation")
        self._start(
            "mission",
            command,
        )

    def stop_stack(self) -> None:
        try:
            result = subprocess.run(
                [str(KILL_SCRIPT)],
                cwd=WORKSPACE,
                env=self._command_env(),
                capture_output=True,
                text=True,
                check=False,
            )
        except OSError as error:
            self.status_var.set(f"停止失败: {error}")
            return
        self._append(result.stdout.rstrip())
        if result.stderr:
            self._append(result.stderr.rstrip())
        self.status_var.set("已请求停止全部任务进程")

    def _update_status(self) -> None:
        if not self.map_var.get():
            self.status_var.set("请先选择场地地图 A 或 B")
            return
        mode = (
            "纯里程计 odom（启动后等待稳定起点）"
            if self.odom_var.get()
            else "地图/GICP"
        )
        self.status_var.set(
            f"场地地图 {self.map_var.get()}，模式 {mode}，任务区域流程 {self.flow_var.get()}，等待启动"
        )

    def _on_map_selected(self, map_id: str) -> None:
        self.map_var.set(map_id)
        self._update_status()
        self._refresh_controls()

    def _is_running(self, name: str) -> bool:
        process = self.processes.get(name)
        return process is not None and process.poll() is None

    def _refresh_controls(self) -> None:
        """Keep launch controls aligned with the selected map and active stack."""
        map_selected = bool(self.map_var.get().strip())
        navigation_running = self._is_running("navigation")
        mission_running = self._is_running("mission")
        stack_running = navigation_running or mission_running

        radio_state = "disabled" if stack_running else "normal"
        for radio in self.map_radios:
            radio.configure(state=radio_state)

        if self.flow_box is not None:
            self.flow_box.configure(
                state="readonly"
                if map_selected and not mission_running
                else "disabled"
            )
        if self.odom_switch is not None:
            self.odom_switch.set_enabled(not stack_running)
        if self.navigation_button is not None:
            self.navigation_button.configure(
                state="normal"
                if map_selected and not stack_running
                else "disabled"
            )
        if self.mission_button is not None:
            self.mission_button.configure(
                state="normal"
                if map_selected and not mission_running
                else "disabled"
            )

    def _append(self, text: str) -> None:
        if not text:
            return
        self.output.configure(state=tk.NORMAL)
        self.output.insert(tk.END, text + "\n")
        self.output.see(tk.END)
        self.output.configure(state=tk.DISABLED)

    def _drain_output(self) -> None:
        while True:
            try:
                line = self.output_queue.get_nowait()
            except queue.Empty:
                break
            self._append(line)
            odom_ready = (
                "stable odom origin is ready" in line
                or "READY stable odometry origin" in line
            )
            if self.odom_var.get() and odom_ready:
                active = "mission" if self._is_running("mission") else "navigation"
                self.status_var.set(f"{active} 运行中，纯里程计 odom 起点已稳定")
        self._refresh_controls()
        self.root.after(200, self._drain_output)

    def _close(self) -> None:
        running = [name for name, process in self.processes.items() if process.poll() is None]
        if running and not messagebox.askyesno(
            "退出",
            "任务进程仍在运行，退出 GUI 但不停止它们？",
        ):
            return
        self.root.destroy()


def main() -> int:
    os.environ.setdefault("DISPLAY", ":0")
    os.environ.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    os.environ.setdefault(
        "XAUTHORITY", f"{os.environ['XDG_RUNTIME_DIR']}/gdm/Xauthority"
    )
    os.environ.setdefault(
        "DBUS_SESSION_BUS_ADDRESS",
        f"unix:path={os.environ['XDG_RUNTIME_DIR']}/bus",
    )
    root = tk.Tk()
    MissionPanel(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
