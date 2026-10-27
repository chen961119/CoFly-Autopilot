"""CoFly Ground Station: USB or 3DR X6 connection for parameters and live data."""
import json
import math
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk, font, messagebox

from parameter_view import ParameterConsole
from parameter_file import SUPPORTED_FILE_VERSIONS
from station_protocol import MODES, decode_fields
from wing_geometry import ORDERS, configuration_ready, aircraft_attitudes, wing_surfaces, project_wing
from auto_connect import FlightControllerDetector
from radio_calibration import RadioCalibrationDialog
from terminal_filter import AutomaticQueryFilter
from sd_files import SdFileDialog

SETTINGS_PATH = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "CoFly" / "ground_station.json"


class GroundStation(ParameterConsole):
    def __init__(self, root, settings_path=None):
        self.settings_path = Path(settings_path) if settings_path else SETTINGS_PATH
        try:
            settings = json.loads(self.settings_path.read_text(encoding="utf-8"))
            if not isinstance(settings, dict):
                settings = {}
        except (OSError, ValueError):
            settings = {}
        self.theme = tk.StringVar(root, value=settings.get("theme") if settings.get("theme") in ("室内", "室外") else "室内")
        size = settings.get("font_size", 11)
        self.font_size = tk.IntVar(root, value=size if type(size) is int and 9 <= size <= 18 else 11)
        self.auto_connect = tk.BooleanVar(root, value=settings.get("auto_connect") is True)
        saved_link_type = settings.get("link_type")
        self.link_type = tk.StringVar(root, value=saved_link_type if saved_link_type in ("USB", "3DR X6") else "USB")
        self.auto_detector = None
        self.link_timeouts = 0
        self.next_link_check = 0
        self.link_bauds = {"USB": settings.get("usb_baud", "921600"),
                           "3DR X6": settings.get("radio_baud", "57600")}
        if "baud" in settings:
            self.link_bauds[self.link_type.get()] = settings["baud"]
        self.auto_next_scan = 0
        self.auto_paused = False
        self.local_aircraft = "本机"
        self.attitude_cards = {}
        self.attitude_columns = 0
        self.pages = {}
        self.nav = {}
        self.caps = {}
        self.terminal_filter = AutomaticQueryFilter()
        self.gcs_pending = None
        self.deferred_action = None
        self.radio_calibration = None
        self.radio_save = None
        self.sd_browser = None
        self.last_data = 0
        self.data = {}
        self.configuration = {}
        self.last_configuration = 0
        self.config_supported = True
        self.auto_probe = True
        self.page = "飞行"
        self.metric_vars = {}
        self.action_buttons = []
        super().__init__(root)
        root.title("CoFly 地面站 · USB / 3DR X6")
        root.geometry("1280x850")
        root.minsize(1000, 720)
        self.port.set(str(settings.get("port", self.port.get())))
        self.baud.set(str(self.link_bauds[self.link_type.get()]))
        self.apply_appearance(save=False)
        self.show_page("飞行")
        self.station_timer = root.after(200, self._station_tick)

    def _build_widgets(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        shell = ttk.Frame(self.root)
        shell.pack(fill="both", expand=True)
        top = ttk.Frame(shell, padding=(20, 12))
        top.pack(fill="x")
        ttk.Label(top, text="CoFly", style="Brand.TLabel").pack(side="left")
        ttk.Label(top, text="  地面站 / USB · 3DR X6", style="Muted.TLabel").pack(side="left")
        self.link_label = tk.StringVar(value="● 未连接")
        ttk.Label(top, textvariable=self.link_label, style="Accent.TLabel").pack(side="right")
        ttk.Label(shell, textvariable=self.status, padding=(20, 10), wraplength=1150).pack(side="bottom", fill="x")
        body = ttk.Frame(shell)
        body.pack(fill="both", expand=True)
        sidebar = ttk.Frame(body, padding=12, width=170)
        sidebar.pack(side="left", fill="y")
        content = ttk.Frame(body, padding=(12, 8))
        content.pack(side="left", fill="both", expand=True)
        for section, names in (("飞行", ("飞行",)), ("飞机设置", ("参数", "校准", "遥控器", "串口助手")),
                               ("应用设置", ("通讯连接", "应用设置"))):
            ttk.Label(sidebar, text=section, style="Muted.TLabel").pack(anchor="w", pady=(18, 8))
            for name in names:
                self.nav[name] = ttk.Button(sidebar, text=name, width=16, command=lambda n=name: self.show_page(n))
                self.nav[name].pack(fill="x", pady=3, ipady=6)
                page = ttk.Frame(content)
                page.grid(row=0, column=0, sticky="nsew")
                self.pages[name] = page
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)
        self.parameter_parent = self.pages["参数"]
        self._heading(self.pages["串口助手"], "串口助手", "共用通讯连接页的串口 · 接收日志、发送命令与导出记录")
        self.terminal_parent = self.pages["串口助手"]
        self.connection_parent = ttk.Frame(self.pages["通讯连接"], padding=12)
        self._heading(self.pages["通讯连接"], "通讯连接", "USB 直连选 921600；3DR X6 选地面端 COM 口，默认 57600。请关闭占用端口的其他程序。")
        self.connection_parent.pack(fill="x", pady=12)
        self.connection_parent.columnconfigure(0, weight=1)
        preset = ttk.Frame(self.pages["通讯连接"], padding=(12, 0))
        preset.pack(before=self.connection_parent, fill="x")
        ttk.Label(preset, text="连接方式").pack(side="left")
        self.link_type_box = ttk.Combobox(preset, textvariable=self.link_type,
                                         values=("USB", "3DR X6"), state="readonly", width=12)
        self.link_type_box.pack(side="left", padx=8)
        self.link_type_box.bind("<<ComboboxSelected>>", self._select_link_type)
        ttk.Label(preset, text="支持自动识别 USB / X6；也可手动选择端口连接。").pack(side="left")
        super()._build_widgets()
        self.sd_download_button = ttk.Button(self.terminal_tools, text="下载 SD 卡内容…", command=self._open_sd_files)
        self.sd_download_button.pack(side="right")
        parameter_actions = ttk.Frame(self.parameter_parent)
        parameter_actions.pack(before=self.parameter_parent.winfo_children()[0], fill="x", padx=12, pady=(8, 0))
        self.parameter_read = self.read_button = ttk.Button(parameter_actions, text="读取参数", command=self.read_parameters)
        self.parameter_read.pack(side="left")
        self.parameter_load_file = ttk.Button(parameter_actions, text="从文件读取", command=self.load_parameters_from_file)
        self.parameter_load_file.pack(side="left", padx=8)
        self.parameter_save_file = ttk.Button(parameter_actions, text="保存到文件", command=self.save_parameters_to_file)
        self.parameter_save_file.pack(side="left")
        self._build_flight()
        self._build_calibration()
        self._build_radio()
        self._build_settings()
        self.device_label = tk.StringVar(value="固件能力尚未识别")
        ttk.Label(self.pages["通讯连接"], textvariable=self.device_label, wraplength=740).pack(anchor="w", padx=12, pady=12)
        self.probe_button = ttk.Button(self.pages["通讯连接"], text="识别固件 / 重试", command=lambda: self._request("CAPS"))
        self.reboot_button = ttk.Button(self.pages["通讯连接"], text="重启飞控", command=lambda: self._request("REBOOT"), state="disabled")
        self.reboot_button.pack(anchor="w", padx=12, pady=6)
        ttk.Label(self.pages["通讯连接"], text="重启前需锁定飞机；设备重启后请重新连接。").pack(anchor="w", padx=12)
        self.probe_button.pack(anchor="w", padx=12)
        ttk.Label(self.pages["通讯连接"], text="连接后自动识别固件。旧固件仍可使用参数页；数据与传感器页需要新版固件。", wraplength=740).pack(anchor="w", padx=12, pady=18)

    def _select_link_type(self, _=None):
        if self.auto_detector:
            self.auto_detector.stop()
        self.baud.set("57600" if self.link_type.get() == "3DR X6" else "921600")
        self._save_settings()
        self._update_controls()

    def _heading(self, page, title, subtitle):
        ttk.Label(page, text=title, style="Title.TLabel").pack(anchor="w", pady=(12, 6))
        ttk.Label(page, text=subtitle, style="Muted.TLabel", wraplength=740).pack(anchor="w", pady=(0, 18))

    def _build_flight(self):
        outer = self.pages["飞行"]
        self.flight_scroll = tk.Canvas(outer, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=self.flight_scroll.yview)
        scrollbar.pack(side="right", fill="y")
        self.flight_scroll.pack(side="left", fill="both", expand=True)
        self.flight_scroll.configure(yscrollcommand=scrollbar.set)
        page = ttk.Frame(self.flight_scroll)
        window = self.flight_scroll.create_window(0, 0, window=page, anchor="nw")
        self.flight_scroll.bind("<Configure>", lambda event: (self.flight_scroll.itemconfigure(window, width=event.width), self._layout_attitude_cards(event.width)))
        page.bind("<Configure>", lambda _: self.flight_scroll.configure(scrollregion=self.flight_scroll.bbox("all")))
        self._heading(page, "飞行数据", "各机姿态与翼尖连接构型 · 远端偏航未传输，以 — 显示")
        self.freshness = tk.StringVar(value="等待连接")
        ttk.Label(page, textvariable=self.freshness, style="Accent.TLabel").pack(anchor="w", pady=(0, 6))
        self.attitude_grid = ttk.Frame(page)
        self.attitude_grid.pack(fill="x")
        self._sync_attitude_cards(("本机",))
        dashboard = ttk.Frame(page)
        dashboard.pack(fill="x", pady=6)
        groups = (("本机角速度 · °/s", (("gx", "X"), ("gy", "Y"), ("gz", "Z"))),
                  ("本机加速度 · g", (("ax", "X"), ("ay", "Y"), ("az", "Z"))),
                  ("本机速度 / 位置", (("airspeed", "空速 m/s"), ("groundspeed", "地速 m/s"), ("altitude", "高度 m"))),
                  ("飞机状态", (("mode", "模式"), ("locked", "锁定"), ("angle", "构型角 °"))))
        for index, (title, metrics) in enumerate(groups):
            card = ttk.Labelframe(dashboard, text=title, padding=(8, 4))
            card.grid(row=0, column=index, sticky="nsew", padx=4)
            dashboard.columnconfigure(index, weight=1)
            for key, label in metrics:
                row = ttk.Frame(card)
                row.pack(fill="x", pady=2)
                ttk.Label(row, text=label, style="Muted.TLabel").pack(side="left")
                var = self.metric_vars[key] = tk.StringVar(value="—")
                ttk.Label(row, textvariable=var).pack(side="right", padx=(4, 0))
        formation = ttk.Labelframe(page, text="实时构型 · 滚转 / 俯仰斜视图", padding=8)
        formation.pack(fill="x", pady=6)
        self.configuration_label = tk.StringVar(value="等待多机姿态")
        ttk.Label(formation, textvariable=self.configuration_label, style="Muted.TLabel", wraplength=1050).pack(anchor="w")
        self.wings = tk.Canvas(formation, height=215, highlightthickness=0)
        self.wings.pack(fill="x", expand=True)
        self.wings.bind("<Configure>", lambda _: self._draw_configuration())
        ttk.Label(page, text="翼面倾斜表示滚转；机头箭头与虚线零俯仰参考线的偏差表示俯仰。等翼展示意，不代表位置或航向。",
                  style="Muted.TLabel", wraplength=1050).pack(anchor="w", pady=6)

    def _sync_attitude_cards(self, order):
        if tuple(self.attitude_cards) == tuple(order):
            return
        for card in self.attitude_cards.values():
            card["frame"].destroy()
        self.attitude_cards.clear()
        self.attitude_columns = 0
        for column in range(4):
            self.attitude_grid.columnconfigure(column, weight=0)
        columns = min(4, len(order))
        for index, name in enumerate(order):
            frame = ttk.Labelframe(self.attitude_grid, text=name if name == "本机" else f"{name}机", padding=6)
            frame.grid(row=index // columns, column=index % columns, sticky="nsew", padx=4, pady=4)
            self.attitude_grid.columnconfigure(index % columns, weight=1, uniform="attitude")
            state = tk.StringVar(value="等待姿态数据")
            ttk.Label(frame, textvariable=state, style="Muted.TLabel").pack(anchor="w")
            canvas = tk.Canvas(frame, width=190, height=120, highlightthickness=0)
            canvas.pack(fill="x", expand=True)
            text = tk.StringVar(value="滚转 —   俯仰 —\n偏航 —")
            ttk.Label(frame, textvariable=text, anchor="center").pack(fill="x", pady=(4, 0))
            self.attitude_cards[name] = {"frame": frame, "canvas": canvas, "text": text, "state": state}
            canvas.bind("<Configure>", lambda _, n=name: self._draw_horizon(n))
        self._layout_attitude_cards(max(700, self.flight_scroll.winfo_width()))

    def _layout_attitude_cards(self, width):
        if not self.attitude_cards:
            return
        columns = max(1, min(4, len(self.attitude_cards), width // max(230, self.font_size.get() * 24)))
        if columns == self.attitude_columns:
            return
        self.attitude_columns = columns
        for column in range(4):
            self.attitude_grid.columnconfigure(column, weight=1 if column < columns else 0, uniform="attitude" if column < columns else "")
        for index, card in enumerate(self.attitude_cards.values()):
            card["frame"].grid_configure(row=index // columns, column=index % columns)


    def _build_calibration(self):
        page = self.pages["校准"]
        self._heading(page, "传感器校准", "识别当前固件的实际采样路径；USB 校准与传感器设置接口预留至下一版本。")
        self.sensor_tree = ttk.Treeview(page, columns=("sensor", "state"), show="headings", height=7, selectmode="browse")
        self.sensor_tree.heading("sensor", text="传感器 / 来源")
        self.sensor_tree.heading("state", text="配置与可用性")
        self.sensor_tree.column("sensor", width=210)
        self.sensor_tree.column("state", width=520)
        self.sensor_tree.pack(fill="x", pady=12)
        controls = ttk.Frame(page)
        controls.pack(fill="x", pady=12)
        self.sensor_setting = tk.StringVar(value="DEFAULT")
        ttk.Label(controls, text="设置预留项").pack(side="left")
        ttk.Combobox(controls, textvariable=self.sensor_setting, values=("DEFAULT",), state="readonly", width=15).pack(side="left", padx=10)
        for label, verb in (("校准所选传感器（预留）", "CALIBRATE"), ("应用设置（预留）", "SENSOR_SET")):
            button = ttk.Button(controls, text=label, command=lambda v=verb: self._sensor_action(v))
            button.pack(side="left", padx=4)
            self.action_buttons.append(button)
        self.sensor_note = tk.StringVar(value="连接并识别固件后显示传感器。")
        ttk.Label(page, textvariable=self.sensor_note, wraplength=740).pack(anchor="w", pady=12)
        self.calibration_radio_button = ttk.Button(page, text="校准遥控器", command=self._calibrate_radio, state="disabled")
        self.calibration_radio_button.pack(anchor="w", pady=8)

    def _build_radio(self):
        page = self.pages["遥控器"]
        self._heading(page, "遥控器", "八路输入统一归一化至 [-1,1]，油门最低为 -1；CH5–8 为辅助1–4。")
        self.radio_label = tk.StringVar(value="接收机：未识别")
        ttk.Label(page, textvariable=self.radio_label).pack(anchor="w", pady=8)
        self.radio_calibrate_button = ttk.Button(page, text="校准遥控器", command=self._calibrate_radio, state="disabled")
        self.radio_calibrate_button.pack(anchor="w", pady=8)
        self.rc_bars = []
        self.rc_normalized = []
        for index, name in enumerate(("滚转", "俯仰", "油门", "偏航", "辅助1", "辅助2", "辅助3", "辅助4"), 1):
            row = ttk.Frame(page)
            row.pack(fill="x", pady=5)
            ttk.Label(row, text=f"CH{index}  {name}", width=16).pack(side="left")
            bar = ttk.Progressbar(row, maximum=1000)
            bar.pack(side="left", fill="x", expand=True, padx=16)
            var = self.metric_vars[f"rc{index}"] = tk.StringVar(value="— μs")
            ttk.Label(row, textvariable=var, width=12).pack(side="right")
            normalized = tk.StringVar(value="—")
            ttk.Label(row, textvariable=normalized, width=9).pack(side="right")
            self.rc_normalized.append(normalized)
            self.rc_bars.append(bar)
    def _build_settings(self):
        page = self.pages["应用设置"]
        self._heading(page, "应用设置", "配色、字体大小与自动连接选项，设置自动保存到本机。")
        row = ttk.Frame(page)
        row.pack(fill="x", pady=12)
        ttk.Label(row, text="配色", width=16).pack(side="left")
        for value in ("室内", "室外"):
            ttk.Radiobutton(row, text=value, variable=self.theme, value=value, command=self.apply_appearance).pack(side="left", padx=12)
        row = ttk.Frame(page)
        row.pack(fill="x", pady=12)
        ttk.Label(row, text="字体大小", width=16).pack(side="left")
        box = ttk.Combobox(row, textvariable=self.font_size, values=tuple(range(9, 19)), state="readonly", width=8)
        box.pack(side="left", padx=12)
        box.bind("<<ComboboxSelected>>", lambda _: self.apply_appearance())
        row = ttk.Frame(page)
        row.pack(fill="x", pady=12)
        ttk.Label(row, text="自动连接飞控", width=16).pack(side="left")
        for label, value in (("是", True), ("否", False)):
            ttk.Radiobutton(row, text=label, variable=self.auto_connect, value=value,
                            command=self._change_auto_connect).pack(side="left", padx=12)
        ttk.Label(page, text="开启后自动识别 USB / X6 上的 CoFly 固件；两者可用时优先 USB。连接后保持当前链路，中断后重新选择。手动断开暂停自动连接，可重新选择“是”恢复。",
                  style="Muted.TLabel", wraplength=850).pack(anchor="w", pady=8)


    def _change_auto_connect(self):
        self.auto_paused = False
        self.auto_next_scan = 0
        if not self.auto_connect.get() and self.auto_detector:
            self.auto_detector.stop()
        self._save_settings()
        self.status.set("自动连接已开启，正在检测 USB / X6" if self.auto_connect.get() else "自动连接已关闭")

    def _auto_connect_tick(self, now):
        enabled = self.auto_connect.get() and not self.auto_paused
        if self.auto_detector and not self.auto_detector.is_alive():
            link = self.auto_detector.result
            stopped = self.auto_detector.stopping.is_set()
            self.auto_detector = None
            self.auto_next_scan = now + 2
            if enabled and not stopped and link and not self.worker:
                self.link_type.set(link.link_type)
                self.port.set(link.port)
                self.baud.set(str(link.baudrate))
                self._save_settings()
                super().toggle_connection()
                return
        if not enabled or self.worker:
            if self.auto_detector:
                self.auto_detector.stop()
            return
        if not self.auto_detector and now >= self.auto_next_scan:
            try:
                usb_baud = int(self.link_bauds["USB"])
                radio_baud = int(self.link_bauds["3DR X6"])
                if min(usb_baud, radio_baud) <= 0:
                    return
            except (ValueError, TypeError):
                return
            self.auto_detector = FlightControllerDetector(self.port.get(), usb_baud, radio_baud)
            self.auto_detector.start()
            self._update_controls()

    def toggle_connection(self):
        manual_disconnect = bool(self.worker)
        if self.auto_detector:
            self.auto_detector.stop()
        if manual_disconnect:
            self.auto_paused = True
        else:
            self._save_settings()
        super().toggle_connection()
        if manual_disconnect and self.auto_connect.get():
            self.status.set("已手动断开，自动连接暂停；在应用设置重新选择“是”恢复")

    def show_page(self, name):
        self._cancel_edit()
        self.page = name
        self.pages[name].tkraise()
        for key, button in self.nav.items():
            button.configure(style="Selected.TButton" if key == name else "TButton")

    def apply_appearance(self, save=True):
        dark = self.theme.get() == "室内"
        bg, panel, fg, muted, accent = ("#151c26", "#202b39", "#edf3fa", "#a9b8ca", "#75c8f4") if dark else ("#edf1f5", "#ffffff", "#142539", "#42566c", "#075e91")
        self.colors = (bg, panel, fg, muted, accent)
        size = self.font_size.get()
        self.root.minsize(max(1000, size * 90), max(850, size * 60))
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            font.nametofont(name).configure(family="Microsoft YaHei UI", size=size)
        style = ttk.Style(self.root)
        style.configure(".", background=bg, foreground=fg, font=("Microsoft YaHei UI", size))
        style.configure("TFrame", background=bg)
        style.configure("TLabel", background=bg, foreground=fg)
        style.configure("TLabelframe", background=bg, bordercolor=muted)
        style.configure("TLabelframe.Label", background=bg, foreground=accent)
        for kind in ("TButton", "TCombobox", "TEntry", "Treeview", "Treeview.Heading"):
            style.configure(kind, background=panel, fieldbackground=panel, foreground=fg, insertcolor=fg)
            style.map(kind, background=[("selected", "#245d85"), ("active", "#245d85")], foreground=[("disabled", muted), ("selected", "white"), ("active", "white")])
        style.map("TCombobox", fieldbackground=[("readonly", panel), ("disabled", bg)],
                  foreground=[("disabled", muted), ("readonly", fg)],
                  selectbackground=[("readonly", panel)], selectforeground=[("readonly", fg)])
        style.configure("Horizontal.TProgressbar", background=accent, troughcolor=panel, bordercolor=muted)
        style.configure("Treeview", rowheight=size * 2 + 8)
        style.configure("Selected.TButton", background="#245d85", foreground="white")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", size + 10, "bold"))
        style.configure("Brand.TLabel", foreground=accent, font=("Microsoft YaHei UI", size + 9, "bold"))
        style.configure("Muted.TLabel", foreground=muted)
        style.configure("Accent.TLabel", foreground=accent)
        style.configure("Value.TLabel", font=("Microsoft YaHei UI", size + 3, "bold"))
        self.terminal.configure(background=panel, foreground=fg, insertbackground=fg, font=("Consolas", size))
        self.root.configure(background=bg)
        self.flight_scroll.configure(background=bg)
        self.attitude_columns = 0
        self._layout_attitude_cards(self.flight_scroll.winfo_width())
        self._draw_horizon()
        self._draw_configuration()
        if save:
            self._save_settings()

    def _save_settings(self):
        self.link_bauds[self.link_type.get()] = self.baud.get()
        try:
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.settings_path.with_suffix(".tmp")
            temporary.write_text(json.dumps({"theme": self.theme.get(), "font_size": self.font_size.get(), "port": self.port.get(), "baud": self.baud.get(), "auto_connect": self.auto_connect.get(), "link_type": self.link_type.get(), "usb_baud": self.link_bauds["USB"], "radio_baud": self.link_bauds["3DR X6"]}, ensure_ascii=False), encoding="utf-8")
            temporary.replace(self.settings_path)
        except OSError as exc:
            self.status.set(f"设置保存失败：{exc}")

    def _update_controls(self):
        super()._update_controls()
        if hasattr(self, "parameter_read"):
            polling = self.gcs_pending is not None and self.gcs_pending["verb"] in ("DATA", "CONFIG")
            idle = self.connected and self.pending is None and (self.gcs_pending is None or polling) and self.file_import is None and self.radio_calibration is None and self.deferred_action is None and self.radio_save is None and self.sd_browser is None
            self.radio_calibrate_button.configure(state="normal" if idle and self.caps.get("radio_calibration") == "SUPPORTED" and self.caps.get("radio") != "DISABLED" else "disabled")
            self.calibration_radio_button.configure(state=self.radio_calibrate_button.cget("state"))
            self.read_button.configure(state="normal" if idle else "disabled")
            self.parameter_read.configure(state="normal" if idle else "disabled")
            files_ready = idle and bool(self.parameters) and self.parameter_file_version in SUPPORTED_FILE_VERSIONS
            self.parameter_load_file.configure(state="normal" if files_ready else "disabled")
            self.parameter_save_file.configure(state="normal" if files_ready else "disabled")
            self.probe_button.configure(state="normal" if idle else "disabled")
            self.reboot_button.configure(state="normal" if idle and self.caps.get("reboot") == "SUPPORTED" else "disabled")
            sd_ready = (idle and self.link_type.get() == "USB" and self.caps.get("sd_files") == "USB_V1"
                        and self.data.get("locked") == 1 and time.monotonic() - self.last_data <= 2)
            self.sd_download_button.configure(state="normal" if sd_ready else "disabled")
            for button in self.action_buttons:
                button.configure(state="normal" if idle and self.caps else "disabled")
        if self.sd_browser:
            self.send_button.configure(state="disabled")
            self.command_box.configure(state="disabled")
            self._cancel_edit()
        self.link_type_box.configure(state="disabled" if self.worker else "readonly")
        self.link_label.set(f"● {self.link_type.get()} 已连接" if self.connected else
                            "● 自动检测飞控" if self.auto_connect.get() and not self.auto_paused else "● 未连接")

    def _start_request(self, *args):
        if self._defer_for_poll(self._start_request, *args):
            return
        if self.gcs_pending or self.radio_calibration or self.radio_save or self.sd_browser:
            self.status.set("正在接收串口数据，请稍后再试")
            return
        super()._start_request(*args)

    def _defer_for_poll(self, callback, *args):
        if self.deferred_action is not None:
            return True
        if self.gcs_pending and self.gcs_pending["verb"] in ("DATA", "CONFIG"):
            self.deferred_action = (callback, args)
            self._update_controls()
            return True
        return False

    def _run_deferred_action(self):
        if self.deferred_action is None or self.gcs_pending is not None:
            return False
        callback, args = self.deferred_action
        self.deferred_action = None
        if self.connected:
            callback(*args)
        self._update_controls()
        return True

    def _calibrate_radio(self):
        if self._defer_for_poll(self._calibrate_radio):
            return
        if self.sd_browser or self.radio_calibration or not self.connected or self.pending or self.gcs_pending or self.file_import:
            return
        if time.monotonic() - self.last_data > 0.5 or self.data.get("locked") != 1 or self.data.get("radio_valid") != 1:
            self.status.set("请先锁定飞机并打开遥控器，等待有效实时数据后再校准")
            return
        if messagebox.askokcancel("遥控器校准", "在校准之前，你应该把所有的微调和辅助微调量设为零。\n单击“确定开始校准”。", parent=self.root):
            # Modal dialogs run Tk's event loop; recheck connection/state afterwards.
            if not self.connected or self.data.get("locked") != 1 or self.data.get("radio_valid") != 1:
                self.status.set("连接或遥控状态已变化，请重新开始校准")
                return
            self.radio_calibration = RadioCalibrationDialog(self)
            self._update_controls()

    def _terminal_payload(self, kind, payload):
        return self.terminal_filter.feed(kind, payload)

    def _open_sd_files(self):
        if self._defer_for_poll(self._open_sd_files):
            return
        if (self.sd_browser or not self.connected or self.pending or self.gcs_pending or self.file_import
                or self.radio_calibration or self.radio_save):
            return
        if self.link_type.get() != "USB" or self.caps.get("sd_files") != "USB_V1":
            self.status.set("请通过 USB 连接，并更新支持 SD 文件下载的飞控固件")
            return
        if self.data.get("locked") != 1 or time.monotonic() - self.last_data > 2:
            self.status.set("请先锁定飞机，等待锁定状态更新后再打开 SD 文件窗口")
            return
        self._cancel_edit()
        self.sd_browser = SdFileDialog(self)
        self._update_controls()
        self.sd_browser.load("/")

    def send_command(self):
        if not self.sd_browser:
            super().send_command()

    def _request(self, verb, *args, automatic=False):
        if not automatic and verb not in ("DATA", "CONFIG") and self._defer_for_poll(self._request, verb, *args):
            return
        if self.sd_browser or not self.connected or self.pending or self.gcs_pending or self.file_import is not None:
            return
        self.request_id = (self.request_id + 1) & 0xffffffff
        self.gcs_pending = {"id": self.request_id, "verb": verb, "automatic": automatic,
                            "deadline": time.monotonic() + (10 if verb == "RC_SAVE" else 3)}
        if automatic:
            self.terminal_filter.register(self.request_id, verb)
        self.worker.send((f"GCS {verb} {self.request_id}" + (" " + " ".join(args) if args else "") + "\n").encode("ascii"))
        self._update_controls()

    def save_radio_calibration(self, values):
        self.radio_save = {"values": values, "deadline": time.monotonic() + 5,
                           "valid_since": None, "last_data": None, "last_ms": None, "frames": 0}
        self.status.set("校准数据已保留，等待遥控信号连续有效后写入 SD 卡…")
        self._update_controls()

    def _try_radio_save(self, data, now):
        job = self.radio_save
        if job is None:
            return
        if data.get("locked") != 1:
            self.radio_save = None
            self.status.set("飞机已解锁，取消校准保存；原参数保持不变")
            return
        if data.get("radio_valid") != 1 or (job["last_data"] is not None and now - job["last_data"] > 0.5):
            job["valid_since"], job["frames"] = None, 0
        job["last_data"] = now
        if data.get("radio_valid") != 1:
            return
        if data["ms"] == job["last_ms"]:
            job["valid_since"], job["frames"] = None, 0
            return
        job["last_ms"] = data["ms"]
        if job["valid_since"] is None:
            job["valid_since"] = now
        job["frames"] += 1
        if now < job["deadline"] and now - job["valid_since"] >= 1 and job["frames"] >= 6:
            self.status.set("遥控信号有效，正在写入校准参数…")
            self._request("RC_SAVE", *job["values"])

    def _sensor_action(self, verb):
        selected = self.sensor_tree.selection()
        active = {"IMU": self.caps.get("imu") == "BMI088_SPI", "EXTIMU": self.caps.get("external") == "ENABLED",
                  "AIRSPEED": self.caps.get("airspeed") == "MS4525", "ROTATE": self.caps.get("rotate") == "MT6701"}
        if not selected or not active.get(selected[0], False):
            self.status.set("请选择当前启用且正在采样的传感器")
            return
        args = (selected[0],) if verb == "CALIBRATE" else (selected[0], self.sensor_setting.get())
        self._request(verb, *args)

    def _handle_reply(self, reply):
        if self.sd_browser and self.sd_browser.transport.handle(reply):
            return
        request_id, kind, fields = reply
        pending = self.gcs_pending
        if not pending or request_id != pending["id"]:
            return super()._handle_reply(reply)
        self.gcs_pending = None
        try:
            if kind == "ERROR":
                self.link_timeouts = 0
                self.next_link_check = time.monotonic() + 3
                code = fields[0] if fields else "UNKNOWN"
                if pending["verb"] == "RC_SAVE":
                    if code == "RADIO_INVALID" and self.radio_save and time.monotonic() < self.radio_save["deadline"]:
                        self.radio_save.update(valid_since=None, last_data=None, last_ms=None, frames=0)
                        self.status.set("保存时遥控信号暂时无效，保留本次校准数据并等待恢复…")
                        self._update_controls()
                        return
                    self.radio_save = None
                    messagebox.showerror("遥控器校准保存失败", f"未确认保存成功（{code}）。请检查飞机锁定状态、接收机和 SD 卡，然后重新校准。", parent=self.root)
                self.status.set(f"遥控校准保存失败：{code}；原遥控参数保持不变" if pending["verb"] == "RC_SAVE" else
                                "该接口暂未实现，待下一版本开放；飞控状态未修改。" if code == "NOT_IMPLEMENTED" else
                                "请先锁定飞机，再重启飞控" if code == "LOCK_REQUIRED" else
                                "待保存参数写入失败，已取消重启；请检查存储介质" if code == "SAVE_FAILED" else f"固件拒绝地面站请求：{code}")
                if pending["verb"] == "CAPS":
                    self.device_label.set("固件不支持地面站协议；参数页仍可使用。请更新固件后重试识别。")
                elif pending["verb"] == "CONFIG":
                    self.config_supported = False
                    self.configuration_label.set("固件不支持构型接口，请更新固件")
            else:
                if kind != pending["verb"]:
                    raise ValueError("回复类型与请求不匹配")
                values = decode_fields(kind, fields)
                self.link_timeouts = 0
                self.next_link_check = time.monotonic() + 3
                if kind == "RC_SAVE":
                    self.radio_save = None
                    self.parameters.clear()
                    self.parameter_file_version = None
                    self._render_table()
                    self.status.set("遥控器校准参数已写入 SD 卡并生效；请重新读取参数")
                    messagebox.showinfo("遥控器校准", "新参数已写入 SD 卡并生效。", parent=self.root)
                if kind == "REBOOT":
                    self.disconnect()
                    self.status.set("飞控已接受重启指令；设备重启后请重新连接")
                if kind == "CAPS" and not (pending.get("automatic") and self.caps):
                    self.caps = values
                    self._render_sensors()
                    self.device_label.set(f"CoFly 串口协议 v1 · {values['imu']} · {values['radio']}")
                    self.radio_label.set(f"接收机：{values['radio']} · 原始 PWM 输入与归一化指令")
                    self.status.set("固件已识别，数据面板已启用")
                elif kind == "DATA":
                    self.data = values
                    self.last_data = time.monotonic()
                    self._render_data()
                    if self.radio_calibration:
                        self.radio_calibration.feed(values, self.last_data)
                    self._try_radio_save(values, self.last_data)
                elif kind == "CONFIG":
                    self.configuration = values
                    self.last_configuration = time.monotonic()
                    if "aircraft_id" in values:
                        identity = chr(64 + values["aircraft_id"]) if values["config_valid"] else "未配置"
                        self.device_label.set(f"当前生效身份：{identity} · {values['count']} 机 · 串口协议 v1")
                    identity = values.get("aircraft_id", 1 if values["master"] else 0)
                    self.local_aircraft = chr(64 + identity) if identity else "本机"
                    self._sync_attitude_cards(ORDERS[values["count"]] if identity else ("本机", *ORDERS[values["count"]]))
                    self._draw_horizon()
                    self._draw_configuration()
        except (ValueError, IndexError, KeyError) as exc:
            if pending["verb"] == "RC_SAVE":
                self.radio_save = None
            self.status.set(f"地面站响应错误：{exc}")
        self._update_controls()
        if self._run_deferred_action():
            return
        if kind == "DATA" and self.page == "飞行" and self.config_supported and self.radio_save is None:
            self._request("CONFIG", automatic=True)

    def _render_sensors(self):
        self.sensor_tree.delete(*self.sensor_tree.get_children())
        for key, name, state in (("IMU", "内置 IMU / 陀螺仪 / 加速度计", self.caps["imu"]),
                                  ("EXTIMU", "外置 IMU", self.caps["external"]),
                                  ("AIRSPEED", "空速", self.caps["airspeed"]),
                                  ("ROTATE", "构型转角", self.caps["rotate"]),
                                  ("BAROMETER", "气压计", self.caps["barometer"])):
            self.sensor_tree.insert("", "end", iid=key, values=(name, state))
        self.sensor_note.set("识别的是编译配置与采样路径，未进行硬件探测。校准、设置按钮只请求预留接口，不调用现有阻塞式校准。")

    def _render_data(self):
        for key, var in self.metric_vars.items():
            value = self.data.get(key)
            if key == "mode":
                text = MODES.get(value, "—")
            elif key == "locked":
                text = "已锁定" if value == 1 else "未锁定" if value == 0 else "—"
            elif key == "angle" and self.data.get("angle_valid") != 1:
                text = "无效"
            elif key.startswith("rc"):
                text = f"{value:.0f} μs" if value is not None else "— μs"
            elif key in ("gx", "gy", "gz", "ax", "ay", "az") and self.caps.get("imu") != "BMI088_SPI":
                text = "—"
            else:
                text = f"{value:.2f}" if value is not None else "—"
            var.set(text)
        for index, var in enumerate(self.rc_normalized, 1):
            value = self.data.get(f"n{index}")
            var.set(f"{value:+.3f}" if value is not None else "—")
        for index, bar in enumerate(self.rc_bars, 1):
            bar["value"] = max(0, min(1000, self.data.get(f"rc{index}", 1000) - 1000))
        self._draw_horizon()

    def _draw_horizon(self, name=None):
        attitudes = aircraft_attitudes(self.configuration, self.data, self.local_aircraft)
        names = (name,) if name else tuple(self.attitude_cards)
        for aircraft in names:
            card = self.attitude_cards.get(aircraft)
            if not card:
                continue
            values = attitudes.get(aircraft, {})
            def value(key):
                v = values.get(key)
                return f"{v:+.1f}°" if v is not None else "—"
            card["text"].set(f"滚转 {value('roll')}   俯仰 {value('pitch')}\n偏航 {value('yaw')}")
            valid = values.get("roll") is not None and values.get("pitch") is not None
            card["state"].set(("本机 IMU" if aircraft == self.local_aircraft else "机间姿态 · 偏航未传输") if valid else "未收到有效姿态 / 已过期")
            canvas = card["canvas"]
            canvas.delete("all")
            w, h = max(canvas.winfo_width(), 100), max(canvas.winfo_height(), 100)
            canvas.configure(background="#263442")
            if not valid:
                canvas.create_text(w/2, h/2, text="等待姿态数据", fill="#d1deea")
                continue
            cx, cy = w/2, h/2
            angle = math.radians(values["roll"])
            pitch = max(-60, min(60, values["pitch"])) * h / 120
            extent = max(w, h) * 3
            def point(x, y):
                return (cx + x * math.cos(angle) - (y + pitch) * math.sin(angle),
                        cy + x * math.sin(angle) + (y + pitch) * math.cos(angle))
            for bounds, color in (((-extent,-extent,extent,0),"#276b96"), ((-extent,0,extent,extent),"#866547")):
                x0,y0,x1,y1=bounds
                canvas.create_polygon(*sum((point(x,y) for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))),()), fill=color, outline="")
            canvas.create_line(*point(-extent,0), *point(extent,0), fill="white", width=2)
            for tick in (-20,-10,10,20):
                y=-tick*h/120
                canvas.create_line(*point(-18,y), *point(18,y), fill="white")
                canvas.create_text(*point(32,y), text=str(tick), fill="white", font=("Microsoft YaHei UI",8))
            canvas.create_line(cx-45,cy,cx-14,cy,cx,cy+6,cx+14,cy,cx+45,cy,fill="#ffda64",width=3)

    def _draw_configuration(self):
        if not hasattr(self, "wings"):
            return
        canvas = self.wings
        canvas.delete("all")
        bg, panel, fg, muted, accent = self.colors
        canvas.configure(background=panel)
        w, h = max(200, canvas.winfo_width()), max(120, canvas.winfo_height())
        if not configuration_ready(self.configuration):
            if not self.config_supported:
                message = "请更新支持构型接口的固件"
            elif self.configuration and self.configuration.get("config_valid") == 0:
                message = "请在参数页设置飞机编号和数量，然后重启"
            elif self.configuration and not self.configuration["master"]:
                message = "完整构型需连接 A 主机"
            else:
                message = "等待有效的相邻飞机姿态"
            canvas.create_text(w / 2, h / 2, text=message, fill=muted, font=("Microsoft YaHei UI", self.font_size.get()))
            self.configuration_label.set(message)
            return
        surfaces = wing_surfaces(self.configuration)
        if not surfaces:
            has_pitch = "p_a" in self.configuration
            message = "等待各机有效姿态，完整构型暂不可用" if has_pitch else "当前固件只有滚转构型，请更新固件以显示俯仰"
            canvas.create_text(w/2,h/2,text=message,fill=muted)
            self.configuration_label.set(message)
            return
        points = [project_wing(p) for surface in surfaces for p in (*surface["corners"], surface["nose"])]
        xmin,xmax=min(p[0] for p in points),max(p[0] for p in points)
        ymin,ymax=min(p[1] for p in points),max(p[1] for p in points)
        scale=min((w-100)/max(.6,xmax-xmin),(h-75)/max(.8,ymax-ymin))
        def screen(point):
            x,y=project_wing(point)
            return w/2+(x-(xmin+xmax)/2)*scale,h/2+(y-(ymin+ymax)/2)*scale
        canvas.create_line(25,h/2,w-25,h/2,fill=muted,dash=(3,6))
        palette={"A":"#ffda64","B":"#75c8f4","C":"#92d9b3","D":"#baa4f2","E":"#f3ab82","F":"#8bc5e6","G":"#e4a5d2"}
        for surface in sorted(surfaces,key=lambda item:item["center"][1],reverse=True):
            name=surface["name"]
            color=palette[name]
            corners=[screen(p) for p in surface["corners"]]
            canvas.create_polygon(*sum(corners,()),fill=color,outline=fg,width=1,tags=("wing",name))
            canvas.create_line(*screen(surface["left"]),*screen(surface["right"]),fill=fg,width=2)
            center = surface["center"]
            level_nose = (center[0], center[1] + .45, center[2])
            canvas.create_line(*screen(center), *screen(level_nose), fill=muted, dash=(3, 3), tags=("pitch_zero", name))
            canvas.create_line(*screen(center),*screen(surface["nose"]),fill=fg,width=2,arrow="last",tags=("nose",name))
            x,y=screen(surface["center"])
            canvas.create_text(x,y+28,text=f"{name}  P{surface['pitch']:+.1f}°",fill=fg,
                               font=("Microsoft YaHei UI",max(8,self.font_size.get()-1)),tags=("wing_label",name))
        self.configuration_label.set(f"{self.configuration['count']} 机 · " + "—".join(ORDERS[self.configuration["count"]]) +
                                     " · IMU 滚转 / 俯仰 · 箭头指向机头")

    def _station_tick(self):
        now = time.monotonic()
        if self.sd_browser:
            self.sd_browser.tick()
        self._auto_connect_tick(now)
        if self.gcs_pending and now > self.gcs_pending["deadline"]:
            verb = self.gcs_pending["verb"]
            self.gcs_pending = None
            if verb == "RC_SAVE":
                self.radio_save = None  # Do not retry an unacknowledged SD write.
            self.status.set("地面站请求超时；请检查固件或重试识别" if verb == "CAPS" else "串口请求超时，结果未确认")
            self._update_controls()
            if verb in ("CAPS", "DATA", "CONFIG"):
                self.link_timeouts += 1
                if self.auto_connect.get() and not self.auto_paused and self.link_timeouts >= 3:
                    self.disconnect()
                    self.status.set("飞控连续三次无响应，正在重新检测 USB / X6；未确认操作不会重发")
        self._run_deferred_action()
        if self.radio_save and now >= self.radio_save["deadline"] and not (self.gcs_pending and self.gcs_pending["verb"] == "RC_SAVE"):
            self.radio_save = None
            self.status.set("五秒内遥控信号未恢复稳定，未保存校准参数；请检查接收机连接")
            self._update_controls()
        if self.radio_calibration and not self.radio_calibration.closed and now - self.last_data > 2:
            self.radio_calibration.cancel("数传数据中断，校准已取消，原参数保持不变")
        if self.connected and not self.pending and not self.gcs_pending and self.file_import is None and self.sd_browser is None:
            if self.auto_probe:
                self.auto_probe = False
                self._request("CAPS", automatic=True)
            elif self.caps and (self.page in ("飞行", "遥控器", "校准")
                               or (self.page == "串口助手" and self.link_type.get() == "USB" and self.caps.get("sd_files") == "USB_V1")
                               or self.radio_calibration or self.radio_save) and not (self.radio_calibration and self.radio_calibration.closed):
                self._request("DATA", automatic=True)
            elif self.auto_connect.get() and not self.auto_paused and now >= self.next_link_check:
                self._request("CAPS", automatic=True)
        age = now - self.last_data if self.last_data else float("inf")
        fresh = self.connected and age <= 2
        self.freshness.set(f"● 实时 · 更新于 {age:.1f} 秒前" if fresh else "● 数据已过期 / 等待飞控数据")
        if not fresh and self.data:
            self.data.clear()
            self._render_data()
            self._update_controls()
        if self.configuration and now - self.last_configuration > 2:
            self.configuration.clear()
            self._draw_horizon()
            self._draw_configuration()
        self.station_timer = self.root.after(200, self._station_tick)

    def disconnect(self):
        if self.sd_browser:
            self.sd_browser.destroy()
        self.terminal_filter = AutomaticQueryFilter()
        self.link_timeouts = 0
        self.next_link_check = 0
        self.radio_save = None
        self.deferred_action = None
        if self.radio_calibration:
            self.radio_calibration.cancel("串口已断开，校准已取消")
        self.auto_next_scan = time.monotonic() + 2
        self.gcs_pending = None
        self.caps.clear()
        self.data.clear()
        self.configuration.clear()
        self.last_configuration = 0
        self.config_supported = True
        self.last_data = 0
        self.auto_probe = True
        super().disconnect()
        self._render_data()
        self._draw_configuration()
        self.sensor_tree.delete(*self.sensor_tree.get_children())
        self.device_label.set("固件能力尚未识别")
        self.radio_label.set("接收机：未识别")
        self.sensor_note.set("连接并识别固件后显示传感器。")

    def close(self):
        if self.sd_browser:
            if self.connected and self.worker:
                self.request_id = self.request_id % 0xffffffff + 1
                self.worker.send(f"SD CANCEL {self.request_id}\n".encode("ascii"))
            self.sd_browser.destroy()
        if self.auto_detector:
            self.auto_detector.stop()
            self.auto_detector.join(timeout=0.5)
        self.root.after_cancel(self.station_timer)
        self._save_settings()
        super().close()


def main():
    root = tk.Tk()
    station = GroundStation(root)
    if "--self-test" in sys.argv:
        root.withdraw()
        root.after(300, station.close)
    root.mainloop()


if __name__ == "__main__":
    main()
