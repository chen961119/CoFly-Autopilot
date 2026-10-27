"""CoFly Autopilot flight-data browser and command-line reader."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import os
from pathlib import Path
import queue
import sys

import numpy as np

from data import LogError, compute_spectrum, display_series, load_log


LOG_SUFFIXES = (".csv", ".txt", ".log", ".dat")
SETTINGS_PATH = Path(os.environ.get("LOCALAPPDATA", Path.home() / ".config")) / "CoFly" / "flight_data_browser.json"


def last_directory():
    try:
        settings = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
        saved = settings.get("last_directory") if isinstance(settings, dict) else None
        if isinstance(saved, str) and saved:
            folder = Path(saved)
            if folder.is_dir():
                return folder
    except (OSError, ValueError):
        pass
    return Path.cwd()


def remember_directory(folder):
    SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    SETTINGS_PATH.write_text(json.dumps({"last_directory": str(folder)}, ensure_ascii=False), encoding="utf-8")


def export_csv(log, destination, columns, bounds):
    indices = [log.columns.index(name) for name in columns]
    left, right = np.searchsorted(log.time, bounds[0]), np.searchsorted(log.time, bounds[1], side="right")
    with Path(destination).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["Time_elapsed(s)", *columns])
        for start in range(left, right, 8192):
            stop = min(start + 8192, right)
            for elapsed, values in zip(log.time[start:stop], log.data[start:stop, indices]):
                writer.writerow([format(elapsed, ".17g"), *[format(value, ".17g") for value in values]])


class Browser:
    def __init__(self, root, files=(), options=None):
        import tkinter as tk
        from tkinter import ttk
        from tkinterdnd2 import DND_FILES
        from matplotlib import font_manager, rcParams
        from matplotlib.figure import Figure
        from chart_canvas import ChartCanvas
        from chart_toolbar import ChartToolbar
        from ui_style import COLORS, apply_theme

        self.ui_font = apply_theme(root)

        available_fonts = {font.name for font in font_manager.fontManager.ttflist}
        preferred_fonts = ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei", "Arial Unicode MS")
        rcParams["font.sans-serif"] = [name for name in preferred_fonts if name in available_fonts] + ["DejaVu Sans"]

        self.root = root
        self.options = options or {}
        self.last_directory = last_directory()
        self.logs = {}
        self.file_order = []
        self.file_items = {}
        self.requests = {}
        self.futures = {}
        self.request_id = 0
        self.signatures = {}
        self.entries = {}
        self.transforms = {}
        self.transform_exclusions = {}
        self.transform_check_vars = {}
        self.groups = [set()]
        self.group_ids = [0]
        self.next_group_id = 1
        self.layout_rows = self.layout_columns = 1
        self.spare_groups = []
        self.group_views = {}
        self.axes = []
        self.lines = []
        self.plotted_groups = []
        self.initial_views = {}
        self.series_styles = {}
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="flight-log")
        self.events = queue.Queue()
        self.syncing = False
        self.replace_selection_group = None
        self.selection_job = None
        self.closed = False
        self.fft_job = None
        self.view_job = None
        self.dirty_view_axes = set()
        self.navigation_start = None
        self.drag_axes = None
        root.title("CoFly Autopilot · Flight Data Browser")
        root.geometry("1450x900")
        root.minsize(1040, 700)
        root.protocol("WM_DELETE_WINDOW", self.close)

        header = ttk.Frame(root, padding=(18, 14, 18, 12))
        header.pack(fill="x")
        ttk.Label(header, text="CoFly · 飞行数据浏览器", style="Title.TLabel").pack(side="left")
        self.overview = tk.StringVar(value="未加载文件")
        ttk.Label(header, textvariable=self.overview, style="Count.TLabel").pack(side="right")
        pane = tk.PanedWindow(root, orient="horizontal", background=COLORS["background"], borderwidth=0,
                              sashwidth=8, sashrelief="flat", opaqueresize=False, sashcursor="sb_h_double_arrow",
                              proxybackground=COLORS["accent"], proxyrelief="flat", proxyborderwidth=0)
        self.pane = pane
        pane.pack(fill="both", expand=True, padx=14, pady=(0, 10))
        left = ttk.Frame(pane, padding=(0, 0, 8, 0))
        right = ttk.Frame(pane, style="Card.TFrame", padding=10)
        pane.add(left, stretch="never")
        pane.add(right, stretch="always", minsize=360)
        self.sidebar = left
        files_card = ttk.Frame(left, style="Card.TFrame", padding=12)
        files_card.pack(fill="both", expand=True)
        ttk.Label(files_card, text="数据与变量", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        file_actions = ttk.Frame(files_card, style="Surface.TFrame")
        file_actions.pack(fill="x", pady=(0, 8))
        for label, callback in (("选择文件…", self.select_files), ("上移", lambda: self.move_files(-1)),
                                ("下移", lambda: self.move_files(1)), ("删除所选", self.remove_files)):
            button_style = "Primary.TButton" if callback == self.select_files else (
                "Danger.TButton" if callback == self.remove_files else "TButton")
            ttk.Button(file_actions, text=label, command=callback, style=button_style).pack(side="left", padx=(0, 4))
        self.search = tk.StringVar()
        self.search_entry = ttk.Entry(files_card, textvariable=self.search, style="Search.TEntry")
        self.search_entry.pack(fill="x", pady=(0, 10))
        self.search_clear = tk.Button(
            self.search_entry, text="⊗", command=self.clear_search, takefocus=False,
            font=(self.ui_font, 14), foreground=COLORS["muted"], background=COLORS["surface"],
            activeforeground=COLORS["accent"], activebackground=COLORS["surface"],
            borderwidth=0, highlightthickness=0, relief="flat", cursor="hand2", padx=0, pady=0)
        self.search_clear.place(relx=1, x=-4, rely=0.5, anchor="e", width=24, height=24)
        self.search_hint = tk.Label(
            self.search_entry, text="搜索变量名，例如 att、ang_vel、yaw",
            foreground=COLORS["muted"], background=COLORS["surface"], borderwidth=0, font=(self.ui_font, 9))
        self.search_hint.bind("<Button-1>", lambda _event: self.search_entry.focus_set())
        self.search_entry.bind("<FocusIn>", self.update_search_hint)
        self.search_entry.bind("<FocusOut>", self.update_search_hint)
        self.search.trace_add("write", self.search_changed)
        self.update_search_hint()
        tree_frame = ttk.Frame(files_card, style="Surface.TFrame")
        tree_frame.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(tree_frame, show="tree", selectmode="extended", height=6)
        self.tree.tag_configure("file", background="#f1f5fb", foreground=COLORS["text"],
                                font=(self.ui_font, 9, "bold"))
        self.tree.tag_configure("alternate", background="#fbfcfe")
        scroll = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.tree.bind("<Button-1>", self.tree_pressed)
        self.tree.bind("<Double-1>", self.tree_pressed)
        self.tree.bind("<<TreeviewSelect>>", self.selection_changed)

        controls = ttk.Frame(left, style="Card.TFrame", padding=12)
        controls.pack(fill="x", pady=(10, 0))
        ttk.Label(controls, text="子图与显示", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        layout_row = ttk.Frame(controls, style="Surface.TFrame")
        layout_row.pack(fill="x", pady=(0, 10))
        self.row_count, self.column_count = tk.StringVar(value="1"), tk.StringVar(value="1")
        for label, variable in (("行", self.row_count), ("列", self.column_count)):
            ttk.Label(layout_row, text=label).pack(side="left", padx=(0, 5))
            editor = ttk.Spinbox(layout_row, from_=1, to=8, textvariable=variable, width=4,
                                 command=self.apply_layout)
            editor.pack(side="left", padx=(0, 10))
            editor.bind("<Return>", self.apply_layout)
            editor.bind("<KP_Enter>", self.apply_layout)
        group_row = ttk.Frame(controls, style="Surface.TFrame")
        group_row.pack(fill="x", pady=(0, 10))
        self.group_box = ttk.Combobox(group_row, state="readonly", width=8)
        self.group_box.pack(side="left", padx=(0, 5))
        self.group_box.bind("<<ComboboxSelected>>", self.change_group)
        ttk.Button(group_row, text="清除子图", command=self.clear_group).pack(side="left", padx=(0, 4))
        self.delete_group_button = ttk.Button(group_row, text="删除子图", command=self.delete_group,
                                              style="Danger.TButton")
        self.delete_group_button.pack(side="left")
        transform_selection = ttk.Frame(controls, style="Surface.TFrame")
        transform_selection.pack(fill="x", pady=(0, 8))
        self.transform_summary = tk.StringVar(value="变量：0/0")
        self.transform_selector = ttk.Menubutton(transform_selection, textvariable=self.transform_summary, width=12)
        self.transform_selector.pack(side="left")
        self.transform_menu = tk.Menu(self.transform_selector, tearoff=False)
        self.transform_selector.configure(menu=self.transform_menu)
        transform = ttk.Frame(controls, style="Surface.TFrame")
        transform.pack(fill="x")
        self.scale, self.offset = tk.StringVar(value="1"), tk.StringVar(value="0")
        self.transform_inputs = {}
        for name, label, variable in (("scale", "缩放", self.scale), ("offset", "偏移", self.offset)):
            ttk.Label(transform, text=label).pack(side="left", padx=(0, 5))
            editor = ttk.Spinbox(transform, textvariable=variable, width=7)
            editor.pack(side="left", padx=(0, 10))
            editor.bind("<<Increment>>", lambda _event, field=name: self.step_transform(field, 1))
            editor.bind("<<Decrement>>", lambda _event, field=name: self.step_transform(field, -1))
            editor.bind("<Return>", self.apply_transform)
            editor.bind("<KP_Enter>", self.apply_transform)
            self.transform_inputs[name] = editor
        ttk.Button(transform, text="恢复", command=self.reset_transform).pack(side="left")
        self.hide_legend = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text="隐藏图例", variable=self.hide_legend,
                        command=lambda: self.toggle_display("legend")).pack(anchor="w", pady=(10, 0))
        self.smooth = tk.BooleanVar(value=False)
        self.fft_enabled = tk.BooleanVar(value=False)
        toggles = ttk.Frame(controls, style="Surface.TFrame")
        toggles.pack(fill="x", pady=(10, 0))
        ttk.Checkbutton(toggles, text="显示平滑", variable=self.smooth,
                        command=lambda: self.toggle_display("smooth")).pack(side="left")
        ttk.Checkbutton(toggles, text="区段 FFT", variable=self.fft_enabled,
                        command=lambda: self.toggle_display("fft")).pack(side="left", padx=(14, 0))
        self.status = tk.StringVar(value="选择或拖入一个或多个 CSV/TXT/LOG/DAT 文件。")
        ttk.Label(root, textvariable=self.status, anchor="w", style="Status.TLabel").pack(side="bottom", fill="x", before=pane)

        chart_header = ttk.Frame(right, style="Surface.TFrame")
        chart_header.pack(fill="x", padx=6, pady=(2, 6))
        ttk.Label(chart_header, text="数据视图", style="Section.TLabel").pack(side="left")
        self.view_summary = tk.StringVar(value="未选择变量")
        ttk.Label(chart_header, textvariable=self.view_summary, style="Muted.TLabel").pack(side="right")
        self.figure = Figure(figsize=(10, 8), dpi=100, facecolor=COLORS["surface"])
        self.canvas = ChartCanvas(self.figure, master=right)
        self.toolbar = ChartToolbar(self.canvas, right, self.report_action, self.export_current_csv,
                                    self.restore_current_view)
        self.toolbar.configure(background=COLORS["surface"])
        for widget in self.toolbar.winfo_children():
            if isinstance(widget, (tk.Button, tk.Checkbutton)):
                widget.configure(background=COLORS["surface"], activebackground=COLORS["selected"],
                                 relief="flat", borderwidth=1, highlightthickness=0)
            elif isinstance(widget, (tk.Label, tk.Frame)):
                widget.configure(background=COLORS["surface"])
        self.toolbar.style_mode_buttons()
        self.canvas.get_tk_widget().configure(highlightthickness=0)
        self.toolbar.pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("scroll_event", self.scroll_zoom)
        self.canvas.mpl_connect("button_press_event", self.select_plot)
        self.canvas.mpl_connect("button_press_event", self.navigation_pressed)
        self.canvas.mpl_connect("motion_notify_event", self.navigation_moved)
        self.canvas.mpl_connect("button_release_event", self.navigation_released)
        self.canvas.mpl_connect("resize_event", self.figure_resized)
        self.update_groups()
        self.plot()
        root.update_idletasks()
        sidebar_min = left.winfo_reqwidth()
        pane.paneconfigure(left, minsize=sidebar_min, width=sidebar_min)
        root.minsize(max(1040, sidebar_min + 360 + 36), 700)
        root.drop_target_register(DND_FILES)
        root.dnd_bind("<<Drop>>", self.drop_paths)
        self.poll_job = root.after(100, self.poll)
        if files:
            self.open_paths(files)

    @property
    def pending(self):
        return len(self.requests)

    def report_action(self, action):
        self.status.set(f"最近操作：{action}")

    def report_view(self, action, ax):
        target = f"子图 {self.axes.index(ax) + 1}" if ax in self.axes else "FFT"
        left, right = ax.get_xlim()
        lower, upper = ax.get_ylim()
        self.report_action(f"{action} · {target} · X [{left:.4g}, {right:.4g}] · Y [{lower:.4g}, {upper:.4g}]")

    def update_search_hint(self, _event=None):
        if not self.search.get() and self.root.focus_get() is not self.search_entry:
            self.search_hint.place(x=5, rely=0.5, anchor="w")
        else:
            self.search_hint.place_forget()

    def search_changed(self, *_args):
        self.finish_tree_selection()
        self.update_search_hint()
        self.refresh_tree()
        self.report_action(f"搜索变量：{self.search.get()}" if self.search.get() else "清除变量搜索")

    def clear_search(self):
        self.search_entry.focus_set()
        self.search.set("")

    def select_files(self):
        from tkinter import filedialog
        initialdir = self.last_directory if self.last_directory.is_dir() else Path.cwd()
        paths = filedialog.askopenfilenames(
            parent=self.root, title="选择飞行数据文件", initialdir=str(initialdir),
            filetypes=[("CSV 文件", "*.csv"), ("飞行日志", "*.csv *.txt *.log *.dat")])
        if paths:
            self.open_paths(paths)
        else:
            self.report_action("取消文件选择")

    def drop_paths(self, event):
        from tkinterdnd2 import COPY, REFUSE_DROP
        paths = self.root.tk.splitlist(event.data)
        return COPY if self.open_paths(paths) else REFUSE_DROP

    def open_paths(self, paths):
        files, ignored = [], []
        for raw in paths:
            try:
                path = Path(raw).expanduser().resolve()
                if path.is_file() and path.suffix.lower() in LOG_SUFFIXES:
                    files.append(path)
                else:
                    ignored.append(raw)
            except (OSError, ValueError):
                ignored.append(raw)
        settings_error = None
        accepted, duplicates = False, []
        if files:
            self.last_directory = files[0].parent
            try:
                remember_directory(self.last_directory)
            except OSError as exc:
                settings_error = exc
            for file in dict.fromkeys(files):
                if self.filename_conflict(file):
                    duplicates.append(file)
                else:
                    accepted = self.load(file) or accepted
        if ignored:
            self.status.set(f"已忽略 {len(ignored)} 个目录、无效路径或不支持的文件；请选择 CSV/TXT/LOG/DAT 文件。")
        elif settings_error:
            self.status.set(f"文件已选择，但无法保存下次打开的位置：{settings_error}")
        if duplicates:
            self.report_duplicate_files(duplicates)
        return accepted

    def filename_conflict(self, path):
        return any(other != path and other.name.casefold() == path.name.casefold()
                   for other in self.logs.keys() | self.requests.keys())

    def report_duplicate_files(self, paths):
        from tkinter import messagebox
        names = "、".join(path.name for path in paths)
        message = f"无法载入：{names}。已有同名文件正在读取或已加载，请先重命名文件或移除原同名文件。"
        self.status.set(message)
        messagebox.showerror("无法载入同名文件", message, parent=self.root)

    def load(self, file):
        path = Path(file).expanduser().resolve()
        if self.filename_conflict(path):
            self.report_duplicate_files([path])
            return False
        options = dict(self.options)
        try:
            stat = path.stat()
        except OSError as exc:
            self.status.set(str(exc))
            return False
        signature = (stat.st_size, stat.st_mtime_ns, tuple(sorted(options.items())))
        if path in self.requests:
            self.status.set(f"正在读取：{path.name}；待完成 {self.pending} 个文件…")
            return True
        if self.signatures.get(path) == signature:
            self.status.set(f"已加载，直接复用内存：{path.name}")
            return True
        self.request_id += 1
        request_id = self.request_id
        self.requests[path] = request_id
        if path not in self.file_order:
            self.file_order.append(path)
        self.refresh_tree()
        self.status.set(f"后台读取 {path.name}；待完成 {self.pending} 个文件…")

        def work():
            try:
                result = load_log(path, **options)
                self.events.put((request_id, path, signature, result, None))
            except Exception as exc:
                self.events.put((request_id, path, signature, None, str(exc)))
        self.futures[path] = self.executor.submit(work)
        return True

    def poll(self):
        from tkinter import messagebox
        while not self.events.empty():
            request_id, path, signature, log, error = self.events.get_nowait()
            if self.requests.get(path) != request_id:
                continue
            self.requests.pop(path)
            self.futures.pop(path, None)
            if error:
                if path not in self.logs:
                    self.file_order.remove(path)
                self.refresh_tree()
                self.status.set(f"加载失败：{path.name}")
                messagebox.showerror("日志读取失败", f"{path}\n\n{error}", parent=self.root)
                continue
            self.logs[path] = log
            self.signatures[path] = signature
            self.refresh_tree()
            self.plot()
            self.report_action(f"加载完成 · {path.name} · {len(log.time):,} 行 × {len(log.columns)} 列 · "
                               f"{log.sample_rate:.3f} Hz · {log.data.nbytes / 2**20:.1f} MiB"
                               f" · {len(log.warnings)} 条提示 · 待完成 {self.pending}")
        if not self.closed:
            self.poll_job = self.root.after(100, self.poll)

    def refresh_tree(self):
        selected_files = {self.file_items[item] for item in self.tree.selection() if item in self.file_items}
        selected_entries = self.selected()
        expanded = {path: self.tree.item(item, "open") for item, path in self.file_items.items()}
        self.syncing = True
        self.tree.delete(*self.tree.get_children())
        self.entries.clear()
        self.file_items.clear()
        query = self.search.get().casefold()
        for path in self.file_order:
            label = path.name + ("（加载中…）" if path in self.requests else "")
            parent = self.tree.insert("", "end", iid=f"file:{path}", text=label,
                                      open=expanded.get(path, True), tags=("file",))
            self.file_items[parent] = path
            log = self.logs.get(path)
            if log is None:
                continue
            for i, column in enumerate(log.columns):
                if query and query not in column.casefold():
                    continue
                item = self.tree.insert(parent, "end", iid=f"variable:{path}:{i}", text=column,
                                        tags=("alternate",) if i % 2 else ())
                self.entries[item] = (path, i)
        restored = [item for item, path in self.file_items.items() if path in selected_files]
        if restored:
            restored.extend(item for item, entry in self.entries.items() if entry in selected_entries)
            self.tree.selection_set(restored)
        else:
            self.select_group()
        self.syncing = False
        self.overview.set(f"{len(self.logs)} 个文件" + (f" · {self.pending} 个读取中" if self.pending else ""))

    def selected(self):
        return {self.entries[item] for item in self.tree.selection() if item in self.entries}

    def tree_pressed(self, event):
        self.finish_tree_selection()
        item = self.tree.identify_row(event.y)
        element = self.tree.identify_element(event.x, event.y)
        if item not in self.file_items or not element.endswith(("indicator", "disclosure")):
            if item in self.entries:
                # Let Tk handle focus, Ctrl toggles and Shift ranges. A plain
                # click replaces the whole group, including filtered-out items.
                if self.groups and not event.state & 0x0005: # Shift / Control.
                    self.replace_selection_group = self.group_ids[max(0, self.group_box.current())]
                    self.selection_job = self.root.after_idle(self.finish_tree_selection)
                path, column = self.entries[item]
                self.report_action(f"点选变量 · {path.name} / {self.logs[path].columns[column]}")
            elif item in self.file_items:
                self.report_action(f"点选文件 · {self.file_items[item].name}")
            return
        # The default Treeview press selects the file header before toggling it,
        # discarding variable selections from this and other files.
        self.tree.focus_set()
        self.tree.focus(item)
        if self.tree.get_children(item):
            if self.tree.tk.getboolean(self.tree.item(item, "open")):
                self.tree.item(item, open=False)
                self.tree.event_generate("<<TreeviewClose>>")
            else:
                self.tree.event_generate("<<TreeviewOpen>>")
                self.tree.item(item, open=True)
            action = "展开文件" if self.tree.tk.getboolean(self.tree.item(item, "open")) else "折叠文件"
            self.report_action(f"{action} · {self.file_items[item].name}")
        return "break"

    def finish_tree_selection(self):
        if self.selection_job is not None:
            self.root.after_cancel(self.selection_job)
            self.selection_job = None
        if self.replace_selection_group is None:
            return
        index = max(0, self.group_box.current())
        if self.groups and self.group_ids[index] == self.replace_selection_group:
            # Also handle clicking an already-selected visible item when hidden
            # curves still need to be removed and Tk sends no selection change.
            self.selection_changed()
        self.replace_selection_group = None

    def selected_files(self):
        return {self.file_items[item] if item in self.file_items else self.entries[item][0]
                for item in self.tree.selection() if item in self.file_items or item in self.entries}

    def move_files(self, direction):
        selected = self.selected_files()
        if not selected:
            self.status.set("请先在文件树中选择要移动的文件或其变量。")
            return
        indices = range(1, len(self.file_order)) if direction < 0 else range(len(self.file_order) - 2, -1, -1)
        for index in indices:
            neighbor = index + direction
            if self.file_order[index] in selected and self.file_order[neighbor] not in selected:
                self.file_order[index], self.file_order[neighbor] = self.file_order[neighbor], self.file_order[index]
        self.refresh_tree()
        self.plot()
        self.report_action(f"{'上移' if direction < 0 else '下移'}所选文件 · {len(selected)} 个")

    def remove_files(self):
        selected = self.selected_files()
        if not selected:
            self.status.set("请先在文件树中选择要删除的文件或其变量。")
            return
        for path in selected:
            future = self.futures.pop(path, None)
            if future:
                future.cancel()
            self.requests.pop(path, None)
            self.logs.pop(path, None)
            self.signatures.pop(path, None)
        self.file_order = [path for path in self.file_order if path not in selected]
        self.transforms = {entry: value for entry, value in self.transforms.items() if entry[0] not in selected}
        self.groups = [{entry for entry in group if entry[0] not in selected} for group in self.groups]
        self.spare_groups = [(group_id, {entry for entry in group if entry[0] not in selected})
                             for group_id, group in self.spare_groups]
        self.refresh_tree()
        self.plot()
        self.report_action(f"删除所选文件 · 已移除 {len(selected)} 个；待完成 {self.pending} 个文件")

    def ordered_entries(self, entries):
        positions = {path: index for index, path in enumerate(self.file_order)}
        return sorted(entries, key=lambda entry: (positions.get(entry[0], len(positions)), entry[1]))

    def select_group(self):
        index = max(self.group_box.current(), 0)
        group = self.groups[index] if self.groups else set()
        was_syncing = self.syncing
        self.syncing = True
        self.tree.selection_set([item for item, key in self.entries.items() if key in group])
        self.syncing = was_syncing

    def selection_changed(self, _event=None):
        if self.syncing:
            return
        if any(item in self.file_items for item in self.tree.selection()) and not self.selected():
            return
        if not self.groups:
            if not self.selected():
                return
            self.append_group()
            self.update_groups()
        index = max(self.group_box.current(), 0)
        replace = self.replace_selection_group == self.group_ids[index]
        self.replace_selection_group = None
        # Search refreshes and modified clicks retain hidden assignments;
        # an ordinary variable click starts a new selection for the group.
        visible = set(self.entries.values())
        updated = self.selected() if replace else (self.groups[index] - visible) | self.selected()
        changed = updated != self.groups[index]
        added = updated - self.groups[index]
        removed = self.groups[index] - updated
        if changed:
            self.groups[index] = updated
        if changed or _event is None:
            self.plot()
        if changed:
            affected = self.ordered_entries(added or removed)
            names = "、".join(f"{path.name}/{self.logs[path].columns[column]}" for path, column in affected[:2])
            if len(affected) > 2:
                names += f" 等 {len(affected)} 个"
            self.report_action(f"{'点选变量' if added else '取消选择'} · 子图 {index + 1} · {names} · 共 {len(updated)} 个变量")

    def update_groups(self, active=None):
        active = self.group_box.current() if active is None else active
        self.group_box["values"] = [f"子图 {i + 1}" for i in range(len(self.groups))]
        if self.groups:
            self.group_box.current(min(max(0, active), len(self.groups) - 1))
        else:
            self.group_box.set("")
        self.delete_group_button.configure(state="normal" if self.groups else "disabled")

    def change_group(self, _event=None):
        self.select_group()
        self.refresh_transform_selector()
        self.style_active_group()
        self.update_fft()
        if self.groups:
            self.report_action(f"切换目标 · 子图 {self.group_box.current() + 1}")

    def select_plot(self, event):
        if event.button != 1 or event.inaxes not in self.axes:
            return
        self.group_box.current(self.axes.index(event.inaxes))
        self.change_group()

    def export_current_csv(self):
        from tkinter import filedialog
        selected = self.ordered_entries(self.groups[max(0, self.group_box.current())]) if self.groups else []
        paths = {path for path, _ in selected}
        if len(paths) != 1:
            self.status.set("导出 CSV 时，请在当前子图中选择同一个文件的变量。")
            return
        log = self.logs[selected[0][0]]
        destination = filedialog.asksaveasfilename(
            parent=self.root, title="导出 CSV", initialdir=str(log.path.parent),
            initialfile=log.path.stem + "_selected.csv", defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")])
        if not destination:
            self.report_action("取消 CSV 导出")
            return
        if Path(destination).resolve() in self.logs:
            self.status.set("导出失败：不能覆盖已加载的原始日志。")
            return
        try:
            export_csv(log, destination, [log.columns[column] for _, column in selected],
                       self.axes[max(0, self.group_box.current())].get_xlim())
        except (OSError, ValueError) as exc:
            self.status.set(f"导出 CSV 失败：{exc}")
            return
        self.report_action(f"导出 CSV · {Path(destination).name}（原始数值，当前时间范围）")

    def apply_layout(self, _event=None):
        try:
            rows, columns = int(self.row_count.get()), int(self.column_count.get())
            if not 1 <= rows <= 8 or not 1 <= columns <= 8:
                raise ValueError()
        except ValueError:
            self.status.set("子图行数和列数必须是 1～8 的整数。")
            return "break"
        active = self.group_box.current()
        count = rows * columns
        if count < len(self.groups):
            self.spare_groups = list(zip(self.group_ids[count:], self.groups[count:])) + self.spare_groups
            del self.groups[count:]
            del self.group_ids[count:]
        while len(self.groups) < count:
            if self.spare_groups:
                group_id, group = self.spare_groups.pop(0)
                self.group_ids.append(group_id)
                self.groups.append(group)
            else:
                self.append_group()
        self.layout_rows, self.layout_columns = rows, columns
        self.update_groups(active)
        self.select_group()
        self.plot()
        self.report_action(f"应用布局 · {rows} 行 × {columns} 列 · {count} 个子图"
                           + ("；超出位置的变量分配已暂存，扩大布局可恢复" if self.spare_groups else ""))
        return "break"

    def append_group(self):
        self.groups.append(set())
        self.group_ids.append(self.next_group_id)
        self.next_group_id += 1

    def clear_group(self):
        if not self.groups:
            return
        self.groups[max(0, self.group_box.current())].clear()
        self.select_group()
        self.plot()
        self.report_action(f"清除子图 · 子图 {self.group_box.current() + 1}")

    def delete_group(self):
        self.finish_tree_selection()
        if not self.groups:
            return
        index = max(0, self.group_box.current())
        removed_id = self.group_ids.pop(index)
        del self.groups[index]
        # Compact unused outer rows/columns while keeping the existing column
        # order. A partially filled final column contains no extra empty axes.
        self.layout_rows = min(self.layout_rows, len(self.groups)) if self.groups else 1
        self.layout_columns = ((len(self.groups) + self.layout_rows - 1) // self.layout_rows
                               if self.groups else 1)
        self.row_count.set(str(self.layout_rows))
        self.column_count.set(str(self.layout_columns))
        self.update_groups(index)
        self.select_group()
        self.plot()
        self.group_views.pop(removed_id, None)
        self.transform_exclusions.pop(removed_id, None)
        self.report_action(f"删除子图 · 原子图 {index + 1}；后续子图已前移，剩余 {len(self.groups)} 个")

    def toggle_display(self, option):
        if option == "legend":
            for ax in self.figure.axes:
                legend = ax.get_legend()
                if legend is not None:
                    legend.set_visible(not self.hide_legend.get())
            self.canvas.draw_idle()
            self.report_action("隐藏图例" if self.hide_legend.get() else "显示图例")
            return
        self.plot()
        enabled = self.smooth.get() if option == "smooth" else self.fft_enabled.get()
        name = "显示平滑" if option == "smooth" else "区段 FFT"
        self.report_action(f"{'开启' if enabled else '关闭'}{name}")

    def step_transform(self, field, direction):
        try:
            scale, offset = float(self.scale.get()), float(self.offset.get())
            if not np.isfinite([scale, offset]).all():
                raise ValueError()
            if field == "scale":
                scale *= 2 if direction > 0 else 0.5
            else:
                index = max(0, self.group_box.current())
                if index >= len(self.axes):
                    self.status.set("请先选择变量，再按当前子图的纵轴范围调整偏移。")
                    return "break"
                lower, upper = self.axes[index].get_ylim()
                offset += direction * abs(upper - lower) * 0.1
            if not np.isfinite([scale, offset]).all():
                raise ValueError()
        except ValueError:
            self.status.set("缩放与偏移必须是有限数值。")
            return "break"
        self.scale.set(format(scale, ".15g"))
        self.offset.set(format(offset, ".15g"))
        self.apply_transform()
        return "break"

    def refresh_transform_selector(self):
        import tkinter as tk
        self.transform_menu.delete(0, "end")
        self.transform_check_vars = {}
        group = self.groups[max(0, self.group_box.current())] if self.groups else set()
        self.transform_selector.configure(state="normal" if group else "disabled")
        if not group and self.groups:
            self.transform_exclusions.pop(self.group_ids[max(0, self.group_box.current())], None)
        if group:
            group_id = self.group_ids[max(0, self.group_box.current())]
            excluded = self.transform_exclusions.setdefault(group_id, set())
            excluded.intersection_update(group)
            self.transform_menu.add_command(label="全选", command=self.select_all_transform_variables)
            self.transform_menu.add_separator()
            for entry in self.ordered_entries(group):
                checked = tk.BooleanVar(self.root, value=entry not in excluded)
                self.transform_check_vars[entry] = checked
                label = f"{entry[0].name} | {self.logs[entry[0]].columns[entry[1]]}"
                self.transform_menu.add_checkbutton(label=label, variable=checked,
                    command=lambda key=entry, gid=group_id, var=checked:
                        self.set_transform_variable(gid, key, var.get()))
        self.update_transform_summary()

    def update_transform_summary(self):
        index = max(0, self.group_box.current())
        group = self.groups[index] if self.groups else set()
        excluded = self.transform_exclusions.get(self.group_ids[index], set()) if self.groups else set()
        self.transform_summary.set(f"变量：{len(group - excluded)}/{len(group)}")

    def set_transform_variable(self, group_id, entry, checked):
        excluded = self.transform_exclusions.setdefault(group_id, set())
        if checked:
            excluded.discard(entry)
        else:
            excluded.add(entry)
        self.update_transform_summary()

    def select_all_transform_variables(self):
        if not self.groups:
            return
        index = max(0, self.group_box.current())
        self.transform_exclusions[self.group_ids[index]] = set()
        self.refresh_transform_selector()

    def transform_targets(self):
        self.finish_tree_selection()
        if not self.groups:
            return set()
        index = max(0, self.group_box.current())
        return self.groups[index] - self.transform_exclusions.get(self.group_ids[index], set())

    def apply_transform(self, _event=None):
        try:
            scale, offset = float(self.scale.get()), float(self.offset.get())
            if not np.isfinite([scale, offset]).all():
                raise ValueError()
        except ValueError:
            self.status.set("缩放与偏移必须是有限数值。")
            return "break"
        entries = self.transform_targets()
        if not entries:
            self.status.set("请先在变量菜单中勾选要缩放或偏移的变量。")
            return "break"
        for entry in entries:
            self.transforms[entry] = (scale, offset)
        self.plot()
        self.report_action(f"应用显示变换 · 缩放 ×{scale:g}，偏移 {offset:+g} · {len(entries)} 个变量")
        return "break"

    def reset_transform(self):
        entries = self.transform_targets()
        if not entries:
            self.status.set("请先在变量菜单中勾选要恢复的变量。")
            return
        for entry in entries:
            self.transforms.pop(entry, None)
        self.scale.set("1")
        self.offset.set("0")
        self.plot()
        self.report_action(f"恢复默认缩放与偏移 · {len(entries)} 个变量")

    def series(self, entry):
        path, column = entry
        log = self.logs[path]
        scale, offset = self.transforms.get(entry, (1, 0))
        values = log.data[:, column]
        if scale != 1 or offset != 0:
            values = values * scale + offset
        return log, values

    def series_label(self, entry):
        path, column = entry
        log = self.logs[path]
        scale, offset = self.transforms.get(entry, (1, 0))
        label = f"{path.name} | {log.columns[column]}"
        changes = []
        if scale != 1:
            changes.append(f"缩放 ×{scale:g}")
        if offset != 0:
            changes.append(f"偏移 {offset:+g}")
        return label + (f" [{', '.join(changes)}]" if changes else "")

    def line_style(self, entry):
        from ui_style import CURVE_COLORS
        if entry not in self.series_styles:
            index = len(self.series_styles)
            self.series_styles[entry] = {
                "color": CURVE_COLORS[index % len(CURVE_COLORS)],
                "linestyle": "-",
            }
        return self.series_styles[entry]

    def style_active_group(self):
        from ui_style import COLORS
        active = max(0, self.group_box.current())
        dense = len(self.groups) + int(self.fft_enabled.get()) > 4
        for index, ax in enumerate(self.axes):
            ax.spines["left"].set_color(COLORS["accent"] if index == active else COLORS["border"])
            ax.spines["left"].set_linewidth(1.3 if index == active else 0.8)
            color = COLORS["accent"] if index == active else COLORS["text"]
            if dense:
                for text in ax.texts:
                    if text.get_gid() == "panel-title":
                        text.set_color(color)
            else:
                ax.set_title(f"子图 {index + 1}", loc="left", fontsize=10, fontweight="bold", color=color, pad=10)
        self.canvas.draw_idle()

    def plot(self):
        from ui_style import COLORS, style_axes, style_legend
        self.group_views.update({group_id: (signature, (ax.get_xlim(), ax.get_ylim()), bool(ax.lines))
                                 for (group_id, signature), ax in zip(self.plotted_groups, self.axes)})
        if self.view_job:
            self.root.after_cancel(self.view_job)
            self.view_job = None
        self.dirty_view_axes.clear()
        self.figure.clear()
        self.axes, self.lines = [], []
        self.plotted_groups, self.initial_views = [], {}
        count = sum(len(group) for group in self.groups)
        self.view_summary.set(f"{self.layout_rows} 行 × {self.layout_columns} 列 · {len(self.groups)} 个子图 · {count} 条曲线")
        self.refresh_transform_selector()
        if not self.groups:
            self.fft_ax = None
            self.figure.text(0.5, 0.54, "尚无子图", ha="center", fontsize=16, color=COLORS["text"])
            self.figure.text(0.5, 0.48, "设置行列布局以绘制变量", ha="center", fontsize=10, color=COLORS["muted"])
            self.toolbar.update()
            self.canvas.draw_idle()
            return
        grid = self.figure.add_gridspec(self.layout_rows + int(self.fft_enabled.get()), self.layout_columns)
        dense = len(self.groups) + int(self.fft_enabled.get()) > 4
        for group_index, group in enumerate(self.groups):
            column, row = divmod(group_index, self.layout_rows)
            ax = self.figure.add_subplot(grid[row, column])
            self.axes.append(ax)
            group_id = self.group_ids[group_index]
            signature = (self.smooth.get(), frozenset(
                (entry, self.transforms.get(entry, (1, 0)), id(self.logs.get(entry[0])))
                for entry in group))
            # Stable IDs preserve each subplot's own view across layout changes.
            self.plotted_groups.append((group_id, signature))
            previous = self.group_views.get(group_id)
            style_axes(ax, f"子图 {group_index + 1}", active=group_index == max(0, self.group_box.current()), dense=dense)
            ax.set_ylabel("显示值", color=COLORS["muted"])
            ax.tick_params(labelbottom=True)
            ax.set_xlabel("相对时间 (s)", color=COLORS["muted"])
            for entry in self.ordered_entries(group):
                if entry[0] not in self.logs or entry[1] >= len(self.logs[entry[0]].columns):
                    continue
                log, values = self.series(entry)
                width = max(1, round(log.sample_rate * 0.2)) if self.smooth.get() else 1
                x, y = display_series(log.time, values, smooth_samples=width)
                label = self.series_label(entry)
                line, = ax.plot(x, y, linewidth=1.35, alpha=0.95, solid_capstyle="round",
                                label=label, **self.line_style(entry))
                self.lines.append((line, entry))
            if ax.lines:
                style_legend(ax, dense=dense).set_visible(not self.hide_legend.get())
            else:
                ax.text(0.5, 0.5, "选择变量以绘制曲线", transform=ax.transAxes,
                        ha="center", va="center", fontsize=11, color=COLORS["muted"])
                ax.grid(False)
                ax.tick_params(labelbottom=False, labelleft=False, bottom=False, left=False)
            limits = self.time_limits(group_index)
            if limits is not None:
                ax.set_xlim(limits)
            # Keep full-data defaults separately from the current independent view.
            self.initial_views[ax] = (ax.get_xlim(), ax.get_ylim())
            if previous is not None and previous[2]:
                ax.set_xlim(self.bounded_limits(previous[1][0], limits))
                if signature == previous[0]:
                    ax.set_ylim(previous[1][1])
        if self.fft_enabled.get():
            self.fft_ax = self.figure.add_subplot(grid[-1, :])
            self.update_fft()
        else:
            self.fft_ax = None
        self.layout_plot()
        self.toolbar.update()
        self.toolbar.push_current()
        for ax in self.axes:
            ax.callbacks.connect("xlim_changed", self.view_changed)
        self.canvas.draw_idle()

    def layout_plot(self):
        if self.axes:
            dense = len(self.groups) + int(self.fft_enabled.get()) > 4
            self.figure.tight_layout(pad=1.0 if dense else 1.6, h_pad=0.6 if dense else 2.0)

    def figure_resized(self, _event):
        self.layout_plot()

    def time_limits(self, group_index=None):
        if not self.groups:
            return None
        group_index = max(0, self.group_box.current()) if group_index is None else group_index
        paths = {path for path, column in self.groups[group_index]
                 if path in self.logs and column < len(self.logs[path].columns)}
        if not paths:
            return None
        lower = min(float(self.logs[path].time[0]) for path in paths)
        upper = max(float(self.logs[path].time[-1]) for path in paths)
        return (lower, upper) if upper > lower else None

    def bounded_time_xlim(self, xlim, group_index=None):
        return self.bounded_limits(xlim, self.time_limits(group_index))

    def restore_current_view(self):
        if not self.axes:
            return
        ax = self.axes[max(0, self.group_box.current())]
        xlim, ylim = self.initial_views[ax]
        if not np.array_equal((ax.get_xlim(), ax.get_ylim()), (xlim, ylim)):
            self.toolbar.push_current()
            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
            self.toolbar.push_current()
            self.canvas.draw_idle()
        self.report_view("恢复当前子图初始视图", ax)

    @staticmethod
    def bounded_limits(view, bounds):
        # Shared by the full time extent and each axis's initial display extent.
        if bounds is None:
            return tuple(view)
        lower, upper = sorted(bounds)
        left, right = sorted(view)
        width = right - left
        if width >= upper - lower:
            result = (lower, upper)
        else:
            left = min(max(left, lower), upper - width)
            result = (left, min(upper, left + width))
        return result[::-1] if view[0] > view[1] else result

    def navigation_pressed(self, event):
        self.navigation_start = None
        if self.drag_axes is not None:
            self.drag_axes.end_pan()
            self.drag_axes = None
        if self.toolbar.mode and event.button in (1, 3) and event.inaxes in self.figure.axes:
            ax = event.inaxes
            self.navigation_start = (str(self.toolbar.mode), ax, ax.get_xlim(), ax.get_ylim())
        elif not self.toolbar.mode and event.button == 1 and event.inaxes in self.figure.axes and event.inaxes.has_data():
            ax = event.inaxes
            self.navigation_start = ("drag", ax, ax.get_xlim(), ax.get_ylim())
            self.drag_axes = ax
            ax.start_pan(event.x, event.y, 1)

    def navigation_moved(self, event):
        if self.drag_axes is None:
            return
        if 1 not in (event.buttons or set()) or self.drag_axes not in self.figure.axes:
            self.navigation_released(event)
            return
        self.drag_axes.drag_pan(1, event.key, event.x, event.y)
        self.canvas.draw_idle()

    def navigation_released(self, _event):
        if self.drag_axes is not None:
            self.drag_axes.end_pan()
            self.drag_axes = None
        start = self.navigation_start
        self.navigation_start = None
        if start is None:
            return
        mode, ax, xlim, ylim = start
        if ax in self.figure.axes and (ax.get_xlim() != xlim or ax.get_ylim() != ylim):
            if mode == "drag":
                self.toolbar.push_current()
            self.report_view("拖动平移" if mode == "drag" else "Zoom 框选缩放", ax)

    def scroll_zoom(self, event):
        ax = event.inaxes
        if ax not in self.figure.axes or not ax.has_data() or not event.step:
            return
        if event.xdata is None or event.ydata is None or not np.isfinite(event.step):
            return
        factor = 1.2 ** -float(np.clip(event.step, -20, 20))
        xlim = event.xdata + (np.asarray(ax.get_xlim()) - event.xdata) * factor
        ylim = event.ydata + (np.asarray(ax.get_ylim()) - event.ydata) * factor
        if not np.isfinite([xlim, ylim]).all() or xlim[0] == xlim[1] or ylim[0] == ylim[1]:
            return
        initial = self.initial_views.get(ax)
        x_bounds = self.time_limits(self.axes.index(ax)) if ax in self.axes else initial[0] if initial else None
        xlim = self.bounded_limits(xlim, x_bounds)
        ylim = self.bounded_limits(ylim, initial[1] if initial else None)
        if np.array_equal(xlim, ax.get_xlim()) and np.array_equal(ylim, ax.get_ylim()):
            return
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        self.toolbar.push_current()
        self.canvas.draw_idle()
        self.report_view("Zoom 滚轮放大" if event.step > 0 else "Zoom 滚轮缩小", ax)

    def view_changed(self, _ax):
        if _ax not in self.axes:
            return
        bounded = self.bounded_time_xlim(_ax.get_xlim(), self.axes.index(_ax))
        if not np.array_equal(_ax.get_xlim(), bounded):
            _ax.set_xlim(bounded, emit=False)
        self.dirty_view_axes.add(_ax)
        if self.view_job:
            self.root.after_cancel(self.view_job)
        self.view_job = self.root.after(180, self.refresh_view)

    def refresh_view(self):
        if self.view_job:
            self.root.after_cancel(self.view_job)
        self.view_job = None
        if not self.axes:
            return
        targets = self.dirty_view_axes or set(self.axes)
        self.dirty_view_axes = set()
        for line, entry in self.lines:
            if line.axes not in targets:
                continue
            bounds = line.axes.get_xlim()
            log, values = self.series(entry)
            left = max(0, int(np.searchsorted(log.time, bounds[0])) - 1)
            right = min(len(log.time), int(np.searchsorted(log.time, bounds[1], side="right")) + 1)
            width = max(1, round(log.sample_rate * 0.2)) if self.smooth.get() else 1
            line.set_data(*display_series(log.time[left:right], values[left:right], smooth_samples=width))
        if self.axes[max(0, self.group_box.current())] in targets:
            self.update_fft()
        self.canvas.draw_idle()

    def update_fft(self):
        from ui_style import COLORS, style_axes, style_legend
        if not self.fft_enabled.get() or not hasattr(self, "fft_ax") or self.fft_ax is None or not self.axes:
            return
        ax = self.fft_ax
        ax.clear()
        dense = len(self.groups) + int(self.fft_enabled.get()) > 4
        style_axes(ax, "频域分析 · FFT", dense=dense)
        errors = []
        for entry in self.ordered_entries(self.groups[max(0, self.group_box.current())]):
            if entry[0] not in self.logs:
                continue
            log, values = self.series(entry)
            label = self.series_label(entry)
            try:
                spectrum = compute_spectrum(log.time, values,
                                            self.axes[max(0, self.group_box.current())].get_xlim())
                ax.plot(spectrum.frequency, spectrum.amplitude, linewidth=1.35, label=label, **self.line_style(entry))
            except LogError as exc:
                errors.append(f"{log.path.name}/{log.columns[entry[1]]}: {exc}")
        ax.set_xlabel("频率 (Hz)", color=COLORS["muted"])
        ax.set_ylabel("幅值", color=COLORS["muted"])
        if ax.lines:
            style_legend(ax, dense=dense).set_visible(not self.hide_legend.get())
        else:
            ax.text(0.5, 0.5, "\n".join(errors[:2]) if errors else "选择变量与时间范围以查看频谱", transform=ax.transAxes,
                    ha="center", va="center", fontsize=10, color=COLORS["muted"])
        if errors and ax.lines:
            ax.text(0.01, 0.03, "\n".join(errors[:2]), transform=ax.transAxes, fontsize=8,
                    color=COLORS["danger"], clip_on=True).set_in_layout(False)
        self.initial_views[ax] = (ax.get_xlim(), ax.get_ylim())
        self.canvas.draw_idle()

    def close(self):
        self.closed = True
        if self.drag_axes is not None:
            self.drag_axes.end_pan()
            self.drag_axes = None
        self.canvas.cancel_pending_draws()
        for job in (self.poll_job, self.view_job, self.fft_job, self.selection_job):
            if job:
                self.root.after_cancel(job)
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


def main():
    # Keep Chinese diagnostics and redirected JSON readable on Windows too.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="CoFly 飞行日志浏览、摘要与区段 FFT")
    parser.add_argument("files", nargs="*", type=Path)
    parser.add_argument("--summary", action="store_true", help="输出 JSON 摘要，不打开窗口")
    parser.add_argument("--fft", help="输出指定列在所选区段的频谱峰值")
    parser.add_argument("--from", dest="start", type=float, default=0)
    parser.add_argument("--to", dest="stop", type=float, default=float("inf"))
    parser.add_argument("--time-column")
    parser.add_argument("--time-unit", choices=("auto", "s", "ms", "us"), default="auto")
    parser.add_argument("--fallback-hz", type=float, default=50)
    parser.add_argument("--encoding", default="utf-8-sig")
    parser.add_argument("--delimiter", help="默认识别逗号、制表符或分号")
    parser.add_argument("--skip-bad-rows", action="store_true")
    args = parser.parse_args()
    options = {key: getattr(args, key) for key in ("time_column", "time_unit", "fallback_hz", "encoding", "delimiter", "skip_bad_rows")}
    if args.summary or args.fft:
        if not args.files:
            parser.error("命令行分析需要至少一个文件。")
        if not np.isfinite(args.start) or np.isnan(args.stop) or args.start >= args.stop:
            parser.error("时间区段必须满足 --from < --to。")
        results = []
        try:
            for file in args.files:
                log = load_log(file, **options)
                summary = log.summary()
                if args.fft:
                    spectrum = compute_spectrum(log.time, log.column(args.fft), (args.start, args.stop))
                    summary["fft"] = dict(column=args.fft, peak_hz=spectrum.peak_frequency,
                                          peak_amplitude=spectrum.peak_amplitude, samples=spectrum.samples,
                                          resampled=spectrum.resampled)
                results.append(summary)
        except (OSError, ValueError, UnicodeError) as exc:
            parser.exit(2, f"读取失败：{exc}\n")
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        from tkinterdnd2 import TkinterDnD
        root = TkinterDnD.Tk()
        Browser(root, args.files, options)
        root.mainloop()


if __name__ == "__main__":
    main()
