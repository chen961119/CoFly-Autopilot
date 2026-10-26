"""Read-only SD browser and checked, retryable USB downloads."""
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path, PurePosixPath
import os
import re
import tempfile
import time
import tkinter as tk
from tkinter import filedialog, ttk
import zlib

from protocol import SD_MAX_BLOCK_SIZE

@dataclass(frozen=True)
class RemoteEntry:
    name: str
    size: int
    modified: object
    directory: bool

    @classmethod
    def parse(cls, fields):
        if len(fields) != 4 or fields[3] not in ("DIR", "FILE"):
            raise ValueError("文件条目格式错误")
        name = bytes.fromhex(fields[0]).decode("utf-8")
        if (not name or name in (".", "..") or any(c in name for c in '/\\\x00')
                or any(ord(c) < 32 for c in name)):
            raise ValueError("文件名无效")
        size = int(fields[1])
        if not 0 <= size <= 0xffffffffffffffff:
            raise ValueError("文件大小无效")
        modified = None if fields[2] == "-" else datetime.strptime(fields[2], "%Y-%m-%dT%H:%M:%S")
        return cls(name, size, modified, fields[3] == "DIR")

    @property
    def kind(self):
        extension = PurePosixPath(self.name).suffix[1:].upper()
        return "文件夹" if self.directory else f"{extension} 文件" if extension else "文件"


def encode_path(path):
    data = path.encode("utf-8")
    if len(data) > 255:
        raise ValueError("文件路径超过飞控支持的 255 字节")
    return data.hex()


def safe_local_name(name):
    # FAT names can include Windows device names. Keep all output inside the
    # chosen directory, using a visible replacement for Windows-only characters.
    name = re.sub(r'[<>:"/\\|?*\x00-\x1f]', "_", name).rstrip(" .")
    if not name or name in (".", ".."):
        raise ValueError("无法保存此文件名")
    if re.fullmatch(r"(?i:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])", name.split(".")[0]):
        name = "_" + name
    return name


class DownloadFile:
    """A file becomes visible under its final name only after end-to-end CRC."""
    def __init__(self, destination, entry, size):
        if size != entry.size:
            raise ValueError("文件大小已变化，请刷新文件列表")
        self.entry, self.destination, self.size = entry, Path(destination), size
        self.offset = self.crc = 0
        self.file = tempfile.NamedTemporaryFile(mode="w+b", prefix=".cofly-", suffix=".part",
                                               dir=self.destination, delete=False)
        self.temporary = Path(self.file.name)

    def append(self, fields):
        if len(fields) != 3:
            raise ValueError("数据块格式错误")
        offset, data, checksum = int(fields[0]), bytes.fromhex(fields[1]), int(fields[2], 16)
        if offset != self.offset or not 1 <= len(data) <= SD_MAX_BLOCK_SIZE or self.offset + len(data) > self.size:
            raise ValueError("数据块长度或位置错误")
        if zlib.crc32(data) != checksum:
            raise ValueError("数据块 CRC 校验失败")
        if self.file.write(data) != len(data):
            raise OSError("电脑未完整写入文件")
        self.crc = zlib.crc32(data, self.crc)
        self.offset += len(data)

    def finish(self, fields):
        if (len(fields) != 2 or int(fields[0]) != self.size or self.offset != self.size
                or int(fields[1], 16) != self.crc):
            raise ValueError("文件长度或完整 CRC 校验失败")
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        if self.entry.modified:
            stamp = self.entry.modified.timestamp()
            os.utime(self.temporary, (stamp, stamp))
        name = safe_local_name(self.entry.name)
        base = Path(name)
        index = 0
        while True:
            target = self.destination / (name if index == 0 else f"{base.stem} ({index}){base.suffix}")
            if target.exists():
                index += 1
                continue
            try:
                if os.name == "nt":
                    # Windows rename fails atomically if a destination exists.
                    os.rename(self.temporary, target)
                else:
                    os.link(self.temporary, target)
                    self.temporary.unlink()
                return target
            except FileExistsError:
                index += 1

    def abort(self):
        self.file.close()
        self.temporary.unlink(missing_ok=True)


class SdTransport:
    def __init__(self, station, failure):
        self.station, self.failure = station, failure
        self.pending = None
        self.alive = False
        self.last_activity = time.monotonic()

    def request(self, verb, args=(), callback=None):
        if self.pending:
            raise RuntimeError("SD 请求尚未完成")
        self.station.request_id = self.station.request_id % 0xffffffff + 1
        request_id = self.station.request_id
        data = (f"SD {verb} {request_id}" + (" " + " ".join(args) if args else "") + "\n").encode("ascii")
        self.pending = dict(id=request_id, verb=verb, data=data, callback=callback,
                            attempts=0, deadline=0)
        self.resend()

    def resend(self):
        pending = self.pending
        pending["attempts"] += 1
        pending["deadline"] = time.monotonic() + 3
        self.last_activity = time.monotonic()
        self.station.terminal_filter.register(pending["id"], "SD")
        self.station.worker.send(pending["data"])

    def handle(self, reply):
        request_id, kind, fields = reply
        if not self.pending or request_id != self.pending["id"]:
            return False
        pending = self.pending
        if kind == "ERROR":
            self.pending = None
            self.alive = False
            messages = {"LOCK_REQUIRED": "飞机已解锁，已停止浏览和下载。请锁定后刷新。",
                        "NO_SD": "SD 卡不可用", "READ_FAILED": "SD 卡读取失败",
                        "OPEN_FAILED": "文件或目录无法打开", "FILE_CHANGED": "文件已变化，请刷新",
                        "SESSION_EXPIRED": "SD 会话已超时，请刷新", "PATH": "SD 路径无效"}
            self.failure(messages.get(fields[0] if fields else "", "飞控拒绝请求：" + " / ".join(fields)))
            return True
        try:
            expected = {"LIST": ("SD_BEGIN",), "NEXT": ("SD_ENTRY", "SD_END"),
                        "OPEN": ("SD_OPEN",), "READ": ("SD_DATA", "SD_EOF"),
                        "KEEP": ("SD_KEEP",), "CANCEL": ("SD_CANCEL",)}[pending["verb"]]
            if kind not in expected:
                raise ValueError("SD 回复类型不匹配")
            # Verify before consuming the request. A damaged block is requested
            # again with the same ID; firmware replays its cached response.
            if kind == "SD_DATA":
                if len(fields) != 3 or zlib.crc32(bytes.fromhex(fields[1])) != int(fields[2], 16):
                    raise ValueError("数据块 CRC 校验失败")
        except (ValueError, KeyError):
            if pending["attempts"] < 3:
                self.resend()
            else:
                self.pending = None
                self.alive = False
                self.failure("SD 数据损坏，三次重试后仍未通过校验")
            return True
        self.pending = None
        self.alive = kind != "SD_CANCEL"
        try:
            if pending["callback"]:
                pending["callback"](kind, fields)
        except (OSError, ValueError, IndexError, UnicodeError) as exc:
            self.failure(str(exc))
        return True

    def tick(self):
        now = time.monotonic()
        if self.pending and now >= self.pending["deadline"]:
            if self.pending["attempts"] < 3:
                self.resend()
            else:
                self.pending = None
                self.alive = False
                self.failure("SD 请求超时，已停止；请检查连接后刷新")
        elif self.alive and not self.pending and now - self.last_activity >= 3:
            self.request("KEEP")


class SdFileDialog:
    def __init__(self, station):
        self.station = station
        try:
            self.block_size = max(1, min(SD_MAX_BLOCK_SIZE, int(station.caps.get("sd_block", "256"))))
        except (ValueError, TypeError):
            self.block_size = 256
        self.transport = SdTransport(station, self.fail)
        self.path = "/"
        self.entries = {}
        self.sort_column, self.reverse = "name", False
        self.busy = self.closing = self.closed = False
        self.releasing_after_error = False
        self.download = None
        self.queue = deque()
        self.queued_start = None
        self.queued_path = None
        self.completed = self.total_bytes = self.done_bytes = 0
        self.window = tk.Toplevel(station.root)
        self.window.title("下载 SD 卡内容")
        self.window.geometry("920x620")
        self.window.minsize(700, 450)
        self.window.transient(station.root)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.window.grab_set()
        outer = ttk.Frame(self.window, padding=16)
        outer.pack(fill="both", expand=True)
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill="x", pady=(0, 10))
        self.up_button = ttk.Button(toolbar, text="上一级", command=self.up)
        self.up_button.pack(side="left")
        self.refresh_button = ttk.Button(toolbar, text="刷新", command=lambda: self.load(self.path))
        self.refresh_button.pack(side="left", padx=8)
        self.path_text = tk.StringVar(value="/")
        ttk.Label(toolbar, textvariable=self.path_text).pack(side="left", padx=8)
        grid = ttk.Frame(outer)
        grid.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(grid, columns=("name", "size", "modified", "kind"),
                                 show="headings", selectmode="extended")
        self.labels = dict(name="名称", size="大小", modified="修改时间", kind="类型")
        for column, width in zip(self.labels, (350, 130, 180, 130)):
            self.tree.heading(column, text=self.labels[column], command=lambda c=column: self.sort(c))
            self.tree.column(column, width=width, minwidth=80, anchor="e" if column == "size" else "w")
        self.tree.grid(row=0, column=0, sticky="nsew")
        vertical = ttk.Scrollbar(grid, orient="vertical", command=self.tree.yview)
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(grid, orient="horizontal", command=self.tree.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.tree.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        grid.rowconfigure(0, weight=1)
        grid.columnconfigure(0, weight=1)
        self.tree.bind("<Double-1>", self.enter)
        self.tree.bind("<<TreeviewSelect>>", lambda _: self.update_controls())
        self.tree.bind("<Control-a>", self.select_all)
        ttk.Label(outer, text="Ctrl / Shift 多选，点击列标题切换升序 / 降序；双击文件夹进入。按当前显示顺序下载选中的文件。",
                  wraplength=850).pack(anchor="w", pady=(10, 4))
        self.note = tk.StringVar(value="正在读取 SD 卡…")
        ttk.Label(outer, textvariable=self.note, wraplength=850).pack(anchor="w", pady=4)
        self.progress = ttk.Progressbar(outer, maximum=100)
        self.progress.pack(fill="x", pady=6)
        buttons = ttk.Frame(outer)
        buttons.pack(fill="x", pady=(6, 0))
        self.download_button = ttk.Button(buttons, text="下载所选文件…", command=self.choose_destination)
        self.download_button.pack(side="right")
        self.cancel_button = ttk.Button(buttons, text="取消下载", command=self.cancel_download)
        self.cancel_button.pack(side="right", padx=8)
        self.close_button = ttk.Button(buttons, text="关闭", command=self.close)
        self.close_button.pack(side="left")

    def update_controls(self):
        enabled = not self.busy and not self.closing and not self.queued_start and self.queued_path is None
        self.up_button.configure(state="normal" if enabled and self.path != "/" else "disabled")
        self.refresh_button.configure(state="normal" if enabled else "disabled")
        selected = [self.entries[i] for i in self.tree.selection() if i in self.entries and not self.entries[i].directory]
        self.download_button.configure(text=f"下载所选文件（{len(selected)}）…",
                                       state="normal" if enabled and selected else "disabled")
        self.cancel_button.configure(state="normal" if self.download or self.queue or self.queued_start else "disabled")

    def sort(self, column=None):
        if column:
            self.reverse = not self.reverse if column == self.sort_column else False
            self.sort_column = column
        keys = {"name": lambda e: e.name.casefold(), "size": lambda e: e.size,
                "modified": lambda e: e.modified or datetime.min, "kind": lambda e: e.kind}
        rows = sorted(self.entries, key=lambda i: (keys[self.sort_column](self.entries[i]), self.entries[i].name.casefold()),
                      reverse=self.reverse)
        rows.sort(key=lambda i: not self.entries[i].directory)
        for index, row in enumerate(rows):
            self.tree.move(row, "", index)
        for key, label in self.labels.items():
            self.tree.heading(key, text=label + (" ▼" if self.reverse else " ▲") if key == self.sort_column else label)

    def select_all(self, _=None):
        self.tree.selection_set(self.tree.get_children())
        return "break"

    def up(self):
        self.load(str(PurePosixPath(self.path).parent))

    def enter(self, event):
        row = self.tree.identify_row(event.y)
        if not self.busy and row in self.entries and self.entries[row].directory:
            self.load(str(PurePosixPath(self.path) / self.entries[row].name))

    def load(self, path):
        if self.busy or self.closing or self.queued_start:
            return
        if self.transport.pending:
            self.queued_path = path
            self.update_controls()
            return
        try:
            encoded = encode_path(path)
        except ValueError as exc:
            self.note.set(str(exc))
            return
        self.busy = True
        self.path, self.entries = path, {}
        self.releasing_after_error = False
        self.path_text.set(path)
        self.tree.delete(*self.tree.get_children())
        self.note.set("正在读取文件列表…")
        self.progress["value"] = 0
        self.update_controls()
        self.transport.request("LIST", (encoded,), self.list_reply)

    def list_reply(self, kind, fields):
        if kind == "SD_ENTRY":
            entry = RemoteEntry.parse(fields)
            if any(e.name == entry.name for e in self.entries.values()):
                raise ValueError("文件列表包含重复名称")
            row = self.tree.insert("", "end", values=(entry.name, "—" if entry.directory else f"{entry.size:,} B",
                                    entry.modified.strftime("%Y-%m-%d %H:%M:%S") if entry.modified else "未知", entry.kind))
            self.entries[row] = entry
            self.note.set(f"正在读取文件列表…已找到 {len(self.entries)} 项")
        elif kind == "SD_END":
            self.busy = False
            self.sort()
            self.note.set(f"共 {len(self.entries)} 项。选择文件后下载；缺少有效时间戳时显示“未知”。浏览期间暂停日志写入。")
            self.update_controls()
            return
        self.transport.request("NEXT", callback=self.list_reply)

    def choose_destination(self):
        if self.busy or self.closing or self.queued_start or self.queued_path is not None:
            return
        selected = set(self.tree.selection())
        entries = [self.entries[i] for i in self.tree.get_children() if i in selected and not self.entries[i].directory]
        if not entries:
            return
        destination = filedialog.askdirectory(title="选择下载文件的保存位置", parent=self.window, mustexist=True)
        if destination and not self.closed and not self.closing and self.station.connected:
            self.queued_start = (entries, destination)
            self.update_controls()

    def tick(self):
        self.transport.tick()
        if self.queued_path is not None and not self.transport.pending and not self.closing:
            path, self.queued_path = self.queued_path, None
            self.load(path)
        if self.queued_start and not self.transport.pending and not self.closing:
            entries, self.destination = self.queued_start
            self.queued_start = None
            self.queue = deque(entries)
            self.completed = self.done_bytes = 0
            self.total_bytes = sum(e.size for e in entries)
            self.total_files = len(entries)
            self.started_at = time.monotonic()
            self.last_progress = 0
            self.busy = True
            self.next_file()

    def next_file(self):
        if not self.queue:
            self.busy = False
            self.progress["value"] = 100
            self.note.set(f"下载完成：{self.completed} 个文件，已通过完整 CRC 校验。保存位置：{self.destination}")
            self.station.status.set(self.note.get())
            self.update_controls()
            return
        self.current_entry = self.queue.popleft()
        try:
            encoded = encode_path(str(PurePosixPath(self.path) / self.current_entry.name))
        except ValueError as exc:
            self.fail(str(exc))
            return
        self.note.set(f"正在打开 {self.current_entry.name}…")
        self.update_controls()
        self.transport.request("OPEN", (encoded,), self.file_reply)

    def file_reply(self, kind, fields):
        if kind == "SD_OPEN":
            if len(fields) != 1:
                raise ValueError("文件大小响应错误")
            self.download = DownloadFile(self.destination, self.current_entry, int(fields[0]))
        elif kind == "SD_DATA":
            self.download.append(fields)
        elif kind == "SD_EOF":
            self.download.finish(fields)
            self.done_bytes += self.download.size
            self.download = None
            self.completed += 1
            self.next_file()
            return
        now = time.monotonic()
        if kind == "SD_OPEN" or now - self.last_progress >= 0.1:
            done = self.done_bytes + self.download.offset
            self.progress["value"] = done * 100 / max(1, self.total_bytes)
            rate = done / max(0.001, now - self.started_at) / 1024
            self.note.set(f"正在下载 {self.completed + 1}/{self.total_files}：{self.current_entry.name} · "
                          f"{self.download.offset:,}/{self.download.size:,} B · {rate:.1f} KB/s · 总进度 {self.progress['value']:.1f}%")
            self.last_progress = now
        args = (str(self.download.offset),)
        if self.block_size > 256:
            args += (str(self.block_size),)
        self.transport.request("READ", args, self.file_reply)

    def clear_download(self):
        self.queue.clear()
        self.queued_start = None
        self.queued_path = None
        if self.download:
            try:
                self.download.abort()
            except OSError as exc:
                message = f"未完成文件清理失败：{exc}"
                self.station.status.set(message)
                return message
            finally:
                self.download = None
        return ""

    def fail(self, message):
        release_failed = self.releasing_after_error
        self.transport.pending = None
        self.transport.alive = False
        cleanup_error = self.clear_download()
        if cleanup_error:
            message += "；" + cleanup_error
        self.busy = False
        self.note.set(f"{message}。已完成的 {self.completed} 个文件保留。")
        self.station.status.set(self.note.get())
        if self.closing:
            self.destroy()
        else:
            # Release logger/file handles even after validation or local disk errors.
            if not release_failed:
                self.releasing_after_error = True
                self.transport.request("CANCEL", callback=lambda *_: self.update_controls())
            self.update_controls()

    def cancel_download(self):
        self.clear_download()
        self.transport.pending = None
        self.busy = True
        self.update_controls()
        self.transport.request("CANCEL", callback=self.cancelled)

    def cancelled(self, *_):
        self.busy = False
        self.load(self.path)
        self.note.set(f"下载已取消，已完成的 {self.completed} 个文件保留；正在刷新列表…")

    def close(self):
        if self.closed or self.closing:
            return
        self.clear_download()
        self.closing = True
        self.transport.pending = None
        self.note.set("正在结束 SD 会话…")
        self.update_controls()
        self.transport.request("CANCEL", callback=lambda *_: self.destroy())

    def destroy(self):
        if self.closed:
            return
        self.clear_download()
        self.closed = True
        self.transport.pending = None
        self.transport.alive = False
        self.window.destroy()
        self.station.sd_browser = None
        self.station._update_controls()
