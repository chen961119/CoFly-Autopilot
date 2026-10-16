"""Nine-step radio calibration. Samples stay on the PC until explicit SD save."""
import math
import time
from collections import deque
import tkinter as tk
from tkinter import font, messagebox, ttk

STEPS = (
    "将油门杆完全向下拉。",
    "将油门杆向上推到底并保持不动。",
    "将油门杆向下拉到底并保持不动。",
    "将偏航杆向右推到底并保持不动。",
    "将偏航杆向左推到底并保持不动。",
    "将滚转杆向右推到底并保持不动。",
    "将滚转杆向左推到底并保持不动。",
    "将俯仰杆向下拉到底并保持不动。",
    "将俯仰杆向上推到底并保持不动。",
)
CHANNELS = (3, 3, 3, 4, 4, 1, 1, 2, 2)
ENDPOINT_LOW = 1200
ENDPOINT_HIGH = 1800


class RadioCalibration:
    def __init__(self):
        self.step = 0
        self.samples = deque()
        self.positions = []
        self.last_ms = None
        self.signal_invalid_since = None

    def feed(self, data, now):
        if data.get("locked") != 1:
            raise ValueError("飞机已解除锁定，校准已取消")
        if data.get("radio_valid") != 1:
            # A single SBUS lost-frame flag is not a sustained disconnect.
            # Discard the hold window so bad or pre-dropout samples cannot
            # qualify after recovery, but keep the current calibration step.
            self.samples.clear()
            self.last_ms = None
            if self.signal_invalid_since is None:
                self.signal_invalid_since = now
            if now - self.signal_invalid_since >= 2:
                raise ValueError("遥控接收机连续两秒无有效信号，校准已取消")
            return False
        self.signal_invalid_since = None
        values = tuple(data[f"rc{i}"] for i in range(1, 5))
        if any(not math.isfinite(v) or not 800 <= v <= 2200 for v in values):
            raise ValueError("遥控通道超出有效范围，校准已取消")
        ms = data["ms"]
        if self.last_ms is not None:
            delta = (int(ms) - int(self.last_ms)) & 0xffffffff
            if delta == 0:
                self.samples.clear()
                return True
            if delta > 2000:
                self.samples.clear()
        self.last_ms = ms
        if self.samples and now - self.samples[-1][0] > 0.5:
            self.samples.clear()
        self.samples.append((now, values))
        while len(self.samples) > 1 and now - self.samples[1][0] >= 1:
            self.samples.popleft()
        return True

    def stable(self):
        if len(self.samples) < 6 or self.samples[-1][0] - self.samples[0][0] < 1:
            return False
        channels = range(4) if self.step == 0 else (CHANNELS[self.step] - 1,)
        return all(max(v[c] for _, v in self.samples) - min(v[c] for _, v in self.samples) <= 12
                   for c in channels)

    def capture_position(self):
        """Use the same position checks for button readiness and capture."""
        if not self.stable():
            raise ValueError("请在指定位置保持稳定至少一秒")
        position = tuple(round(sum(v[c] for _, v in self.samples) / len(self.samples)) for c in range(4))
        c = CHANNELS[self.step] - 1
        if ENDPOINT_LOW < position[c] < ENDPOINT_HIGH:
            raise ValueError("尚未到达端点范围，请将指定摇杆推到底")
        if self.step == 0 and any(not 1300 <= position[i] <= 1700 for i in (0, 1, 3)):
            raise ValueError("请将滚转、俯仰和偏航杆放在中位")
        if self.step == 2 and abs(position[c] - self.positions[0][c]) > 30:
            raise ValueError("两次油门最低位置不一致，请将油门杆向下拉到底")
        if self.step in (1, 4, 6, 8) and abs(position[c] - self.positions[-1][c]) < 400:
            raise ValueError("两端行程不足 400 μs，请确认摇杆已推到底且通道顺序为滚转、俯仰、油门、偏航")
        if self.step in (3, 5, 7) and abs(position[c] - self.positions[0][c]) < 200:
            raise ValueError("摇杆行程不足，请将指定摇杆推到底")
        if self.step in (4, 6, 8) and (position[c] - self.positions[0][c]) * (self.positions[-1][c] - self.positions[0][c]) >= 0:
            raise ValueError("两端必须位于中位两侧，请检查摇杆方向")
        return position

    def readiness(self):
        try:
            self.capture_position()
        except ValueError as exc:
            return False, str(exc)
        return True, "位置和行程检查通过，已稳定，可以点击下一步"

    def capture(self):
        position = self.capture_position()
        self.positions.append(position)
        self.step += 1
        self.samples.clear()

    def parameters(self):
        if self.step != 9:
            raise ValueError("九步校准尚未完成")
        result = []
        # Pitch pull-back (step 8) is positive; push-forward (step 9) is negative.
        for c, positive, negative in ((0, 5, 6), (1, 7, 8), (2, 1, 2), (3, 3, 4)):
            high, low = self.positions[positive][c], self.positions[negative][c]
            minimum, maximum = sorted((high, low))
            trim = round((minimum + maximum) / 2) if c == 2 else self.positions[0][c]
            if not (800 <= minimum <= 1500 <= maximum <= 2200 and
                    trim - minimum > 100 and maximum - trim > 100):
                raise ValueError(f"CH{c + 1} 的端点或中位无效，请重新校准")
            result.extend((minimum, maximum, trim, 1 if high > low else -1))
        return tuple(str(v) for v in result)


class RadioCalibrationDialog:
    def __init__(self, station):
        self.station = station
        self.model = RadioCalibration()
        self.closed = False
        self.window = tk.Toplevel(station.root)
        self.window.title("遥控器校准")
        # Size once for the selected font, then keep it fixed while calibrating.
        scale = max(1, abs(font.nametofont("TkDefaultFont").cget("size")) / 11)
        width, height = round(520 * scale), round(220 * scale)
        self.window.geometry(f"{width}x{height}")
        self.window.resizable(False, False)
        self.window.transient(station.root)
        self.window.protocol("WM_DELETE_WINDOW", self.cancel)
        self.window.grab_set()
        body = ttk.Frame(self.window, padding=14)
        body.pack(fill="both", expand=True)
        self.prompt = tk.StringVar()
        ttk.Label(body, textvariable=self.prompt, wraplength=width - 32).pack(anchor="w", pady=(0, 8))
        self.note = tk.StringVar(value="等待实时遥控数据…")
        ttk.Label(body, textvariable=self.note, wraplength=width - 32).pack(anchor="w", pady=(0, 8))
        actions = ttk.Frame(body)
        actions.pack(side="bottom", fill="x")
        self.next = ttk.Button(actions, text="下一步", command=self.advance, state="disabled")
        self.next.pack(side="left")
        ttk.Button(actions, text="取消校准", command=self.cancel).pack(side="right")
        self.show_step()

    def show_step(self):
        extra = "\n请将滚转、俯仰和偏航杆放在中位，以采集中位值。" if self.model.step == 0 else ""
        self.prompt.set(f"第 {self.model.step + 1} / 9 步：{STEPS[self.model.step]}{extra}\n保持稳定至少一秒后，单击“下一步”。")
        self.next.configure(state="disabled")
        self.note.set("正在等待摇杆到达指定位置并保持稳定…")

    def feed(self, data, now):
        if self.closed:
            return
        try:
            signal_valid = self.model.feed(data, now)
        except (ValueError, KeyError) as exc:
            self.cancel(str(exc))
            return
        if not signal_valid:
            self.next.configure(state="disabled")
            self.note.set("遥控信号暂时无效，正在等待恢复；请保持当前步骤的位置。")
            return
        ready, reason = self.model.readiness()
        self.next.configure(state="normal" if ready else "disabled")
        channel = CHANNELS[self.model.step]
        self.note.set(f"CH{channel}：{data[f'rc{channel}']:.0f} μs · {reason}")

    def advance(self):
        if self.closed:
            return
        if self.station._defer_for_poll(self.advance):
            return
        if self.station.gcs_pending or self.station.pending:
            return
        if not self.station.connected or self.station.last_data == 0 or time.monotonic() - self.station.last_data > 0.5:
            self.cancel("遥控数据已过期，校准已取消")
            return
        try:
            self.model.capture()
            if self.model.step < 9:
                self.show_step()
                return
            values = self.model.parameters()
        except ValueError as exc:
            if self.model.step == 9:
                self.cancel(str(exc))
            else:
                self.note.set(str(exc))
            return
        self.closed = True
        self.window.destroy()
        if messagebox.askyesno("遥控器校准", "所有校准工作已完成，是否将新参数写入sd卡参数中？", parent=self.station.root):
            if self.station.connected and not self.station.pending and not self.station.gcs_pending and self.station.file_import is None:
                self.station.save_radio_calibration(values)
            else:
                self.station.status.set("连接状态已变化，未发送校准保存请求；请重新校准")
        else:
            self.station.status.set("校准已完成，未保存；原参数保持不变")
        self.station.radio_calibration = None
        self.station._update_controls()

    def cancel(self, reason="校准已取消，原参数保持不变"):
        if not self.closed:
            self.closed = True
            self.window.destroy()
        self.station.radio_calibration = None
        self.station.status.set(reason)
        self.station._update_controls()
