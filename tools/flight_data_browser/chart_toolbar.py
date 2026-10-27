"""Report explicit toolbar actions without treating redraws as user actions."""
import tkinter as tk

from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk


class ChartToolbar(NavigationToolbar2Tk):
    toolitems = (
        ("Back", "上一视图", "back", "back"),
        ("Forward", "下一视图", "forward", "forward"),
        ("Home", "恢复初始视图", "home", "home"),
        (None, None, None, None),
        ("Zoom", "框选缩放：左键放大，右键缩小", "zoom_to_rect", "zoom"),
        ("Subplots", "设置图表布局", "subplots", "configure_subplots"),
        ("Save", "保存图片", "filesave", "save_figure"),
        ("CSV", "导出当前子图所选变量为 CSV", None, "export_csv"),
    )

    def __init__(self, canvas, window, on_action, on_export):
        self.on_action = on_action
        self.on_export = on_export
        super().__init__(canvas, window)

    def _Button(self, text, image_file, toggle, command):
        if text != "CSV":
            return super()._Button(text, image_file, toggle, command)
        button = tk.Button(self, text="CSV", font="TkDefaultFont", command=command, width=3,
                           padx=5, pady=3, relief="flat", overrelief="groove", borderwidth=1)
        button._image_file = None
        button.pack(side=tk.LEFT)
        return button

    def export_csv(self):
        self.on_export()

    def style_mode_buttons(self):
        from ui_style import COLORS
        for name in ("Zoom",):
            button = self._buttons.get(name)
            if button is None:
                continue
            selected = bool(button.var.get())
            border = COLORS["accent"] if selected else COLORS["surface"]
            button.configure(
                background=COLORS["selected"] if selected else COLORS["surface"],
                selectcolor=COLORS["selected"], activebackground=COLORS["selected"],
                relief="sunken" if selected else "flat", offrelief="flat",
                overrelief="sunken" if selected else "groove", borderwidth=1,
                highlightthickness=1, highlightbackground=border, highlightcolor=border)

    def _update_buttons_checked(self):
        super()._update_buttons_checked()
        self.style_mode_buttons()

    def pan(self, *args):
        # Panning is a direct left-drag gesture, without a separate tool mode.
        return

    def zoom(self, *args):
        previous = self.mode
        super().zoom(*args)
        if self.mode != previous:
            self.on_action("启用 Zoom 框选缩放" if self.mode else "关闭 Zoom 框选缩放")

    def home(self, *args):
        super().home(*args)
        self.on_action("恢复初始视图")

    def back(self, *args):
        super().back(*args)
        self.on_action("返回上一视图")

    def forward(self, *args):
        super().forward(*args)
        self.on_action("前进到下一视图")
