"""Coalesce Tk resize events before asking Matplotlib to render again."""
from types import SimpleNamespace

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class ChartCanvas(FigureCanvasTkAgg):
    resize_delay_ms = 100

    def __init__(self, *args, **kwargs):
        self._resize_job = None
        self._resize_size = None
        super().__init__(*args, **kwargs)

    def resize(self, event):
        if event.width <= 0 or event.height <= 0:
            return
        self._resize_size = (event.width, event.height)
        if self._resize_job is not None:
            self._tkcanvas.after_cancel(self._resize_job)
        if self._idle_draw_id is not None:
            self._tkcanvas.after_cancel(self._idle_draw_id)
            self._idle_draw_id = None
        self._resize_job = self._tkcanvas.after(self.resize_delay_ms, self._finish_resize)

    def _finish_resize(self):
        self._resize_job = None
        width, height = self._resize_size
        if (width, height) == self.get_width_height(physical=True):
            self.draw_idle()
        else:
            super().resize(SimpleNamespace(width=width, height=height))

    def draw_idle(self):
        if self._resize_job is None:
            super().draw_idle()

    def cancel_pending_draws(self):
        for name in ("_resize_job", "_idle_draw_id"):
            job = getattr(self, name, None)
            if job is not None:
                self._tkcanvas.after_cancel(job)
                setattr(self, name, None)
