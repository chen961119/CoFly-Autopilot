"""Shared desktop and chart styling for the flight-data browser."""
from tkinter import font, ttk


COLORS = {
    "background": "#f1f5f9", "surface": "#ffffff", "subtle": "#f8fafc",
    "text": "#1e293b", "muted": "#64748b", "border": "#dbe3ee",
    "accent": "#2563eb", "selected": "#e8f0ff", "danger": "#b84646",
}
CURVE_COLORS = ("#2563eb", "#0d9488", "#e18a24", "#8b5cf6",
                "#d45178", "#0891b2", "#7c9629", "#a56843")


def apply_theme(root):
    families = set(font.families(root))
    family = next((name for name in ("Microsoft YaHei UI", "Microsoft YaHei", "Segoe UI", "Noto Sans CJK SC")
                   if name in families), font.nametofont("TkDefaultFont", root=root).actual("family"))
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont"):
        font.nametofont(name, root=root).configure(family=family, size=10)
    style = ttk.Style(root)
    style.theme_use("clam")
    c = COLORS
    root.configure(background=c["background"])
    style.configure(".", font=(family, 10), background=c["background"], foreground=c["text"])
    style.configure("TFrame", background=c["background"])
    style.configure("Card.TFrame", background=c["surface"], bordercolor=c["border"],
                    lightcolor=c["surface"], darkcolor=c["surface"], borderwidth=1, relief="solid")
    style.configure("Surface.TFrame", background=c["surface"])
    style.configure("TLabel", background=c["surface"], foreground=c["text"])
    style.configure("Title.TLabel", background=c["background"], font=(family, 17, "bold"))
    style.configure("Section.TLabel", font=(family, 10, "bold"))
    style.configure("Muted.TLabel", foreground=c["muted"], font=(family, 9))
    style.configure("Count.TLabel", background=c["selected"], foreground=c["accent"], padding=(10, 5), font=(family, 9))
    style.configure("Status.TLabel", background="#e8eef5", foreground=c["muted"], padding=(16, 7), font=(family, 9))
    style.configure("TButton", padding=(9, 6), width=0, font=(family, 9), background=c["subtle"],
                    bordercolor=c["border"], lightcolor=c["subtle"], darkcolor=c["subtle"],
                    foreground=c["text"], borderwidth=1, relief="flat", focuscolor=c["accent"])
    style.map("TButton", background=[("pressed", "#dae6f8"), ("active", c["selected"])],
              foreground=[("disabled", "#94a3b8")], bordercolor=[("active", "#b1c9ed")])
    style.configure("Primary.TButton", background=c["accent"], foreground="white", bordercolor=c["accent"],
                    lightcolor=c["accent"], darkcolor=c["accent"])
    style.map("Primary.TButton", background=[("pressed", "#1e40af"), ("active", "#1d4ed8")],
              foreground=[("disabled", "#cbd5e1"), ("!disabled", "white")])
    style.configure("Danger.TButton", foreground=c["danger"])
    style.map("Danger.TButton", background=[("active", "#fff1f2")])
    for widget in ("TEntry", "TSpinbox", "TCombobox"):
        style.configure(widget, padding=5, fieldbackground=c["surface"], background=c["subtle"],
                        foreground=c["text"], bordercolor=c["border"], lightcolor=c["surface"],
                        darkcolor=c["surface"], insertcolor=c["text"], arrowsize=12)
        style.map(widget, bordercolor=[("focus", c["accent"])], fieldbackground=[("readonly", c["subtle"])],
                  selectbackground=[("!disabled", c["selected"])], selectforeground=[("!disabled", c["text"])])
    # Clam uses white text for readonly + focus; our light field needs dark text
    # when the popup closes without selecting another subplot, too.
    style.map("TCombobox", foreground=[("disabled", c["muted"]), ("!disabled", c["text"])])
    style.configure("Search.TEntry", padding=(5, 5, 32, 5))
    style.configure("TMenubutton", padding=(6, 4), background=c["subtle"], foreground=c["text"],
                    bordercolor=c["border"], lightcolor=c["surface"], darkcolor=c["surface"],
                    arrowsize=8, arrowpadding=2)
    style.map("TMenubutton", background=[("active", c["selected"])],
              foreground=[("disabled", c["muted"]), ("!disabled", c["text"])])
    style.configure("TCheckbutton", background=c["surface"], foreground=c["text"], font=(family, 9), padding=3)
    style.map("TCheckbutton", background=[("active", c["surface"])])
    style.configure("Treeview", background=c["surface"], fieldbackground=c["surface"],
                    foreground=c["text"], rowheight=28, borderwidth=0, font=(family, 9))
    style.map("Treeview", background=[("selected", c["selected"])], foreground=[("selected", "#1d4ed8")])
    style.configure("Vertical.TScrollbar", background="#cbd5e1", troughcolor=c["surface"],
                    bordercolor=c["surface"], lightcolor=c["surface"], darkcolor=c["surface"], arrowsize=11)
    root.option_add("*TCombobox*Listbox.font", (family, 10))
    return family


def style_axes(ax, title, active=False, dense=None):
    c = COLORS
    if dense is None:
        grid = ax.get_subplotspec().get_gridspec()
        dense = grid.nrows * grid.ncols > 4
    ax.set_facecolor(c["surface"])
    ax.set_axisbelow(True)
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines["left"].set_color(c["accent"] if active else c["border"])
    ax.spines["left"].set_linewidth(1.3 if active else 0.8)
    ax.spines["bottom"].set_color(c["border"])
    ax.spines["bottom"].set_linewidth(0.8)
    ax.tick_params(axis="both", labelsize=7 if dense else 9, labelcolor=c["muted"],
                   color=c["border"], length=2 if dense else 3, pad=3 if dense else 5)
    ax.grid(axis="y", color="#e6edf5", linewidth=0.7)
    ax.grid(axis="x", color="#f0f4f9", linewidth=0.7)
    title_color = c["accent"] if active else c["text"]
    if dense:
        ax.text(0.012, 0.95, title, transform=ax.transAxes, va="top", fontsize=8,
                fontweight="bold", color=title_color, gid="panel-title")
    else:
        ax.set_title(title, loc="left", fontsize=10, fontweight="bold", color=title_color, pad=10)
    ax.xaxis.label.set_color(c["muted"])
    ax.yaxis.label.set_color(c["muted"])
    ax.xaxis.label.set_size(8 if dense else 9)
    ax.yaxis.label.set_size(8 if dense else 9)
    ax.margins(x=0.015, y=0.1)


def style_legend(ax, dense=None):
    if dense is None:
        grid = ax.get_subplotspec().get_gridspec()
        dense = grid.nrows * grid.ncols > 4
    legend = ax.legend(fontsize=7 if dense else 8, loc="upper right", frameon=True, fancybox=True,
                       framealpha=0.94, facecolor="white", edgecolor=COLORS["border"],
                       borderpad=0.6, labelspacing=0.4, handlelength=2.4)
    for text in legend.get_texts():
        text.set_color(COLORS["text"])
    return legend
