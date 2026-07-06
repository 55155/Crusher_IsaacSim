"""
Tablet Shape Viewer — desktop app
Dependencies:  pip install pyvista pyvistaqt PyQt5

Run:    python viewer_desktop.py
"""

import os
import sys
import subprocess

# ── PyQt5 ────────────────────────────────────────────────────────
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout, QVBoxLayout, QGridLayout,
    QSlider, QLabel, QFrame, QSizePolicy,
    QLineEdit,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor, QPalette

# ── PyVista ──────────────────────────────────────────────────────
import pyvista as pv
from pyvistaqt import QtInteractor
import numpy as np
import trimesh

# ────────────────────────────────────────────────────────────────
# Script location (Codes/) → paths relative to parent (tablets_stl/)
_HERE   = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.normpath(os.path.join(_HERE, "..", "stl"))

RADII   = [4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5]
ASPECTS = [1.0, 1.17, 1.33, 1.5, 1.67, 1.83, 2.0, 2.17, 2.33, 2.5]
CURVS   = [0.08, 0.11, 0.14, 0.17, 0.20, 0.23, 0.26, 0.29, 0.32, 0.35]

AR_DESCS = ['Circular', 'Near-circular ellipse', 'Ellipse', 'Elliptical', 'Gentle ellipse',
            'Elliptical caplet', 'Caplet', 'Long caplet', 'Very long caplet', 'Oblong']
CV_DESCS = ['Very flat', 'Flat', 'Standard flat', 'Standard convex', 'Standard',
            'Regular convex', 'Convex', 'Strong convex', 'Very convex', 'Spherical']

# ── Density-based contact hardness constants (synced with crusher_tablet_sim.py) ──
DENSITY_REF_SOFT    = 900.0    # kg/m³  soft reference
DENSITY_REF_HARD    = 1800.0   # kg/m³  hard reference
SOLREF_TAU_SOFT     = 0.020    # s
SOLREF_TAU_HARD     = 0.002    # s
BICONVEX_VOL_FACTOR = 0.82


def _estimate_volume_mm3(R_mm, AR, CV):
    """Estimate biconvex tablet volume [mm³]."""
    import math
    cd = CV * 2.0 * R_mm
    th = R_mm * 0.20 + 2.0 * cd
    return (4.0 / 3.0) * math.pi * (R_mm * AR) * R_mm * (th / 2.0) * BICONVEX_VOL_FACTOR


def _mass_to_density(mass_mg, R_mm, AR, CV):
    """Mass (mg) + shape → density [kg/m³]."""
    vol_m3 = _estimate_volume_mm3(R_mm, AR, CV) * 1e-9
    return (mass_mg * 1e-6) / vol_m3


def _density_to_tau(rho):
    """Density → MuJoCo solref time constant τ [s]  (Power-law, Hertzian)."""
    import math
    rho   = max(DENSITY_REF_SOFT, min(DENSITY_REF_HARD, rho))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    tau   = SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha
    return max(SOLREF_TAU_HARD, min(SOLREF_TAU_SOFT, tau))


# ── Color constants (achromatic / grayscale) ──────────────────────
C_BG     = "#0d1117"
C_PANEL  = "#161b22"
C_BORDER = "#21262d"
C_TEXT   = "#e6edf3"
C_MUTED  = "#8b949e"
# Accent colors — grayscale replacements (formerly blue/green/orange/purple)
C_BLUE   = "#d0d7de"   # R      accent
C_GREEN  = "#b1bac4"   # AR     accent
C_ORANGE = "#9aa5b1"   # CV     accent
C_PURPLE = "#adbac7"   # generic accent
# Buttons — neutral gray scheme
C_BTN     = "#30363d"
C_BTN_HOV = "#484f58"
C_BTN_PRS = "#21262d"


# ── Style utilities ────────────────────────────────────────────
def label(text, size=12, color=C_TEXT, bold=False):
    w = QLabel(text)
    w.setStyleSheet(
        f"color:{color}; font-size:{size}px;"
        f"{'font-weight:700;' if bold else ''}"
        "background:transparent;"
    )
    return w


def divider():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color:{C_BORDER};")
    return line


def styled_slider(accent_color):
    s = QSlider(Qt.Horizontal)
    s.setRange(0, 9)
    s.setSingleStep(1)
    s.setPageStep(1)
    s.setFixedHeight(28)
    s.setStyleSheet(f"""
        QSlider::groove:horizontal {{
            height: 4px;
            background: {C_BORDER};
            border-radius: 2px;
        }}
        QSlider::sub-page:horizontal {{
            background: {accent_color};
            border-radius: 2px;
        }}
        QSlider::handle:horizontal {{
            background: {accent_color};
            width: 18px; height: 18px;
            margin: -7px 0;
            border-radius: 9px;
            border: 2px solid {C_PANEL};
        }}
        QSlider::handle:horizontal:hover {{
            border: 3px solid {accent_color};
            background: white;
        }}
    """)
    return s


def dim_card(title, val_id):
    """Dimension display card → (card widget, value QLabel)"""
    card = QWidget()
    card.setStyleSheet(
        f"background:{C_BG}; border:1px solid {C_BORDER}; border-radius:8px;"
    )
    layout = QVBoxLayout(card)
    layout.setContentsMargins(12, 8, 12, 8)
    layout.setSpacing(2)

    lbl_title = QLabel(title.upper())
    lbl_title.setStyleSheet(f"color:{C_MUTED}; font-size:10px; letter-spacing:1px; background:transparent; border:none;")
    lbl_val = QLabel("—")
    lbl_val.setObjectName(val_id)
    lbl_val.setStyleSheet(f"color:{C_TEXT}; font-size:16px; font-weight:700; background:transparent; border:none;")

    layout.addWidget(lbl_title)
    layout.addWidget(lbl_val)
    return card, lbl_val


# ── Main window ────────────────────────────────────────────────
class TabletViewer(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tablet Shape Viewer")
        self.resize(1280, 760)
        self._first_load    = True   # reset camera only once
        self._last_density  = None   # density computed from mass input (passed to Crusher)
        self._sim_proc      = None   # running simulation process
        self._setup_palette()
        self._build_ui()
        self._update()

    # ── Palette (app-wide dark theme) ──────────────────────────
    def _setup_palette(self):
        pal = QPalette()
        pal.setColor(QPalette.Window,          QColor(C_BG))
        pal.setColor(QPalette.WindowText,      QColor(C_TEXT))
        pal.setColor(QPalette.Base,            QColor(C_PANEL))
        pal.setColor(QPalette.AlternateBase,   QColor(C_BG))
        pal.setColor(QPalette.ToolTipBase,     QColor(C_BG))
        pal.setColor(QPalette.ToolTipText,     QColor(C_TEXT))
        pal.setColor(QPalette.Text,            QColor(C_TEXT))
        pal.setColor(QPalette.Button,          QColor(C_PANEL))
        pal.setColor(QPalette.ButtonText,      QColor(C_TEXT))
        pal.setColor(QPalette.Highlight,       QColor(C_BLUE))
        pal.setColor(QPalette.HighlightedText, QColor("#000000"))
        self.setPalette(pal)
        self.setStyleSheet(f"QMainWindow {{ background:{C_BG}; }}")

    # ── UI construction ─────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        root.setStyleSheet(f"background:{C_BG};")
        self.setCentralWidget(root)

        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # ── Left panel ─────────────────────────────────────────
        panel = QWidget()
        panel.setFixedWidth(290)
        panel.setStyleSheet(
            f"background:{C_PANEL}; border-right:1px solid {C_BORDER};"
        )
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(22, 26, 22, 26)
        pl.setSpacing(0)

        # Title
        pl.addWidget(label("Pharmaceutical Design", 9, C_BLUE, True))
        pl.addSpacing(4)
        title = QLabel("Tablet Shape\nViewer")
        title.setStyleSheet(f"color:{C_TEXT}; font-size:22px; font-weight:700; background:transparent;")
        pl.addWidget(title)
        pl.addSpacing(20)
        pl.addWidget(divider())
        pl.addSpacing(18)

        # ── 3 sliders ───────────────────────────────────────────
        pl.addWidget(label("PARAMETERS", 9, C_MUTED, True))
        pl.addSpacing(14)

        sliders_info = [
            ("R",  "Minor radius",  C_BLUE,   4, "val_r",  "desc_r"),
            ("AR", "Aspect ratio",  C_GREEN,  3, "val_ar", "desc_ar"),
            ("CV", "Curvature",     C_ORANGE, 2, "val_cv", "desc_cv"),
        ]
        self.sliders = {}
        self.val_labels = {}
        self.desc_labels = {}

        for param, sub, color, default, vid, did in sliders_info:
            # Header row
            row = QWidget()
            row.setStyleSheet("background:transparent;")
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)

            name_lbl = QLabel(f"{param}  <span style='color:{C_MUTED};font-weight:400;font-size:11px'>{sub}</span>")
            name_lbl.setStyleSheet(f"color:{color}; font-size:13px; font-weight:700; background:transparent;")
            name_lbl.setTextFormat(Qt.RichText)

            val_lbl = QLabel("—")
            val_lbl.setStyleSheet(f"color:{color}; font-size:20px; font-weight:700; background:transparent;")
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            rl.addWidget(name_lbl)
            rl.addWidget(val_lbl)
            pl.addWidget(row)

            # Slider
            sl = styled_slider(color)
            sl.setValue(default)
            sl.valueChanged.connect(self._update)
            pl.addWidget(sl)

            # Description
            desc_lbl = QLabel("")
            desc_lbl.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
            pl.addWidget(desc_lbl)
            pl.addSpacing(16)

            self.sliders[param]      = sl
            self.val_labels[param]   = val_lbl
            self.desc_labels[param]  = desc_lbl

        pl.addWidget(divider())
        pl.addSpacing(18)

        # ── Dimension cards ──────────────────────────────────────
        pl.addWidget(label("DIMENSIONS", 9, C_MUTED, True))
        pl.addSpacing(12)

        grid_w = QWidget()
        grid_w.setStyleSheet("background:transparent;")
        grid = QGridLayout(grid_w)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(8)

        dims = [
            ("Minor diameter", "d_minor"), ("Major diameter", "d_major"),
            ("Total thickness", "d_thick"), ("Spherical radius",  "d_rs"),
        ]
        self.dim_labels = {}
        for i, (title, key) in enumerate(dims):
            card, val = dim_card(title, key)
            grid.addWidget(card, i // 2, i % 2)
            self.dim_labels[key] = val

        pl.addWidget(grid_w)
        pl.addSpacing(18)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── Hardness (mass input) ─────────────────────────────────
        pl.addWidget(label("HARDNESS", 9, C_MUTED, True))
        pl.addSpacing(10)

        # Mass input row
        mass_row = QWidget(); mass_row.setStyleSheet("background:transparent;")
        mass_rl  = QHBoxLayout(mass_row); mass_rl.setContentsMargins(0,0,0,0); mass_rl.setSpacing(6)
        mass_rl.addWidget(label("Mass", 11, C_TEXT))
        self.edit_mass = QLineEdit()
        self.edit_mass.setPlaceholderText("mg")
        self.edit_mass.setMaximumWidth(80)
        self.edit_mass.setStyleSheet(f"""
            QLineEdit {{
                background: {C_BG};
                color: {C_TEXT};
                border: 1px solid {C_BORDER};
                border-radius: 5px;
                padding: 4px 8px;
                font-size: 13px;
                font-weight: 700;
            }}
            QLineEdit:focus {{
                border: 1px solid {C_BLUE};
            }}
        """)
        self.edit_mass.textChanged.connect(self._recalc_hardness)
        mass_rl.addWidget(self.edit_mass)
        mass_rl.addWidget(label("mg", 11, C_MUTED))
        mass_rl.addStretch()
        pl.addWidget(mass_row)
        pl.addSpacing(6)

        # Density / τ display row
        self.lbl_density = label("Density  —", 11, C_MUTED)
        self.lbl_tau     = label("τ  —",       11, C_MUTED)
        pl.addWidget(self.lbl_density)
        pl.addWidget(self.lbl_tau)
        pl.addSpacing(8)

        # Hardness gauge (QFrame background + inner fill widget)
        gauge_outer = QFrame()
        gauge_outer.setFixedHeight(12)
        gauge_outer.setStyleSheet(
            f"background:{C_BG}; border:1px solid {C_BORDER}; border-radius:5px;"
        )
        gauge_inner_layout = QHBoxLayout(gauge_outer)
        gauge_inner_layout.setContentsMargins(1,1,1,1)
        gauge_inner_layout.setSpacing(0)
        self._gauge_bar = QWidget()
        self._gauge_bar.setFixedHeight(8)
        self._gauge_bar.setStyleSheet(f"background:{C_MUTED}; border-radius:3px;")
        self._gauge_spacer = QWidget()
        self._gauge_spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        gauge_inner_layout.addWidget(self._gauge_bar)
        gauge_inner_layout.addWidget(self._gauge_spacer)
        pl.addWidget(gauge_outer)
        self._gauge_outer = gauge_outer

        self.lbl_hardness = label("", 10, C_MUTED)
        self.lbl_hardness.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_hardness)

        pl.addSpacing(18)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── Filename / status ────────────────────────────────────
        pl.addWidget(label("STL FILE", 9, C_MUTED, True))
        pl.addSpacing(8)

        self.lbl_fname = QLabel("—")
        self.lbl_fname.setStyleSheet(
            f"color:{C_PURPLE}; font-size:11px; font-family:Consolas;"
            f"background:{C_BG}; border:1px solid {C_BORDER}; border-radius:6px;"
            "padding:7px 10px;"
        )
        self.lbl_fname.setWordWrap(True)
        pl.addWidget(self.lbl_fname)

        self.lbl_status = QLabel("Preparing...")
        self.lbl_status.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_status)

        pl.addSpacing(12)

        # ── MuJoCo launch button ─────────────────────────────────
        from PyQt5.QtWidgets import QPushButton
        self.btn_mujoco = QPushButton("▶  Open in MuJoCo")
        self.btn_mujoco.setEnabled(False)
        self.btn_mujoco.setStyleSheet(f"""
            QPushButton {{
                background: {C_BTN};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 9px;
                font-size: 12px;
                font-weight: 700;
            }}
            QPushButton:hover    {{ background: {C_BTN_HOV}; }}
            QPushButton:pressed  {{ background: {C_BTN_PRS}; }}
            QPushButton:disabled {{ background: {C_BTN_PRS}; color: {C_MUTED}; }}
        """)
        self.btn_mujoco.clicked.connect(self._launch_mujoco)
        pl.addWidget(self.btn_mujoco)

        # ── Crusher simulation buttons (headless + viewer) ───────
        crusher_row = QWidget()
        crusher_row.setStyleSheet("background:transparent;")
        crusher_rl = QHBoxLayout(crusher_row)
        crusher_rl.setContentsMargins(0, 0, 0, 0)
        crusher_rl.setSpacing(6)

        self.btn_crusher_hl = QPushButton("▶  Headless")
        self.btn_crusher_hl.setEnabled(False)
        self.btn_crusher_hl.setStyleSheet(f"""
            QPushButton {{
                background: {C_BTN};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 9px;
                font-size: 12px;
                font-weight: 700;
            }}
            QPushButton:hover    {{ background: {C_BTN_HOV}; }}
            QPushButton:pressed  {{ background: {C_BTN_PRS}; }}
            QPushButton:disabled {{ background: {C_BTN_PRS}; color: {C_MUTED}; }}
        """)
        self.btn_crusher_hl.clicked.connect(lambda: self._launch_crusher_sim("headless"))

        self.btn_crusher_vw = QPushButton("▶  Viewer")
        self.btn_crusher_vw.setEnabled(False)
        self.btn_crusher_vw.setStyleSheet(f"""
            QPushButton {{
                background: {C_BTN};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 9px;
                font-size: 12px;
                font-weight: 700;
            }}
            QPushButton:hover    {{ background: {C_BTN_HOV}; }}
            QPushButton:pressed  {{ background: {C_BTN_PRS}; }}
            QPushButton:disabled {{ background: {C_BTN_PRS}; color: {C_MUTED}; }}
        """)
        self.btn_crusher_vw.clicked.connect(lambda: self._launch_crusher_sim("viewer"))

        crusher_rl.addWidget(self.btn_crusher_hl)
        crusher_rl.addWidget(self.btn_crusher_vw)
        pl.addWidget(crusher_row)

        self.btn_stop = QPushButton("■  Stop simulation")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(f"""
            QPushButton {{
                background: {C_BTN};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px;
                font-size: 12px;
                font-weight: 700;
            }}
            QPushButton:hover    {{ background: {C_BTN_HOV}; }}
            QPushButton:pressed  {{ background: {C_BTN_PRS}; }}
            QPushButton:disabled {{ background: {C_BTN_PRS}; color: {C_MUTED}; }}
        """)
        self.btn_stop.clicked.connect(self._stop_crusher_sim)
        pl.addWidget(self.btn_stop)

        self.lbl_mujoco = QLabel("")
        self.lbl_mujoco.setStyleSheet(f"color:{C_MUTED}; font-size:10px; background:transparent;")
        self.lbl_mujoco.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_mujoco)

        pl.addSpacing(18)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── Convex decomposition section ──────────────────────────
        pl.addWidget(label("COLLISION MESH", 9, C_MUTED, True))
        pl.addSpacing(10)

        from PyQt5.QtWidgets import QPushButton
        self.btn_decomp = QPushButton("Run convex decomposition")
        self.btn_decomp.setStyleSheet(f"""
            QPushButton {{
                background: {C_BTN};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px;
                font-size: 12px;
                font-weight: 600;
            }}
            QPushButton:hover {{ background: {C_BTN_HOV}; }}
            QPushButton:pressed {{ background: {C_BTN_PRS}; }}
            QPushButton:disabled {{ background: {C_BTN_PRS}; color: {C_MUTED}; }}
        """)
        self.btn_decomp.clicked.connect(self._run_decomposition)
        pl.addWidget(self.btn_decomp)

        self.btn_export = QPushButton("Export STL")
        self.btn_export.setEnabled(False)
        self.btn_export.setStyleSheet(f"""
            QPushButton {{
                background: {C_BTN};
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px;
                font-size: 12px;
                font-weight: 600;
            }}
            QPushButton:hover {{ background: {C_BTN_HOV}; }}
            QPushButton:pressed {{ background: {C_BTN_PRS}; }}
            QPushButton:disabled {{ background: {C_BTN_PRS}; color: {C_MUTED}; }}
        """)
        self.btn_export.clicked.connect(self._export_hulls)
        pl.addWidget(self.btn_export)
        pl.addSpacing(8)

        self.lbl_decomp = QLabel("Not decomposed")
        self.lbl_decomp.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
        self.lbl_decomp.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_decomp)

        self._hulls = []       # decomposed trimesh list (for export)
        self._hull_fpath = ""  # current decomposed STL path

        pl.addSpacing(18)
        pl.addWidget(divider())
        pl.addSpacing(10)

        # ── Shortcuts panel ──────────────────────────────────────────
        pl.addWidget(label("SHORTCUTS", 9, C_MUTED, True))
        pl.addSpacing(8)

        shortcuts_data = [
            # (key_text, desc_text, is_header)
            ("Mouse",               "",                        True),
            ("Left-drag",           "Rotate",                  False),
            ("Right-drag",          "Pan",                     False),
            ("Scroll",              "Zoom in / out",           False),
            ("Rendering",           "",                        True),
            ("W",                   "★ Toggle wireframe",      False),
            ("S",                   "Solid (surface) render",  False),
            ("Camera",              "",                        True),
            ("R",                   "Reset camera",            False),
            ("F",                   "Focus picked point",      False),
            ("Other",               "",                        True),
            ("Q",                   "Close viewer window",     False),
            ("P",                   "Pick point",              False),
            ("C",                   "Perspective ↔ Parallel",  False),
            ("I",                   "Toggle axes",             False),
        ]

        sc_widget = QWidget()
        sc_widget.setStyleSheet(
            f"background:{C_BG}; border:1px solid {C_BORDER}; border-radius:6px;"
        )
        sc_layout = QVBoxLayout(sc_widget)
        sc_layout.setContentsMargins(10, 8, 10, 8)
        sc_layout.setSpacing(1)

        for key_txt, desc_txt, is_header in shortcuts_data:
            if is_header:
                hdr = QLabel(f"  {key_txt}")
                hdr.setStyleSheet(
                    f"color:{C_BLUE}; font-size:10px; font-weight:700;"
                    f"background:transparent; padding-top:5px;"
                )
                sc_layout.addWidget(hdr)
            else:
                row_w = QWidget()
                row_w.setStyleSheet("background:transparent;")
                row_l = QHBoxLayout(row_w)
                row_l.setContentsMargins(0, 0, 0, 0)
                row_l.setSpacing(4)

                is_star = "★" in desc_txt
                key_color  = C_ORANGE if is_star else C_MUTED
                desc_color = C_TEXT   if is_star else C_MUTED

                lbl_key = QLabel(key_txt)
                lbl_key.setFixedWidth(90)
                lbl_key.setStyleSheet(
                    f"color:{key_color}; font-size:10px; font-family:Consolas;"
                    f"font-weight:700; background:transparent;"
                )
                lbl_desc = QLabel(desc_txt)
                lbl_desc.setStyleSheet(
                    f"color:{desc_color}; font-size:10px; background:transparent;"
                )
                row_l.addWidget(lbl_key)
                row_l.addWidget(lbl_desc)
                sc_layout.addWidget(row_w)

        pl.addWidget(sc_widget)

        pl.addStretch()

        # ── 3D viewport ────────────────────────────────────────────
        self.plotter = QtInteractor(
            parent=root,
            auto_update=False,
        )
        # Viewport background
        self.plotter.set_background("#0d1117", top="#20242b")
        self.plotter.enable_anti_aliasing('ssaa')
        self.plotter.add_axes(interactive=False)

        # ── Custom lighting (stronger shading contrast) ──────────
        # Remove default lights, set manually
        self.plotter.remove_all_lights()

        # Key light: upper-left → strong contrast
        key = pv.Light(position=(8, 12, 10), focal_point=(0, 0, 0),
                       intensity=1.0, color='white')
        key.positional = False
        self.plotter.add_light(key)

        # Fill light: opposite side → avoid pure black
        fill = pv.Light(position=(-6, -4, -5), focal_point=(0, 0, 0),
                        intensity=0.25, color='white')
        fill.positional = False
        self.plotter.add_light(fill)

        # Rim light: from behind → emphasize silhouette
        rim = pv.Light(position=(0, -10, 6), focal_point=(0, 0, 0),
                       intensity=0.2, color='white')
        rim.positional = False
        self.plotter.add_light(rim)

        root_layout.addWidget(panel)
        root_layout.addWidget(self.plotter, stretch=1)

    # ── Hardness calc (mass input → density → τ) ───────────────
    def _recalc_hardness(self):
        """Mass (mg) + current sliders (R, AR, CV) → compute density·τ and update UI."""
        ri = self.sliders["R"].value()
        ai = self.sliders["AR"].value()
        ci = self.sliders["CV"].value()
        R  = RADII[ri]; AR = ASPECTS[ai]; CV = CURVS[ci]

        try:
            mass_mg = float(self.edit_mass.text().strip())
            if mass_mg <= 0:
                raise ValueError
            rho = _mass_to_density(mass_mg, R, AR, CV)
            tau = _density_to_tau(rho)
            self._last_density = rho   # used when launching Crusher

            # Update text
            self.lbl_density.setText(
                f"Density  <span style='color:{C_BLUE};font-weight:700'>"
                f"{rho:.0f} kg/m³</span>"
            )
            self.lbl_density.setTextFormat(Qt.RichText)
            self.lbl_tau.setText(
                f"τ  <span style='color:{C_GREEN};font-weight:700'>"
                f"{tau:.4f} s</span>"
            )
            self.lbl_tau.setTextFormat(Qt.RichText)

            # Update gauge
            ratio = (rho - DENSITY_REF_SOFT) / (DENSITY_REF_HARD - DENSITY_REF_SOFT)
            ratio = max(0.0, min(1.0, ratio))
            pct   = int(ratio * 100)

            if pct < 25:
                color, txt = "#6e7681", f"Soft  ({pct}%)"
            elif pct < 50:
                color, txt = "#8b949e", f"Medium  ({pct}%)"
            elif pct < 75:
                color, txt = "#adbac7", f"Hard  ({pct}%)"
            else:
                color, txt = "#e6edf3", f"Very Hard  ({pct}%)"

            total_w = self._gauge_outer.width() - 4   # exclude padding
            bar_w   = max(4, int(total_w * ratio))
            self._gauge_bar.setFixedWidth(bar_w)
            self._gauge_bar.setStyleSheet(
                f"background:{color}; border-radius:3px;"
            )
            self.lbl_hardness.setText(txt)
            self.lbl_hardness.setStyleSheet(
                f"color:{color}; font-size:10px; background:transparent;"
            )

        except (ValueError, ZeroDivisionError):
            self._last_density = None
            self.lbl_density.setText("Density  —")
            self.lbl_density.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
            self.lbl_tau.setText("τ  —")
            self.lbl_tau.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
            self._gauge_bar.setFixedWidth(4)
            self._gauge_bar.setStyleSheet(f"background:{C_MUTED}; border-radius:3px;")
            self.lbl_hardness.setText("Enter mass")
            self.lbl_hardness.setStyleSheet(
                f"color:{C_MUTED}; font-size:10px; background:transparent;"
            )

    # ── Model update ─────────────────────────────────────────
    def _update(self):
        ri = self.sliders["R"].value()
        ai = self.sliders["AR"].value()
        ci = self.sliders["CV"].value()

        R  = RADII[ri]
        AR = ASPECTS[ai]
        CV = CURVS[ci]

        # Value labels
        self.val_labels["R"].setText(f"{R:.1f} mm")
        self.val_labels["AR"].setText(f"{AR:.2f}")
        self.val_labels["CV"].setText(f"{CV:.2f}")

        # Descriptions
        self.desc_labels["R"].setText(f"Diameter  {R*2:.1f} mm")
        self.desc_labels["AR"].setText(f"{AR_DESCS[ai]}  —  major axis {R*2*AR:.1f} mm")
        self.desc_labels["CV"].setText(CV_DESCS[ci])

        # Derived dimensions
        cd = CV * 2 * R
        bh = R * 0.20
        th = bh + 2 * cd
        Rs = (R*R + cd*cd) / (2*cd)

        self.dim_labels["d_minor"].setText(f"{R*2:.1f} mm")
        self.dim_labels["d_major"].setText(f"{R*2*AR:.1f} mm")
        self.dim_labels["d_thick"].setText(f"{th:.2f} mm")
        self.dim_labels["d_rs"].setText(f"{Rs:.1f} mm")

        # Load file
        fname = f"tablet_R{R:.1f}_AR{AR:.2f}_CV{CV:.2f}.stl"
        fpath = os.path.join(STL_DIR, fname)
        self.lbl_fname.setText(fname)
        self._load_mesh(fpath)

        # Recompute density if mass entered when slider changes
        self._recalc_hardness()

    def _load_mesh(self, fpath):
        self.plotter.clear()
        self.plotter.clear_actors()

        if not os.path.exists(fpath):
            self.lbl_status.setText("✗ File not found — generate the STL first")
            self.lbl_status.setStyleSheet("color:#e6edf3; font-size:11px; background:transparent;")
            self.btn_mujoco.setEnabled(False)
            self.btn_crusher_hl.setEnabled(False)
            self.btn_crusher_vw.setEnabled(False)
            self.plotter.render()
            return

        try:
            mesh = pv.read(fpath)

            # Center align (mesh.center is a tuple → unpack directly)
            cx, cy, cz = mesh.center
            mesh.translate((-cx, -cy, -cz), inplace=True)

            self.plotter.add_mesh(
                mesh,
                color="#d6d6d6",
                ambient=0.08,
                diffuse=0.85,
                specular=0.7,
                specular_power=40,
                smooth_shading=True,
            )

            # ── Scale bar (10mm reference) ──────────────────────
            # 10mm bar below the tablet → real-size comparison
            bar_y = -18.0   # fixed position at bottom (mm)
            bar = pv.Line((-5, bar_y, 0), (5, bar_y, 0))
            self.plotter.add_mesh(bar, color="#adbac7",
                                  line_width=3, render_lines_as_tubes=True)
            self.plotter.add_point_labels(
                [(0, bar_y - 1.5, 0)], ["10 mm"],
                font_size=10, text_color="#adbac7",
                fill_shape=False, always_visible=True,
                shape_opacity=0,
            )

            # Set camera only on first load (fixed after)
            if self._first_load:
                # Fix camera distance for the largest tablet (R=8.5, AR=2.5)
                self.plotter.camera.position = (0, -90, 40)
                self.plotter.camera.focal_point = (0, 0, 0)
                self.plotter.camera.up = (0, 0, 1)
                self._first_load = False

            self.plotter.render()

            self.lbl_status.setText("✓ Loaded")
            self.lbl_status.setStyleSheet(
                "color:#adbac7; font-size:11px; background:transparent;"
            )
            self.btn_mujoco.setEnabled(True)
            self.btn_crusher_hl.setEnabled(True)
            self.btn_crusher_vw.setEnabled(True)
            self.lbl_mujoco.setText("")
        except Exception as e:
            self.lbl_status.setText(f"✗ Error: {e}")
            self.lbl_status.setStyleSheet(
                "color:#e6edf3; font-size:11px; background:transparent;"
            )
            self.btn_mujoco.setEnabled(False)
            self.btn_crusher_hl.setEnabled(False)
            self.btn_crusher_vw.setEnabled(False)

    # ── Convex decomposition ─────────────────────────────────────
    def _run_decomposition(self):
        fpath = os.path.join(STL_DIR, self.lbl_fname.text())
        if not os.path.exists(fpath):
            self.lbl_decomp.setText("✗ STL file not found")
            return

        self.btn_decomp.setEnabled(False)
        self.lbl_decomp.setText("⏳ Decomposing...")
        QApplication.processEvents()

        try:
            # Load original with trimesh
            tm = trimesh.load(fpath, force='mesh')

            # ── Convex decomposition attempt order ──────────────
            # 1st: coacd (pip install coacd)
            # 2nd: trimesh built-in VHACD
            # 3rd: single convex hull (always works)
            hulls = []
            method = ""

            try:
                import coacd
                tm_c = coacd.Mesh(
                    np.array(tm.vertices, dtype=np.float64),
                    np.array(tm.faces,    dtype=np.int32)
                )
                parts = coacd.run_coacd(tm_c, threshold=0.05, max_convex_hull=8)
                hulls = [trimesh.Trimesh(vertices=np.array(v), faces=np.array(f))
                         for v, f in parts]
                method = "CoACD"
            except ImportError:
                pass

            if not hulls:
                try:
                    hulls = trimesh.decomposition.convex_decomposition(
                        tm, maxhulls=6, maxverts=64)
                    if not isinstance(hulls, list):
                        hulls = [hulls]
                    method = "VHACD"
                except Exception:
                    pass

            if not hulls:
                hulls = [tm.convex_hull]
                method = "Single convex hull"

            self._hulls      = hulls
            self._hull_fpath = fpath

            # ── Memory estimate ──────────────────────────────────
            total_verts = sum(len(h.vertices) for h in hulls)
            total_faces = sum(len(h.faces)    for h in hulls)
            # PhysX: ~48 bytes per vertex
            mem_kb = (total_verts * 48) / 1024

            # ── Visualize in viewport ────────────────────────────
            self.plotter.clear_actors()

            # Original mesh (translucent gray wireframe)
            orig = pv.read(fpath)
            cx, cy, cz = orig.center
            orig.translate((-cx, -cy, -cz), inplace=True)
            self.plotter.add_mesh(orig, color="#888888", opacity=0.15,
                                  style='wireframe')

            # Distinct grayscale shade per hull
            HULL_COLORS = ["#e6edf3","#c9d1d9","#adbac7",
                           "#8b949e","#6e7681","#d0d7de",
                           "#9aa5b1","#767d86"]
            for i, hull in enumerate(hulls):
                pv_hull = pv.PolyData(
                    np.array(hull.vertices, dtype=np.float64),
                    np.hstack([
                        np.full((len(hull.faces), 1), 3, dtype=np.int32),
                        np.array(hull.faces, dtype=np.int32),
                    ]).ravel()
                )
                pv_hull.translate((-cx, -cy, -cz), inplace=True)
                self.plotter.add_mesh(
                    pv_hull,
                    color=HULL_COLORS[i % len(HULL_COLORS)],
                    opacity=0.75,
                    smooth_shading=True,
                    ambient=0.1, diffuse=0.8, specular=0.4,
                )

            self.plotter.render()

            # Show stats
            self.lbl_decomp.setText(
                f"✓ {method}  |  {len(hulls)} hulls\n"
                f"Verts {total_verts} / Faces {total_faces}\n"
                f"Est. memory: ~{mem_kb:.1f} KB"
            )
            self.lbl_decomp.setStyleSheet(
                f"color:#adbac7; font-size:11px; background:transparent;"
            )
            self.btn_export.setEnabled(True)

        except Exception as e:
            self.lbl_decomp.setText(f"✗ Error: {e}")
            self.lbl_decomp.setStyleSheet(
                f"color:#e6edf3; font-size:11px; background:transparent;"
            )
        finally:
            self.btn_decomp.setEnabled(True)

    # ── Export hull STL ──────────────────────────────────────────
    def _export_hulls(self):
        if not self._hulls:
            return
        base = os.path.splitext(self._hull_fpath)[0]
        for i, hull in enumerate(self._hulls):
            out = f"{base}_hull{i:02d}.stl"
            hull.export(out)
        self.lbl_decomp.setText(
            self.lbl_decomp.text() + f"\n📁 Saved {len(self._hulls)} STL files"
        )

    # ── Launch MuJoCo viewer ──────────────────────────────────────
    def _launch_mujoco(self):
        fpath = os.path.join(STL_DIR, self.lbl_fname.text())
        if not os.path.exists(fpath):
            self.lbl_mujoco.setText("✗ STL file not found")
            return

        launcher = os.path.join(_HERE, "launch_mujoco.py")
        if not os.path.exists(launcher):
            self.lbl_mujoco.setText("✗ launch_mujoco.py not found")
            return

        # Run as separate process → no PyVista viewer blocking
        import platform
        kwargs = {}
        if platform.system() == "Windows":
            kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE
        subprocess.Popen([sys.executable, launcher, fpath], **kwargs)
        self.lbl_mujoco.setText("✓ MuJoCo viewer running…")
        self.lbl_mujoco.setStyleSheet(
            "color:#adbac7; font-size:10px; background:transparent;"
        )

    # ── Launch Crusher simulation (velocity control) ────────────────────
    def _launch_crusher_sim(self, mode="headless"):
        fpath = os.path.join(STL_DIR, self.lbl_fname.text())
        if not os.path.exists(fpath):
            self.lbl_mujoco.setText("✗ STL file not found")
            return

        script_name = (
            "crusher_velocity_ctrl.py"
            if mode == "headless"
            else "crusher_velocity_ctrl_viewer.py"
        )
        sim_script = os.path.normpath(
            os.path.join(_HERE, "..", "..",
                         "MuJoCo_PlayGround", "20260603", script_name)
        )
        if not os.path.exists(sim_script):
            self.lbl_mujoco.setText(f"✗ {script_name} not found")
            return

        # Stop any already-running process first
        self._stop_crusher_sim(silent=True)

        import platform
        kwargs = {}
        if platform.system() == "Windows":
            kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE

        cmd = [sys.executable, sim_script, fpath]
        if self._last_density is not None:
            cmd += ["--density", f"{self._last_density:.2f}"]

        self._sim_proc = subprocess.Popen(cmd, **kwargs)

        # Update button states while running
        self.btn_crusher_hl.setEnabled(False)
        self.btn_crusher_vw.setEnabled(False)
        self.btn_stop.setEnabled(True)

        label_mode = "Headless" if mode == "headless" else "Viewer"
        if self._last_density is not None:
            tau = _density_to_tau(self._last_density)
            self.lbl_mujoco.setText(
                f"✓ [{label_mode}]  ρ={self._last_density:.0f} kg/m³  τ={tau:.4f}s"
            )
        else:
            self.lbl_mujoco.setText(f"✓ [{label_mode}] Crusher simulation running…")
        self.lbl_mujoco.setStyleSheet(
            "color:#adbac7; font-size:10px; background:transparent;"
        )

    # ── Stop Crusher simulation ───────────────────────────────────────
    def _stop_crusher_sim(self, silent=False):
        if self._sim_proc is not None:
            try:
                self._sim_proc.terminate()
            except Exception:
                pass
            self._sim_proc = None

        # Restore launch buttons based on STL load state
        stl_ok = os.path.exists(os.path.join(STL_DIR, self.lbl_fname.text()))
        self.btn_crusher_hl.setEnabled(stl_ok)
        self.btn_crusher_vw.setEnabled(stl_ok)
        self.btn_stop.setEnabled(False)

        if not silent:
            self.lbl_mujoco.setText("■ Simulation stopped")
            self.lbl_mujoco.setStyleSheet(
                "color:#8b949e; font-size:10px; background:transparent;"
            )

    def closeEvent(self, event):
        self._stop_crusher_sim(silent=True)
        self.plotter.close()
        super().closeEvent(event)


# ── Run ──────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))

    win = TabletViewer()
    win.show()

    sys.exit(app.exec_())
