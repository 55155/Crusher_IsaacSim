"""
Tablet Shape Viewer — 데스크탑 앱
의존성:  pip install pyvista pyvistaqt PyQt5

실행:    python viewer_desktop.py
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
# 스크립트 위치(Codes/) → 상위(tablets_stl/) 기준 상대경로
_HERE   = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.normpath(os.path.join(_HERE, "..", "stl"))

RADII   = [4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5]
ASPECTS = [1.0, 1.17, 1.33, 1.5, 1.67, 1.83, 2.0, 2.17, 2.33, 2.5]
CURVS   = [0.08, 0.11, 0.14, 0.17, 0.20, 0.23, 0.26, 0.29, 0.32, 0.35]

AR_DESCS = ['원형', '원에 가까운 타원', '타원', '타원형', '완만한 타원',
            '타원 캡렛', '캡렛', '긴 캡렛', '매우 긴 캡렛', '장방형']
CV_DESCS = ['매우 편평', '편평', '표준 편평', '표준 볼록', '표준',
            '일반 볼록', '볼록', '강한 볼록', '매우 볼록', '구면형']

# ── 밀도 기반 접촉 경도 상수 (crusher_tablet_sim.py 와 동기화) ──
DENSITY_REF_SOFT    = 900.0    # kg/m³  연질 기준
DENSITY_REF_HARD    = 1800.0   # kg/m³  경질 기준
SOLREF_TAU_SOFT     = 0.020    # s
SOLREF_TAU_HARD     = 0.002    # s
BICONVEX_VOL_FACTOR = 0.82


def _estimate_volume_mm3(R_mm, AR, CV):
    """biconvex 알약 부피 추정 [mm³]."""
    import math
    cd = CV * 2.0 * R_mm
    th = R_mm * 0.20 + 2.0 * cd
    return (4.0 / 3.0) * math.pi * (R_mm * AR) * R_mm * (th / 2.0) * BICONVEX_VOL_FACTOR


def _mass_to_density(mass_mg, R_mm, AR, CV):
    """무게(mg) + 형상 → 밀도 [kg/m³]."""
    vol_m3 = _estimate_volume_mm3(R_mm, AR, CV) * 1e-9
    return (mass_mg * 1e-6) / vol_m3


def _density_to_tau(rho):
    """밀도 → MuJoCo solref 시정수 τ [s]  (Power-law, Hertzian)."""
    import math
    rho   = max(DENSITY_REF_SOFT, min(DENSITY_REF_HARD, rho))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    tau   = SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha
    return max(SOLREF_TAU_HARD, min(SOLREF_TAU_SOFT, tau))


# ── 색상 상수 ──────────────────────────────────────────────────
C_BG     = "#0d1117"
C_PANEL  = "#161b22"
C_BORDER = "#21262d"
C_TEXT   = "#e6edf3"
C_MUTED  = "#8b949e"
C_BLUE   = "#58a6ff"
C_GREEN  = "#3fb950"
C_ORANGE = "#f78166"
C_PURPLE = "#a371f7"


# ── 스타일 유틸 ────────────────────────────────────────────────
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
    """치수 표시 카드 → (카드 위젯, 값 QLabel)"""
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


# ── 메인 윈도우 ────────────────────────────────────────────────
class TabletViewer(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tablet Shape Viewer")
        self.resize(1280, 760)
        self._first_load    = True   # 최초 1회만 카메라 리셋
        self._last_density  = None   # 무게 입력으로 계산된 밀도 (Crusher 전달용)
        self._setup_palette()
        self._build_ui()
        self._update()

    # ── 팔레트 (앱 전체 다크 테마) ──────────────────────────────
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

    # ── UI 구성 ─────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        root.setStyleSheet(f"background:{C_BG};")
        self.setCentralWidget(root)

        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # ── 왼쪽 패널 ──────────────────────────────────────────
        panel = QWidget()
        panel.setFixedWidth(290)
        panel.setStyleSheet(
            f"background:{C_PANEL}; border-right:1px solid {C_BORDER};"
        )
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(22, 26, 22, 26)
        pl.setSpacing(0)

        # 제목
        pl.addWidget(label("Pharmaceutical Design", 9, C_BLUE, True))
        pl.addSpacing(4)
        title = QLabel("Tablet Shape\nViewer")
        title.setStyleSheet(f"color:{C_TEXT}; font-size:22px; font-weight:700; background:transparent;")
        pl.addWidget(title)
        pl.addSpacing(20)
        pl.addWidget(divider())
        pl.addSpacing(18)

        # ── 슬라이더 3개 ────────────────────────────────────────
        pl.addWidget(label("PARAMETERS", 9, C_MUTED, True))
        pl.addSpacing(14)

        sliders_info = [
            ("R",  "단반경",  C_BLUE,   4, "val_r",  "desc_r"),
            ("AR", "형태비",  C_GREEN,  3, "val_ar", "desc_ar"),
            ("CV", "곡률",    C_ORANGE, 2, "val_cv", "desc_cv"),
        ]
        self.sliders = {}
        self.val_labels = {}
        self.desc_labels = {}

        for param, kor, color, default, vid, did in sliders_info:
            # 헤더 행
            row = QWidget()
            row.setStyleSheet("background:transparent;")
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 0, 0, 0)

            name_lbl = QLabel(f"{param}  <span style='color:{C_MUTED};font-weight:400;font-size:11px'>{kor}</span>")
            name_lbl.setStyleSheet(f"color:{color}; font-size:13px; font-weight:700; background:transparent;")
            name_lbl.setTextFormat(Qt.RichText)

            val_lbl = QLabel("—")
            val_lbl.setStyleSheet(f"color:{color}; font-size:20px; font-weight:700; background:transparent;")
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            rl.addWidget(name_lbl)
            rl.addWidget(val_lbl)
            pl.addWidget(row)

            # 슬라이더
            sl = styled_slider(color)
            sl.setValue(default)
            sl.valueChanged.connect(self._update)
            pl.addWidget(sl)

            # 설명
            desc_lbl = QLabel("")
            desc_lbl.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
            pl.addWidget(desc_lbl)
            pl.addSpacing(16)

            self.sliders[param]      = sl
            self.val_labels[param]   = val_lbl
            self.desc_labels[param]  = desc_lbl

        pl.addWidget(divider())
        pl.addSpacing(18)

        # ── 치수 카드 ────────────────────────────────────────────
        pl.addWidget(label("DIMENSIONS", 9, C_MUTED, True))
        pl.addSpacing(12)

        grid_w = QWidget()
        grid_w.setStyleSheet("background:transparent;")
        grid = QGridLayout(grid_w)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(8)

        dims = [
            ("단축 직경", "d_minor"), ("장축 직경", "d_major"),
            ("전체 두께", "d_thick"), ("구면 반경",  "d_rs"),
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

        # ── 경도 (무게 입력) ──────────────────────────────────────
        pl.addWidget(label("HARDNESS  경도", 9, C_MUTED, True))
        pl.addSpacing(10)

        # 무게 입력 행
        mass_row = QWidget(); mass_row.setStyleSheet("background:transparent;")
        mass_rl  = QHBoxLayout(mass_row); mass_rl.setContentsMargins(0,0,0,0); mass_rl.setSpacing(6)
        mass_rl.addWidget(label("무게", 11, C_TEXT))
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

        # 밀도 / τ 표시 행
        self.lbl_density = label("밀도  —", 11, C_MUTED)
        self.lbl_tau     = label("τ  —",    11, C_MUTED)
        pl.addWidget(self.lbl_density)
        pl.addWidget(self.lbl_tau)
        pl.addSpacing(8)

        # 경도 게이지 (QFrame 배경 + 내부 채움 위젯)
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

        # ── 파일명 / 상태 ────────────────────────────────────────
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

        self.lbl_status = QLabel("준비 중...")
        self.lbl_status.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_status)

        pl.addSpacing(12)

        # ── MuJoCo 실행 버튼 ─────────────────────────────────────
        from PyQt5.QtWidgets import QPushButton
        self.btn_mujoco = QPushButton("▶  MuJoCo에서 열기")
        self.btn_mujoco.setEnabled(False)
        self.btn_mujoco.setStyleSheet(f"""
            QPushButton {{
                background: #b45309;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 9px;
                font-size: 12px;
                font-weight: 700;
            }}
            QPushButton:hover    {{ background: #d97706; }}
            QPushButton:pressed  {{ background: #92400e; }}
            QPushButton:disabled {{ background: #21262d; color: {C_MUTED}; }}
        """)
        self.btn_mujoco.clicked.connect(self._launch_mujoco)
        pl.addWidget(self.btn_mujoco)

        # ── Crusher 시뮬레이션 버튼 ──────────────────────────────
        self.btn_crusher = QPushButton("⚙  Crusher 시뮬레이션")
        self.btn_crusher.setEnabled(False)
        self.btn_crusher.setStyleSheet(f"""
            QPushButton {{
                background: #6e40c9;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 9px;
                font-size: 12px;
                font-weight: 700;
            }}
            QPushButton:hover    {{ background: #8957e5; }}
            QPushButton:pressed  {{ background: #553098; }}
            QPushButton:disabled {{ background: #21262d; color: {C_MUTED}; }}
        """)
        self.btn_crusher.clicked.connect(self._launch_crusher_sim)
        pl.addWidget(self.btn_crusher)

        self.lbl_mujoco = QLabel("")
        self.lbl_mujoco.setStyleSheet(f"color:{C_MUTED}; font-size:10px; background:transparent;")
        self.lbl_mujoco.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_mujoco)

        pl.addSpacing(18)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── 볼록 분해 섹션 ────────────────────────────────────────
        pl.addWidget(label("COLLISION MESH", 9, C_MUTED, True))
        pl.addSpacing(10)

        from PyQt5.QtWidgets import QPushButton
        self.btn_decomp = QPushButton("볼록 분해 실행")
        self.btn_decomp.setStyleSheet(f"""
            QPushButton {{
                background: #1f6feb;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px;
                font-size: 12px;
                font-weight: 600;
            }}
            QPushButton:hover {{ background: #388bfd; }}
            QPushButton:pressed {{ background: #1158c7; }}
            QPushButton:disabled {{ background: #21262d; color: {C_MUTED}; }}
        """)
        self.btn_decomp.clicked.connect(self._run_decomposition)
        pl.addWidget(self.btn_decomp)

        self.btn_export = QPushButton("STL 내보내기")
        self.btn_export.setEnabled(False)
        self.btn_export.setStyleSheet(f"""
            QPushButton {{
                background: #238636;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px;
                font-size: 12px;
                font-weight: 600;
            }}
            QPushButton:hover {{ background: #2ea043; }}
            QPushButton:pressed {{ background: #196127; }}
            QPushButton:disabled {{ background: #21262d; color: {C_MUTED}; }}
        """)
        self.btn_export.clicked.connect(self._export_hulls)
        pl.addWidget(self.btn_export)
        pl.addSpacing(8)

        self.lbl_decomp = QLabel("분해 전")
        self.lbl_decomp.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
        self.lbl_decomp.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_decomp)

        self._hulls = []       # 분해된 trimesh 목록 (내보내기용)
        self._hull_fpath = ""  # 현재 분해된 STL 경로

        pl.addSpacing(18)
        pl.addWidget(divider())
        pl.addSpacing(10)

        # ── 단축키 패널 ──────────────────────────────────────────────
        pl.addWidget(label("SHORTCUTS", 9, C_MUTED, True))
        pl.addSpacing(8)

        shortcuts_data = [
            # (key_text, desc_text, is_header)
            ("마우스",              "",                        True),
            ("좌클릭 드래그",       "회전",                    False),
            ("우클릭 드래그",       "팬 (이동)",               False),
            ("스크롤",              "줌 인 / 아웃",            False),
            ("렌더링",              "",                        True),
            ("W",                   "★ 와이어프레임 토글",     False),
            ("S",                   "솔리드(표면) 렌더링",     False),
            ("카메라",              "",                        True),
            ("R",                   "카메라 리셋",             False),
            ("F",                   "선택 점 포커스",          False),
            ("기타",                "",                        True),
            ("Q",                   "뷰어 창 닫기",            False),
            ("P",                   "점 선택 (Pick)",          False),
            ("C",                   "원근 ↔ 평행 투영",        False),
            ("I",                   "좌표축 표시 토글",        False),
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
                key_color  = C_ORANGE if is_star else "#f78166"
                desc_color = C_GREEN  if is_star else C_MUTED

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

        # ── 3D 뷰포트 ────────────────────────────────────────────
        self.plotter = QtInteractor(
            parent=root,
            auto_update=False,
        )
        # 뷰포트 배경
        self.plotter.set_background("#0d1117", top="#1c2333")
        self.plotter.enable_anti_aliasing('ssaa')
        self.plotter.add_axes(interactive=False)

        # ── 커스텀 조명 (음영 대비 강화) ─────────────────────────
        # 기본 조명 제거 후 직접 설정
        self.plotter.remove_all_lights()

        # 주 조명: 좌상단 → 강한 명암 생성
        key = pv.Light(position=(8, 12, 10), focal_point=(0, 0, 0),
                       intensity=1.0, color='white')
        key.positional = False
        self.plotter.add_light(key)

        # 보조 조명: 반대편 → 완전한 검정 방지
        fill = pv.Light(position=(-6, -4, -5), focal_point=(0, 0, 0),
                        intensity=0.25, color='#aac4ff')
        fill.positional = False
        self.plotter.add_light(fill)

        # 림 조명: 뒤에서 → 실루엣 강조
        rim = pv.Light(position=(0, -10, 6), focal_point=(0, 0, 0),
                       intensity=0.2, color='#ffe8c0')
        rim.positional = False
        self.plotter.add_light(rim)

        root_layout.addWidget(panel)
        root_layout.addWidget(self.plotter, stretch=1)

    # ── 경도 계산 (무게 입력 → 밀도 → τ) ───────────────────────
    def _recalc_hardness(self):
        """무게(mg) + 현재 슬라이더(R, AR, CV) → 밀도·τ 계산 후 UI 갱신."""
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
            self._last_density = rho   # Crusher 실행 시 사용

            # 텍스트 갱신
            self.lbl_density.setText(
                f"밀도  <span style='color:{C_BLUE};font-weight:700'>"
                f"{rho:.0f} kg/m³</span>"
            )
            self.lbl_density.setTextFormat(Qt.RichText)
            self.lbl_tau.setText(
                f"τ  <span style='color:{C_GREEN};font-weight:700'>"
                f"{tau:.4f} s</span>"
            )
            self.lbl_tau.setTextFormat(Qt.RichText)

            # 게이지 갱신
            ratio = (rho - DENSITY_REF_SOFT) / (DENSITY_REF_HARD - DENSITY_REF_SOFT)
            ratio = max(0.0, min(1.0, ratio))
            pct   = int(ratio * 100)

            if pct < 25:
                color, txt = C_GREEN,  f"Soft  ({pct}%)"
            elif pct < 50:
                color, txt = "#f1c40f", f"Medium  ({pct}%)"
            elif pct < 75:
                color, txt = C_ORANGE, f"Hard  ({pct}%)"
            else:
                color, txt = "#e74c3c", f"Very Hard  ({pct}%)"

            total_w = self._gauge_outer.width() - 4   # 패딩 제외
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
            self.lbl_density.setText("밀도  —")
            self.lbl_density.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
            self.lbl_tau.setText("τ  —")
            self.lbl_tau.setStyleSheet(f"color:{C_MUTED}; font-size:11px; background:transparent;")
            self._gauge_bar.setFixedWidth(4)
            self._gauge_bar.setStyleSheet(f"background:{C_MUTED}; border-radius:3px;")
            self.lbl_hardness.setText("무게를 입력하세요")
            self.lbl_hardness.setStyleSheet(
                f"color:{C_MUTED}; font-size:10px; background:transparent;"
            )

    # ── 모델 업데이트 ─────────────────────────────────────────
    def _update(self):
        ri = self.sliders["R"].value()
        ai = self.sliders["AR"].value()
        ci = self.sliders["CV"].value()

        R  = RADII[ri]
        AR = ASPECTS[ai]
        CV = CURVS[ci]

        # 값 레이블
        self.val_labels["R"].setText(f"{R:.1f} mm")
        self.val_labels["AR"].setText(f"{AR:.2f}")
        self.val_labels["CV"].setText(f"{CV:.2f}")

        # 설명
        self.desc_labels["R"].setText(f"직경  {R*2:.1f} mm")
        self.desc_labels["AR"].setText(f"{AR_DESCS[ai]}  —  장축 {R*2*AR:.1f} mm")
        self.desc_labels["CV"].setText(CV_DESCS[ci])

        # 파생 치수
        cd = CV * 2 * R
        bh = R * 0.20
        th = bh + 2 * cd
        Rs = (R*R + cd*cd) / (2*cd)

        self.dim_labels["d_minor"].setText(f"{R*2:.1f} mm")
        self.dim_labels["d_major"].setText(f"{R*2*AR:.1f} mm")
        self.dim_labels["d_thick"].setText(f"{th:.2f} mm")
        self.dim_labels["d_rs"].setText(f"{Rs:.1f} mm")

        # 파일 로드
        fname = f"tablet_R{R:.1f}_AR{AR:.2f}_CV{CV:.2f}.stl"
        fpath = os.path.join(STL_DIR, fname)
        self.lbl_fname.setText(fname)
        self._load_mesh(fpath)

        # 슬라이더 변경 시 무게가 입력돼 있으면 밀도 재계산
        self._recalc_hardness()

    def _load_mesh(self, fpath):
        self.plotter.clear()
        self.plotter.clear_actors()

        if not os.path.exists(fpath):
            self.lbl_status.setText("✗ 파일 없음 — STL을 먼저 생성하세요")
            self.lbl_status.setStyleSheet("color:#f85149; font-size:11px; background:transparent;")
            self.btn_mujoco.setEnabled(False)
            self.btn_crusher.setEnabled(False)
            self.plotter.render()
            return

        try:
            mesh = pv.read(fpath)

            # 중심 정렬 (mesh.center는 tuple → 직접 분해)
            cx, cy, cz = mesh.center
            mesh.translate((-cx, -cy, -cz), inplace=True)

            self.plotter.add_mesh(
                mesh,
                color="#dcd8ce",
                ambient=0.08,
                diffuse=0.85,
                specular=0.7,
                specular_power=40,
                smooth_shading=True,
            )

            # ── 스케일 바 (10mm 기준선) ──────────────────────────
            # 정제 아래쪽에 10mm 막대 표시 → 실제 크기 비교 가능
            bar_y = -18.0   # 화면 하단 고정 위치 (mm)
            bar = pv.Line((-5, bar_y, 0), (5, bar_y, 0))
            self.plotter.add_mesh(bar, color="#58a6ff",
                                  line_width=3, render_lines_as_tubes=True)
            self.plotter.add_point_labels(
                [(0, bar_y - 1.5, 0)], ["10 mm"],
                font_size=10, text_color="#58a6ff",
                fill_shape=False, always_visible=True,
                shape_opacity=0,
            )

            # 최초 로드 시에만 카메라 위치 설정 (이후엔 고정)
            if self._first_load:
                # 가장 큰 정제 기준(R=8.5, AR=2.5)으로 카메라 거리 고정
                self.plotter.camera.position = (0, -90, 40)
                self.plotter.camera.focal_point = (0, 0, 0)
                self.plotter.camera.up = (0, 0, 1)
                self._first_load = False

            self.plotter.render()

            self.lbl_status.setText("✓ 로드 완료")
            self.lbl_status.setStyleSheet(
                "color:#3fb950; font-size:11px; background:transparent;"
            )
            self.btn_mujoco.setEnabled(True)
            self.btn_crusher.setEnabled(True)
            self.lbl_mujoco.setText("")
        except Exception as e:
            self.lbl_status.setText(f"✗ 오류: {e}")
            self.lbl_status.setStyleSheet(
                "color:#f85149; font-size:11px; background:transparent;"
            )
            self.btn_mujoco.setEnabled(False)
            self.btn_crusher.setEnabled(False)

    # ── 볼록 분해 ─────────────────────────────────────────────────
    def _run_decomposition(self):
        fpath = os.path.join(STL_DIR, self.lbl_fname.text())
        if not os.path.exists(fpath):
            self.lbl_decomp.setText("✗ STL 파일 없음")
            return

        self.btn_decomp.setEnabled(False)
        self.lbl_decomp.setText("⏳ 분해 중...")
        QApplication.processEvents()

        try:
            # trimesh로 원본 로드
            tm = trimesh.load(fpath, force='mesh')

            # ── 볼록 분해 시도 순서 ──────────────────────────────
            # 1순위: coacd (pip install coacd)
            # 2순위: trimesh 내장 VHACD
            # 3순위: 단일 볼록 껍질 (항상 가능)
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
                method = "단일 볼록 껍질"

            self._hulls      = hulls
            self._hull_fpath = fpath

            # ── 메모리 추정 ──────────────────────────────────────
            total_verts = sum(len(h.vertices) for h in hulls)
            total_faces = sum(len(h.faces)    for h in hulls)
            # PhysX 기준: 정점당 ~48 bytes
            mem_kb = (total_verts * 48) / 1024

            # ── 뷰포트에 시각화 ──────────────────────────────────
            self.plotter.clear_actors()

            # 원본 메시 (반투명 회색 와이어프레임)
            orig = pv.read(fpath)
            cx, cy, cz = orig.center
            orig.translate((-cx, -cy, -cz), inplace=True)
            self.plotter.add_mesh(orig, color="#888888", opacity=0.15,
                                  style='wireframe')

            # Hull별로 다른 색상
            HULL_COLORS = ["#ff6b6b","#4ecdc4","#45b7d1",
                           "#96e6a1","#ffd93d","#c77dff",
                           "#ff9a3c","#74b9ff"]
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

            # 통계 표시
            self.lbl_decomp.setText(
                f"✓ {method}  |  {len(hulls)}개 hull\n"
                f"정점 {total_verts} / 면 {total_faces}\n"
                f"예상 메모리: ~{mem_kb:.1f} KB"
            )
            self.lbl_decomp.setStyleSheet(
                f"color:#3fb950; font-size:11px; background:transparent;"
            )
            self.btn_export.setEnabled(True)

        except Exception as e:
            self.lbl_decomp.setText(f"✗ 오류: {e}")
            self.lbl_decomp.setStyleSheet(
                f"color:#f85149; font-size:11px; background:transparent;"
            )
        finally:
            self.btn_decomp.setEnabled(True)

    # ── Hull STL 내보내기 ──────────────────────────────────────────
    def _export_hulls(self):
        if not self._hulls:
            return
        base = os.path.splitext(self._hull_fpath)[0]
        for i, hull in enumerate(self._hulls):
            out = f"{base}_hull{i:02d}.stl"
            hull.export(out)
        self.lbl_decomp.setText(
            self.lbl_decomp.text() + f"\n📁 {len(self._hulls)}개 STL 저장 완료"
        )

    # ── MuJoCo 뷰어 실행 ──────────────────────────────────────────
    def _launch_mujoco(self):
        fpath = os.path.join(STL_DIR, self.lbl_fname.text())
        if not os.path.exists(fpath):
            self.lbl_mujoco.setText("✗ STL 파일 없음")
            return

        launcher = os.path.join(_HERE, "launch_mujoco.py")
        if not os.path.exists(launcher):
            self.lbl_mujoco.setText("✗ launch_mujoco.py 없음")
            return

        # 별도 프로세스로 실행 → PyVista 뷰어 블로킹 없음
        import platform
        kwargs = {}
        if platform.system() == "Windows":
            kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE
        subprocess.Popen([sys.executable, launcher, fpath], **kwargs)
        self.lbl_mujoco.setText("✓ MuJoCo 뷰어 실행 중…")
        self.lbl_mujoco.setStyleSheet(
            "color:#3fb950; font-size:10px; background:transparent;"
        )

    # ── Crusher 시뮬레이션 실행 (velocity control) ────────────────────
    def _launch_crusher_sim(self):
        fpath = os.path.join(STL_DIR, self.lbl_fname.text())
        if not os.path.exists(fpath):
            self.lbl_mujoco.setText("✗ STL 파일 없음")
            return

        sim_script = os.path.normpath(
            os.path.join(_HERE, "..", "..",
                         "MuJoCo_PlayGround", "20260603", "crusher_velocity_ctrl.py")
        )
        if not os.path.exists(sim_script):
            self.lbl_mujoco.setText("✗ crusher_velocity_ctrl.py 없음")
            return

        import platform
        kwargs = {}
        if platform.system() == "Windows":
            kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE

        cmd = [sys.executable, sim_script, fpath]
        if self._last_density is not None:
            cmd += ["--density", f"{self._last_density:.2f}"]

        subprocess.Popen(cmd, **kwargs)

        if self._last_density is not None:
            tau = _density_to_tau(self._last_density)
            self.lbl_mujoco.setText(
                f"✓ 8 RPM 속도제어  ρ={self._last_density:.0f} kg/m³  τ={tau:.4f}s"
            )
        else:
            self.lbl_mujoco.setText("✓ Crusher 시뮬레이션 실행 중… (8 RPM)")
        self.lbl_mujoco.setStyleSheet(
            "color:#a371f7; font-size:10px; background:transparent;"
        )

    def closeEvent(self, event):
        self.plotter.close()
        super().closeEvent(event)


# ── 실행 ──────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))

    win = TabletViewer()
    win.show()

    sys.exit(app.exec_())
