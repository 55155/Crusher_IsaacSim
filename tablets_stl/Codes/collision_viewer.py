"""
Collision Mesh Viewer — 볼록 분해 전용 도구
=========================================
의존성:  pip install pyvista pyvistaqt PyQt5 trimesh numpy
선택적:  pip install coacd        (고품질 분해, 권장)

실행:    python collision_viewer.py
         python collision_viewer.py path/to/tablet.stl
"""

import os
import sys
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout, QVBoxLayout, QGridLayout,
    QLabel, QFrame, QPushButton, QComboBox,
    QFileDialog, QCheckBox,
    QTableWidget, QTableWidgetItem, QHeaderView,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor, QPalette

import pyvista as pv
from pyvistaqt import QtInteractor
import trimesh

# ── 경로 설정 (스크립트 위치(Codes/) → 상위(tablets_stl/) 기준) ──────
BASE_DIR      = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
STL_DIR       = os.path.join(BASE_DIR, "stl")
COLLISION_DIR = os.path.join(BASE_DIR, "collision")

# ── 색상 상수 ────────────────────────────────────────────────────────
C_BG     = "#0d1117"
C_PANEL  = "#161b22"
C_BORDER = "#21262d"
C_TEXT   = "#e6edf3"
C_MUTED  = "#8b949e"
C_BLUE   = "#58a6ff"
C_GREEN  = "#3fb950"
C_ORANGE = "#f78166"
C_PURPLE = "#a371f7"
C_RED    = "#f85149"
C_YELLOW = "#e3b341"

# 최대 20가지 Hull 색상 (구별 쉽도록 고대비 팔레트)
HULL_COLORS = [
    "#ff6b6b", "#4ecdc4", "#45b7d1", "#96e6a1",
    "#ffd93d", "#c77dff", "#ff9a3c", "#74b9ff",
    "#ff8fab", "#06d6a0", "#ffb347", "#b39ddb",
    "#ef476f", "#26c6da", "#ffe082", "#80cbc4",
    "#7bc67e", "#ce93d8", "#118ab2", "#ffd166",
]

# ── 분해 프리셋 ──────────────────────────────────────────────────────
#   threshold : CoACD 오차 허용치 (낮을수록 정밀, 느림)
#   max_hulls : 최대 볼록 껍질 수
#   vhacd_res : VHACD voxel 해상도 (CoACD 미설치 시 사용)
PRESETS = {
    "거친    (max  8 hull)": dict(threshold=0.05, max_hulls=8,  vhacd_res=100_000),
    "표준    (max 16 hull)": dict(threshold=0.02, max_hulls=16, vhacd_res=200_000),
    "정밀    (max 32 hull)": dict(threshold=0.01, max_hulls=32, vhacd_res=400_000),
    "최정밀  (max 64 hull)": dict(threshold=0.005, max_hulls=64, vhacd_res=800_000),
}

VIEW_MODES = ["원본 메시", "볼록 껍질", "겹침 보기", "면 법선"]


# ── 유틸 위젯 헬퍼 ───────────────────────────────────────────────────
def lbl(text, size=12, color=C_TEXT, bold=False):
    w = QLabel(text)
    w.setStyleSheet(
        f"color:{color}; font-size:{size}px;"
        + ("font-weight:700;" if bold else "")
        + "background:transparent;"
    )
    return w


def divider():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color:{C_BORDER};")
    return line


def btn(text, bg="#1f6feb", hov="#388bfd", prs="#1158c7"):
    b = QPushButton(text)
    b.setStyleSheet(f"""
        QPushButton           {{ background:{bg};  color:white; border:none;
                                 border-radius:6px; padding:8px;
                                 font-size:12px; font-weight:600; }}
        QPushButton:hover     {{ background:{hov}; }}
        QPushButton:pressed   {{ background:{prs}; }}
        QPushButton:disabled  {{ background:#21262d; color:{C_MUTED}; }}
    """)
    return b


def tm_to_pv(mesh: trimesh.Trimesh) -> pv.PolyData:
    """trimesh → PyVista PolyData 변환"""
    faces = np.hstack([
        np.full((len(mesh.faces), 1), 3, dtype=np.int32),
        mesh.faces.astype(np.int32),
    ])
    return pv.PolyData(mesh.vertices.astype(float), faces)


# ── 메인 윈도우 ──────────────────────────────────────────────────────
class CollisionViewer(QMainWindow):

    def __init__(self, init_file=None):
        super().__init__()
        self.setWindowTitle("Collision Mesh Viewer")
        self.resize(1400, 820)

        self._fpath    = None          # 현재 열린 STL 경로
        self._tm_orig  = None          # trimesh 원본 메시
        self._hulls    = []            # 분해된 trimesh hull 목록
        self._view_mode = 1            # 기본: 볼록 껍질
        self._cx = self._cy = self._cz = 0.0

        self._setup_palette()
        self._build_ui()

        if init_file and os.path.exists(init_file):
            self._load_file(init_file)

    # ── 다크 팔레트 ───────────────────────────────────────────────
    def _setup_palette(self):
        pal = QPalette()
        pal.setColor(QPalette.Window,          QColor(C_BG))
        pal.setColor(QPalette.WindowText,      QColor(C_TEXT))
        pal.setColor(QPalette.Base,            QColor(C_PANEL))
        pal.setColor(QPalette.AlternateBase,   QColor(C_BG))
        pal.setColor(QPalette.Text,            QColor(C_TEXT))
        pal.setColor(QPalette.Button,          QColor(C_PANEL))
        pal.setColor(QPalette.ButtonText,      QColor(C_TEXT))
        pal.setColor(QPalette.Highlight,       QColor(C_BLUE))
        pal.setColor(QPalette.HighlightedText, QColor("#000000"))
        self.setPalette(pal)
        self.setStyleSheet(f"QMainWindow {{ background:{C_BG}; }}")

    # ── UI 빌드 ───────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        root.setStyleSheet(f"background:{C_BG};")
        self.setCentralWidget(root)

        layout = QHBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        layout.addWidget(self._make_panel())
        layout.addWidget(self._make_viewport(), stretch=1)

    # ── 좌측 패널 ─────────────────────────────────────────────────
    def _make_panel(self):
        panel = QWidget()
        panel.setFixedWidth(330)
        panel.setStyleSheet(
            f"background:{C_PANEL}; border-right:1px solid {C_BORDER};"
        )
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(20, 24, 20, 20)
        pl.setSpacing(0)

        # 제목
        pl.addWidget(lbl("Physics Simulation", 9, C_BLUE, True))
        pl.addSpacing(4)
        title = QLabel("Collision Mesh\nViewer")
        title.setStyleSheet(
            f"color:{C_TEXT}; font-size:22px; font-weight:700; background:transparent;"
        )
        pl.addWidget(title)
        pl.addSpacing(18)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── 파일 열기 ─────────────────────────────────────────
        pl.addWidget(lbl("STL FILE", 9, C_MUTED, True))
        pl.addSpacing(8)

        b_open = btn("STL 파일 열기 …", "#238636", "#2ea043", "#196127")
        b_open.clicked.connect(self._open_file)
        pl.addWidget(b_open)
        pl.addSpacing(6)

        self.lbl_fname = QLabel("파일을 선택하세요")
        self.lbl_fname.setStyleSheet(
            f"color:{C_PURPLE}; font-size:10px; font-family:Consolas;"
            f"background:{C_BG}; border:1px solid {C_BORDER}; border-radius:6px;"
            "padding:6px 10px;"
        )
        self.lbl_fname.setWordWrap(True)
        pl.addWidget(self.lbl_fname)

        self.lbl_orig_stats = lbl("—", 10, C_MUTED)
        pl.addWidget(self.lbl_orig_stats)
        pl.addSpacing(14)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── 분해 설정 ─────────────────────────────────────────
        pl.addWidget(lbl("DECOMPOSITION", 9, C_MUTED, True))
        pl.addSpacing(10)

        self.cmb_preset = QComboBox()
        self.cmb_preset.addItems(list(PRESETS.keys()))
        self.cmb_preset.setCurrentIndex(1)   # 표준 기본값
        self.cmb_preset.setStyleSheet(f"""
            QComboBox {{
                background:{C_BG}; color:{C_TEXT};
                border:1px solid {C_BORDER}; border-radius:6px;
                padding:6px 10px; font-size:11px;
            }}
            QComboBox::drop-down {{ border:none; width:20px; }}
            QComboBox QAbstractItemView {{
                background:{C_PANEL}; color:{C_TEXT};
                border:1px solid {C_BORDER};
                selection-background-color:#1f6feb;
            }}
        """)
        pl.addWidget(self.cmb_preset)
        pl.addSpacing(8)

        self.btn_decomp = btn("▶  볼록 분해 실행")
        self.btn_decomp.setEnabled(False)
        self.btn_decomp.clicked.connect(self._run_decomposition)
        pl.addWidget(self.btn_decomp)
        pl.addSpacing(6)

        self.lbl_decomp_status = lbl("파일을 먼저 불러오세요", 11, C_MUTED)
        self.lbl_decomp_status.setAlignment(Qt.AlignCenter)
        pl.addWidget(self.lbl_decomp_status)
        pl.addSpacing(14)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── 시각화 모드 4개 ───────────────────────────────────
        pl.addWidget(lbl("VISUALIZATION MODE", 9, C_MUTED, True))
        pl.addSpacing(10)

        mode_w = QWidget()
        mode_w.setStyleSheet("background:transparent;")
        mg = QGridLayout(mode_w)
        mg.setContentsMargins(0, 0, 0, 0)
        mg.setSpacing(6)

        self._mode_btns = []
        for i, name in enumerate(VIEW_MODES):
            b = QPushButton(name)
            b.setCheckable(True)
            b.setChecked(i == self._view_mode)
            b.setStyleSheet(self._mode_style(i == self._view_mode))
            b.clicked.connect(lambda _, idx=i: self._set_view_mode(idx))
            self._mode_btns.append(b)
            mg.addWidget(b, i // 2, i % 2)

        pl.addWidget(mode_w)
        pl.addSpacing(10)

        self.chk_orig = QCheckBox("원본 와이어프레임 겹치기")
        self.chk_orig.setStyleSheet(
            f"color:{C_TEXT}; font-size:11px; background:transparent;"
        )
        self.chk_orig.stateChanged.connect(lambda _: self._refresh())
        pl.addWidget(self.chk_orig)
        pl.addSpacing(14)
        pl.addWidget(divider())
        pl.addSpacing(14)

        # ── Hull 통계 테이블 ──────────────────────────────────
        pl.addWidget(lbl("HULL STATISTICS", 9, C_MUTED, True))
        pl.addSpacing(8)

        self.tbl = QTableWidget(0, 4)
        self.tbl.setHorizontalHeaderLabels(["Hull", "정점", "면", "부피 %"])
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl.setFixedHeight(155)
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        self.tbl.setStyleSheet(f"""
            QTableWidget {{
                background:{C_BG}; color:{C_TEXT};
                border:1px solid {C_BORDER}; border-radius:6px;
                font-size:11px; gridline-color:{C_BORDER};
            }}
            QHeaderView::section {{
                background:{C_PANEL}; color:{C_MUTED};
                border:none; padding:4px;
                font-size:10px; font-weight:700;
                border-bottom:1px solid {C_BORDER};
            }}
            QTableWidget::item {{ padding:3px; }}
        """)
        pl.addWidget(self.tbl)
        pl.addSpacing(6)

        self.lbl_mem = lbl("예상 메모리: —", 11, C_MUTED)
        pl.addWidget(self.lbl_mem)
        pl.addSpacing(14)
        pl.addWidget(divider())
        pl.addSpacing(12)

        # ── Hull 내보내기 ─────────────────────────────────────
        self.btn_export = btn("Hull STL 내보내기", "#6e40c9", "#8b5cf6", "#5a32a3")
        self.btn_export.setEnabled(False)
        self.btn_export.clicked.connect(self._export_hulls)
        pl.addWidget(self.btn_export)
        pl.addSpacing(6)

        self.lbl_export = lbl("", 10, C_MUTED)
        self.lbl_export.setAlignment(Qt.AlignCenter)
        self.lbl_export.setWordWrap(True)
        pl.addWidget(self.lbl_export)

        pl.addStretch()
        return panel

    # ── PyVista 뷰포트 ────────────────────────────────────────────
    def _make_viewport(self):
        self.plotter = QtInteractor(auto_update=False)
        self.plotter.set_background("#0d1117", top="#1c2333")
        self.plotter.enable_anti_aliasing('ssaa')
        self.plotter.add_axes(interactive=False)

        self.plotter.remove_all_lights()
        for pos, intensity, color in [
            ((8,  12, 10), 1.00, 'white'),
            ((-6, -4, -5), 0.28, '#aac4ff'),
            ((0, -10,  6), 0.20, '#ffe8c0'),
        ]:
            l = pv.Light(position=pos, focal_point=(0,0,0),
                         intensity=intensity, color=color)
            l.positional = False
            self.plotter.add_light(l)

        return self.plotter

    # ── 모드 버튼 스타일 ──────────────────────────────────────────
    def _mode_style(self, active):
        if active:
            return (f"QPushButton {{ background:#1f6feb; color:white; border:none;"
                    f" border-radius:6px; padding:7px; font-size:11px; font-weight:600; }}")
        return (f"QPushButton {{ background:{C_BG}; color:{C_TEXT};"
                f" border:1px solid {C_BORDER}; border-radius:6px;"
                f" padding:7px; font-size:11px; }}"
                f"QPushButton:hover {{ background:#21262d; }}")

    # ── 파일 열기 ─────────────────────────────────────────────────
    def _open_file(self):
        start = STL_DIR if os.path.isdir(STL_DIR) else os.path.expanduser("~")
        path, _ = QFileDialog.getOpenFileName(
            self, "STL 파일 선택", start, "STL Files (*.stl)"
        )
        if path:
            self._load_file(path)

    def _load_file(self, path):
        try:
            self._tm_orig = trimesh.load(path, force='mesh')
            self._fpath   = path
            self._hulls   = []

            # 중심 계산
            c = self._tm_orig.centroid
            self._cx, self._cy, self._cz = float(c[0]), float(c[1]), float(c[2])

            self.lbl_fname.setText(os.path.basename(path))
            v = len(self._tm_orig.vertices)
            f = len(self._tm_orig.faces)
            self.lbl_orig_stats.setText(f"원본 정점 {v:,}  /  면 {f:,}")
            self.btn_decomp.setEnabled(True)
            self.btn_export.setEnabled(False)
            self._set_decomp_status("분해 프리셋을 선택 후 실행하세요", C_MUTED)
            self._tbl_clear()
            self._render_original()

        except Exception as e:
            self._set_decomp_status(f"✗ 로드 오류: {e}", C_RED)

    # ── 볼록 분해 실행 ────────────────────────────────────────────
    def _run_decomposition(self):
        if self._tm_orig is None:
            return

        p = PRESETS[self.cmb_preset.currentText()]
        self.btn_decomp.setEnabled(False)
        self._set_decomp_status("⏳ 분해 중 …", C_YELLOW)
        QApplication.processEvents()

        hulls, method = [], ""

        try:
            # ① CoACD (pip install coacd)
            try:
                import coacd
                mesh_c = coacd.Mesh(
                    np.array(self._tm_orig.vertices, dtype=np.float64),
                    np.array(self._tm_orig.faces,    dtype=np.int32),
                )
                parts = coacd.run_coacd(
                    mesh_c,
                    threshold=p['threshold'],
                    max_convex_hull=p['max_hulls'],
                    preprocess_resolution=50,
                )
                hulls  = [trimesh.Trimesh(vertices=np.array(v), faces=np.array(f))
                          for v, f in parts]
                method = "CoACD"
            except ImportError:
                pass

            # ② VHACD (trimesh 내장, v-hacd 바이너리 필요)
            if not hulls:
                try:
                    result = trimesh.decomposition.convex_decomposition(
                        self._tm_orig,
                        maxhulls=p['max_hulls'],
                        resolution=p['vhacd_res'],
                        maxverts=256,
                    )
                    hulls  = result if isinstance(result, list) else [result]
                    method = "VHACD"
                except Exception:
                    pass

            # ③ 단일 볼록 껍질 (fallback)
            if not hulls:
                hulls  = [self._tm_orig.convex_hull]
                method = "단일 볼록 껍질"

            self._hulls = hulls
            self._update_table()

            n = len(hulls)
            self._set_decomp_status(f"✓ {method}  —  {n}개 hull 생성", C_GREEN)
            self.btn_export.setEnabled(True)
            self._set_view_mode(self._view_mode)   # 현재 모드로 갱신

        except Exception as e:
            self._set_decomp_status(f"✗ 오류: {e}", C_RED)
        finally:
            self.btn_decomp.setEnabled(True)

    # ── 통계 테이블 ───────────────────────────────────────────────
    def _tbl_clear(self):
        self.tbl.setRowCount(0)
        self.lbl_mem.setText("예상 메모리: —")

    def _update_table(self):
        self._tbl_clear()
        orig_vol = self._tm_orig.volume if self._tm_orig.is_watertight else None
        total_v = total_f = 0

        for i, h in enumerate(self._hulls):
            v, f = len(h.vertices), len(h.faces)
            total_v += v
            total_f += f

            if orig_vol and orig_vol > 0:
                try:
                    vol_pct = f"{h.volume / orig_vol * 100:.1f} %"
                except Exception:
                    vol_pct = "—"
            else:
                vol_pct = "—"

            row = self.tbl.rowCount()
            self.tbl.insertRow(row)
            col_hex = HULL_COLORS[i % len(HULL_COLORS)]
            for col, val in enumerate([f"# {i+1}", str(v), str(f), vol_pct]):
                item = QTableWidgetItem(val)
                item.setForeground(QColor(col_hex if col == 0 else C_TEXT))
                item.setTextAlignment(Qt.AlignCenter)
                self.tbl.setItem(row, col, item)

        mem_kb = (total_v * 48) / 1024   # PhysX ~48 bytes/vertex
        self.lbl_mem.setText(
            f"총 정점 {total_v:,} / 면 {total_f:,}   ·   예상 메모리 ≈ {mem_kb:.1f} KB"
        )

    # ── 시각화 모드 전환 ──────────────────────────────────────────
    def _set_view_mode(self, mode):
        self._view_mode = mode
        for i, b in enumerate(self._mode_btns):
            b.setStyleSheet(self._mode_style(i == mode))
            b.setChecked(i == mode)
        self._refresh()

    def _refresh(self):
        [self._render_original,
         self._render_hulls,
         self._render_overlay,
         self._render_normals][self._view_mode]()

    # ── 공통 헬퍼 ─────────────────────────────────────────────────
    def _translated_orig(self) -> pv.PolyData:
        pv_m = tm_to_pv(self._tm_orig)
        pv_m.translate((-self._cx, -self._cy, -self._cz), inplace=True)
        return pv_m

    def _translated_hull(self, h: trimesh.Trimesh) -> pv.PolyData:
        pv_h = tm_to_pv(h)
        pv_h.translate((-self._cx, -self._cy, -self._cz), inplace=True)
        return pv_h

    def _overlay_orig_wire(self):
        """원본 와이어프레임 덧그리기 (체크박스 연동)"""
        if self.chk_orig.isChecked() and self._tm_orig is not None:
            self.plotter.add_mesh(
                self._translated_orig(),
                color="#ffffff", opacity=0.12,
                style='wireframe', line_width=0.6,
            )

    # ── 모드 0 : 원본 메시 ────────────────────────────────────────
    def _render_original(self):
        self.plotter.clear_actors()
        if self._tm_orig is None:
            self.plotter.render()
            return
        pv_m = self._translated_orig()
        self.plotter.add_mesh(
            pv_m, color="#dcd8ce",
            ambient=0.08, diffuse=0.85, specular=0.7, specular_power=40,
            smooth_shading=True,
        )
        if self.chk_orig.isChecked():
            self.plotter.add_mesh(
                pv_m.copy(), color="#ffffff", opacity=0.18,
                style='wireframe', line_width=0.6,
            )
        self.plotter.render()

    # ── 모드 1 : 볼록 껍질 ───────────────────────────────────────
    def _render_hulls(self):
        self.plotter.clear_actors()
        if not self._hulls:
            self._render_original()
            return
        for i, h in enumerate(self._hulls):
            pv_h = self._translated_hull(h)
            col  = HULL_COLORS[i % len(HULL_COLORS)]
            self.plotter.add_mesh(
                pv_h, color=col, opacity=0.88,
                smooth_shading=True,
                ambient=0.10, diffuse=0.80, specular=0.35,
            )
        self._overlay_orig_wire()
        self.plotter.render()

    # ── 모드 2 : 겹침 보기 ───────────────────────────────────────
    def _render_overlay(self):
        """원본 반투명 + 각 hull 반투명 + hull 경계 엣지 강조"""
        self.plotter.clear_actors()
        if self._tm_orig is None:
            self.plotter.render()
            return

        # 원본: 반투명 솔리드 + 와이어프레임
        pv_orig = self._translated_orig()
        self.plotter.add_mesh(pv_orig, color="#dcd8ce", opacity=0.20,
                              smooth_shading=True)
        self.plotter.add_mesh(pv_orig.copy(), color="#cccccc", opacity=0.25,
                              style='wireframe', line_width=0.5)

        if self._hulls:
            for i, h in enumerate(self._hulls):
                pv_h = self._translated_hull(h)
                col  = HULL_COLORS[i % len(HULL_COLORS)]

                # hull 반투명 솔리드
                self.plotter.add_mesh(
                    pv_h, color=col, opacity=0.45,
                    smooth_shading=True,
                    ambient=0.12, diffuse=0.78,
                )
                # hull 경계 엣지 — 충돌면 윤곽선 강조
                edges = pv_h.extract_feature_edges(
                    boundary_edges=True,
                    feature_edges=True,
                    non_manifold_edges=False,
                    feature_angle=20,
                )
                if edges.n_cells > 0:
                    self.plotter.add_mesh(
                        edges, color=col,
                        line_width=1.8, opacity=0.9,
                        render_lines_as_tubes=True,
                    )
        self.plotter.render()

    # ── 모드 3 : 면 법선 ─────────────────────────────────────────
    def _render_normals(self):
        """각 hull의 면 법선을 화살표로 표시 — 충돌 방향 시각화"""
        self.plotter.clear_actors()
        if not self._hulls:
            self._render_original()
            return

        # 화살표 크기: 메시 전체 크기의 8 %
        b = self._translated_orig().bounds   # (xmin,xmax,ymin,ymax,zmin,zmax)
        extent = max(b[1]-b[0], b[3]-b[2], b[5]-b[4])
        arrow_len = extent * 0.08

        for i, h in enumerate(self._hulls):
            pv_h = self._translated_hull(h)
            col  = HULL_COLORS[i % len(HULL_COLORS)]

            # hull 반투명 솔리드
            self.plotter.add_mesh(pv_h, color=col, opacity=0.35,
                                  smooth_shading=True)

            # hull 엣지 (면 경계 윤곽)
            edges = pv_h.extract_feature_edges(
                boundary_edges=True, feature_edges=True,
                non_manifold_edges=False, feature_angle=20,
            )
            if edges.n_cells > 0:
                self.plotter.add_mesh(edges, color=col, line_width=1.5,
                                      opacity=0.8)

            # 면 법선 화살표
            pv_n = pv_h.compute_normals(
                cell_normals=True, point_normals=False, consistent_normals=True
            )
            centers = pv_n.cell_centers()
            centers.point_data['Normals'] = pv_n.cell_data['Normals']
            arrows = centers.glyph(
                orient='Normals',
                scale=False,
                factor=arrow_len,
                geom=pv.Arrow(),
            )
            self.plotter.add_mesh(arrows, color=col, opacity=0.95)

        self._overlay_orig_wire()
        self.plotter.render()

    # ── 내보내기 ──────────────────────────────────────────────────
    def _export_hulls(self):
        if not self._hulls or not self._fpath:
            return
        os.makedirs(COLLISION_DIR, exist_ok=True)
        base = os.path.splitext(os.path.basename(self._fpath))[0]
        saved = []
        for i, h in enumerate(self._hulls):
            out = os.path.join(COLLISION_DIR, f"{base}_hull{i:02d}.stl")
            h.export(out)
            saved.append(out)
        self.lbl_export.setText(
            f"✓ {len(saved)}개 저장\ncollision\\{base}_hull*.stl"
        )
        self.lbl_export.setStyleSheet(
            f"color:{C_GREEN}; font-size:10px; background:transparent;"
        )

    # ── 상태 텍스트 헬퍼 ──────────────────────────────────────────
    def _set_decomp_status(self, text, color):
        self.lbl_decomp_status.setText(text)
        self.lbl_decomp_status.setStyleSheet(
            f"color:{color}; font-size:11px; background:transparent;"
        )

    def closeEvent(self, event):
        self.plotter.close()
        super().closeEvent(event)


# ── 진입점 ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))
    init_file = sys.argv[1] if len(sys.argv) > 1 else None
    win = CollisionViewer(init_file)
    win.show()
    sys.exit(app.exec_())
