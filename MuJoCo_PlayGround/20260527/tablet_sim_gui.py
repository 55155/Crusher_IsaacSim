#!/usr/bin/env python3
"""
tablet_sim_gui.py — Crusher Tablet Simulator GUI 프론트엔드

▶ 기능
    - 알약 물성(무게/형상 또는 밀도) 입력 → 밀도·solref·solimp 실시간 계산
    - 경도 게이지 시각화
    - 밀도→τ 참조 테이블 (현재 밀도 하이라이트)
    - crusher_tablet_sim.py (단일) / batch_tablet_sim.py (배치) 서브프로세스 실행
    - 콘솔 출력 실시간 표시 (색상 분류)

▶ 실행
    python tablet_sim_gui.py
    python tablet_sim_gui.py --stl path/to/tablet.stl   # STL 미리 지정

▶ 의존성
    표준 라이브러리만 사용 (tkinter, subprocess, threading, pathlib)
    numpy, math ─ 밀도 계산 (동일 환경 공유)
"""
from __future__ import annotations   # Python 3.9 에서 X | Y 타입힌트 허용

import argparse
import math
import os
import subprocess
import sys
import threading
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext
import tkinter as tk
from tkinter import ttk

import numpy as np

# ── 경로 ─────────────────────────────────────────────────────────────────────
_HERE   = Path(__file__).parent
STL_DIR = _HERE.parent.parent / "tablets_stl" / "stl"

# ── 물리 상수 (crusher_tablet_sim.py 와 반드시 동기화) ───────────────────────
DENSITY_REF_SOFT    = 900.0    # kg/m³  연질 기준
DENSITY_REF_HARD    = 1800.0   # kg/m³  경질 기준
SOLREF_TAU_SOFT     = 0.020    # s
SOLREF_TAU_HARD     = 0.002    # s
DENSITY_DEFAULT     = 1200.0   # kg/m³
BICONVEX_VOL_FACTOR = 0.82     # biconvex 부피 보정계수

# 참조 테이블 (밀도 → τ)
_REF_TABLE = [
    (900,  "0.0200", "연질 (저압착 포도당)"),
    (1000, "0.0148", ""),
    (1100, "0.0112", ""),
    (1200, "0.0080", "기본값"),
    (1300, "0.0061", ""),
    (1400, "0.0049", ""),
    (1600, "0.0033", ""),
    (1800, "0.0020", "경질 (탄산칼슘계)"),
]


# ── 계산 함수 ─────────────────────────────────────────────────────────────────
def estimate_volume_mm3(R_mm: float, AR: float, CV: float) -> float:
    """biconvex 알약 부피 추정 [mm³]."""
    cd = CV * 2.0 * R_mm
    th = R_mm * 0.20 + 2.0 * cd
    return (4.0 / 3.0) * math.pi * (R_mm * AR) * R_mm * (th / 2.0) * BICONVEX_VOL_FACTOR


def mass_to_density(mass_mg: float, R_mm: float, AR: float, CV: float) -> float:
    """무게(mg) + 형상 → 밀도 [kg/m³]."""
    return (mass_mg * 1e-6) / (estimate_volume_mm3(R_mm, AR, CV) * 1e-9)


def density_to_tau(rho: float) -> float:
    """밀도 → MuJoCo solref τ [s]  (Power-law, Hertzian Contact Theory)."""
    rho   = float(np.clip(rho, DENSITY_REF_SOFT, DENSITY_REF_HARD))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    tau   = SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha
    return float(np.clip(tau, SOLREF_TAU_HARD, SOLREF_TAU_SOFT))


def density_to_dimp(rho: float) -> float:
    """밀도 → solimp d_max."""
    return float(np.interp(rho, [DENSITY_REF_SOFT, DENSITY_REF_HARD], [0.950, 0.999]))


# ─────────────────────────────────────────────────────────────────────────────
# 색상 팔레트 (다크 테마)
# ─────────────────────────────────────────────────────────────────────────────
C = {
    "bg":      "#1c1c2e",
    "bg2":     "#252537",
    "bg3":     "#32324a",
    "bg4":     "#3e3e5a",
    "fg":      "#e2e2f0",
    "fg2":     "#8888aa",
    "accent":  "#7c83fd",
    "accent2": "#a5abff",
    "green":   "#3fb950",
    "yellow":  "#d29922",
    "orange":  "#e67e22",
    "red":     "#f85149",
    "white":   "#ffffff",
}


# ─────────────────────────────────────────────────────────────────────────────
class _DarkStyle:
    """ttk 스타일 일괄 적용 헬퍼."""

    @staticmethod
    def apply(root: tk.Tk):
        root.configure(bg=C["bg"])
        s = ttk.Style(root)
        try:
            s.theme_use("clam")
        except Exception:
            pass

        s.configure(".",
                    background=C["bg"], foreground=C["fg"],
                    fieldbackground=C["bg2"],
                    troughcolor=C["bg3"], bordercolor=C["bg3"],
                    darkcolor=C["bg2"], lightcolor=C["bg4"],
                    selectbackground=C["accent"], selectforeground=C["white"])

        s.configure("TFrame",         background=C["bg"])
        s.configure("TLabel",         background=C["bg"], foreground=C["fg"])
        s.configure("TLabelframe",    background=C["bg"], foreground=C["accent"],
                    bordercolor=C["bg4"], relief="groove")
        s.configure("TLabelframe.Label",
                    background=C["bg"], foreground=C["accent"],
                    font=("Segoe UI", 9, "bold"))
        s.configure("TEntry",
                    fieldbackground=C["bg2"], foreground=C["fg"],
                    insertcolor=C["fg"], selectbackground=C["accent"])
        s.configure("TButton",
                    background=C["bg3"], foreground=C["fg"],
                    padding=[8, 3], relief="flat")
        s.map("TButton",
              background=[("active", C["bg4"]), ("pressed", C["accent"])],
              foreground=[("active", C["white"])])
        s.configure("TCheckbutton", background=C["bg"], foreground=C["fg"])
        s.configure("TRadiobutton", background=C["bg"], foreground=C["fg"])
        s.configure("TSeparator",   background=C["bg4"])

        # Notebook
        s.configure("TNotebook",     background=C["bg"],  borderwidth=0)
        s.configure("TNotebook.Tab",
                    background=C["bg3"], foreground=C["fg2"],
                    padding=[12, 4], borderwidth=0)
        s.map("TNotebook.Tab",
              background=[("selected", C["bg2"]), ("active", C["bg4"])],
              foreground=[("selected", C["fg"]),  ("active", C["fg"])])


# ─────────────────────────────────────────────────────────────────────────────
class TabletSimGUI:
    """메인 GUI 클래스."""

    def __init__(self, root: tk.Tk, init_stl: str = ""):
        self.root = root
        self.root.title("Crusher Tablet Simulator")
        self.root.minsize(820, 660)
        _DarkStyle.apply(root)

        # ── 입력 변수 ─────────────────────────────────────────────────────────
        self.input_mode = tk.StringVar(value="mass")   # "mass" | "density"
        self.sv_mass    = tk.StringVar(value="500")    # mg
        self.sv_dens_in = tk.StringVar(value="1200")   # kg/m³  (직접 입력 모드)
        self.sv_radius  = tk.StringVar(value="8.0")    # mm
        self.sv_ar      = tk.StringVar(value="0.80")
        self.sv_cv      = tk.StringVar(value="0.25")
        self.sv_stl     = tk.StringVar(value=init_stl)

        # 단일 시뮬 설정
        self.sv_save_plots = tk.BooleanVar(value=False)

        # 배치 시뮬 설정
        self.sv_b_density = tk.StringVar(value="1200")
        self.sv_b_stl_dir = tk.StringVar(value="")
        self.sv_b_limit   = tk.StringVar(value="")
        self.sv_parallel  = tk.BooleanVar(value=False)
        self.sv_workers   = tk.StringVar(value="4")
        self.sv_b_save    = tk.BooleanVar(value=False)
        self.sv_b_duration= tk.StringVar(value="10")

        # ── 계산 결과 변수 (읽기 전용) ────────────────────────────────────────
        self.sv_vol  = tk.StringVar(value="—")
        self.sv_rho  = tk.StringVar(value="—")   # 계산된 밀도
        self.sv_tau  = tk.StringVar(value="—")
        self.sv_dimp = tk.StringVar(value="—")

        self._proc_lock = threading.Lock()
        self._build_ui()
        self._bind_traces()
        self._recalc()

    # ─────────────────────────────────────────────────────────────────────────
    # UI 구성
    # ─────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)   # 콘솔이 늘어남

        # ── 헤더 ─────────────────────────────────────────────────────────────
        hdr = tk.Frame(outer, bg=C["bg"])
        hdr.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        tk.Label(hdr, text="⚗  Crusher Tablet Simulator",
                 bg=C["bg"], fg=C["accent"],
                 font=("Segoe UI", 13, "bold")).pack(side="left")
        tk.Label(hdr, text="  —  밀도 기반 접촉 경도 자동 계산",
                 bg=C["bg"], fg=C["fg2"],
                 font=("Segoe UI", 9)).pack(side="left")

        # ── 중단: 입력 + 결과 ─────────────────────────────────────────────────
        mid = ttk.Frame(outer)
        mid.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        mid.columnconfigure(0, weight=2)
        mid.columnconfigure(1, weight=1)

        self._build_input_frame(mid)
        self._build_result_frame(mid)

        # ── 하단: 실행 탭 ─────────────────────────────────────────────────────
        self._build_run_tabs(outer)

        # ── 콘솔 ─────────────────────────────────────────────────────────────
        self._build_console(outer)

    # ── 입력 패널 ─────────────────────────────────────────────────────────────
    def _build_input_frame(self, parent: ttk.Frame):
        f = ttk.LabelFrame(parent, text="📋  알약 물성 입력", padding=10)
        f.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        f.columnconfigure(2, weight=1)

        # 입력 모드 선택
        mode_row = ttk.Frame(f)
        mode_row.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))
        ttk.Label(mode_row, text="입력 방식 ").pack(side="left")
        ttk.Radiobutton(mode_row, text="무게 (mg) 입력",
                        variable=self.input_mode, value="mass").pack(side="left", padx=6)
        ttk.Radiobutton(mode_row, text="밀도 (kg/m³) 직접 입력",
                        variable=self.input_mode, value="density").pack(side="left", padx=6)

        ttk.Separator(f, orient="horizontal").grid(
            row=1, column=0, columnspan=4, sticky="ew", pady=(0, 8))

        # ── 무게 / 밀도 입력 (모드별 전환) ─────────────────────────────────
        self._lbl_mass = self._entry_row(f, 2, "무게",    self.sv_mass,    "mg",
                                          "알약 1정 실측 무게")
        self._lbl_dens_in = self._entry_row(f, 3, "밀도", self.sv_dens_in, "kg/m³",
                                             "900(연질) ~ 1800(경질)")
        self._ent_mass_widget    = self._last_entry
        self._ent_dens_in_widget = None   # 아래에서 캡처 불가 → trace로 처리

        ttk.Separator(f, orient="horizontal").grid(
            row=4, column=0, columnspan=4, sticky="ew", pady=8)

        # ── 형상 파라미터 ──────────────────────────────────────────────────
        tk.Label(f, text="형상 파라미터", bg=C["bg"], fg=C["fg2"],
                 font=("Segoe UI", 8)).grid(row=5, column=0, columnspan=4,
                 sticky="w", padx=4, pady=(0, 2))
        self._entry_row(f, 6, "반지름 R",  self.sv_radius, "mm",    "단반경")
        self._entry_row(f, 7, "종횡비 AR", self.sv_ar,     "",      "장반경/단반경 (0.5 ~ 1.5)")
        self._entry_row(f, 8, "왕관비 CV", self.sv_cv,     "",      "곡률 비율 (0.1 ~ 0.4)")

        ttk.Separator(f, orient="horizontal").grid(
            row=9, column=0, columnspan=4, sticky="ew", pady=8)

        # ── STL 파일 ────────────────────────────────────────────────────────
        ttk.Label(f, text="STL 파일").grid(row=10, column=0, sticky="w", padx=4, pady=3)
        stl_row = ttk.Frame(f)
        stl_row.grid(row=10, column=1, columnspan=3, sticky="ew", padx=4)
        stl_row.columnconfigure(0, weight=1)
        ttk.Entry(stl_row, textvariable=self.sv_stl).grid(row=0, column=0, sticky="ew")
        tk.Button(stl_row, text=" 📂 ", command=self._browse_stl,
                  bg=C["bg3"], fg=C["fg"], relief="flat",
                  activebackground=C["accent"], activeforeground=C["white"],
                  cursor="hand2").grid(row=0, column=1, padx=(4, 0))

        # 모드 초기 반영
        self.input_mode.trace_add("write", lambda *_: self._update_mode_widgets())
        self._update_mode_widgets()

    def _entry_row(self, parent, row, label, svar, unit, hint=""):
        """라벨 + Entry + 단위 한 줄 추가. 마지막 Entry 를 self._last_entry 에 저장."""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=3)
        e = ttk.Entry(parent, textvariable=svar, width=11)
        e.grid(row=row, column=1, sticky="w", padx=4, pady=3)
        ttk.Label(parent, text=unit).grid(row=row, column=2, sticky="w", padx=(0, 4), pady=3)
        ttk.Label(parent, text=hint, foreground=C["fg2"],
                  font=("Segoe UI", 8)).grid(row=row, column=3, sticky="w", pady=3)
        self._last_entry = e
        return ttk.Label(parent, text=label)   # 반환값은 사용 안 함 (하위 호환)

    def _update_mode_widgets(self):
        """무게/밀도 입력 행 활성화 상태 전환."""
        is_mass = self.input_mode.get() == "mass"
        # Entry 위젯을 직접 순회해 state 조정
        for child in self.root.winfo_children():
            self._toggle_state_recursive(child, is_mass)
        self._recalc()

    def _toggle_state_recursive(self, widget, is_mass):
        """sv_mass / sv_dens_in 에 연결된 Entry 의 state 토글 (재귀)."""
        try:
            tv = widget.cget("textvariable")
            if str(tv) == str(self.sv_mass):
                widget.configure(state="normal" if is_mass else "readonly")
            elif str(tv) == str(self.sv_dens_in):
                widget.configure(state="readonly" if is_mass else "normal")
        except Exception:
            pass
        for child in widget.winfo_children():
            self._toggle_state_recursive(child, is_mass)

    # ── 결과 패널 ─────────────────────────────────────────────────────────────
    def _build_result_frame(self, parent: ttk.Frame):
        f = ttk.LabelFrame(parent, text="📊  계산 결과 (실시간)", padding=10)
        f.grid(row=0, column=1, sticky="nsew")
        f.columnconfigure(1, weight=1)

        self._result_row(f, 0, "추정 부피",   self.sv_vol,  "mm³")
        self._result_row(f, 1, "추정 밀도",   self.sv_rho,  "kg/m³")
        self._result_row(f, 2, "solref  τ",   self.sv_tau,  "s")
        self._result_row(f, 3, "solimp  d",   self.sv_dimp, "")

        # 경도 게이지
        ttk.Separator(f, orient="horizontal").grid(
            row=4, column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Label(f, text="경도 수준").grid(row=5, column=0, sticky="w", padx=4, pady=(0, 2))
        gauge_f = tk.Frame(f, bg=C["bg"])
        gauge_f.grid(row=5, column=1, columnspan=2, sticky="w", padx=4)
        self._gauge_canvas = tk.Canvas(gauge_f, width=160, height=20,
                                        bg=C["bg2"], highlightthickness=1,
                                        highlightbackground=C["bg4"])
        self._gauge_canvas.pack(side="left")
        self._gauge_lbl = tk.Label(gauge_f, text="", bg=C["bg"], fg=C["fg"],
                                    font=("Segoe UI", 9, "bold"), width=16, anchor="w")
        self._gauge_lbl.pack(side="left", padx=(8, 0))

        # 밀도 → τ 참조 테이블
        ttk.Separator(f, orient="horizontal").grid(
            row=6, column=0, columnspan=3, sticky="ew", pady=8)
        tk.Label(f, text="밀도 → τ  참조", bg=C["bg"], fg=C["fg2"],
                 font=("Segoe UI", 8, "bold")).grid(
            row=7, column=0, columnspan=3, sticky="w", padx=4, pady=(0, 4))
        self._build_ref_table(f, start_row=8)

    def _result_row(self, parent, row, label, svar, unit):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=3)
        tk.Label(parent, textvariable=svar,
                 bg=C["bg"], fg=C["accent2"],
                 font=("Consolas", 11, "bold")).grid(
            row=row, column=1, sticky="w", padx=4, pady=3)
        ttk.Label(parent, text=unit, foreground=C["fg2"]).grid(
            row=row, column=2, sticky="w", padx=(0, 4), pady=3)

    def _build_ref_table(self, parent, start_row: int):
        """밀도 → τ 참조 테이블 (행 전체를 저장해 현재 밀도 하이라이트에 활용)."""
        hdrs = ["밀도", "τ [s]", "비고"]
        for c, h in enumerate(hdrs):
            tk.Label(parent, text=h, bg=C["bg"], fg=C["fg2"],
                     font=("Consolas", 8, "bold")).grid(
                row=start_row, column=c, padx=(4 if c == 0 else 2, 2), sticky="w")

        self._ref_labels: list[tuple[tk.Label, float]] = []
        for i, (rho, tau, note) in enumerate(_REF_TABLE):
            r = start_row + 1 + i
            lbl_rho = tk.Label(parent, text=f"{rho:>5}", bg=C["bg"], fg=C["fg"],
                                font=("Consolas", 8))
            lbl_rho.grid(row=r, column=0, padx=4, sticky="w")
            lbl_tau = tk.Label(parent, text=tau, bg=C["bg"], fg=C["accent"],
                                font=("Consolas", 8, "bold"))
            lbl_tau.grid(row=r, column=1, padx=2, sticky="w")
            lbl_note = tk.Label(parent, text=note, bg=C["bg"], fg=C["fg2"],
                                 font=("Segoe UI", 7))
            lbl_note.grid(row=r, column=2, padx=2, sticky="w")
            self._ref_labels.append((lbl_rho, lbl_tau, lbl_note, rho))

    def _highlight_ref_table(self, current_rho: float | None):
        """현재 밀도에 가장 가까운 참조 행 하이라이트."""
        if not hasattr(self, "_ref_labels"):
            return
        best_idx = -1
        if current_rho is not None:
            dists = [abs(current_rho - r[3]) for r in self._ref_labels]
            best_idx = int(np.argmin(dists))

        for i, (lr, lt, ln, _) in enumerate(self._ref_labels):
            if i == best_idx:
                lr.config(bg=C["bg3"], fg=C["white"])
                lt.config(bg=C["bg3"], fg=C["accent2"])
                ln.config(bg=C["bg3"])
            else:
                lr.config(bg=C["bg"], fg=C["fg"])
                lt.config(bg=C["bg"], fg=C["accent"])
                ln.config(bg=C["bg"])

    # ── 실행 탭 ───────────────────────────────────────────────────────────────
    def _build_run_tabs(self, parent: ttk.Frame):
        nb = ttk.Notebook(parent)
        nb.grid(row=1, column=0, sticky="ew", pady=(0, 8))

        t_single = ttk.Frame(nb, padding=8)
        nb.add(t_single, text="🚀  단일 시뮬레이션")
        self._build_single_tab(t_single)

        t_batch = ttk.Frame(nb, padding=8)
        nb.add(t_batch, text="📦  배치 시뮬레이션")
        self._build_batch_tab(t_batch)

    def _build_single_tab(self, parent: ttk.Frame):
        parent.columnconfigure(99, weight=1)   # 오른쪽 여백

        ttk.Checkbutton(parent, text="그래프 저장 (PNG)",
                         variable=self.sv_save_plots).grid(
            row=0, column=0, padx=(0, 16))

        self.btn_single = tk.Button(
            parent, text="  ▶  시뮬레이션 실행  ",
            command=self._run_single,
            bg=C["accent"], fg=C["white"], relief="flat",
            font=("Segoe UI", 10, "bold"),
            activebackground="#5c63cd", activeforeground=C["white"],
            padx=14, pady=5, cursor="hand2")
        self.btn_single.grid(row=0, column=1, padx=(0, 8))

    def _build_batch_tab(self, parent: ttk.Frame):
        parent.columnconfigure(1, weight=1)

        # 행 0: STL 폴더
        ttk.Label(parent, text="STL 폴더").grid(row=0, column=0, sticky="w", padx=(0, 6))
        stl_row = ttk.Frame(parent)
        stl_row.grid(row=0, column=1, columnspan=5, sticky="ew")
        stl_row.columnconfigure(0, weight=1)
        ttk.Entry(stl_row, textvariable=self.sv_b_stl_dir).grid(
            row=0, column=0, sticky="ew")
        tk.Button(stl_row, text=" 📂 ", command=self._browse_stl_dir,
                  bg=C["bg3"], fg=C["fg"], relief="flat", cursor="hand2").grid(
            row=0, column=1, padx=(4, 0))

        # 행 1: 밀도 / 시간
        ttk.Label(parent, text="밀도").grid(row=1, column=0, sticky="w", padx=(0, 6), pady=(4, 0))
        ttk.Entry(parent, textvariable=self.sv_b_density, width=8).grid(
            row=1, column=1, sticky="w", padx=(0, 2), pady=(4, 0))
        ttk.Label(parent, text="kg/m³").grid(row=1, column=2, sticky="w", padx=(0, 14), pady=(4, 0))
        ttk.Label(parent, text="시뮬 시간").grid(row=1, column=3, sticky="w", padx=(0, 6), pady=(4, 0))
        ttk.Entry(parent, textvariable=self.sv_b_duration, width=6).grid(
            row=1, column=4, sticky="w", padx=(0, 2), pady=(4, 0))
        ttk.Label(parent, text="s").grid(row=1, column=5, sticky="w", pady=(4, 0))

        # 행 2: 개수 / 병렬
        ttk.Label(parent, text="개수 제한").grid(row=2, column=0, sticky="w", padx=(0, 6), pady=(4, 0))
        ttk.Entry(parent, textvariable=self.sv_b_limit, width=8).grid(
            row=2, column=1, sticky="w", padx=(0, 2), pady=(4, 0))
        ttk.Label(parent, text="(비어두면 전체)").grid(row=2, column=2, sticky="w", padx=(0, 14), pady=(4, 0))
        ttk.Checkbutton(parent, text="병렬 실행", variable=self.sv_parallel).grid(
            row=2, column=3, sticky="w", pady=(4, 0))
        ttk.Label(parent, text="워커 수").grid(row=2, column=4, sticky="w", pady=(4, 0))
        ttk.Entry(parent, textvariable=self.sv_workers, width=4).grid(
            row=2, column=5, sticky="w", padx=(4, 0), pady=(4, 0))

        # 행 3: 옵션 + 실행 버튼
        ttk.Checkbutton(parent, text="개별 그래프 저장",
                         variable=self.sv_b_save).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(6, 0))

        self.btn_batch = tk.Button(
            parent, text="  ▶  배치 실행  ",
            command=self._run_batch,
            bg="#2e7d32", fg=C["white"], relief="flat",
            font=("Segoe UI", 10, "bold"),
            activebackground="#1b5e20", activeforeground=C["white"],
            padx=14, pady=5, cursor="hand2")
        self.btn_batch.grid(row=3, column=4, columnspan=2, sticky="e", pady=(6, 0))

    # ── 콘솔 ─────────────────────────────────────────────────────────────────
    def _build_console(self, parent: ttk.Frame):
        parent.rowconfigure(2, weight=1)

        con_frame = ttk.LabelFrame(parent, text="📟  출력", padding=4)
        con_frame.grid(row=2, column=0, sticky="nsew")
        con_frame.columnconfigure(0, weight=1)
        con_frame.rowconfigure(0, weight=1)

        self.console = scrolledtext.ScrolledText(
            con_frame, height=11, wrap=tk.WORD,
            font=("Consolas", 9),
            bg="#0d1117", fg="#c9d1d9",
            insertbackground="#c9d1d9",
            selectbackground=C["bg3"],
            relief="flat", borderwidth=0)
        self.console.grid(row=0, column=0, sticky="nsew")
        self.console.configure(state="disabled")

        # 색상 태그
        self.console.tag_configure("cmd",  foreground="#6e7681")
        self.console.tag_configure("ok",   foreground="#3fb950")
        self.console.tag_configure("err",  foreground="#f85149")
        self.console.tag_configure("warn", foreground="#d29922")
        self.console.tag_configure("info", foreground="#58a6ff")

        clr = tk.Button(con_frame, text="콘솔 지우기",
                        command=self._clear_console,
                        bg=C["bg2"], fg=C["fg2"],
                        relief="flat", padx=8, pady=2, cursor="hand2")
        clr.grid(row=1, column=0, sticky="e", pady=(4, 0))

    # ─────────────────────────────────────────────────────────────────────────
    # 실시간 계산
    # ─────────────────────────────────────────────────────────────────────────
    def _bind_traces(self):
        for sv in [self.sv_mass, self.sv_dens_in,
                   self.sv_radius, self.sv_ar, self.sv_cv]:
            sv.trace_add("write", lambda *_: self._recalc())

    def _recalc(self):
        try:
            R  = float(self.sv_radius.get())
            AR = float(self.sv_ar.get())
            CV = float(self.sv_cv.get())
            if R <= 0 or AR <= 0 or CV <= 0:
                raise ValueError

            vol = estimate_volume_mm3(R, AR, CV)

            if self.input_mode.get() == "mass":
                m   = float(self.sv_mass.get())
                rho = mass_to_density(m, R, AR, CV)
            else:
                rho = float(self.sv_dens_in.get())

            tau  = density_to_tau(rho)
            dimp = density_to_dimp(rho)

            self.sv_vol.set(f"{vol:.1f}")
            self.sv_rho.set(f"{rho:.0f}")
            self.sv_tau.set(f"{tau:.4f}")
            self.sv_dimp.set(f"{dimp:.3f}")

            self._draw_gauge(rho)
            self._highlight_ref_table(rho)

        except Exception:
            for sv in [self.sv_vol, self.sv_rho, self.sv_tau, self.sv_dimp]:
                sv.set("—")
            self._draw_gauge(None)
            self._highlight_ref_table(None)

    # ─────────────────────────────────────────────────────────────────────────
    # 경도 게이지
    # ─────────────────────────────────────────────────────────────────────────
    def _draw_gauge(self, rho: float | None):
        c  = self._gauge_canvas
        W, H = 160, 20
        c.delete("all")

        if rho is None:
            c.create_rectangle(0, 0, W, H, fill=C["bg3"], outline="")
            self._gauge_lbl.config(text="입력 오류", fg=C["fg2"])
            return

        ratio = (rho - DENSITY_REF_SOFT) / (DENSITY_REF_HARD - DENSITY_REF_SOFT)
        ratio = max(0.0, min(1.0, ratio))
        fw    = int(W * ratio)

        # 배경
        c.create_rectangle(0, 0, W, H, fill=C["bg3"], outline="")

        # 바 (green → yellow → orange → red)
        if fw > 0:
            rv = min(255, int(255 * ratio * 1.4))
            gv = min(180, int(180 * (1 - ratio)))
            color = f"#{rv:02x}{gv:02x}20"
            c.create_rectangle(0, 0, fw, H, fill=color, outline="")
            # 마커
            c.create_rectangle(max(0, fw - 2), 0, fw, H,
                               fill=C["white"], outline="")

        # 라벨
        pct = ratio * 100
        if pct < 20:
            txt, fg = f"Soft  ({pct:.0f}%)",      C["green"]
        elif pct < 45:
            txt, fg = f"Medium  ({pct:.0f}%)",    C["yellow"]
        elif pct < 75:
            txt, fg = f"Hard  ({pct:.0f}%)",      C["orange"]
        else:
            txt, fg = f"Very Hard  ({pct:.0f}%)", C["red"]
        self._gauge_lbl.config(text=txt, fg=fg)

    # ─────────────────────────────────────────────────────────────────────────
    # 파일 다이얼로그
    # ─────────────────────────────────────────────────────────────────────────
    def _browse_stl(self):
        init = str(STL_DIR) if STL_DIR.exists() else str(Path.home())
        p = filedialog.askopenfilename(
            title="Tablet STL 선택",
            initialdir=init,
            filetypes=[("STL Files", "*.stl"), ("All Files", "*.*")])
        if p:
            self.sv_stl.set(p)

    def _browse_stl_dir(self):
        init = str(STL_DIR) if STL_DIR.exists() else str(Path.home())
        p = filedialog.askdirectory(title="STL 폴더 선택", initialdir=init)
        if p:
            self.sv_b_stl_dir.set(p)

    # ─────────────────────────────────────────────────────────────────────────
    # 시뮬레이션 실행
    # ─────────────────────────────────────────────────────────────────────────
    def _effective_density(self) -> float | None:
        """현재 입력 모드에서 유효 밀도 반환."""
        try:
            return float(self.sv_rho.get())
        except ValueError:
            return None

    def _run_single(self):
        stl = self.sv_stl.get().strip()
        if not stl:
            messagebox.showwarning("STL 없음",
                                   "단일 시뮬레이션에는 STL 파일이 필요합니다.\n"
                                   "📂 버튼으로 파일을 선택해 주세요.")
            return
        if not Path(stl).exists():
            messagebox.showerror("파일 없음", f"파일을 찾을 수 없습니다:\n{stl}")
            return

        rho = self._effective_density()
        if rho is None:
            messagebox.showerror("계산 오류",
                                  "밀도를 계산할 수 없습니다.\n입력값을 확인해 주세요.")
            return

        script = _HERE / "crusher_tablet_sim.py"
        cmd = [sys.executable, str(script), stl,
               "--density", f"{rho:.2f}"]
        if self.sv_save_plots.get():
            cmd.append("--save-plots")

        self._launch(cmd, self.btn_single)

    def _run_batch(self):
        try:
            rho = float(self.sv_b_density.get())
        except ValueError:
            messagebox.showerror("입력 오류", "밀도 값을 확인해 주세요.")
            return

        script = _HERE / "batch_tablet_sim.py"
        cmd = [sys.executable, str(script), "--density", f"{rho:.2f}"]

        stl_dir = self.sv_b_stl_dir.get().strip()
        if stl_dir:
            cmd += ["--stl-dir", stl_dir]

        duration = self.sv_b_duration.get().strip()
        if duration:
            cmd += ["--duration", duration]

        limit = self.sv_b_limit.get().strip()
        if limit:
            cmd += ["--limit", limit]

        if self.sv_parallel.get():
            cmd += ["--parallel", "--workers", self.sv_workers.get()]

        if self.sv_b_save.get():
            cmd.append("--save-plots")

        self._launch(cmd, self.btn_batch)

    def _launch(self, cmd: list[str], btn: tk.Button):
        btn.configure(state="disabled")
        self._log(f"$ {' '.join(cmd)}\n", "cmd")

        def _worker():
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True, bufsize=1,
                    encoding="utf-8", errors="replace")
                for line in proc.stdout:
                    tag = None
                    lo  = line.lower()
                    if any(k in lo for k in ["error", "오류", "traceback", "exception", "✘"]):
                        tag = "err"
                    elif any(k in lo for k in ["warn", "경고", "[!]"]):
                        tag = "warn"
                    elif any(k in lo for k in ["✔", "완료", "saved", "저장", "done"]):
                        tag = "ok"
                    elif any(k in lo for k in ["◆", "phase", "──"]):
                        tag = "info"
                    self._log(line, tag)

                proc.wait()
                if proc.returncode == 0:
                    self._log("✔  완료\n", "ok")
                else:
                    self._log(f"✘  종료 코드: {proc.returncode}\n", "err")

            except Exception as exc:
                self._log(f"실행 오류: {exc}\n", "err")
            finally:
                self.root.after(0, lambda: btn.configure(state="normal"))

        threading.Thread(target=_worker, daemon=True).start()

    # ─────────────────────────────────────────────────────────────────────────
    # 콘솔 헬퍼
    # ─────────────────────────────────────────────────────────────────────────
    def _log(self, text: str, tag: str | None = None):
        def _do():
            self.console.configure(state="normal")
            self.console.insert("end", text, tag or "")
            self.console.see("end")
            self.console.configure(state="disabled")
        self.root.after(0, _do)

    def _clear_console(self):
        self.console.configure(state="normal")
        self.console.delete("1.0", "end")
        self.console.configure(state="disabled")


# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Crusher Tablet Simulator GUI")
    ap.add_argument("--stl", default="", help="미리 지정할 STL 파일 경로")
    args = ap.parse_args()

    root = tk.Tk()

    # Windows 고DPI 인식
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    TabletSimGUI(root, init_stl=args.stl)
    root.mainloop()


if __name__ == "__main__":
    main()
