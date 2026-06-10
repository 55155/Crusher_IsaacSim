"""
viewer_desktop.py
Crusher Simulator launcher GUI.

Runs either:
  - crusher_velocity_ctrl.py        (headless, 20s fixed)
  - crusher_velocity_ctrl_viewer.py (MuJoCo viewer + realtime plot)
"""

import os, sys, subprocess
import tkinter as tk
from tkinter import filedialog, messagebox

_HERE   = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))

_HEADLESS = os.path.join(_HERE, "crusher_velocity_ctrl.py")
_VIEWER   = os.path.join(_HERE, "crusher_velocity_ctrl_viewer.py")

_KV_DEFAULT = "14.9"


# ── Helpers ───────────────────────────────────────────────────────────
def _browse():
    path = filedialog.askopenfilename(
        title="Select Tablet STL",
        initialdir=STL_DIR if os.path.isdir(STL_DIR) else _HERE,
        filetypes=[("STL Files", "*.stl"), ("All Files", "*.*")],
    )
    if path:
        _stl_var.set(path)


def _get_params():
    """Returns (stl, rpm, kv, density) or None on validation error."""
    stl = _stl_var.get().strip()
    if not stl or not os.path.isfile(stl):
        messagebox.showerror("오류", "유효한 STL 파일을 선택하세요.")
        return None
    try:
        rpm     = float(_rpm_var.get())
        kv      = float(_kv_var.get())
        density = float(_den_var.get())
    except ValueError:
        messagebox.showerror("오류", "RPM / kv / Density 값을 확인하세요.")
        return None
    return stl, rpm, kv, density


def _launch(script):
    params = _get_params()
    if params is None:
        return
    stl, rpm, kv, density = params
    cmd = [sys.executable, script, stl,
           "--rpm",     str(rpm),
           "--kv",      str(kv),
           "--density", str(density)]
    subprocess.Popen(cmd, cwd=_HERE)
    _status_var.set(f"실행: {os.path.basename(script)}")


# ── Build GUI ─────────────────────────────────────────────────────────
root = tk.Tk()
root.title("Crusher Simulator")
root.resizable(False, False)

_PAD = {"padx": 10, "pady": 5}

# ── Title ─────────────────────────────────────────────────────────────
tk.Label(root, text="Crusher Simulator",
         font=("Arial", 14, "bold")).grid(
    row=0, column=0, columnspan=3, pady=(12, 6))

tk.Frame(root, height=1, bg="#cccccc").grid(
    row=1, column=0, columnspan=3, sticky="ew", padx=10)

# ── STL file ──────────────────────────────────────────────────────────
tk.Label(root, text="STL File").grid(row=2, column=0, sticky="e", **_PAD)
_stl_var = tk.StringVar()
tk.Entry(root, textvariable=_stl_var, width=46).grid(row=2, column=1, **_PAD)
tk.Button(root, text="Browse", width=8, command=_browse).grid(
    row=2, column=2, **_PAD)

# ── Parameters ────────────────────────────────────────────────────────
_rpm_var = tk.StringVar(value="8.0")
_kv_var  = tk.StringVar(value=_KV_DEFAULT)
_den_var = tk.StringVar(value="1200")

for row, (label, var, unit) in enumerate([
    ("RPM",            _rpm_var, "rpm"),
    ("kv",             _kv_var,  "N·m·s/rad"),
    ("Density",        _den_var, "kg/m³"),
], start=3):
    tk.Label(root, text=label).grid(row=row, column=0, sticky="e", **_PAD)
    tk.Entry(root, textvariable=var, width=10).grid(row=row, column=1, sticky="w", **_PAD)
    tk.Label(root, text=unit, fg="gray").grid(row=row, column=2, sticky="w")

# ── Divider ───────────────────────────────────────────────────────────
tk.Frame(root, height=1, bg="#cccccc").grid(
    row=6, column=0, columnspan=3, sticky="ew", padx=10, pady=(6, 0))

# ── Buttons ───────────────────────────────────────────────────────────
_btn_frame = tk.Frame(root)
_btn_frame.grid(row=7, column=0, columnspan=3, pady=12)

tk.Button(
    _btn_frame, text="▶  Run Headless", width=20,
    bg="#4a90d9", fg="white", font=("Arial", 10, "bold"),
    command=lambda: _launch(_HEADLESS),
).pack(side="left", padx=10)

tk.Button(
    _btn_frame, text="▶  Run with Viewer", width=20,
    bg="#5cb85c", fg="white", font=("Arial", 10, "bold"),
    command=lambda: _launch(_VIEWER),
).pack(side="left", padx=10)

# ── Status bar ────────────────────────────────────────────────────────
_status_var = tk.StringVar(value="대기 중")
tk.Label(root, textvariable=_status_var, fg="gray", font=("Arial", 9)).grid(
    row=8, column=0, columnspan=3, pady=(0, 10))

root.mainloop()
