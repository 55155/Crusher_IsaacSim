"""
impulse_analysis.py
크랭크-슬라이더 끝점 반력 역산 — 충격량 결정 요소 시각화

물리 모델:
    1. 크랭크-슬라이더 기구학  →  슬라이더 위치 / 속도
    2. 가상일 원리             →  τ_crank / |dx/dθ|  = F_slider(θ)
    3. 접촉 역학 (과감쇠 스프링-댐퍼) → F_contact(t), 충격량 J

Usage:
    python impulse_analysis.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Circle

# ══════════════════════════════════════════════════════════════════════
# 파라미터 (crusher_velocity_ctrl_viewer.py 와 동일)
# ══════════════════════════════════════════════════════════════════════
R        = 0.020          # 크랭크 반경 [m]
L        = 0.080          # 커넥팅 로드 길이 [m]
RPM      = 8.0            # 크랭크 목표 회전수 [RPM]
OMEGA    = RPM * 2*np.pi / 60   # [rad/s]
TAU_MAX  = 12.5           # 크랭크 최대 토크 [N·m]  (forcelim × gear)

# 접촉 파라미터 (tablet_geom & L9_PLATE 모두 동일)
SOLREF_T   = 0.001        # solref[0] : 시간상수 [s]
SOLREF_D   = 2.0          # solref[1] : 감쇠비 ζ
SOLIMP_MAX = 0.999        # solimp 최대 임피던스

# 유효 질량 (슬라이더 어셈블리 추정)
M_EFF = 0.5               # [kg]

# 접촉 각도 (TDC 전 10° — 알약과 plate가 만나는 지점)
THETA_CONTACT_DEG = 10.0

# ══════════════════════════════════════════════════════════════════════
# 기구학 계산
# ══════════════════════════════════════════════════════════════════════
theta     = np.linspace(0, 2*np.pi, 2000)
theta_deg = np.degrees(theta)
sin_t = np.sin(theta)
cos_t = np.cos(theta)
ratio = R / L

# 슬라이더 위치 (크랭크 중심 기준)
x_slider = R * cos_t + L * np.sqrt(1 - ratio**2 * sin_t**2)

# dx/dθ (위치-각도 전달비)
with np.errstate(invalid='ignore'):
    sqrt_term = np.sqrt(np.maximum(1 - ratio**2 * sin_t**2, 1e-12))
dxdth = -R * sin_t - R**2 * sin_t * cos_t / (L * sqrt_term)

# 슬라이더 속도 [m/s]
v_slider = dxdth * OMEGA

# 슬라이더 가속도 [m/s²]
with np.errstate(invalid='ignore'):
    d2xdth2 = (
        -R * cos_t
        - R**2 * (cos_t**2 - sin_t**2) / (L * sqrt_term)
        - (R**4 * sin_t**2 * cos_t**2) / (L**3 * sqrt_term**3)
    )
a_slider = d2xdth2 * OMEGA**2

# 접촉 지점 인덱스
idx_c    = np.argmin(np.abs(theta_deg - THETA_CONTACT_DEG))
v_impact = abs(v_slider[idx_c])        # [m/s]  접촉 순간 슬라이더 속도
F_from_torque = TAU_MAX / max(abs(dxdth[idx_c]), 1e-6)  # [N]  토크→힘 변환

# 가상일 원리에 의한 슬라이더 반력 (전 각도)
F_slider = np.where(np.abs(dxdth) > 1e-4, TAU_MAX / np.abs(dxdth), np.nan)

# ══════════════════════════════════════════════════════════════════════
# 접촉 역학 (과감쇠 스프링-댐퍼, ζ = 2.0)
# 운동 방정식: m·ẍ + c·ẋ + k·x = 0,  x(0)=0, ẋ(0)=v_impact
# ══════════════════════════════════════════════════════════════════════
omega_n = 1.0 / SOLREF_T                        # 고유 각주파수 [rad/s]
zeta    = SOLREF_D                              # 감쇠비
k_eff   = M_EFF * omega_n**2                   # 등가 강성 [N/m]
c_eff   = 2 * zeta * M_EFF * omega_n           # 등가 감쇠 [N·s/m]
omega_d = omega_n * np.sqrt(zeta**2 - 1)       # 과감쇠 특성근 [rad/s]

# 수치 적분 (RK4)  —  x(0)=0, v(0)=v_impact
dt_c  = 1e-6
t_end = 0.025
t_arr = np.arange(0, t_end, dt_c)
xc    = np.zeros(len(t_arr))
vc    = np.zeros(len(t_arr))
vc[0] = v_impact

for i in range(1, len(t_arr)):
    def accel(xi, vi):
        return -(k_eff * xi + c_eff * vi) / M_EFF

    k1x = vc[i-1];               k1v = accel(xc[i-1], vc[i-1])
    k2x = vc[i-1] + k1v*dt_c/2;  k2v = accel(xc[i-1]+k1x*dt_c/2, vc[i-1]+k1v*dt_c/2)
    k3x = vc[i-1] + k2v*dt_c/2;  k3v = accel(xc[i-1]+k2x*dt_c/2, vc[i-1]+k2v*dt_c/2)
    k4x = vc[i-1] + k3v*dt_c;    k4v = accel(xc[i-1]+k3x*dt_c,   vc[i-1]+k3v*dt_c)

    xc[i] = xc[i-1] + (k1x + 2*k2x + 2*k3x + k4x)*dt_c/6
    vc[i] = vc[i-1] + (k1v + 2*k2v + 2*k3v + k4v)*dt_c/6

    if xc[i] < 0 and i > 10:
        xc[i:] = 0; vc[i:] = 0; break

F_contact = np.maximum(k_eff * xc + c_eff * vc, 0)
J_impulse = np.trapz(F_contact, t_arr)
F_peak    = F_contact.max()
t_peak_ms = t_arr[np.argmax(F_contact)] * 1000

# ══════════════════════════════════════════════════════════════════════
# PLOT
# ══════════════════════════════════════════════════════════════════════
DARK  = '#12131a'
PANEL = '#1c1e2a'
TEXT  = '#dce0e8'
MUTED = '#7a8099'
ACC   = '#4fc3f7'   # 파랑
GRN   = '#69db7c'   # 초록
RED   = '#ff6b6b'   # 빨강
YEL   = '#ffd43b'   # 노랑
ORG   = '#ffa94d'   # 주황
PUR   = '#cc5de8'   # 보라

fig = plt.figure(figsize=(18, 13), facecolor=DARK)
gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.50, wspace=0.38,
                        left=0.06, right=0.97, top=0.93, bottom=0.06)

ax_mech  = fig.add_subplot(gs[0, 0])
ax_pos   = fig.add_subplot(gs[0, 1])
ax_vel   = fig.add_subplot(gs[0, 2])
ax_force = fig.add_subplot(gs[1, :2])
ax_cont  = fig.add_subplot(gs[1, 2])
ax_table = fig.add_subplot(gs[2, :])

def _style(ax, title, xlabel, ylabel, ycolor=TEXT):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TEXT, fontsize=9, fontweight='bold', pad=6)
    ax.set_xlabel(xlabel, color=MUTED, fontsize=8)
    ax.set_ylabel(ylabel, color=ycolor, fontsize=8)
    ax.tick_params(colors=TEXT, labelsize=7.5)
    ax.tick_params(axis='y', colors=ycolor)
    ax.grid(True, alpha=0.18, color='#444')
    for sp in ax.spines.values():
        sp.set_color('#333')

# ── ① 기구 다이어그램 ─────────────────────────────────────────────────
ax_mech.set_facecolor(PANEL)
ax_mech.set_xlim(-0.115, 0.135); ax_mech.set_ylim(-0.065, 0.065)
ax_mech.set_aspect('equal'); ax_mech.axis('off')
ax_mech.set_title("Crank-Slider Mechanism", color=TEXT, fontsize=9, fontweight='bold', pad=6)

th_d = np.radians(30)
px1, py1 = R*np.cos(th_d), R*np.sin(th_d)
sx  = R*np.cos(th_d) + L*np.sqrt(1-(R/L*np.sin(th_d))**2)

ax_mech.add_patch(Circle((0,0), R, fill=False, color=ACC, lw=1.2, ls='--', alpha=0.4))
ax_mech.annotate('', xy=(px1, py1), xytext=(0, 0),
                 arrowprops=dict(arrowstyle='->', color=ACC, lw=2.2))
ax_mech.plot([px1, sx], [py1, 0], color=GRN, lw=2.5, solid_capstyle='round')
ax_mech.add_patch(FancyBboxPatch((sx-0.004, -0.013), 0.026, 0.026,
                                  boxstyle='round,pad=0.002',
                                  fc='#37474f', ec=YEL, lw=1.8))
ax_mech.add_patch(Circle((0, 0),  0.004, color=ACC, zorder=5))
ax_mech.add_patch(Circle((px1, py1), 0.003, color=GRN, zorder=5))
ax_mech.add_patch(Circle((sx, 0), 0.003, color=YEL, zorder=5))

ax_mech.text(px1/2-0.012, py1/2+0.010, f'R = {R*1000:.0f} mm', color=ACC, fontsize=7.5)
ax_mech.text((px1+sx)/2-0.024, py1/2-0.002, f'L = {L*1000:.0f} mm', color=GRN, fontsize=7.5)
ax_mech.text(sx+0.010, 0.003, 'Impact\nPlate', color=YEL, fontsize=7, va='center')
ax_mech.text(0, -0.048, f'ω = {RPM} RPM\nτ_max = {TAU_MAX} N·m', color=ORG,
             fontsize=7.5, ha='center')
# 지면 표시
for gx in [-0.014, 0.000, 0.014]:
    ax_mech.plot([gx, gx-0.007], [-0.028, -0.036], color='#666', lw=1)
ax_mech.plot([-0.018, 0.022], [-0.028, -0.028], color='#666', lw=1.3)

# ── ② 슬라이더 위치 ───────────────────────────────────────────────────
stroke_mm = (x_slider - x_slider.min()) * 1000
_style(ax_pos, "Slider Position vs Crank Angle",
       "Crank angle [°]", "Stroke from BDC [mm]", ACC)
ax_pos.plot(theta_deg, stroke_mm, color=ACC, lw=1.8)
ax_pos.axvline(THETA_CONTACT_DEG, color=RED, ls='--', lw=1.3, alpha=0.85)
ax_pos.text(THETA_CONTACT_DEG+4, stroke_mm[idx_c]-1.5,
            f'θ_c = {THETA_CONTACT_DEG:.0f}°', color=RED, fontsize=7.5)
ax_pos.text(185, stroke_mm.max()*0.46,
            f'Stroke\n= 2R = {(x_slider.max()-x_slider.min())*1000:.1f} mm',
            color=YEL, fontsize=8, ha='center',
            bbox=dict(fc=PANEL, ec='#444', boxstyle='round,pad=3'))
ax_pos.set_xlim(0, 360)

# ── ③ 슬라이더 속도 ───────────────────────────────────────────────────
_style(ax_vel, "Slider Velocity vs Crank Angle",
       "Crank angle [°]", "Velocity [mm/s]", GRN)
ax_vel.plot(theta_deg, v_slider*1000, color=GRN, lw=1.8)
ax_vel.axhline(0, color='#555', lw=0.8)
ax_vel.axvline(THETA_CONTACT_DEG, color=RED, ls='--', lw=1.3, alpha=0.85)
ax_vel.scatter([THETA_CONTACT_DEG], [v_impact*1000], color=RED, s=50, zorder=5)
ax_vel.text(THETA_CONTACT_DEG+4, v_impact*1000 + abs(v_slider).max()*0.05,
            f'v_impact\n= {v_impact*1000:.3f} mm/s', color=RED, fontsize=7.5)
ax_vel.set_xlim(0, 360)

# ── ④ 슬라이더 반력 (가상일) ─────────────────────────────────────────
_style(ax_force,
       "Slider Reaction Force  ·  Virtual Work: F_slider = τ_crank / |dx/dθ|",
       "Crank angle [°]", "F_slider [N]", ORG)
F_clip = np.clip(F_slider, 0, 3000)
ax_force.plot(theta_deg, F_clip, color=ORG, lw=1.8)
ax_force.fill_between(theta_deg, 0, F_clip, alpha=0.12, color=ORG)
ax_force.axvline(THETA_CONTACT_DEG, color=RED, ls='--', lw=1.3, alpha=0.85, label=f'θ_contact = {THETA_CONTACT_DEG}°')
ax_force.axhline(F_from_torque, color=YEL, ls=':', lw=1.2,
                 label=f'F at θ_c = {F_from_torque:.1f} N')
ax_force.scatter([THETA_CONTACT_DEG], [min(F_from_torque, 3000)], color=YEL, s=60, zorder=6)
ax_force.set_xlim(0, 360)
ax_force.legend(fontsize=8, facecolor=PANEL, edgecolor='#444', labelcolor=TEXT)
ax_force.text(185, F_clip.max()*0.55,
              f'Near TDC/BDC: dx/dθ → 0\n→  F_slider → ∞  (singular)',
              color=MUTED, fontsize=7.8, ha='center',
              bbox=dict(fc=PANEL, ec='#444', boxstyle='round,pad=3'))

# ── ⑤ 접촉력 프로파일 (RK4) ──────────────────────────────────────────
_style(ax_cont, "Contact Force Profile  (RK4, overdamped ζ=2)",
       "Time [ms]", "F_contact [N]", RED)
t_ms = t_arr * 1000
ax_cont.plot(t_ms, F_contact, color=RED, lw=1.8)
ax_cont.fill_between(t_ms, 0, F_contact, alpha=0.18, color=RED)
ax_cont.axhline(F_peak, color=YEL, ls=':', lw=1.2)
ax_cont.scatter([t_peak_ms], [F_peak], color=YEL, s=55, zorder=5)
ax_cont.text(t_peak_ms + t_end*1000*0.04, F_peak * 0.88,
             f'F_peak = {F_peak:.1f} N\nat t = {t_peak_ms:.2f} ms',
             color=YEL, fontsize=7.5)
ax_cont.text(t_end*1000*0.55, F_peak*0.35,
             f'J = ∫F dt = {J_impulse:.5f} N·s',
             color=ORG, fontsize=8.5, fontweight='bold',
             bbox=dict(fc=PANEL, ec='#555', boxstyle='round,pad=3'))
ax_cont.set_xlim(0, t_end*1000)

# ── ⑥ 충격량 결정 요소 테이블 ────────────────────────────────────────
ax_table.set_facecolor(PANEL)
ax_table.axis('off')
ax_table.set_title(
    "Impulse Decomposition  ·  J = ∫F dt  ←  각 결정 요소",
    color=TEXT, fontsize=10, fontweight='bold', pad=8)
for sp in ax_table.spines.values():
    sp.set_color('#333')

rows = [
    # (카테고리,  파라미터명,  값,  역할 / 연결관계,  색)
    ("구동",   "τ_max  (크랭크 토크 한계)",
               f"{TAU_MAX} N·m",
               f"F_slider = τ / |dx/dθ|  →  θ_c={THETA_CONTACT_DEG}°에서  F = {F_from_torque:.1f} N",
               ORG),
    ("기구학", "R  (크랭크 반경)",
               f"{R*1000:.0f} mm",
               f"Stroke = 2R = {2*R*1000:.0f} mm   |   R/L = {R/L:.3f}",
               ACC),
    ("기구학", "L  (커넥팅 로드)",
               f"{L*1000:.0f} mm",
               f"dx/dθ|_θc = {dxdth[idx_c]*1000:.3f} mm/rad   (클수록 힘 증폭↑)",
               ACC),
    ("기구학", "ω  (크랭크 각속도)",
               f"{RPM} RPM  =  {OMEGA:.4f} rad/s",
               f"v_impact = |dx/dθ|·ω = {v_impact*1000:.4f} mm/s  at θ={THETA_CONTACT_DEG}°",
               GRN),
    ("접촉",   "m_eff  (접촉 유효 질량)",
               f"≈ {M_EFF:.2f} kg",
               f"m·v_impact = {M_EFF*v_impact:.5f} N·s  (impulse 하한)",
               PUR),
    ("접촉",   "k_eff  (등가 강성)",
               f"{k_eff:.0f} N/m  ←  m/(solref[0])²",
               f"solref[0] = {SOLREF_T} s  (tablet + plate 동일 → harmonic mean = 동일)",
               RED),
    ("접촉",   "ζ  (감쇠비)",
               f"{SOLREF_D:.1f}  (overdamped)  ←  solref[1]",
               f"ω_n = 1/solref[0] = {omega_n:.0f} rad/s   |   c_eff = {c_eff:.0f} N·s/m",
               RED),
    ("결과",   "F_peak  (접촉력 피크)",
               f"{F_peak:.2f} N",
               f"peak at t = {t_peak_ms:.3f} ms  after contact",
               YEL),
    ("결과",   "J  = ∫F dt  (충격량)",
               f"{J_impulse:.6f} N·s",
               f"= m_eff · v_impact · g(ζ)   where  g({SOLREF_D}) = {J_impulse/(M_EFF*v_impact):.4f}",
               YEL),
]

col_x  = [0.005, 0.075, 0.260, 0.430]
col_hd = ["분류", "파라미터", "값", "역할 / 연결관계"]
hd_col = ['#888', '#aab', '#aab', '#aab']
y0 = 0.96
dy = 1.0 / (len(rows) + 1.8)

for j, (hd, hx, hc) in enumerate(zip(col_hd, col_x, hd_col)):
    ax_table.text(hx, y0, hd, color=hc, fontsize=8, fontweight='bold',
                  transform=ax_table.transAxes, va='top')
ax_table.axhline(y0 - 0.04, xmin=0.0, xmax=1.0,
                 color='#444', lw=0.8, transform=ax_table.transAxes)

for i, (cat, name, val, role, clr) in enumerate(rows):
    y = y0 - (i + 1.3) * dy
    bg = '#1e2030' if i % 2 == 0 else PANEL
    ax_table.add_patch(FancyBboxPatch((0, y - dy*0.15), 1.0, dy*0.88,
                                      boxstyle='square,pad=0',
                                      fc=bg, ec='none',
                                      transform=ax_table.transAxes, zorder=0))
    ax_table.text(col_x[0], y, cat,  color=MUTED,  fontsize=7.8,
                  transform=ax_table.transAxes, va='center')
    ax_table.text(col_x[1], y, name, color=clr,    fontsize=8.2, fontweight='bold',
                  transform=ax_table.transAxes, va='center')
    ax_table.text(col_x[2], y, val,  color=TEXT,   fontsize=8.2,
                  transform=ax_table.transAxes, va='center')
    ax_table.text(col_x[3], y, role, color=MUTED,  fontsize=7.8,
                  transform=ax_table.transAxes, va='center')

fig.suptitle(
    "Crank-Slider  ·  끝점(End Point) 슬라이더 반력 역산  —  충격량 결정 요소 분석",
    color=TEXT, fontsize=13, fontweight='bold')

out = "MuJoCo_PlayGround/Sim_result/plot/impulse_analysis.png"
import os; os.makedirs(os.path.dirname(out), exist_ok=True)
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor=DARK)
print(f"Saved: {out}")
plt.show()
