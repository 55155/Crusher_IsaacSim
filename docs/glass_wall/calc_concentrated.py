"""중앙 집중하중 두께 설계 + 벽/분체층 구조 강성 비교 (Roark Table 11.4 case 1d: 단순지지 정사각판, 중앙 원형 하중)."""
import numpy as np
E_g, nu = 70e3, 0.22           # MPa
a = 56.0                       # 지지 개구부 [mm]
F = 1500.0                     # 설계 반력 [N]
beta, alpha = 0.435, 0.1267    # a/b=1: 응력계수, 집중하중 처짐계수 (y = alpha W a^2 / (E t^3))
def r_eq(r0, t):               # 얇은 판 보정 (r0 < 0.5 t 일 때 등가 반경)
    return np.sqrt(1.6 * r0**2 + t**2) - 0.675 * t if r0 < 0.5 * t else r0
def sigma(F, r0, t):
    return 3 * F / (2 * np.pi * t**2) * ((1 + nu) * np.log(2 * a / (np.pi * r_eq(r0, t))) + beta)
print("== 1500 N 이 중앙 원(반경 r0)에 몰릴 때 최대응력 [MPa] ==")
ts = (6, 8, 10, 12, 15)
print("r0[mm]  " + "  ".join(f"t={t:>2}" for t in ts))
for r0 in (25, 10, 5, 2.5, 1.0, 0.5):
    print(f"{r0:5.1f}   " + "  ".join(f"{sigma(F, r0, t):5.1f}" for t in ts))
print("\n== 허용응력별 필요 두께 (가장 나쁜 r0=0.5mm, 1500 N) ==")
for allow in (50, 40, 30):
    t = next(t for t in np.arange(4, 30, 0.5) if sigma(F, 0.5, t) <= allow); print(f"allow {allow} MPa -> t >= {t} mm")
print("\n== 집중하중 처짐/강성 (t 별) ==")
for t in ts:
    d = alpha * F * a**2 / (E_g * t**3) * 1e3; print(f"t={t:2d}: 처짐 {d:5.1f} um, k = {F/d:5.0f} N/um")
print("\n== 기존 SUS 벽 외팔보: 뿌리 모멘트 vs 유리판 단위폭 모멘트 ==")
print(f"외팔보 뿌리 M = F*L = {F*35/1e3:.1f} N.m (폭 120mm 전체가 받음 -> {F*35/120:.0f} N.mm/mm)")
q = F / 50**2
print(f"유리 4변지지 균일 Mmax = 0.0479 q a^2 = {0.0479*q*a**2:.0f} N.mm/mm")
print("\n== 분체층 구조 강성 k_bed = E_bed*A/h  vs 벽 ==")
for Eb in (10, 50, 200):
    for h in (1, 3):
        kb = Eb * 2500 / h / 1e3  # N/um
        print(f"E_bed {Eb:3d} MPa h {h} mm: k_bed {kb:6.1f} N/um | 유리t10(균일) 401 -> x{401/kb:5.1f} | 기존SUS 135 -> x{135/kb:5.1f} | 벽이 먹는 변위 {kb/(kb+401)*100:4.1f}%")
