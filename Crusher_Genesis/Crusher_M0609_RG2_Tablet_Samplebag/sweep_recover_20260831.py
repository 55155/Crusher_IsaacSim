"""회수장치 투입 자세 스윕 — 진입 여유 x 하강 웨이포인트 x 하강 속도.

문제
----
분쇄 후 봉투를 회수장치 턱 사이에 내려놓는 구간에서 봉투가 27~30도 기울어 물린다.
설계 의도는 좌우 실링이 각각 4.5mm 씩 물리는 것인데, 실측에서는 봉투 중심이 y 로
6.5mm 밀려 **한쪽 턱만** 물었다.

원인(사용자 지적 2026-08-31): 진입 고도가 낮아 **이송 중에 봉투가 F/M 링크를
쓸고 지나간다.**

    턱 z 대역          world 0.2005 ~ 0.3005   (로컬 48~148mm, 네 링크 공통)
    진입 시 봉투 하단  0.2805                  <- 턱 상단보다 20mm 아래

tilt 가 extract 6.0deg -> toRC 30.5deg 로 커지는 것이 이 간섭이다. 정착 스텝을
400 으로 늘려도 29.6 -> 27.6deg 밖에 안 줄었다 — 감쇠하는 진동이 아니라 간섭으로
생긴 정적 자세이기 때문이다.

스윕 축
------
  CLEAR_Z  진입 시 봉투 하단이 턱 상단보다 얼마나 위에 있어야 하는가
  WAYS     턱 사이 하강을 몇 개의 카테시안 웨이포인트로 쪼갤 것인가
  N_DOWN   하강 스텝 수(느릴수록 흔들림이 덜 들어간다)

판정
----
  tilt          투입 시점 기울기. 낮을수록 좋다(목표 < 10deg)
  수평오차      봉투 중심 vs 턱 물림 중심. 설계 여유가 4.5mm 라 그 이하가 목표
  drop          해제 후 낙하. 0 이어야 물린 것이다
  bag_bottom    목표 z 대비. 봉투가 접히면 크게 뜬다

사용법
------
    python sweep_recover_20260831.py            # 전체(2레인)
    python sweep_recover_20260831.py --lanes 1  # 단일 레인

병렬 상한은 2다(§20-5). 3개 이상은 above 구간이 정체해 결과가 오염된다.
출력: SWEEP_RECOVER/<run>.log
"""
import argparse
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "SWEEP_RECOVER")
PY = sys.executable

# **[수정 2026-09-01] 축을 도달 가능한 범위로 다시 잡았다.**
# "턱을 완전히 넘겨서 떨군다"는 접근은 이 배치에서 성립하지 않는다 — 봉투가
# 핑거 아래 82mm 에 매달리는데 턱 상단이 0.3005 라 핑거가 0.3825 이상이어야
# 하고, 그 (x,y) 에서 팔이 닿는 높이는 0.40 언저리다. 여유가 20mm 뿐이다.
#     CLEAR_Z=20 -> 진입 z=0.4025, IK 오차  5.3mm
#     CLEAR_Z=40 -> 진입 z=0.4225, IK 오차 17.2mm  (도달 불가)
# 그래서 CLEAR_Z 는 끄고(0) **진입 고도 자체**를 도달 범위 안에서 훑는다.
# 대신 yaw 를 축으로 넣는다 — ad_plan(135deg)은 tilt 2.0deg, ae_final(90deg)은
# 29.6deg 였다. 실링 정렬은 90deg 가 맞지만 그 자세에서 왜 기우는지가 미해결이라,
# 두 자세를 같은 조건에서 나란히 본다.
APPROACH_H = [60, 80, 100, 120] # mm, 물림 자세 위로 얼마나 띄워 진입할 것인가
# **[실측 2026-09-01] WAYS 는 무관한 축이라 9 로 고정했다.**
# H80_W9_D900_Y90 과 H80_W21_D900_Y90 이 소수점까지 동일했다(4.4mm / 7.3deg).
# 이송 보간을 고치고 나니 하강 자체는 짧고 거의 수직이라 쪼갤 이유가 없다.
WAYS = [9]
N_DOWN = [600, 900, 1800]       # 하강 스텝
YAW = [90, 135]                 # 실링 정렬(90) vs 저 tilt 로 알려진 자세(135)

BASE = dict(
    RECOVER="1",
    CRUSH_SECONDS="15",
    NO_VIDEO="1",               # 수치만 본다. 이긴 설정만 나중에 영상으로 다시.
    N_RC_SETTLE="150",          # 정착은 효과가 없었다 — 원래대로 줄인다
    N_RC_STILL="150",
    RECOVER_CLEAR_Z_MM="0",     # 턱 상단 역산은 끈다(도달 불가) — APPROACH_H 로 훑는다
    RECOVER_REACH_TOL_MM="15",  # 넘으면 이송을 건너뛰고 로그에 남는다
)


def run_one(job):
    cz, wy, nd, yw = job
    name = f"H{cz}_W{wy}_D{nd}_Y{yw}"
    log = os.path.join(OUT, f"{name}.log")
    if os.path.exists(log) and "[recover] **" in open(log, encoding="utf-8",
                                                      errors="replace").read():
        print(f"[skip] {name}", flush=True)
        return name, cz, wy, nd, yw, None
    env = dict(os.environ, **BASE, RUN_TAG=name,
               RECOVER_APPROACH_H_MM=str(cz), RECOVER_DOWN_WAYS=str(wy),
               N_RC_DOWN=str(nd), RC_WRIST_DEG=str(yw), RC_SHAFT_MODE="set")
    print(f"[run ] {name}  진입={cz}mm  ways={wy}  하강={nd}  yaw={yw}", flush=True)
    with open(log, "w", encoding="utf-8") as f:
        subprocess.run([PY, "-u", os.path.join(HERE, "full_workflow.py")],
                       env=env, stdout=f, stderr=subprocess.STDOUT)
    txt = open(log, encoding="utf-8", errors="replace").read()
    g = {}
    m = re.search(r"투입 완료.*?수평오차=([-\d.]+)mm\s+bag_bottom=([-\d.]+)"
                  r".*?tilt=([-\d.]+)deg", txt, re.S)
    if m:
        g["err"], g["bot"], g["tilt"] = (float(m.group(1)), float(m.group(2)),
                                        float(m.group(3)))
    if "도달 불가" in txt:
        g["skip"] = "도달불가"
    if "봉투 발산" in txt:
        g["skip"] = "발산"
    d = re.search(r"해제 후 낙하 ([-+\d.]+)mm", txt)
    if d:
        g["drop"] = float(d.group(1))
    t = re.search(r"봉투하단 z=([\d.]+)", txt)
    if t:
        g["tgt"] = float(t.group(1))
    print(f"[done] {name} -> {g if g else '실패'}", flush=True)
    return name, cz, wy, nd, yw, (g or None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lanes", type=int, default=2)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    jobs = [(cz, wy, nd, yw) for cz in APPROACH_H for wy in WAYS
            for nd in N_DOWN for yw in YAW]
    print(f"총 {len(jobs)}개 런, 레인 {min(args.lanes, 2)}개\n")
    with ThreadPoolExecutor(max_workers=min(args.lanes, 2)) as ex:
        res = list(ex.map(run_one, jobs))

    print("\n" + "=" * 84)
    print(f"{'런':26s}{'진입':>6}{'ways':>6}{'하강':>7}{'yaw':>6}   "
          f"{'tilt':>8}{'수평오차':>10}{'낙하':>8}{'하단오차':>10}")
    print("-" * 84)
    ok = []
    for name, cz, wy, nd, yw, g in res:
        if not g:
            print(f"{name:26s}{cz:>6}{wy:>6}{nd:>7}{yw:>6}   실패/생략 "
                  f"(SWEEP_RECOVER/{name}.log)")
            continue
        bot_err = (g.get("bot", float("nan")) - g.get("tgt", float("nan"))) * 1000
        print(f"{name:26s}{cz:>6}{wy:>6}{nd:>7}{yw:>6}   "
              f"{g.get('tilt', float('nan')):>7.1f}°{g.get('err', float('nan')):>9.1f}mm"
              f"{g.get('drop', float('nan')):>7.1f}mm{bot_err:>9.1f}mm")
        ok.append((g.get("tilt", 1e9), g.get("err", 1e9), name))
    print("=" * 84)
    if ok:
        ok.sort()
        print(f"tilt 최소: {ok[0][2]}  (tilt {ok[0][0]:.1f}°, 수평오차 {ok[0][1]:.1f}mm)")
        print("설계 여유가 실링당 4.5mm 이므로 수평오차는 그 이하가 목표다.")
        print(f"\n이긴 설정으로 영상 런:")
        n = ok[0][2]
        cz, wy, nd, yw = re.match(r"H(\d+)_W(\d+)_D(\d+)_Y(\d+)", n).groups()
        print(f"  RUN_TAG=best RECOVER=1 CRUSH_SECONDS=15 RECOVER_APPROACH_H_MM={cz} "
              f"RECOVER_DOWN_WAYS={wy} N_RC_DOWN={nd} RC_WRIST_DEG={yw} "
              f"python full_workflow.py")


if __name__ == "__main__":
    main()
