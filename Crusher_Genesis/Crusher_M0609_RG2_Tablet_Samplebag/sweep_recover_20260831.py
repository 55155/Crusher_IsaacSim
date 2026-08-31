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

CLEAR_Z = [20, 40, 60, 90]      # mm, 턱 상단 위 여유
WAYS = [9, 21]                  # 하강 웨이포인트 수
N_DOWN = [900, 1800]            # 하강 스텝
SHAFT = ["set", "control"]      # 샤프트 구동 방식(사용자 지시로 축에 추가)

BASE = dict(
    RECOVER="1",
    CRUSH_SECONDS="15",
    NO_VIDEO="1",               # 수치만 본다. 이긴 설정만 나중에 영상으로 다시.
    N_RC_SETTLE="150",          # 정착은 효과가 없었다 — 원래대로 줄인다
    N_RC_STILL="150",
)


def run_one(job):
    cz, wy, nd, sm = job
    name = f"CZ{cz}_W{wy}_D{nd}_{sm}"
    log = os.path.join(OUT, f"{name}.log")
    if os.path.exists(log) and "[recover] **" in open(log, encoding="utf-8",
                                                      errors="replace").read():
        print(f"[skip] {name}", flush=True)
        return name, cz, wy, nd, sm, None
    env = dict(os.environ, **BASE, RUN_TAG=name,
               RECOVER_CLEAR_Z_MM=str(cz), RECOVER_DOWN_WAYS=str(wy),
               N_RC_DOWN=str(nd), RC_SHAFT_MODE=sm)
    print(f"[run ] {name}  여유={cz}mm  ways={wy}  하강={nd}  샤프트={sm}", flush=True)
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
    d = re.search(r"해제 후 낙하 ([-+\d.]+)mm", txt)
    if d:
        g["drop"] = float(d.group(1))
    t = re.search(r"봉투하단 z=([\d.]+)", txt)
    if t:
        g["tgt"] = float(t.group(1))
    print(f"[done] {name} -> {g if g else '실패'}", flush=True)
    return name, cz, wy, nd, sm, (g or None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lanes", type=int, default=2)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    jobs = [(cz, wy, nd, sm) for cz in CLEAR_Z for wy in WAYS
            for nd in N_DOWN for sm in SHAFT]
    print(f"총 {len(jobs)}개 런, 레인 {min(args.lanes, 2)}개\n")
    with ThreadPoolExecutor(max_workers=min(args.lanes, 2)) as ex:
        res = list(ex.map(run_one, jobs))

    print("\n" + "=" * 84)
    print(f"{'런':26s}{'여유':>6}{'ways':>6}{'하강':>7}{'샤프트':>9}   "
          f"{'tilt':>8}{'수평오차':>10}{'낙하':>8}{'하단오차':>10}")
    print("-" * 84)
    ok = []
    for name, cz, wy, nd, sm, g in res:
        if not g:
            print(f"{name:26s}{cz:>6}{wy:>6}{nd:>7}{sm:>9}   실패 "
                  f"(SWEEP_RECOVER/{name}.log)")
            continue
        bot_err = (g.get("bot", float("nan")) - g.get("tgt", float("nan"))) * 1000
        print(f"{name:26s}{cz:>6}{wy:>6}{nd:>7}{sm:>9}   "
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
        cz, wy, nd, sm = re.match(r"CZ(\d+)_W(\d+)_D(\d+)_(\w+)", n).groups()
        print(f"  RUN_TAG=best RECOVER=1 CRUSH_SECONDS=15 RECOVER_CLEAR_Z_MM={cz} "
              f"RECOVER_DOWN_WAYS={wy} N_RC_DOWN={nd} RC_SHAFT_MODE={sm} "
              f"python full_workflow.py")


if __name__ == "__main__":
    main()
