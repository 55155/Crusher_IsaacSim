"""벽 압착을 control_dofs_force 로 — 힘 x 반사관성 스윕 (2026-08-31, 사용자 지시).

왜 이 스윕인가
--------------
docs/DigitalTwin.md §20 이 "힘제어"라 부른 것은 전부 `control_dofs_position` +
`force_range` 클램프였다. **`control_dofs_force` 는 이 저장소에서 벽에 한 번도
걸린 적이 없다**(전체 grep: 그리퍼 예제 2곳뿐). 둘은 다르다 —

    control_dofs_position + clamp : 목표로 PD 가 밀다 상한에 포화. 목표가
                                    도달 불가면 배리어에 계속 밀어붙인다.
    control_dofs_force            : 일정한 힘으로 민다. 봉투 반력과 균형지는
                                    곳에서 **스스로 선다**.

§20-4-2 가 남긴 결론이 근거다 — ±100N 은 Motor2 최대 스펙이지 운전 힘이 아니고,
**2.5g 필름은 어떤 압착 두께에서도 100N 을 만들지 못한다.** 균형점이 없으니 벽이
항상 끝까지 닫혔다. 야간 스윕은 100 -> 800N 으로 **올리기만** 했고 그 아래는 한
번도 보지 않았다. 참고로 성공하는 그리퍼는 40N 으로 봉투를 잡는다.

FORCE 모드는 부수 효과도 하나 없앤다. `ctrl_mode` 는 POSITION=0/VELOCITY=1/
FORCE=2 이고 암시적 관성 보정은 `ctrl_mode <= VELOCITY` 에서만 걸리므로
(forward_dynamics.py:317), 오늘 크랭크를 죽인 `kv*dt` 부풀림을 안 탄다.

측정하려는 것
------------
정지 위치 = 실효 압착 두께, 그때의 힘 = **필요 압착력의 첫 수치**(§11-5 가 원래
재려던 값). 판정은 `drop` — 그리퍼를 놓은 뒤 봉투가 흘러내린 양이다.

    stalled=Y (하드스톱 전에 스스로 멈춤) + drop ~ 0  이면 성립.

사용법
------
    python sweep_force_20260831.py              # 전체 매트릭스
    python sweep_force_20260831.py --lanes 1    # 단일 레인(느리지만 안전)

병렬 상한은 2다(§20-5: RAM 16.8GB / 런당 3.6GB, VRAM 12.2GB / 런당 3.5GB).
3개 이상 띄우면 above(이송) 구간이 정체해 clamp 까지 가지 못한다 — 압착 실패가
아니라 자원 문제이므로 결과가 오염된다.

출력: SWEEP_FORCE/<run>.log  및  RESULT_<run>/grip-28mm/*.mp4
"""
import argparse
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "SWEEP_FORCE")
PY = sys.executable

# 힘(N) x 반사관성(kg). 힘은 그리퍼의 40N 아래를 훑고, 관성은 §20-4-1 에서
# 관통 속도를 23배 줄인 것이 확인된 축이다(런타임 API 로 걸어야 한다).
FORCES = [2.0, 5.0, 10.0, 20.0, 40.0]
ARMATURES = [10.0, 100.0]

BASE = dict(
    LAYOUT_FROM_STEP="1",
    CLAMP_MODE="force",
    WALL_KINEMATIC="0",          # 힘지령이므로 기구학 구동은 끈다
    LEFTWALL_FLANGE_CONTACT="1",
    HOLD_THROUGH_CLAMP="0",      # drop 을 재려면 놓아야 한다
    GRIP_OFFSET_MM="-28",        # 검증된 파지(중앙 파지는 삽입이 19.5mm 얕았다)
    CRUSH_SECONDS="0",           # 압착만 본다
    WALL_CLOSE_MMPS="8",
)


def run_one(job):
    force, arm = job
    name = f"F{force:g}_A{arm:g}"
    log = os.path.join(OUT, f"{name}.log")
    if os.path.exists(log) and "[SUMMARY]" in open(log, encoding="utf-8",
                                                   errors="replace").read():
        print(f"[skip] {name}", flush=True)
        return name, None
    env = dict(os.environ, **BASE, RUN_TAG=name,
               WALL_PUSH_N=str(force), WALL_ARMATURE=str(arm))
    print(f"[run ] {name}  힘={force}N  armature={arm}kg", flush=True)
    with open(log, "w", encoding="utf-8") as f:
        p = subprocess.run([PY, "-u", os.path.join(HERE, "full_workflow.py")],
                           env=env, stdout=f, stderr=subprocess.STDOUT)
    txt = open(log, encoding="utf-8", errors="replace").read()
    m = re.search(r"^\[SUMMARY\] (.*)$", txt, re.M)
    print(f"[done] {name} -> {'OK' if m else 'rc=' + str(p.returncode)}", flush=True)
    return name, (m.group(1) if m else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lanes", type=int, default=2, help="병렬 레인 수 (상한 2)")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    jobs = [(f, a) for a in ARMATURES for f in FORCES]
    print(f"총 {len(jobs)}개 런, 레인 {min(args.lanes, 2)}개\n")
    with ThreadPoolExecutor(max_workers=min(args.lanes, 2)) as ex:
        results = list(ex.map(run_one, jobs))

    print("\n" + "=" * 96)
    print(f"{'런':12s}{'힘(N)':>7}{'armature':>10}   {'벽 정지':>9}{'본체면':>8}"
          f"{'stalled':>9}{'drop':>8}   판정")
    print("-" * 96)
    for name, sm in results:
        if not sm:
            print(f"{name:12s}{'':>7}{'':>10}   실패 (로그 확인: SWEEP_FORCE/{name}.log)")
            continue
        g = dict(re.findall(r"(\w+)=([^\s]+)", sm))
        f_, a_ = name[1:].split("_A")
        print(f"{name:12s}{f_:>7}{a_:>10}   {g.get('wall','?'):>9}"
              f"{g.get('body_gap','?'):>8}{g.get('stalled','?'):>9}"
              f"{g.get('drop','?'):>8}   {g.get('verdict','?')}")
    print("=" * 96)
    print("성립 기준: stalled=Y (하드스톱 전 자력 정지) + drop ~ 0")
    print("그 정지 위치가 실효 압착 두께, 그때의 힘이 필요 압착력이다.\n")
    print("영상:")
    for name, sm in results:
        d = os.path.join(HERE, f"RESULT_{name}", "grip-28mm")
        if os.path.isdir(d):
            for fn in sorted(os.listdir(d)):
                if fn.endswith(".mp4"):
                    print(f"  RESULT_{name}/grip-28mm/{fn}")


if __name__ == "__main__":
    main()
