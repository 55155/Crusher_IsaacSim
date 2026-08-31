#!/usr/bin/env bash
# 압착 파라미터 야간 스윕 (2026-08-31)
#
# 이 기계는 병렬 폭이 좁다 — RAM 16.8GB(런당 약 3.6GB), VRAM 12.2GB(런당 약 4GB).
# 그래서 **레인 2개**가 상한이다. 레인마다 설정 목록을 순차로 돌린다.
#
# 각 런은 타임아웃을 건다. clamp 에서 IPC 배리어가 갈리면 20분이 넘도록 진행이
# 없는 사례가 여러 번 있었고(§19-4, 이번 -15.5mm 위치제어 런), 그런 설정 하나가
# 밤을 통째로 먹으면 안 된다. 타임아웃은 STALL 로 기록하고 다음으로 넘어간다.
#
# 판정의 핵심 지표는 로그의 [SUMMARY] 줄, 그중에서도 **drop** 이다 — 그리퍼를
# 놓은 뒤 봉투가 흘러내린 양(§18-4 실패 실측 -34.9mm). hold=0 인 런만 유효하다.
set -u

PY="C:/Users/user/AppData/Local/Programs/Python/Python311/python.exe"
DIR="C:/Users/user/Desktop/Crusher_IsaacSim/Crusher_Genesis/Crusher_M0609_RG2_Tablet_Samplebag"
OUT="$DIR/SWEEP"
TIMEOUT="${SWEEP_TIMEOUT:-1500}"        # 런당 25분
mkdir -p "$OUT"
CSV="$OUT/summary.tsv"
[ -f "$CSV" ] || printf 'run\tstatus\tsecs\tsummary\n' > "$CSV"

run_one() {
  local name="$1"; shift
  local log="$OUT/$name.log"
  # 이미 성공한 런은 건너뛴다 — 스크립트를 다시 돌려도 이어서 진행된다.
  if grep -q '^\[SUMMARY\]' "$log" 2>/dev/null; then
    echo "[skip] $name (이미 완료)"; return
  fi
  echo "[run ] $name : $*"
  printf '[cfg] %s\n' "$*" > "$log"
  local t0 t1 rc
  t0=$(date +%s)
  # RUN_TAG 로 런마다 RESULT 디렉터리를 분리한다 — 중간 산출물(_bag_seal_uv.obj 등)
  # 경로가 겹치면 병렬 런끼리 서로의 파일을 덮어써서 전부 죽는다.
  timeout "$TIMEOUT" env "$@" RUN_TAG="$name" LAYOUT_FROM_STEP=1 NO_VIDEO=1 "$PY" -u "$DIR/full_workflow.py" >> "$log" 2>&1
  rc=$?
  t1=$(date +%s)
  local sm status
  sm=$(grep -m1 '^\[SUMMARY\]' "$log" | sed 's/^\[SUMMARY\] //')
  if [ "$rc" = "124" ]; then
    status=STALL
    sm="clamp 마지막 관측: $(grep -E '^\s+\[clamp t=' "$log" | tail -1 | sed 's/^ *//')"
  elif [ -n "$sm" ]; then
    status=OK
  else
    status="ERR($rc)"
    sm=$(grep -m1 -E 'RuntimeError|Error:|Exception' "$log" | cut -c1-160)
  fi
  printf '%s\t%s\t%s\t%s\n' "$name" "$status" "$((t1-t0))" "$sm" >> "$CSV"
  echo "[done] $name -> $status ($((t1-t0))s)"
}

lane_g() {
  # 레인 G — f1(기구학 구동, CLAMP_TARGET=-11.5mm)이 처음으로 PASS 했다:
  #   wall=-11.34mm  본체면 잔여 6.16mm  **drop=-0.0mm**  verdict=PASS
  # 리포지토리 기록상 같은 설정이 FAIL/PASS 로 갈린 전례가 있으므로(§13-11) 재현을
  # 먼저 굳히고, 그 다음 "기구학 구동만으로 된 것인가 / 플랜지 제거도 필요했나"를
  # 대조군으로 가른다.
  run_one g1_kin115_r2 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-11.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one g2_kin115_r3 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-11.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one g3_kin115_flangeON WALL_KINEMATIC=1 CLAMP_TARGET_MM=-11.5 LEFTWALL_FLANGE_CONTACT=1 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one g4_kin125 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-12.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one g5_kin115_r4 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-11.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one g6_kin125_r2 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-12.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
}

lane_g
echo "[sweep-g] 완료"
