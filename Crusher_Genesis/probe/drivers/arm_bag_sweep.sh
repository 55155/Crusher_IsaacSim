#!/bin/bash
# arm_bag_sweep.sh — 로봇팔 파지 구간 격리 변인통제 (사용자 지시, 2026-09-10)
#
# §25 에서 "파우더가 원인"까지 갈랐다. 다만 그 프로브는 봉투를 **SPC**(소프트
# 위치구속)로 잡았고, 실제 공정은 **마찰 파지**다 — 경계조건이 다르다.
# probe_arm_bag.py 는 로봇 + 봉투 + 파우더만 남기고 실제 파지를 재현한다.
#
# 스모크 실측(60알, 흔들기 1초):
#   lift   핑거 |a| 0.351  봉투 |a| 1.254  -> 3.57배
#   shake  핑거 |a| 0.245  봉투 |a| 1.109  -> 4.53배
# 핑거는 매끈한데 봉투만 덜컥인다. 구동이 아니라 **파지 경계**에서 생긴다는 뜻이라
# GRIP_MU(마찰)를 새 후보로 넣었다.
#
# 조 순서는 가치 순. 도중에 끊겨도 앞쪽만으로 판단이 서도록.
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/probe || exit 1
OUT=_armbag_sweep; mkdir -p $OUT

run () {  # run <tag> <env...>
  local tag=$1; shift
  local t0=$SECONDS
  echo "[ARM] $tag 시작  $(date +%H:%M:%S)"
  env SHAKE_S=6.0 "$@" TAG="$tag" $PY -u probe_arm_bag.py > "$OUT/$tag.log" 2>&1
  local rc=$? el=$((SECONDS-t0))
  local lf=$(grep -E "^lift " "$OUT/$tag.log" | head -1 | tr -s ' ')
  local sh=$(grep -E "^shake " "$OUT/$tag.log" | head -1 | tr -s ' ')
  local err=$(grep -E "Traceback|body_count|not valid|out of memory" "$OUT/$tag.log" | head -1)
  echo "[ARM] $tag  exit=$rc  ${el}s  | $lf | $sh ${err:+ERR: $err}"
}

# ── A. 파우더 대조 (핵심). 영상 켜서 눈으로도 본다 ───────────────────────
run A1_g524_v0 N_GRAINS=524
run A2_g0_v0   N_GRAINS=0

# ── B. 구동 방식 — 속도를 같이 써주면 달라지나 ───────────────────────────
run B1_g524_v1 N_GRAINS=524 ARM_VEL=1 NO_VIDEO=1
run B2_g0_v1   N_GRAINS=0   ARM_VEL=1 NO_VIDEO=1

# ── C. 파지 마찰 — 핑거가 매끈한데 봉투만 덜컥이니 여기가 새 후보 ────────
run C1_mu0.3 N_GRAINS=524 GRIP_MU=0.3 NO_VIDEO=1
run C2_mu1.5 N_GRAINS=524 GRIP_MU=1.5 NO_VIDEO=1

# ── D. 낟알 수 — 같은 2g 을 굵게/곱게 ────────────────────────────────────
run D1_g221_R1.0  N_GRAINS=221  GRAIN_RADIUS_MM=1.0 NO_VIDEO=1
run D2_g1768_R0.5 N_GRAINS=1768 GRAIN_RADIUS_MM=0.5 NO_VIDEO=1

# ── E. dt — §25 에서 비단조였다. 파지 경계에서도 그런지 ──────────────────
run E1_dt2.5 N_GRAINS=524 DT_MS=2.5 NO_VIDEO=1

# ── F. 흔들기 세기 — 덜컥거림이 운동 크기에 비례하나 ─────────────────────
run F1_deg16 N_GRAINS=524 SHAKE_DEG=16 NO_VIDEO=1
run F2_hz2   N_GRAINS=524 SHAKE_HZ=2.0 NO_VIDEO=1

echo "[ARM] 완료  $(date +%H:%M:%S)"
