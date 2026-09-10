#!/bin/bash
# above_speed_sweep.sh — 이송 속도 사다리 (사용자 지시, 2026-09-10)
#
# full_workflow 의 above 구간(Q_LIFT -> 슬롯 위 IK 자세)을 격리 씬에서 재현했다.
# 같은 경로를 N_ABOVE 스텝 수만 바꿔 지나간다 = 경로 동일, 속도만 변인.
#
# 판정: 관성력은 v^2 에 비례한다. 스텝을 2배로 늘리면(속도 절반) 가속도 잔차가
# **4배로 줄어야** 물리적으로 정상이다. 덜 줄면 수치 문제다.
#   기준 실측 N_ABOVE=400 에서 봉투 |a| = 1.768 m/s^2
#   v^2 예측:  200 -> 7.07,  400 -> 1.768,  800 -> 0.442,  1600 -> 0.111
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/probe || exit 1
OUT=_above_speed; mkdir -p $OUT
for n in 200 400 800 1600 3200; do
  t0=$SECONDS
  echo "[SPD] N_ABOVE=$n 시작  $(date +%H:%M:%S)"
  env MOVE=above N_GRAINS=524 N_ABOVE=$n NO_VIDEO=1 TAG="above_n$n" \
    $PY -u probe_arm_bag.py > "$OUT/n$n.log" 2>&1
  rc=$?; el=$((SECONDS-t0))
  line=$(grep -E "^above " "$OUT/n$n.log" | head -1 | tr -s ' ')
  err=$(grep -E "Traceback|body_count|not valid" "$OUT/n$n.log" | head -1)
  echo "[SPD] N_ABOVE=$n  exit=$rc  ${el}s  | $line ${err:+ERR: $err}"
done
echo "[SPD] 완료  $(date +%H:%M:%S)"
