#!/bin/bash
# bag_shake_sweep.sh — "봉투가 왜 덜컥거리는가" 변인통제 (사용자 지시, 2026-09-10)
#
# 전 구간(full_workflow)은 한 조 45~60분이라 변인통제가 불가능했다. probe_bag_shake.py
# 는 봉투 + 파우더 + 잡아주는 것만 남기고 10초를 흔든다 — 한 조 10~20분.
#
# 지표 두 개를 같이 본다(부호반전율은 작은 잡음에도 50%로 포화돼 못 쓴다):
#   jerk_ratio   실측 |2차차분| / 지령 사인파의 이론값. 1 이면 지령만큼만 움직임
#   jitter_rms   이동평균 뺀 잔차 RMS [mm] = 눈에 보이는 떨림 폭
#
# 스모크 실측(100알, 2초): spc 3.7/0.856  set 4.1/1.059  setvel 2.7/0.624
# -> setvel 이 set 보다 41% 낫지만, 강체가 없는 spc 도 3.7 이라 **구동과 무관한
#    바탕 떨림**이 따로 있다. 그것이 무엇인지가 이 스윕의 주제다.
#
# 조 순서는 가치 순이다. 도중에 끊겨도 앞쪽만으로 판단이 서도록.
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/probe || exit 1
OUT=_shake_sweep; mkdir -p $OUT

run () {  # run <tag> <env...>
  local tag=$1; shift
  local t0=$SECONDS
  echo "[SHAKE] $tag 시작  $(date +%H:%M:%S)"
  env SECONDS=10 "$@" TAG="$tag" $PY -u probe_bag_shake.py > "$OUT/$tag.log" 2>&1
  local rc=$? el=$((SECONDS-t0))
  local j=$(grep -oE "jerk_ratio *[0-9.]+" "$OUT/$tag.log" | head -1)
  local r=$(grep -oE "jitter_rms *[0-9.]+" "$OUT/$tag.log" | head -1)
  local err=$(grep -E "Traceback|body_count|not valid|out of memory" "$OUT/$tag.log" | head -1)
  echo "[SHAKE] $tag  exit=$rc  ${el}s  ${j:-?}  ${r:-?}  ${err:+ERR: $err}"
}

# ── A. 구동 방식 (파우더 있음). 영상 켜서 눈으로도 본다 ──────────────────
run A1_spc      DRIVE=spc    N_GRAINS=524
run A2_set      DRIVE=set    N_GRAINS=524
run A3_setvel   DRIVE=setvel N_GRAINS=524

# ── B. 파우더 대조군 — 떨림이 파우더 때문인가 ────────────────────────────
run B1_spc_dry     DRIVE=spc    N_GRAINS=0 NO_VIDEO=1
run B2_set_dry     DRIVE=set    N_GRAINS=0 NO_VIDEO=1
run B3_setvel_dry  DRIVE=setvel N_GRAINS=0 NO_VIDEO=1

# ── C. dt — 바탕 떨림이 적분 해상도 때문인가 ─────────────────────────────
run C1_dt10   DRIVE=spc N_GRAINS=524 DT_MS=10.0 NO_VIDEO=1
run C2_dt2.5  DRIVE=spc N_GRAINS=524 DT_MS=2.5  NO_VIDEO=1
run C3_dt1.25 DRIVE=spc N_GRAINS=524 DT_MS=1.25 NO_VIDEO=1

# ── D. 천 파라미터 — 유일하게 살아 있다고 확인된 노브들 ──────────────────
run D1_E1e5    DRIVE=spc N_GRAINS=524 CLOTH_E=1.0e5 NO_VIDEO=1
run D2_E1.6e6  DRIVE=spc N_GRAINS=524 CLOTH_E=1.6e6 NO_VIDEO=1
run D3_bend100 DRIVE=spc N_GRAINS=524 CLOTH_BEND=100  NO_VIDEO=1
run D4_bend1600 DRIVE=spc N_GRAINS=524 CLOTH_BEND=1600 NO_VIDEO=1

# ── E. 흔들기 조건 — 떨림이 가진 주파수에 붙어 있나 ──────────────────────
run E1_f0.5 DRIVE=spc N_GRAINS=524 FREQ_HZ=0.5 NO_VIDEO=1
run E2_f2.0 DRIVE=spc N_GRAINS=524 FREQ_HZ=2.0 NO_VIDEO=1
run E3_a20  DRIVE=spc N_GRAINS=524 AMP_MM=20   NO_VIDEO=1

# ── F. 낟알 크기 — 같은 2g 을 굵게/곱게 ──────────────────────────────────
run F1_R1.0_N221  DRIVE=spc N_GRAINS=221  GRAIN_RADIUS_MM=1.0 NO_VIDEO=1
run F2_R0.5_N1768 DRIVE=spc N_GRAINS=1768 GRAIN_RADIUS_MM=0.5 NO_VIDEO=1

# ── G. 접촉 배리어 — 낟알-천 접촉이 떨림원인가 ───────────────────────────
run G1_dhat5e-5 DRIVE=spc N_GRAINS=524 IPC_D_HAT=5e-5 NO_VIDEO=1
run G2_dhat5e-4 DRIVE=spc N_GRAINS=524 IPC_D_HAT=5e-4 NO_VIDEO=1

echo "[SHAKE] 완료  $(date +%H:%M:%S)"
