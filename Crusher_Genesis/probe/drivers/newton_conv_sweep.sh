#!/bin/bash
# newton_conv_sweep.sh — **알별 접촉력이 몇 N 인지 말할 수 있게 만드는** 스윕
#                        (2026-09-23, 사용자 지시 "다 고쳐보자")
#
# 문제(§27-12): full_workflow 분쇄 런의 알별 접촉력이 참값의 중앙 22배, 2배 이내
# 적중률 8.6%, 방향 일치 cos 0.365 다. 원인은 Newton 이 velocity_tol 50 mm/s 에서
# 끊는데 낟알은 1.33 mm/s 로 움직여, 돌려받는 gradient 가 힘이 아니라 잔차라는 것.
#
# **합격 판정은 하나다.** 자유입자는 증분 포텐셜 최소점에서 f_contact = m(a-g) 가
# 항등식이므로 무리 합력 z 는 총 자중과 같아야 한다:
#       cf_close = sum f_z / 총자중  ->  1.0
# full_workflow 가 이제 이 값을 매 스텝 찍고 런 끝에 요약한다([RESULT] 수렴 검산).
# 실측 기준선: 기본 tol 에서 1,896 (즉 세 자릿수 틀림).
#
# 같이 거는 것 — 배리어(§27-9). 알의 97~99%가 2R+d_hat 에 붙어 서로 닿은 적이
# 없으면 수렴시켜도 재는 것이 배리어 반발력이다. R=1.0mm / 332알(2.086g)은
# 배리어가 풀리는 것이 확인된 조건이고 프로젝트 목표 장입량 2g 과도 맞는다.
#
# 영상은 끈다(NO_VIDEO=1). 묻는 것이 수치뿐이고 렌더가 시간을 크게 먹는다.
# CRUSH_SECONDS=15 는 타격 2회(t=3.75s, 11.25s)를 담는 최소 구간이다 —
# 1주기는 SPC 해제 과도라 어차피 못 쓰므로 2주기를 봐야 한다.
#
# 순서는 가치 순이다. 도중에 끊겨도 앞쪽만으로 판단이 서도록.
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/Crusher_M0609_RG2_Tablet_Samplebag || exit 1
OUT=_newton_conv; mkdir -p $OUT

BASE="STAGE=clamp:crush GRAIN=1 GRAIN_WHEN=start SKIP_TABLET=1 NO_VIDEO=1
      CRUSH_SECONDS=15 GRAIN_RADIUS_MM=1.0 N_GRAINS=332"

run () {  # run <tag> <env...>
  local tag=$1; shift
  local t0=$SECONDS
  echo "[SWEEP] $tag 시작  $(date +%H:%M:%S)"
  env $BASE RUN_TAG="$tag" "$@" $PY -u full_workflow.py > "$OUT/$tag.log" 2>&1
  local rc=$? el=$((SECONDS-t0))
  local cl=$(grep -E "^\[RESULT\] \*\*수렴 검산" "$OUT/$tag.log" | head -1)
  local pk=$(grep -E "^\[RESULT\] \*\*알별 최대 단일 접촉" "$OUT/$tag.log" | head -1)
  local nn=$(grep -E "^\[RESULT\] 최근접 낟알 간격" "$OUT/$tag.log" | head -1)
  local err=$(grep -E "Traceback|GenesisException|out of memory" "$OUT/$tag.log" | head -1)
  echo "[SWEEP] $tag  exit=$rc  ${el}s"
  echo "        ${cl:-(수렴검산 없음)}"
  echo "        ${pk:-(하중 없음)}"
  echo "        ${nn}"
  [ -n "$err" ] && echo "        ERR: $err"
}

# 0) 기준선 — 배선 수정 + R=1.0 만 반영, 솔버는 기본값. 종전 1,896 과 비교할 축.
run conv_base
# 0b) **R 대조군.** 스모크(2s 무부하, R=1.0)에서 검산이 이미 중앙 1.0 이 나왔는데,
#     그건 R 과 구간이 같이 바뀐 값이라 귀속이 안 된다. 같은 15s/타격 2회 조건에서
#     R 만 종전값(0.5mm/500알)으로 되돌려, 수렴이 R 덕인지 아닌지부터 가른다.
run conv_R0.5     GRAIN_RADIUS_MM=0.5 N_GRAINS=500
# 1~3) 허용오차를 낟알 속력 아래로 단계적으로 내린다. 1e-3 은 §25-4 에서 이미
#      돌려본 값(전 공정 +44%)이고, 1e-4/1e-5 는 미검증이다.
run conv_nt1e-3   NEWTON_TOL=1e-3
run conv_nt1e-4   NEWTON_TOL=1e-4
run conv_nt1e-5   NEWTON_TOL=1e-5
# 4) 조여도 안 풀리면 **반복 상한**에 걸린 것이다 — tol 을 더 내려도 안 바뀐다.
#    상한을 올려 그 갈래를 가른다.
run conv_nt1e-5_it NEWTON_TOL=1e-5 NEWTON_MAX_IT=200
# 5) 마찰은 별개 축이다. contact_eps_velocity 기본 10 mm/s 가 낟알 속력보다 커서
#    전 알이 평활화 구간에 있다 — Coulomb 이 아니라 점성 근사로 풀린다.
run conv_nt1e-5_ev NEWTON_TOL=1e-5 EPS_VEL=1e-4

echo "[SWEEP] 완료  $(date +%H:%M:%S)"
