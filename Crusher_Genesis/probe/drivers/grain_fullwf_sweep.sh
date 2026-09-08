#!/bin/bash
# grain_fullwf_sweep.sh — 낟알 병목 변인통제, **full_workflow 환경 기준** (2026-09-08)
#
# 왜 probe 가 아니라 full_workflow 인가(사용자 지시): probe(ipc_grain_coupler.py
# TEST_MODE=bag)에는 Genesis 강체가 아예 없어서, IPC 와 Genesis 두 솔버가 같이 도는
# 실제 조건을 대변하지 못한다. 앞서 probe 에서 돌린 마찰/eps_velocity 스윕이
# 그래서 판단 근거가 못 됐다.
#
# 실측 기준선(2026-09-08):
#   grainfull2  NEWTON_TOL 기본(0.05)  2055s(34분)  배위수 5.16  더미 28.7mm
#   grainfull3  NEWTON_TOL=1e-3        2951s(49분)  배위수 5.24  더미 12.3mm
#   -> newton_tol 은 낟알 구간만이 아니라 **전 공정**을 44% 느리게 한다.
#
# 영상은 끈다(NO_VIDEO=1) — 이 스윕이 묻는 건 수치이고, 렌더가 시간을 크게 먹는다.
# 육안 확인용 영상은 별도 런(grainfull4)에서 받는다.
#
# 순서는 **가치 순**이다. 도중에 끊겨도 앞쪽 결과만으로 판단이 서도록.
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/Crusher_M0609_RG2_Tablet_Samplebag || exit 1
OUT=_fullwf_sweep; mkdir -p $OUT

# 앞선 런(예약된 grainfull4)이 아직 돌고 있으면 기다린다 — GPU 를 같이 쓰면
# 둘 다 느려지고 메모리도 위험하다. 최대 90분까지만 기다린다.
for _i in $(seq 1 180); do
  ps -W 2>/dev/null | grep -q "crusher_genesis/python" || break
  sleep 30
done

run () {  # run <tag> <env...>
  local tag=$1; shift
  local t0=$SECONDS
  echo "[SWEEP] $tag 시작  $(date +%H:%M:%S)"
  env NO_VIDEO=1 RECOVER=1 CRUSH_SECONDS=15 RUN_TAG="$tag" "$@" \
    $PY -u full_workflow.py > "$OUT/$tag.log" 2>&1
  local rc=$? el=$((SECONDS-t0))
  local res=$(grep -E "^\[RESULT\] grain 담김" "$OUT/$tag.log" | head -1)
  local tim=$(grep -E "^\[timing\]" "$OUT/$tag.log" | head -1)
  local err=$(grep -E "Traceback|GenesisException|CUDA|out of memory" "$OUT/$tag.log" | head -1)
  echo "[SWEEP] $tag  exit=$rc  ${el}s  ${res:-(낟알결과없음)}  ${tim}  ${err:+ERR: $err}"
}

# 1) 장입량 확대 — N_GRAINS 기본값을 올릴지의 근거. 비용이 어떻게 스케일하나.
run N1000        GRAIN=1 N_GRAINS=1000 NEWTON_TOL=1e-3
# 2) 프로젝트 기준 장입량 2g (소금 rho=2160, 1mm 구 1768알)
run N1768rho2160 GRAIN=1 N_GRAINS=1768 GRAIN_RHO=2160 NEWTON_TOL=1e-3
# 3) newton_tol 의 이득이 어디서 오나 — 1e-2 로도 충분한가(그러면 훨씬 싸다)
run nt1e-2       GRAIN=1 N_GRAINS=500 NEWTON_TOL=1e-2
# 4,5) newton_tol 을 **기본값으로 박아도 되는가** — 낟알 없는 기존 공정에
#      영향이 없어야 한다. 압착 벽 반력/봉투 거동을 두 조로 비교한다.
run nograin_base GRAIN=0
run nograin_nt   GRAIN=0 NEWTON_TOL=1e-3
# 6) 압밀이 수렴했나 — 정착 시간을 3배로 주면 더미가 더 눌러앉나
run settle6s     GRAIN=1 N_GRAINS=500 NEWTON_TOL=1e-3 GRAIN_SECONDS=6
# 7) d_hat/R 비율 영향 — IPC_D_HAT 은 정제/봉투까지 바꾸므로 못 건드린다.
#    대신 낟알을 키워 비율만 0.2 -> 0.1 로 내린다(질량이 8배가 되는 건 교란 요인).
run R1.0         GRAIN=1 N_GRAINS=500 NEWTON_TOL=1e-3 GRAIN_RADIUS_MM=1.0

echo "[SWEEP] 완료  $(date +%H:%M:%S)"
