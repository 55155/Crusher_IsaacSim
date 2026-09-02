#!/bin/bash
# 2026-09-02 야간 런 — SPC 정점 부착(패치 2)을 full_workflow 에 적용한 검증.
# 파일명에 조건을 넣는다: <케이스>_<설명>.
PY=C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe
cd "$(dirname "$0")"
OUT=RESULT/attach_20260902
mkdir -p "$OUT"

run () {  # run <tag> <설명> <env...>
  local tag=$1; shift; local desc=$1; shift
  echo "===================================================================="
  echo "[run ] $tag — $desc"
  env "$@" RUN_TAG="$tag" $PY -u full_workflow.py > "$OUT/$tag.log" 2>&1
  echo "[exit] $? "
  grep -E "shape 고정|bag dbg|투입 완료|해제 후 낙하|RPM|\[saved\]" "$OUT/$tag.log" | head -12
}

# A. 기준선 — 패치 없이(옛 하드 핀). 카메라 추적 비교용.
run "A_baseline_hardpin"  "하드 핀(기존)"        BAG_ATTACH=0 CRUSH_SECONDS=15 RECOVER=1
# B. 패치 적용 — 같은 조건. A 와 나란히 비교.
run "B_attach_k1e4"       "SPC k=1e4"           BAG_ATTACH=1 BAG_ATTACH_K=1e4 CRUSH_SECONDS=15 RECOVER=1
# C. 구속 세기 상향
run "C_attach_k1e6"       "SPC k=1e6"           BAG_ATTACH=1 BAG_ATTACH_K=1e6 CRUSH_SECONDS=15 RECOVER=1
# D. 구속 세기 하향
run "D_attach_k1e2"       "SPC k=1e2"           BAG_ATTACH=1 BAG_ATTACH_K=1e2 CRUSH_SECONDS=15 RECOVER=1
# E. 장시간 압착 — 패치 적용 상태에서 60초 운전
run "E_attach_crush60"    "SPC k=1e4, 압착 60초" BAG_ATTACH=1 BAG_ATTACH_K=1e4 CRUSH_SECONDS=60 RECOVER=1

echo "=== 야간 런 완료 ==="
ls -la "$OUT"/*.mp4 2>/dev/null | tail -30
