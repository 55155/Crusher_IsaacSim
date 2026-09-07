#!/bin/bash
# 무게 스윕 — 낟알 개수만 바꾼다. 나머지(크기/밀도/배치부피/개구/봉투) 전부 고정.
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/Powder_flip_test
OUT=_mass_sweep; mkdir -p $OUT
run () {  # run <알수> <목표질량표기>
  local n=$1 tag=$2 t0=$SECONDS
  TEST_MODE=bag BAG_CONSTRAINT_MODE=spc GRAIN_PLACEMENT=poisson \
  N_GRAINS=$n GRAIN_RADIUS_MM=0.5 GRAIN_RHO=2160 \
  GRAIN_FILL_R_MM=10 GRAIN_FILL_H_MM=90 \
    $PY -u ipc_grain_coupler.py > "$OUT/m$tag.log" 2>&1
  local rc=$? el=$((SECONDS-t0))
  echo "[SWEEP] ${tag}g N=$n exit=$rc ${el}s  $(grep -E '^\[RESULT\] TEST_MODE' $OUT/m$tag.log | head -1)"
  grep -E '^\[mom \]' "$OUT/m$tag.log" | head -2
}
run 221  0.25
run 442  0.50
run 884  1.00
run 1768 2.00
echo "[SWEEP] 완료"
