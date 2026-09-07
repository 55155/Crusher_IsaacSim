#!/bin/bash
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/Powder_flip_test
OUT=_ecc; mkdir -p $OUT
run () {  # run <cy_mm> <tag>
  local cy=$1 tag=$2 t0=$SECONDS
  TEST_MODE=bag BAG_CONSTRAINT_MODE=spc GRAIN_PLACEMENT=poisson \
  N_GRAINS=1768 GRAIN_RADIUS_MM=0.5 GRAIN_RHO=2160 \
  GRAIN_FILL_R_MM=10 GRAIN_FILL_H_MM=90 GRAIN_FILL_CY_MM=$cy \
    $PY -u ipc_grain_coupler.py > "$OUT/$tag.log" 2>&1
  echo "[ECC] $tag (cy=${cy}mm) exit=$? $((SECONDS-t0))s  $(grep -E '^\[RESULT\] TEST_MODE' $OUT/$tag.log|head -1)"
  grep -E '^\[bag\] 배치|^\[saved\]|^\[mom \] 시계열' "$OUT/$tag.log"
}
run 0  center
run 20 ecc20
echo "[ECC] 완료"
