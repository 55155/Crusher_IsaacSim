#!/bin/bash
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/Powder_flip_test
OUT=_grain_budget; mkdir -p $OUT
run () {  # run <N> <radius_mm>
  local n=$1 r=$2 tag="N${1}_R${2}" t0=$SECONDS
  TEST_MODE=bag BAG_CONSTRAINT_MODE=spc N_GRAINS=$n GRAIN_RADIUS_MM=$r \
    $PY -u ipc_grain_coupler.py > "$OUT/$tag.log" 2>&1
  local rc=$? el=$((SECONDS-t0))
  local res=$(grep -E "^\[RESULT\] TEST_MODE" "$OUT/$tag.log" | head -1)
  local err=$(grep -E "Traceback|GenesisException|CUDA|out of memory" "$OUT/$tag.log" | head -1)
  echo "[BUDGET] $tag  exit=$rc  ${el}s  ${res:-(결과없음)} ${err:+ERR: $err}"
}
run 60   1.5
run 200  1.5
run 500  1.5
run 1000 1.5
run 500  0.8
echo "[BUDGET] 완료"
