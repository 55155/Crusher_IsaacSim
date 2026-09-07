#!/bin/bash
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/Powder_flip_test
for w in 0 1; do
  t0=$SECONDS
  TEST_MODE=crusher WITH_GRAINS=$w N_GRAINS=90 GRAIN_RADIUS_MM=1.5 GRAIN_RHO=2160 \
    $PY -u ipc_grain_coupler.py > "_crusher_g$w.log" 2>&1
  echo "[CRUSH] 낟알=$w exit=$? $((SECONDS-t0))s"
  grep -E "^\[crush\]|^\[saved\]|World is not valid|Traceback" "_crusher_g$w.log" | head -12
done
