#!/bin/bash
PY="C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe"
cd /c/Crusher_isaacsim/Crusher_Genesis/Powder_flip_test
# (a) 반력 계측 확인 — 낟알 ON/OFF
for w in 0 1; do
  TEST_MODE=crusher WITH_GRAINS=$w WITH_BAG=0 N_GRAINS=90 GRAIN_RADIUS_MM=1.5 GRAIN_RHO=2160 \
    $PY -u ipc_grain_coupler.py > "_ab_g$w.log" 2>&1
  echo "[A] 낟알=$w exit=$?"; grep -E "^\[crush\] k=(   0|400|524)" "_ab_g$w.log"
done
# (b) 봉투를 슬롯에 넣고 낟알 담은 뒤 압착
TEST_MODE=crusher WITH_GRAINS=1 WITH_BAG=1 N_GRAINS=90 GRAIN_RADIUS_MM=1.5 GRAIN_RHO=2160 \
  $PY -u ipc_grain_coupler.py > "_ab_bag.log" 2>&1
echo "[B] 봉투+낟알 exit=$?"
grep -E "^\[crush\]|^\[saved\]|World is not valid|Traceback|Error" "_ab_bag.log" | head -14
