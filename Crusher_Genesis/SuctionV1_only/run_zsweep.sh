#!/bin/bash
# 흡착 위치(입구 테두리 아래 z)를 5단계로 훑으며 봉투 형상을 본다.
# 파일명에 조건을 넣는다: zsweep_rim<mm>_K<strength>.mp4
PY=C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe
cd "$(dirname "$0")"
for RIM in 5 15 25 40 60; do
  TAG=$(printf "rim%02dmm_K1e4" "$RIM")
  echo "[run ] $TAG  (입구 테두리 아래 ${RIM}mm 흡착)"
  ATTACH=1 ATTACH_K=1e4 CUP_BELOW_RIM_MM=$RIM TRACE=0 \
    OUT_SUBDIR=zsweep RUN_TAG=$TAG \
    $PY -u suction_bagopen_20260901.py > "RESULT/zsweep/$TAG.log" 2>&1
  echo "[done] $TAG  $(grep -E '^\[result\] mouth' "RESULT/zsweep/$TAG.log")"
  echo "       $(grep -E '^\[bbox\]' "RESULT/zsweep/$TAG.log" | sed 's/\[bbox\] 봉투 //')"
done
echo "=== zsweep 완료 ==="
ls -la RESULT/zsweep/*.mp4 2>/dev/null
