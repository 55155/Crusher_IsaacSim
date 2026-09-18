#!/bin/bash
# run_repeat5.sh — **같은 설정 5회 반복**으로 비결정성 분포를 잰다 (2026-09-14).
#
# 사용자 요청 "같은 설정으로 5번 돌려서 분포 내줘". 변인은 하나도 안 바꾼다 —
# 바뀌는 것은 RUN_TAG(출력 디렉터리)뿐이다. env 는 RESULT_crush_grain2/3/4 를
# 만든 런과 동일하게 맞췄다(그 런들의 로그 헤더에서 역산):
#     STAGE=clamp:crush      -> CLAMP_ONLY=1, 팔 시퀀스 생략, 봉투를 슬롯에 직접 스폰
#     GRAIN=1 GRAIN_WHEN=start  -> 낟알 500알 R=0.5mm 를 봉투 안에 미리 채운 채 시작
#     SKIP_TABLET=1          -> 정제 제외(봉투 FEM + IPC + 낟알만)
#     CRUSH_SECONDS=60       -> 8 RPM 으로 60초 = 12,000 스텝, 타격 8회
# 나머지(BAG_ATTACH=1, BAG_ATTACH_K=1e4, GS_SEED=0, IPC_D_HAT=1e-4 ...)는 기본값이고
# 기본값이 곧 그 런들의 값이다. 시드도 같다 — **시드가 같아도 갈린다는 게 요점**이다.
#
# 런당 약 28분, 총 2시간 20분 전후. 출력 런당 약 108MB.
PY=C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe
cd "$(dirname "$0")"
OUT=RESULT/repeat5_20260914
mkdir -p "$OUT"

for i in 1 2 3 4 5; do
  tag="rep$i"
  echo "===================================================================="
  echo "[run ] $tag  ($(date '+%H:%M:%S'))"
  env STAGE=clamp:crush GRAIN=1 GRAIN_WHEN=start SKIP_TABLET=1 \
      CRUSH_SECONDS=60 RUN_TAG="repeat5_$tag" \
      $PY -u full_workflow.py > "$OUT/$tag.log" 2>&1
  echo "[exit] $?  ($(date '+%H:%M:%S'))"
  grep -E "^\[saved\]|^\[RESULT\]|알당 반력|낟알 무리 합력" "$OUT/$tag.log" | head -8
done

echo "=== 5회 반복 완료 ==="
ls -la RESULT_repeat5_rep*/grip-28mm/crush_*.npz 2>/dev/null
