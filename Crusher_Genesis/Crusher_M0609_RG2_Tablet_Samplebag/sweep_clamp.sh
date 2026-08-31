#!/usr/bin/env bash
# 압착 파라미터 야간 스윕 (2026-08-31)
#
# 이 기계는 병렬 폭이 좁다 — RAM 16.8GB(런당 약 3.6GB), VRAM 12.2GB(런당 약 4GB).
# 그래서 **레인 2개**가 상한이다. 레인마다 설정 목록을 순차로 돌린다.
#
# 각 런은 타임아웃을 건다. clamp 에서 IPC 배리어가 갈리면 20분이 넘도록 진행이
# 없는 사례가 여러 번 있었고(§19-4, 이번 -15.5mm 위치제어 런), 그런 설정 하나가
# 밤을 통째로 먹으면 안 된다. 타임아웃은 STALL 로 기록하고 다음으로 넘어간다.
#
# 판정의 핵심 지표는 로그의 [SUMMARY] 줄, 그중에서도 **drop** 이다 — 그리퍼를
# 놓은 뒤 봉투가 흘러내린 양(§18-4 실패 실측 -34.9mm). hold=0 인 런만 유효하다.
set -u

PY="C:/Users/user/AppData/Local/Programs/Python/Python311/python.exe"
DIR="C:/Users/user/Desktop/Crusher_IsaacSim/Crusher_Genesis/Crusher_M0609_RG2_Tablet_Samplebag"
OUT="$DIR/SWEEP"
TIMEOUT="${SWEEP_TIMEOUT:-1500}"        # 런당 25분
mkdir -p "$OUT"
CSV="$OUT/summary.tsv"
[ -f "$CSV" ] || printf 'run\tstatus\tsecs\tsummary\n' > "$CSV"

run_one() {
  local name="$1"; shift
  local log="$OUT/$name.log"
  # 이미 성공한 런은 건너뛴다 — 스크립트를 다시 돌려도 이어서 진행된다.
  if grep -q '^\[SUMMARY\]' "$log" 2>/dev/null; then
    echo "[skip] $name (이미 완료)"; return
  fi
  echo "[run ] $name : $*"
  printf '[cfg] %s\n' "$*" > "$log"
  local t0 t1 rc
  t0=$(date +%s)
  # RUN_TAG 로 런마다 RESULT 디렉터리를 분리한다 — 중간 산출물(_bag_seal_uv.obj 등)
  # 경로가 겹치면 병렬 런끼리 서로의 파일을 덮어써서 전부 죽는다.
  timeout "$TIMEOUT" env "$@" RUN_TAG="$name" LAYOUT_FROM_STEP=1 NO_VIDEO=1 "$PY" -u "$DIR/full_workflow.py" >> "$log" 2>&1
  rc=$?
  t1=$(date +%s)
  local sm status
  sm=$(grep -m1 '^\[SUMMARY\]' "$log" | sed 's/^\[SUMMARY\] //')
  if [ "$rc" = "124" ]; then
    status=STALL
    sm="clamp 마지막 관측: $(grep -E '^\s+\[clamp t=' "$log" | tail -1 | sed 's/^ *//')"
  elif [ -n "$sm" ]; then
    status=OK
  else
    status="ERR($rc)"
    sm=$(grep -m1 -E 'RuntimeError|Error:|Exception' "$log" | cut -c1-160)
  fi
  printf '%s\t%s\t%s\t%s\n' "$name" "$status" "$((t1-t0))" "$sm" >> "$CSV"
  echo "[done] $name -> $status ($((t1-t0))s)"
}

lane_a() {
  # 레인 A — 본 가설: 플랜지를 뺐으니 압착이 성립하는가. 힘을 올려가며 본다.
  run_one a1_base   LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one a2_F200   LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=200 HOLD_THROUGH_CLAMP=0
  run_one a3_F400   LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=400 HOLD_THROUGH_CLAMP=0
  run_one a4_base_r2 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one a5_F200_r2 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=200 HOLD_THROUGH_CLAMP=0
  run_one a6_F800   LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=800 HOLD_THROUGH_CLAMP=0
}

lane_b() {
  # 레인 B — 대조군(플랜지 ON)과 닫힘 속도. 속도를 늦추면 배리어 충격이 준다.
  run_one b1_flangeON LEFTWALL_FLANGE_CONTACT=1 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one b2_v2       LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=2 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one b3_v2_F200  LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=2 WALL_FORCE_LIM_N=200 HOLD_THROUGH_CLAMP=0
  run_one b4_v4_F200  LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=4 WALL_FORCE_LIM_N=200 HOLD_THROUGH_CLAMP=0
  run_one b5_v2_r2    LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=2 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one b6_v2_F400  LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=2 WALL_FORCE_LIM_N=400 HOLD_THROUGH_CLAMP=0
}


lane_c() {
  # 레인 C — 랙-피니언 반사 관성(armature). 레인 A/B 가 힘·속도와 무관하게
  # 전부 관통으로 끝났고, 원인이 축이 너무 가벼운 것(0.312kg)으로 좁혀졌다.
  run_one c1_arm10  WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one c2_arm50  WALL_ARMATURE=50 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one c3_arm10_F200 WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=200 HOLD_THROUGH_CLAMP=0
  run_one c4_arm10_r2 WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one c5_arm200 WALL_ARMATURE=200 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
}

lane_d() {
  # 레인 D — 관성 + 낮은 kv(안정 한계 아래), 그리고 관성만으로 되는지의 대조.
  run_one d1_arm2    WALL_ARMATURE=2  LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one d2_kv100   WALL_ARMATURE=0  WALL_KV=100 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one d3_arm10_v2 WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=2 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one d4_arm50_r2 WALL_ARMATURE=50 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one d5_arm50_F200 WALL_ARMATURE=50 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=200 HOLD_THROUGH_CLAMP=0
}

lane_e() {
  # 레인 E — 강체<->IPC 프록시 소프트 구속 강성. 힘/속도/관성을 다 바꿔도 첫 접촉
  # 에서 같은 모양으로 터졌으므로, 남은 후보인 양방향 커플링 루프를 흔들어 본다.
  run_one e1_cs10    IPC_CONSTRAINT_STRENGTH=10   WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one e2_cs1000  IPC_CONSTRAINT_STRENGTH=1000 WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one e3_cs10000 IPC_CONSTRAINT_STRENGTH=10000 WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one e4_cs1000_v2 IPC_CONSTRAINT_STRENGTH=1000 WALL_ARMATURE=10 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=2 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
  run_one e5_cs1000_arm50 IPC_CONSTRAINT_STRENGTH=1000 WALL_ARMATURE=50 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 WALL_FORCE_LIM_N=100 HOLD_THROUGH_CLAMP=0
}

lane_f() {
  # 레인 F — 벽을 기구학적으로 구동(set_dofs_position). 힘으로 미는 한 접촉
  # 임펄스가 벽을 튕겨낼 수 있다는 것이 A~E 로 확인됐으므로, 아예 되밀 수 없게
  # 만든다. 실기 랙-피니언 대감속기도 역구동이 안 되므로 이쪽에 가깝다.
  # 정지 기준이 힘이 아니라 위치이므로 CLAMP_TARGET 을 깊이별로 훑는다.
  #   -11.5mm = 본체면 잔여 6.0mm (6mm 봉투에 막 닿음)
  #   -13.5mm = 잔여 4.0mm (2mm 압착)
  #   -15.5mm = 잔여 2.0mm (필름 두 겹 이론 하한)
  run_one f1_kin_115 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-11.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one f2_kin_135 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-13.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one f3_kin_155 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-15.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one f4_kin_135_r2 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-13.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
  run_one f5_kin_135_v2 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-13.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=2 HOLD_THROUGH_CLAMP=0
  run_one f6_kin_145 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-14.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
}

case "${1:-both}" in
  a) lane_a ;;
  b) lane_b ;;
  c) lane_c ;;
  d) lane_d ;;
  cd) lane_c & lane_d & wait ;;
  e) lane_e ;;
  f) lane_f ;;
  ef) lane_e & lane_f & wait ;;
  ce) lane_c & lane_e & wait ;;
  *) lane_a & lane_b & wait ;;
esac
echo "[sweep] 완료 — $CSV"
