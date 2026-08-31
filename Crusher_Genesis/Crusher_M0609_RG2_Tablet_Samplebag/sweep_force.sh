#!/usr/bin/env bash
# 힘 상한을 **내려서** 균형점이 실제로 존재하는 구간을 찾는다 — 단일 레인.
#
# 어젯밤 스윕은 100 -> 800N 으로 올리기만 했다. 그런데 +-100N 은 Motor2 의 최대
# 스펙이지 운전 힘이 아니고, **2.5g 짜리 필름은 어떤 압착 두께에서도 100N 을
# 만들지 못한다.** 균형점이 없으니 벽은 항상 끝까지 닫히고 배리어가 갈린다
# (h1: armature=100 으로 관통 속도가 -3,237 -> -143mm/s 로 줄었지만 여전히 넘어감.
#  그 가속에 필요한 힘 79N 은 100N 상한 안쪽이라 이번엔 액추에이터가 범인이다).
#
# 참고로 성공하는 그리퍼는 40N 으로 봉투를 잡는다.
#
# 고정: armature=100(런타임 API, kv 안정 한계 40,125 로 kv=500 이 안전 영역),
#       플랜지 ON(실기 형상), 속도지령 8mm/s, control_dofs_position 힘제어.
# 판정: stalled=Y(하드스톱 전에 멈춤) + drop~0 이면 성립. 정지 위치가 곧 실효
#       압착 두께 측정값이 된다(§11-5).
set -u
export SWEEP_TIMEOUT=900        # 정상 런 205초. 갈리는 설정을 오래 붙들지 않는다.
source /dev/stdin <<< "$(sed -n '/^PY=/,/^}/p' sweep_clamp.sh)"
for F in 2 5 10 20 40; do
  run_one "i_F${F}" WALL_ARMATURE=100 WALL_FORCE_LIM_N=$F LEFTWALL_FLANGE_CONTACT=1 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
done
