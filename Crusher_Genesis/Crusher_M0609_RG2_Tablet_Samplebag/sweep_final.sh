#!/usr/bin/env bash
# 남은 미결 하나를 **단일 레인**으로 돌린다 — 레인을 3~4개로 과다구독하면
# above(이송) 구간이 정체해 clamp 까지 못 간다(이번 밤의 STALL 은 전부 그것이다).
set -u
source /dev/stdin <<< "$(sed -n '/^PY=/,/^}/p' sweep_clamp.sh)"
run_one g3_kin115_flangeON WALL_KINEMATIC=1 CLAMP_TARGET_MM=-11.5 LEFTWALL_FLANGE_CONTACT=1 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
run_one g4_kin125         WALL_KINEMATIC=1 CLAMP_TARGET_MM=-12.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
run_one g5_kin135         WALL_KINEMATIC=1 CLAMP_TARGET_MM=-13.5 LEFTWALL_FLANGE_CONTACT=0 WALL_CLOSE_MMPS=8 HOLD_THROUGH_CLAMP=0
