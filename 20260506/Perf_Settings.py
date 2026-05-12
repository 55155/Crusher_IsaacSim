"""
Perf_Settings.py  -  Isaac Sim Script Editor
시뮬레이션 성능 조정 (프레임 드랍 완화)

Play 상태에서 실행 가능. 즉시 적용됨.
"""

import carb

s = carb.settings.get_settings()

# ── 1. Physics 스텝 수 ─────────────────────────────────────────────
# 기본값: 60 Hz. 낮출수록 CPU 부담 감소, 단 물리 정밀도도 낮아짐
PHYSICS_HZ = 30   # ← 여기 조정 (60 / 30 / 20)
s.set("/physics/timeStepsPerSecond", PHYSICS_HZ)

# ── 2. USD 동기화 끄기 ────────────────────────────────────────────
# 매 스텝마다 결과를 USD로 올리는 작업 → 렌더링과 무관하게 비용 큼
# False로 하면 viewport 업데이트가 줄어들지만 렌더 속도 크게 향상
s.set("/physics/updateToUsd", True)           # False 시 시각 피드백 없어짐, 일단 True 유지
s.set("/physics/updateVelocitiesToUsd", False) # 속도 벡터는 굳이 USD에 안 써도 됨

# ── 3. 비동기 시뮬레이션 ──────────────────────────────────────────
# True: physics와 rendering을 분리해서 병렬 처리
s.set("/physics/asyncSimRender", True)

# ── 현재 설정값 출력 ──────────────────────────────────────────────
hz   = s.get("/physics/timeStepsPerSecond")
usd  = s.get("/physics/updateToUsd")
vel  = s.get("/physics/updateVelocitiesToUsd")
asyn = s.get("/physics/asyncSimRender")
print(f"timeStepsPerSecond  : {hz}")
print(f"updateToUsd         : {usd}")
print(f"updateVelocitiesToUsd: {vel}")
print(f"asyncSimRender      : {asyn}")
print("완료. 효과 없으면 PHYSICS_HZ를 더 낮춰보세요.")
