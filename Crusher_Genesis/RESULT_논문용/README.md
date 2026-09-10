# RESULT_논문용 — 논문 그림용 산출물

여기 있는 것은 **논문 그림 후보로 골라 둔 것**이다. 재생성하려면 각 항목의
"만든 법"을 그대로 다시 돌리면 된다. 실험 기록 자체는 `probe/README.md` 와
`docs/DigitalTwin.md` 를 본다.

---

## 1. 파우더가 봉투 거동을 지배한다 (2026-09-10)

봉투를 좌우로 흔들었을 때, **파우더 2g 유무만** 바꾼 대조다. 나머지(천 재질,
구동 방식, dt, 흔들기 조건)는 전부 동일하다.

| 파일 | 내용 |
|---|---|
| `compare_t*.png` | 좌 = 빈 봉투, 우 = 파우더 2g. 같은 시각 나란히 |
| `shake_empty_t*.png` | 빈 봉투 단독 |
| `shake_powder2g_t*.png` | 파우더 2g 단독 |

시각은 흔들기 사인파(40mm, 1Hz) 기준 `t=0.25s`(+극점), `0.75s`(−극점),
`0.50s`(중앙 통과), `5.25s`/`9.75s`(정상 상태)다.

### 실측

|  | 빈 봉투 | 파우더 2g |
|---|---|---|
| 봉투 폭 | 6.2 ~ 13.8mm (평균 **8.0**) | 6.4 ~ 51.6mm (평균 **31.5**) |
| 봉투 높이 | 89.8 ~ 90.1mm (거의 불변) | 76.8 ~ 90.1mm (**13mm 수축**) |
| COM 가속도 잔차 중앙값 | **1.67** m/s² | **4.79** m/s² |
| jerk_ratio | **1.1** (지령대로만 움직임) | **3.0** |

빈 봉투는 사실상 지령한 궤적만 따라가고 형상이 거의 안 변한다. 파우더가 들어가면
폭이 4배로 부풀고, 높이가 13mm 줄고, 가속도 잔차가 3배가 된다.

### 만든 법

    cd Crusher_Genesis/probe
    PY=C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe

    DRIVE=spc N_GRAINS=524 SECONDS=10 TAG=A1_spc            $PY probe_bag_shake.py
    DRIVE=spc N_GRAINS=0   SECONDS=10 TAG=B1_spc_dry_vid    $PY probe_bag_shake.py

프레임은 `RESULT_shake/shake_<TAG>_<ts>.mp4` 에서 `ffmpeg -ss <t> -frames:v 1` 로 뽑았다
(영상은 실시간 재생, 28.57fps). ffmpeg 은 `imageio_ffmpeg` 번들 것을 쓴다 —
PATH 에 ffmpeg 이 없다.

### 주의 — 그림 설명에 쓸 때

- 구동 방식(`set_dofs_position` 순간이동 / 속도 포함 / SPC 직접)은 **결과에 영향이
  없었다**(파우더 없는 조에서 1.63/1.64/1.67 m/s² 로 동일). 처음엔 순간이동을
  원인으로 의심했는데 대조군이 그 가설을 기각했다.
- 흔들림 크기를 실기와 대조한 적은 **아직 없다.** "파우더가 봉투 거동을 지배한다"
  까지가 이 그림이 말할 수 있는 것이고, 그 크기가 정량적으로 맞는지는 별개다.
- 전체 20조 스윕은 `probe/drivers/bag_shake_sweep.sh`, 결과는
  `probe/_shake_sweep/*.log` 와 `probe/RESULT_shake/*.npz`.
