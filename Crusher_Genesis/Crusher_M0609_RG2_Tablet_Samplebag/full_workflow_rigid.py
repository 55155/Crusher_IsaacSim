"""full_workflow_rigid.py — full_workflow.py 와 같은 공정을, **모든 바디를
rigid 로** 푸는 버전. 커플러(IPC) 없음, FEM 없음.

목적은 재료 충실도가 아니라 **공정 검증**이다 (docs/DigitalTwin.md §12):
기구 배치 · IK 도달성 · 매니퓰레이터 조작정책 · 간섭 — 전부 재료와 무관한
항목인데, IPC+FEM 조합은 1회 실행에 16분+ 라 배치를 1mm 옮길 때마다 그 비용을
내고 있었다. 이 파일은 같은 시퀀스를 **약 80초**에 돌린다(약 12배).

`DT` 와 페이즈 스텝수(`N_*`)를 `full_workflow.py` 와 **동일한 상수로 공유**하므로,
여기서 확정한 궤적·자세·타이밍이 FEM 버전으로 1:1 로 넘어간다. 배치·자세
상수도 전부 `full_workflow` 에서 import 한다 — 이 파일에는 "rigid 로 바꾸느라
달라진 것"만 있다.

**FEM 버전과 다른 점 (전부 §12 에 근거 기록):**

1. **봉투** — `Samplebag_seal_pouch3.stl` 은 두께 0의 open shell(watertight=False,
   hull 을 뜨면 입구가 막힌 슬래브)이라 rigid 로 그대로 못 쓴다. 같은 치수·같은
   위상을 **box primitive 5장**으로 재생성한 프록시를 쓴다(§12-3).
   벽 두께를 바꿔도 density 를 역산해 FEM 봉투와 질량이 정확히 일치한다(2.597g).
2. **정제** — FEM 캡슐 대신 MJCF `type="capsule"`(= SDF primitive, §조합6).
   settle 이후에는 봉투에 종속된 화물로 다룬다(`RB_TABLET_CARGO`, §12-6).
3. **구동** — `set_dofs_position` 텔레포트 금지, `control_dofs_position`(PD) +
   `gravity_compensation=1.0`. 위치 강제는 접촉 임펄스를 깨뜨린다(§7-7, §12-4).
4. **커플러** — IPC 대신 LegacyCoupler(재질간 플래그 전부 False). 리지드-리지드
   접촉은 리지드 솔버가 자체 처리하므로 `needs_coup=False` 로 커플러용 SDF
   생성도 건너뛴다.
5. **봉투 자립** — FEM 의 정점 제약 대신 prep~close 구간 pose 하드 홀드.

**Left_Wall 충돌은 기본 OFF** (`RB_LEFTWALL_COLLISION=1` 로 켤 수 있음).
켜면 비볼록 hull 이 다른 크러셔 부품과 간섭해 슬라이드 조인트를 -10mm 에
잼시켜 **슬롯이 아예 안 열린다**(§12-7). 켜려면 볼록분해가 선행돼야 한다.

env:
  Y_OFFSET_MM           슬롯 IK 타깃 Y 스윕 (기본 0)
  WALL_OPEN_MM          슬롯 개방량 mm (기본 6.0). +7.1mm 가 기구 하드 스톱이라
                        그 이상 지령해도 통로는 약 19.1mm 가 상한(§12-7).
  RB_TABLET_CARGO       정제를 봉투 종속 화물로 (기본 1)
  RB_LEFTWALL_COLLISION Left_Wall 충돌 (기본 0 — 켜면 잼)
  VIEWER                1 이면 뷰어

출력: Result_NoCoupling_OnlyRigidbody/rigid_workflow_<...>_{overview,bagcam,sideview}.mp4
"""
import os, sys, tempfile, time
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# 배치·자세·페이즈 상수는 전부 FEM 버전과 공유한다(궤적 1:1 대응의 전제).
import full_workflow as fw

# ── 출력: FEM 결과(RESULT/)와 섞이지 않도록 전용 디렉터리 ────────────────────
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "Result_NoCoupling_OnlyRigidbody")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")

Y_OFFSET_MM = float(os.environ.get("Y_OFFSET_MM", "0"))
Y_OFFSET = Y_OFFSET_MM * 1e-3
# 슬롯 개방량(+가 개방). 정지 형상 gap(12mm) 에 더해져 실제 통로가 된다.
WALL_OFFSET = float(os.environ.get("WALL_OPEN_MM", "6.0")) * 1e-3
RB_TABLET_CARGO = os.environ.get("RB_TABLET_CARGO", "1") == "1"
RB_LEFTWALL_COLLISION = os.environ.get("RB_LEFTWALL_COLLISION", "0") == "1"

_STEM = f"rigid_workflow_wall{WALL_OFFSET*1e3:+.1f}mm_yoff{Y_OFFSET_MM:+.1f}mm_{_TS}"
MP4_OVERVIEW = os.path.join(OUT_DIR, f"{_STEM}_overview.mp4")
MP4_BAGCAM = os.path.join(OUT_DIR, f"{_STEM}_bagcam.mp4")
MP4_SIDE = os.path.join(OUT_DIR, f"{_STEM}_sideview.mp4")

# ── rigid 프록시 파라미터 (docs/DigitalTwin.md §12-3) ────────────────────────
# 패널별 (두께, 성장방향). 성장방향 +1=바깥 / -1=안쪽:
#  - front/back: 바깥. 안쪽으로 키우면 공동이 6->4mm 인데 정제 캡슐 지름이 정확히
#    4mm 라 여유 0. 바깥이면 외형 8mm 이나 통로(17~19mm) 대비 충분하고, FEM 봉투도
#    cloth thickness 1mm 가 접촉 오프셋으로 붙어 실효 8mm 라 동등하다.
#  - 좌우: 안쪽. 폭 64mm 가 슬롯 Y 여유 65mm 대비 0.5mm/쪽뿐이라 바깥으로 키우면
#    삽입 자체가 불가능해진다.
#  - 바닥: 안쪽 + 두껍게. 바깥으로 키우면 외형 바닥이 SHELF_TOP 아래로 내려가
#    스폰 시 선반과 겹친다.
RB_PANELS = {
    "front":  (fw.CLOTH_THICK, +1),
    "back":   (fw.CLOTH_THICK, +1),
    "left":   (fw.CLOTH_THICK, -1),
    "right":  (fw.CLOTH_THICK, -1),
    "bottom": (0.004, -1),
}
BAG_HALF_T = 0.003  # 봉투 표면 메시의 두께 반값(로컬 Z, 실측 ±3mm)
# 질량 보존: 패널 두께를 바꿔도 density 를 역산해 FEM 봉투와 정확히 맞춘다.
RB_CLOTH_AREA = (2 * (2 * fw.BAG_PANEL_HALF_W) * (2 * fw.BAG_PANEL_HALF_H)
                 + 2 * (2 * BAG_HALF_T) * (2 * fw.BAG_PANEL_HALF_H)
                 + (2 * fw.BAG_PANEL_HALF_W) * (2 * BAG_HALF_T))
RB_TARGET_MASS = fw.CLOTH_RHO * fw.CLOTH_THICK * RB_CLOTH_AREA

# 리지드 솔버가 쓰는 마찰(IPC 의 coup_friction 은 커플러 전용이라 여기선 무효).
RB_FRICTION = 1.0
# PD 게인. kp=4500 에서는 above/lift 같은 빠른 구간의 추종 지연이 74~82mrad(4~4.7도)
# 까지 벌어져 슬롯 정렬에 그대로 실렸다 — 지연은 속도/kp 에 비례하므로 올려서 누른다.
RB_ARM_KP, RB_ARM_KV = 20000.0, 1200.0
RB_FING_KP, RB_FING_KV = 60.0, 3.0

# Left_Wall 은 원본 MJCF 에서 contype=0/conaffinity=0 이라 리지드 솔버가 충돌을
# 걸러낸다. 되살리면 클램프 접촉을 볼 수 있지만 §12-7 의 잼이 생긴다.
WALL_GEOM_LEFTWALL = "L2_Left_Wall1_1"

# full_workflow.py 는 이 값을 main() 안에서 정의해 import 로 못 가져온다. 값과
# 근거는 그쪽과 동일: rigid-only 정밀 스윕으로 찾은 핑거-Crusher 충돌 경계로,
# 52mm(wall_top+16mm)는 접촉 0건, 50mm 부터 접촉 7건 — 안전한 최소 여유.
INSERT_MARGIN_ABOVE_CENTER = 0.052


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _prepare_crusher_mjcf():
    """fw._prepare_crusher_mjcf 와 같되, 옵션으로 Left_Wall 충돌을 되살린다."""
    dst = fw._prepare_crusher_mjcf()
    if not RB_LEFTWALL_COLLISION:
        return dst
    tree = ET.parse(dst); root = tree.getroot()
    wb = root.find("worldbody")
    for g in wb.iter("geom"):
        if g.get("mesh") == WALL_GEOM_LEFTWALL:
            g.attrib.pop("contype", None)
            g.attrib.pop("conaffinity", None)
    tree.write(dst)
    print(f"[crusher] {WALL_GEOM_LEFTWALL} 충돌 ON — §12-7 잼 주의")
    return dst


def _panel_t(name):
    return RB_PANELS[name][0]


def _panel_grow_out(name):
    """바깥으로 키운 두께 — 외형 AABB 를 그만큼 넓힌다."""
    t, grow = RB_PANELS[name]
    return max(0, grow) * t


def _panel_grow_in(name):
    """안쪽으로 키운 두께 — 공동을 그만큼 깎는다."""
    t, grow = RB_PANELS[name]
    return max(0, -grow) * t


def prepare_rigid_bag_mjcf():
    """5-panel 봉투 프록시 MJCF 생성 (docs/DigitalTwin.md §12-3).

    STL 실측과 정확히 일치: 로컬 원점 중심, X ±32mm(폭) / Y ±45mm(높이, +45 가
    입구) / Z ±3mm(두께). 각 패널의 **안쪽 면**이 원래 cloth 표면 위치에 놓이도록
    지정 방향으로만 두께를 키운다.
    """
    hw, hh, ht = fw.BAG_PANEL_HALF_W, fw.BAG_PANEL_HALF_H, BAG_HALF_T

    def d(name):  # 패널 중심이 cloth 표면에서 밀려나는 양(부호 포함)
        t, grow = RB_PANELS[name]
        return grow * t / 2

    panels = [
        ("front",  (0.0, 0.0, ht + d("front")),   (hw, hh, _panel_t("front") / 2)),
        ("back",   (0.0, 0.0, -ht - d("back")),   (hw, hh, _panel_t("back") / 2)),
        ("left",   (-hw - d("left"), 0.0, 0.0),   (_panel_t("left") / 2, hh, ht)),
        ("right",  (hw + d("right"), 0.0, 0.0),   (_panel_t("right") / 2, hh, ht)),
        ("bottom", (0.0, -hh - d("bottom"), 0.0), (hw, _panel_t("bottom") / 2, ht)),
    ]
    vol = sum(8 * s[0] * s[1] * s[2] for _, _, s in panels)
    density = RB_TARGET_MASS / vol

    def rgba(name):
        c = fw.SEAL_COLOR if name in ("left", "right") else fw.BAG_BODY_COLOR
        return " ".join(f"{v / 255:.3f}" for v in c) + " 0.85"

    geoms = "\n".join(
        f'      <geom name="bag_{n}" type="box" pos="{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}" '
        f'size="{s[0]:.6f} {s[1]:.6f} {s[2]:.6f}" rgba="{rgba(n)}"/>'
        for n, p, s in panels
    )
    xml = f"""<mujoco model="rigid_bag_proxy">
  <compiler angle="radian"/>
  <default>
    <geom density="{density:.4f}" friction="{RB_FRICTION} 0.02 0.001" contype="1" conaffinity="1"/>
  </default>
  <worldbody>
    <body name="bag_proxy" pos="0 0 0">
      <freejoint name="bag_free"/>
{geoms}
    </body>
  </worldbody>
</mujoco>
"""
    dst = os.path.join(tempfile.mkdtemp(prefix="rigid_bag_"), "rigid_bag.xml")
    with open(dst, "w", encoding="utf-8") as f:
        f.write(xml)

    thick = " ".join(f"{n}={_panel_t(n)*1e3:.1f}mm{'out' if RB_PANELS[n][1] > 0 else 'in'}"
                     for n, _, _ in panels)
    out_t = 2 * ht + _panel_grow_out("front") + _panel_grow_out("back")
    cav_h = 2 * hh - _panel_grow_in("bottom")
    print(f"[rigid-bag] 5-panel proxy: {thick}")
    print(f"[rigid-bag] density={density:.1f} vol={vol*1e9:.0f}mm^3 "
          f"mass={vol*density*1e3:.3f}g (FEM cloth 등가 {RB_TARGET_MASS*1e3:.3f}g)")
    print(f"[rigid-bag] 외형 폭={2*hw*1e3:.1f}mm 두께={out_t*1e3:.1f}mm  공동 높이={cav_h*1e3:.1f}mm")
    return dst, out_t


def prepare_rigid_tablet_mjcf():
    """정제 프록시 — MJCF capsule geom(= SDF primitive, §조합6)."""
    r, half_cyl = fw.CAP_RADIUS_MM * 1e-3, fw.CAP_CYL_H_MM * 1e-3 / 2
    xml = f"""<mujoco model="rigid_tablet">
  <compiler angle="radian"/>
  <worldbody>
    <body name="tablet_proxy" pos="0 0 0">
      <freejoint name="tablet_free"/>
      <geom name="tablet_cap" type="capsule" size="{r:.6f} {half_cyl:.6f}"
            density="{fw.TABLET_RHO}" friction="{fw.TABLET_FRICTION} 0.02 0.001"
            contype="1" conaffinity="1" rgba="0.90 0.90 0.85 1"/>
    </body>
  </worldbody>
</mujoco>
"""
    dst = os.path.join(tempfile.mkdtemp(prefix="rigid_tablet_"), "rigid_tablet.xml")
    with open(dst, "w", encoding="utf-8") as f:
        f.write(xml)
    vol = np.pi * r * r * (2 * half_cyl) + 4 / 3 * np.pi * r ** 3
    print(f"[rigid-tablet] capsule R={r*1e3:.2f}mm L_total={(2*half_cyl+2*r)*1e3:.2f}mm "
          f"mass={vol*fw.TABLET_RHO*1e3:.4f}g")
    return dst


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" RIGID workflow (no coupling, all rigid bodies) — viewer={use_viewer}")
    print(f" 슬롯 개방 지령={WALL_OFFSET*1e3:+.1f}mm  Y_OFFSET={Y_OFFSET_MM:+.1f}mm  "
          f"tablet_cargo={int(RB_TABLET_CARGO)}  leftwall_col={int(RB_LEFTWALL_COLLISION)}")
    print("=" * 60)

    crusher_xml = _prepare_crusher_mjcf()
    robot_xml = fw._prepare_robot_mjcf()
    bag_xml, bag_outer_t = prepare_rigid_bag_mjcf()
    tablet_xml = prepare_rigid_tablet_mjcf()

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=fw.DT, gravity=(0, 0, -9.81)),
        # 씬에 연성 솔버가 하나도 없으므로 IPC 를 통째로 뺀다. 리지드-리지드 접촉은
        # 리지드 솔버가 자체 처리하고, 재질간 커플링 플래그는 전부 False.
        coupler_options=gs.options.LegacyCouplerOptions(
            rigid_mpm=False, rigid_sph=False, rigid_pbd=False, rigid_fem=False,
            mpm_sph=False, mpm_pbd=False, fem_mpm=False, fem_sph=False,
        ),
        vis_options=gs.options.VisOptions(
            background_color=(0.93, 0.94, 0.96),
            ambient_light=(0.16, 0.16, 0.18),
            lights=[
                {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 6.0},
                {"type": "directional", "dir": (1, 1, -0.6), "color": (1.0, 1.0, 1.0), "intensity": 1.2},
            ],
        ),
        show_viewer=use_viewer,
    )

    def rmat(friction=RB_FRICTION, **kw):
        # needs_coup=False: 커플러가 쓸 SDF 생성을 건너뛴다(연성 솔버가 없으므로).
        return gs.materials.Rigid(needs_coup=False, friction=friction, **kw)

    for p in fw.PLATE_POSITIONS:
        scene.add_entity(
            gs.morphs.Mesh(file=fw.PLATE_PATH, fixed=True, pos=p),
            material=rmat(),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )

    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=fw.CRUSHER_POS, euler=fw.CRUSHER_EULER,
                       decimate=True, convexify=True),
        material=rmat(),
        surface=gs.surfaces.Default(smooth=False),
    )

    scene.add_entity(gs.morphs.Plane(visualization=False), material=rmat())
    scene.add_entity(
        gs.morphs.Box(size=fw.SHELF_SIZE, pos=fw.SHELF_POS, fixed=True),
        material=rmat(friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )
    for mjcf, pos in ((fw.FIXTURE_MJCF, fw.FIXTURE_POS),
                      (fw.RECOVERY2_MJCF, fw.RECOVERY2_POS),
                      (fw.SUCTION_MJCF, fw.SUCTIONV1_POS)):
        scene.add_entity(gs.morphs.MJCF(file=mjcf, pos=pos, decimate=False), material=rmat())

    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, pos=tuple(fw.ROBOT_OFFSET), decimate=False),
        # PD 로 구동하므로 중력보상을 켠다 — 안 주면 정상상태 오차로 팔이 처져
        # IK 목표에서 벗어난다(§12-4).
        material=rmat(gravity_compensation=1.0),
    )
    bag = scene.add_entity(
        gs.morphs.MJCF(file=bag_xml, pos=fw.BAG_POS, euler=fw.BAG_EULER), material=rmat())
    tablet = scene.add_entity(
        gs.morphs.MJCF(file=tablet_xml, pos=fw.TABLET_POS), material=rmat(friction=fw.TABLET_FRICTION))

    cam_over = scene.add_camera(res=(1280, 960), pos=fw.OVERVIEW_CAM_POS,
                               lookat=fw.OVERVIEW_CAM_LOOK, fov=48, GUI=False)
    cam_bag = scene.add_camera(res=(960, 720), pos=tuple(np.array(fw.BAG_POS) + fw.BAGCAM_OFFSET),
                               lookat=fw.BAG_POS, fov=40, GUI=False)
    cam_side = scene.add_camera(res=(960, 720), pos=fw.OVERVIEW_CAM_POS,
                                lookat=fw.OVERVIEW_CAM_LOOK, fov=45, GUI=False)

    print("\n[build] scene.build() 시작...")
    t_build = time.time()
    scene.build(n_envs=0)
    build_s = time.time() - t_build
    t_steps = time.time()
    print(f"[build] 성공 ({build_s:.1f}s)")

    # 봉투 자세 하드 홀드 — rigid 프록시는 64x8mm 바닥면에 90mm 높이로 서 있어
    # 그냥 두면 넘어진다. FEM 의 정점 제약과 같은 역할(§12-4).
    bag_hold_pos = _npy(bag.get_pos()).squeeze().copy()
    bag_hold_quat = _npy(bag.get_quat()).squeeze().copy()
    print(f"[bag] 자세 고정(hold): pos={bag_hold_pos} quat={bag_hold_quat}")

    # ── Crusher 슬롯 위치 계산 (fw 와 동일) ──────────────────────────────────
    wb_lo, wb_hi = fw.crusher_mesh_world_aabb(fw.WALL_BACK_MESH)
    wl_lo, wl_hi = fw.crusher_mesh_world_aabb(fw.WALL_LEFT_MESH, fw.LEFTWALL_BODY_POS,
                                              fw.LEFTWALL_GEOM_POS)
    gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
    gap_cx = (gap_lo_x + gap_hi_x) / 2.0
    gap_width = gap_hi_x - gap_lo_x
    y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
    gap_cy = (y_lo + y_hi) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])
    print(f"[slot] gap_cx={gap_cx:.4f} gap_cy={gap_cy:.4f} gap_width(정지)={gap_width*1000:.1f}mm "
          f"wall_top_z={wall_top_z:.4f}")

    crusher_joints = {j.name: j for j in crusher.joints if j.name}

    def _scalar_dof(name):
        d = crusher_joints[name].dofs_idx_local
        return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d

    crank_dof = _scalar_dof(fw.CRANK_JOINT)
    wall_dof = _scalar_dof(fw.WALL_JOINT)
    crusher.set_dofs_kp(np.array([fw.CRANK_KP]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kv(np.array([fw.CRANK_KV]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kp(np.array([fw.WALL_KP]), dofs_idx_local=[wall_dof])
    crusher.set_dofs_kv(np.array([fw.WALL_KV]), dofs_idx_local=[wall_dof])

    left_link = robot.get_link(fw.FINGER_LINKS[0])
    q_grasp, q_lift = fw.Q_GRASP, fw.Q_LIFT
    robot.set_dofs_position(np.concatenate([q_grasp, [fw.FING_OPEN] * 6]))

    # PD 게인. RG2 mimic 은 MJCF <equality><joint> 를 Genesis 가 안 지키므로
    # 핑거 6 DOF 에 같은 목표각을 주는 fw 방식을 그대로 유지한다.
    n_dof = robot.n_dofs
    robot.set_dofs_kp(np.concatenate([np.full(6, RB_ARM_KP), np.full(n_dof - 6, RB_FING_KP)]))
    robot.set_dofs_kv(np.concatenate([np.full(6, RB_ARM_KV), np.full(n_dof - 6, RB_FING_KV)]))
    print(f"[robot] PD 구동: n_dofs={n_dof} arm kp/kv={RB_ARM_KP}/{RB_ARM_KV} "
          f"finger kp/kv={RB_FING_KP}/{RB_FING_KV} (gravity_compensation=1.0)")

    cam_over.start_recording(save_to_filename=MP4_OVERVIEW, fps=30)
    cam_bag.start_recording(save_to_filename=MP4_BAGCAM, fps=30)
    cam_side.start_recording(save_to_filename=MP4_SIDE, fps=30)

    # ── 상태 지표 ────────────────────────────────────────────────────────────
    hw, hh = fw.BAG_PANEL_HALF_W, fw.BAG_PANEL_HALF_H
    _LOCAL_LO = np.array([-hw - _panel_grow_out("left"), -hh - _panel_grow_out("bottom"),
                          -BAG_HALF_T - _panel_grow_out("back")])
    _LOCAL_HI = np.array([hw + _panel_grow_out("right"), hh,  # +hh 는 입구(open)
                          BAG_HALF_T + _panel_grow_out("front")])
    _CAV_LO = np.array([-hw + _panel_grow_in("left"), -hh + _panel_grow_in("bottom"), -BAG_HALF_T])
    _CAV_HI = np.array([hw - _panel_grow_in("right"), hh, BAG_HALF_T])

    def _bag_com():
        return _npy(bag.get_pos()).squeeze()

    def _bag_corners():
        p, R = _bag_com(), _quat_to_R(_npy(bag.get_quat()).squeeze())
        c = np.array([[_LOCAL_LO[0] if i & 1 else _LOCAL_HI[0],
                       _LOCAL_LO[1] if i & 2 else _LOCAL_HI[1],
                       _LOCAL_LO[2] if i & 4 else _LOCAL_HI[2]] for i in range(8)])
        return p + (R @ c.T).T

    def _bag_pose():
        """봉투 자세를 각도로 직접 잰다 — AABB 폭/높이는 회전이 섞이면 해석이
        안 된다(기울면 Z-extent 가 실제 높이 91mm 를 넘어버림).
          tilt  = 높이축(로컬 +Y)이 world +Z 에서 몇 도 기울었나 (0 이 이상)
          twist = 두께축(로컬 +Z)이 슬롯 두께 방향(world X)에서 몇 도 틀어졌나
        """
        R = _quat_to_R(_npy(bag.get_quat()).squeeze())
        tilt = np.degrees(np.arccos(np.clip(abs(R[:, 1] @ np.array([0., 0., 1.])), 0, 1)))
        twist = np.degrees(np.arccos(np.clip(abs(R[:, 2] @ np.array([1., 0., 0.])), 0, 1)))
        return float(tilt), float(twist)

    def _tablet_status():
        p, bp = _npy(tablet.get_pos()).squeeze(), _bag_com()
        L = (_quat_to_R(_npy(bag.get_quat()).squeeze()).T @ (p - bp)) * 1e3
        lo, hi, r = _CAV_LO * 1e3, _CAV_HI * 1e3, fw.CAP_RADIUS_MM
        out = [ax for i, ax in enumerate("xyz") if L[i] < lo[i] - r or L[i] > hi[i] + r]
        return f"  tablet_local=({L[0]:+.1f},{L[1]:+.1f},{L[2]:+.1f})mm " + \
               ("IN" if not out else "OUT:" + "".join(out))

    def _status():
        tilt, twist = _bag_pose()
        return f"  tilt={tilt:.1f}deg twist={twist:.1f}deg{_tablet_status()}"

    # ── 스텝 구동 ────────────────────────────────────────────────────────────
    bag_held = [True]
    tablet_cargo = [None]

    def drive_robot(q, f):
        # 텔레포트 금지 — 위치 강제는 접촉 임펄스를 깨뜨린다(§7-7).
        robot.control_dofs_position(np.concatenate([q, [f] * 6]))

    def step_sim():
        scene.step()
        if bag_held[0]:
            bag.set_pos(bag_hold_pos)
            bag.set_quat(bag_hold_quat)
            bag.set_dofs_velocity(np.zeros(bag.n_dofs))
        if tablet_cargo[0] is not None:
            bp, bq = _bag_com(), _npy(bag.get_quat()).squeeze()
            tablet.set_pos(bp + _quat_to_R(bq) @ tablet_cargo[0])
            tablet.set_quat(bq)
            tablet.set_dofs_velocity(np.zeros(tablet.n_dofs))

    def render_cams():
        cam_over.render()
        bc = _bag_com()
        cam_bag.set_pose(pos=tuple(bc + fw.BAGCAM_OFFSET), lookat=tuple(bc), up=(0, 0, 1))
        cam_bag.render()
        cam_side.render()

    def run_arm(name, q0, q1, f0, f1, n, crank_q=None, wall_q=None, trace=False):
        q = q1
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            drive_robot(q, f0 + (f1 - f0) * s)
            if crank_q is not None:
                crusher.control_dofs_position(np.array([crank_q]), dofs_idx_local=[crank_dof])
            if wall_q is not None:
                crusher.control_dofs_position(np.array([wall_q]), dofs_idx_local=[wall_dof])
            step_sim()
            render_cams()
            if trace and k % 40 == 0:
                print(f"    [{name} k={k:4d}] bag_z={_bag_com()[2]*1e3:+.1f}mm{_status()}")
        q_err = float(np.abs(_npy(robot.get_dofs_position())[:6] - q).max())
        print(f"[phase] {name:8s} @done  bag_com={_bag_com()}  finger_z="
              f"{float(_npy(left_link.get_pos()).squeeze()[2]):.4f}  "
              f"q_err={q_err*1e3:.2f}mrad{_status()}")

    # ── Phase 0: prep — 크랭크 -180도, Left_Wall 개방 ─────────────────────────
    print(f"\n[phase] 0 prep ({fw.N_PREP*fw.DT:.1f}s) — 크랭크 0->{fw.CRANK_START_Q:+.3f}rad, "
          f"Left_Wall 0->{WALL_OFFSET*1000:+.1f}mm(개방)")
    for k in range(fw.N_PREP):
        s = (k + 1) / fw.N_PREP
        crusher.control_dofs_position(np.array([fw.CRANK_START_Q * s]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_position(np.array([WALL_OFFSET * s]), dofs_idx_local=[wall_dof])
        drive_robot(q_grasp, fw.FING_OPEN)
        step_sim()
        render_cams()
    cq = _npy(crusher.get_dofs_position())[crank_dof]
    wq = _npy(crusher.get_dofs_position())[wall_dof]
    # 실제 통로 폭은 지령이 아니라 **실측 wq** 로 계산해야 한다 — 잼이 있으면
    # 둘이 크게 어긋나고, 그 상태로는 봉투가 절대 안 들어간다(§12-7).
    open_gap = gap_width + wq
    print(f"[phase] prep     @done  crank={cq:+.3f}rad  wall={wq*1000:+.2f}mm "
          f"(지령 {WALL_OFFSET*1000:+.2f}mm, 오차 {(wq-WALL_OFFSET)*1000:+.2f}mm)")
    print(f"[slot] 실제 통로 = {gap_width*1000:.1f}(정지) + {wq*1000:+.2f}(벽) = {open_gap*1000:.1f}mm"
          f"  vs 봉투 두께 {bag_outer_t*1e3:.1f}mm → 여유 {(open_gap-bag_outer_t)/2*1000:+.1f}mm/쪽")
    if abs(wq - WALL_OFFSET) > 1e-3:
        print(f"[slot] **주의** 벽이 지령을 못 따라간다 — 기구 잼 또는 하드 스톱 "
              f"(RB_LEFTWALL_COLLISION={int(RB_LEFTWALL_COLLISION)}; +7.1mm 는 알려진 하드 스톱)")

    # ── Phase 1-6: 정제 낙하 -> 봉투 파지 -> 리프트 ───────────────────────────
    run_arm("drop", q_grasp, q_grasp, fw.FING_OPEN, fw.FING_OPEN, fw.N_DROP,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle", q_grasp, q_grasp, fw.FING_OPEN, fw.FING_OPEN, fw.N_SETTLE,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET)

    if RB_TABLET_CARGO:
        # 담긴 상태의 로컬 오프셋을 고정 — 이후 정제는 봉투를 따라다니는 화물.
        # "담기"는 §조합5/6/9(FEM+IPC)에서 이미 검증된 항목이라 여기서 다시 풀지
        # 않는다. 게다가 공동 두께 6mm 에 캡슐 지름 4mm 라 여유가 1mm/쪽뿐이고,
        # hold 해제 순간 정제가 front 패널을 뚫고 튀어나간다(§12-6).
        bp = _bag_com()
        tablet_cargo[0] = _quat_to_R(_npy(bag.get_quat()).squeeze()).T @ \
            (_npy(tablet.get_pos()).squeeze() - bp)
        print(f"[tablet] cargo 종속 (로컬 오프셋 {tablet_cargo[0]*1e3} mm)")

    run_arm("close", q_grasp, q_grasp, fw.FING_OPEN, fw.FING_CLOSE, fw.N_CLOSE,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET)
    # 핑거가 다 닫힌 뒤에 홀드를 푼다 — 붙잡을 게 있는 상태에서 풀기 위함(§12-4).
    bag_held[0] = False
    print("[bag] hold 해제(핑거 닫힘 완료) — 이제부터 순수 마찰 파지")

    run_arm("grasp", q_grasp, q_grasp, fw.FING_CLOSE, fw.FING_CLOSE, fw.N_GRASP,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("lift", q_grasp, q_lift, fw.FING_CLOSE, fw.FING_CLOSE, fw.N_LIFT,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("hold", q_lift, q_lift, fw.FING_CLOSE, fw.FING_CLOSE, fw.N_HOLD,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET)

    # ── Phase 7-8: above -> insert (IK) ──────────────────────────────────────
    wall_center_z = (wall_top_z + wb_lo[2]) / 2.0
    above_z = wall_top_z + 0.20
    insert_z = wall_center_z + INSERT_MARGIN_ABOVE_CENTER
    target_xy = np.array([gap_cx, gap_cy + Y_OFFSET])
    print(f"[slot] wall_center_z={wall_center_z:.4f}  insert_z(finger)={insert_z:.4f}")

    side_z = (above_z + wall_center_z) / 2.0
    cam_side.set_pose(pos=(gap_cx, gap_cy + fw.SIDECAM_Y_OFFSET, side_z),
                      lookat=(gap_cx, gap_cy, side_z), up=(0, 0, 1))

    def ik(z):
        return _npy(robot.inverse_kinematics(
            link=left_link, pos=np.array([target_xy[0], target_xy[1], z]),
            quat=fw.VERTICAL_QUAT, local_point=fw.FINGER_TCP_LOCAL,
            dofs_idx_local=np.arange(6)))[:6]

    qpos_above = ik(above_z)
    print(f"\n[ik] above arm_q={qpos_above}")
    run_arm("above", q_lift, qpos_above, fw.FING_CLOSE, fw.FING_CLOSE, fw.N_ABOVE,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)

    qpos_insert = ik(insert_z)
    print(f"[ik] insert arm_q={qpos_insert}")
    run_arm("insert", qpos_above, qpos_insert, fw.FING_CLOSE, fw.FING_CLOSE, fw.N_INSERT,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle2", qpos_insert, qpos_insert, fw.FING_CLOSE, fw.FING_CLOSE, fw.N_SETTLE2,
            crank_q=fw.CRANK_START_Q, wall_q=WALL_OFFSET)

    # ── Phase 9-10: clamp -> release ─────────────────────────────────────────
    print(f"\n[phase] 9 clamp ({fw.N_CLAMP*fw.DT:.1f}s) — Left_Wall {WALL_OFFSET*1000:+.1f}mm -> "
          f"{fw.CLAMP_TARGET*1000:+.1f}mm")
    for k in range(fw.N_CLAMP):
        s = (k + 1) / fw.N_CLAMP
        crusher.control_dofs_position(np.array([WALL_OFFSET + (fw.CLAMP_TARGET - WALL_OFFSET) * s]),
                                      dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([fw.CRANK_START_Q]), dofs_idx_local=[crank_dof])
        drive_robot(qpos_insert, fw.FING_CLOSE)
        step_sim()
        render_cams()
    wq_final = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[phase] clamp    @done  wall={wq_final*1000:+.2f}mm{_status()}")
    if not RB_LEFTWALL_COLLISION:
        print("[phase] clamp    (주의) Left_Wall 충돌 OFF — 벽이 봉투를 통과한다. "
              "클램프 접촉을 보려면 볼록분해 후 RB_LEFTWALL_COLLISION=1")

    run_arm("release", qpos_insert, qpos_insert, fw.FING_CLOSE, fw.FING_OPEN, fw.N_RELEASE,
            crank_q=fw.CRANK_START_Q, wall_q=fw.CLAMP_TARGET, trace=True)

    # ── 판정 — rigid 기준 ────────────────────────────────────────────────────
    # fw 의 판정식은 "봉투 바닥이 wall_center_z 근방"이라는 **양방향** 창이라,
    # 봉투가 포켓 바닥까지 제대로 내려가면 오히려 FAIL 이 뜬다(§12-7). rigid
    # 에서는 "포켓 안으로 충분히 들어갔는가 + 자세가 서 있는가"로 본다.
    corners = _bag_corners()
    bottom_z = float(corners[:, 2].min())
    tilt, twist = _bag_pose()
    pocket_floor_z = float(wb_lo[2])
    entered = bottom_z < wall_top_z - 0.02          # 벽 윗면보다 20mm 이상 아래
    seated = bottom_z < wall_center_z + 0.01        # 포켓 중앙 이하까지 내려옴
    upright = tilt < 15.0 and twist < 15.0
    verdict = "PASS" if (entered and seated and upright) else "FAIL"
    reasons = []
    if not entered:
        reasons.append(f"슬롯 진입 실패(bottom_z={bottom_z:.4f} vs wall_top_z={wall_top_z:.4f} — 벽 위에 얹힘)")
    if not seated:
        reasons.append(f"삽입 깊이 부족(bottom_z={bottom_z:.4f} > wall_center_z+10mm)")
    if not upright:
        reasons.append(f"자세 불량(tilt={tilt:.1f}deg twist={twist:.1f}deg, 임계 15deg)")
    print(f"\n[RESULT] verdict={verdict}  ({'; '.join(reasons) if reasons else '삽입 성공, 자세 정상'})")
    print(f"[RESULT] wall_open 지령={WALL_OFFSET*1e3:+.1f}mm 실제={wq*1e3:+.2f}mm  통로={open_gap*1e3:.1f}mm")
    print(f"[RESULT] bag_bottom_z={bottom_z:.4f}  (wall_top={wall_top_z:.4f} "
          f"wall_center={wall_center_z:.4f} pocket_floor={pocket_floor_z:.4f})")
    print(f"[RESULT] tilt={tilt:.1f}deg  twist={twist:.1f}deg")

    steps_s = time.time() - t_steps
    n_steps = (fw.N_PREP + fw.N_DROP + fw.N_SETTLE + fw.N_CLOSE + fw.N_GRASP + fw.N_LIFT
               + fw.N_HOLD + fw.N_ABOVE + fw.N_INSERT + fw.N_SETTLE2 + fw.N_CLAMP + fw.N_RELEASE)
    print(f"\n[timing] build={build_s:.1f}s  steps={steps_s:.1f}s "
          f"({n_steps} steps, {steps_s/n_steps*1e3:.1f}ms/step)")

    cam_over.stop_recording()
    cam_bag.stop_recording()
    cam_side.stop_recording()
    for p in (MP4_OVERVIEW, MP4_BAGCAM, MP4_SIDE):
        print(f"[saved] {os.path.basename(p)}")
    print(f"[saved] -> {OUT_DIR}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")
