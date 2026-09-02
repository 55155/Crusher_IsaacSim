"""
fem_ipc_workarounds.py — Genesis FEMEntity.set_vertex_constraints() 의 IPC
커플러 체크 버그를 우회한다.

**버그(2026-07-15 확인, Genesis 설치본 fem_entity.py:900-901)**:
    if isinstance(self.sim.coupler, IPCCoupler):
        gs.raise_exception("This method is only supported by IPC coupler.")
조건이 뒤집혀 있다 — "IPC 커플러에서만 지원됨"이라는 메시지와 반대로,
**IPC 커플러를 쓸 때 예외를 던진다**(실측: `bag.set_vertex_constraints(...)`
호출 시 `GenesisException('This method is only supported by IPC coupler.')`
발생, 우리 프로젝트는 전부 IPC 커플러를 쓰므로 이 메서드가 사실상 항상
막혀 있었다).

**우회 방식**: Genesis 설치본(site-packages)은 전혀 안 건드리고, 원본 메서드의
로직(버그인 isinstance 체크 한 줄만 제외)을 그대로 복제해 런타임에
`FEMEntity.set_vertex_constraints`를 이 사본으로 몽키패치 교체한다. 패치 후에는
`entity.set_vertex_constraints(...)`를 원래 API 그대로 호출하면 된다.
`update_constraint_targets`/`remove_vertex_constraints`는 이 버그가 없어
(소스 확인됨) 패치 불필요.

사용법:
    from fem_ipc_workarounds import patch_fem_vertex_constraints
    patch_fem_vertex_constraints()   # import genesis 이후, scene.build() 이전 아무때나 1회

    bag.set_vertex_constraints(verts_idx_local=fixed_idx, is_soft_constraint=False)
    ...
    bag.remove_vertex_constraints()  # 원본 그대로, 패치 불필요
"""
_patched = False


def patch_fem_vertex_constraints():
    global _patched
    if _patched:
        return

    import torch
    import genesis as gs
    import genesis.engine.entities.fem_entity as fem_entity_module
    from genesis.engine.entities.rigid_entity.rigid_link import RigidLink

    FEMEntity = fem_entity_module.FEMEntity

    def _fixed_set_vertex_constraints(
        self, verts_idx_local, target_poss=None, link=None, is_soft_constraint=False, stiffness=0.0, envs_idx=None
    ):
        """원본 FEMEntity.set_vertex_constraints 사본 — IPCCoupler 체크(버그)만 제거."""
        if self._solver._use_implicit_solver and not self._solver._enable_vertex_constraints:
            gs.raise_exception(
                "This feature is disabled. Please set 'enable_vertex_constraints=True' when using FEM implicit solver."
            )
        # (원본의 `if isinstance(self.sim.coupler, IPCCoupler): raise` 는 로직이
        # 뒤집힌 버그라 여기서 의도적으로 생략한다.)

        if not self._solver._constraints_initialized:
            self._solver.init_constraints()

        use_current_poss = target_poss is None
        envs_idx = self._scene._sanitize_envs_idx(envs_idx)
        verts_idx_local = self._sanitize_verts_idx_local(verts_idx_local, envs_idx)
        verts_idx = verts_idx_local + self._v_start
        target_poss = self._sanitize_verts_tensor(target_poss, gs.tc_float, verts_idx, envs_idx, (3,))

        if use_current_poss:
            self._kernel_get_verts_pos(self._sim.cur_substep_local, target_poss, verts_idx)

        if link is None:
            link_idx = -1
            link_init_pos = torch.zeros((self._sim._B, 3), dtype=gs.tc_float, device=gs.device)
            link_init_quat = torch.zeros((self._sim._B, 4), dtype=gs.tc_float, device=gs.device)
        else:
            assert isinstance(link, RigidLink), "Only RigidLink is supported for vertex constraints."
            link_idx = link.idx
            link_init_pos = link.get_pos()
            link_init_quat = link.get_quat()
            if self._scene.n_envs == 0:
                link_init_pos = link_init_pos[None]
                link_init_quat = link_init_quat[None]

        self._solver._kernel_set_vertex_constraints(
            self._sim.cur_substep_local,
            verts_idx,
            target_poss,
            is_soft_constraint,
            stiffness,
            link_idx,
            link_init_pos,
            link_init_quat,
            envs_idx,
        )

    FEMEntity.set_vertex_constraints = _fixed_set_vertex_constraints
    _patched = True
    print("[fem_ipc_workarounds] FEMEntity.set_vertex_constraints 패치 완료 (IPC 커플러 버그 우회)")


def patch_ipc_vertex_attach(strength_rate=100.0):
    """IPC 커플러에 FEM 정점 부착을 배선한다 (2026-09-02).

    **왜 필요한가**: IPC 커플러를 켜면 천은 uipc 가 스텝하는데, `coupler.py` 전체에
    FEM 정점 구속을 참조하는 코드가 한 줄도 없다(거기 constraint 는 전부 강체
    링크용이다). 그래서 `set_vertex_constraints` 는 IPC 씬에서 이렇게 동작한다:

        소프트  아무 일도 안 일어남 (stiffness 1e4~1e8 결과가 소수점까지 동일)
        하드    자기 정점만 목표로 순간이동, 이웃이 안 따라와 메시가 찢김
                (90mm 봉투 bbox 가 470~545mm 로. 무구속 대조군은 형상 유지)

    **우회**: uipc 에 이미 있는 `SoftPositionConstraint` 를 천 메시에 배선한다.
    새 물리가 아니라 Genesis 가 안 연결한 배선을 잇는 것이다. 강체 링크는 Genesis
    가 이미 같은 메커니즘(`is_constrained` + `aim_transform`)을 쓰고 있다 — 다만
    그건 두 솔버 동기화 장치이지 부착 기능이 아니라, 천에는 필요가 없어서 안
    만들어졌다.

    적용 후 정점 전체에 속성이 생기고 **기본값은 전부 꺼짐**이다:
        is_constrained (n,)      0
        aim_position   (n,3,1)   각 정점의 현재 위치
    켜도 제자리라 아무 일도 안 일어난다. 누구를 잡을지는 런타임에 고른다.

    `_ipc_world.init()` 이후에는 씬을 못 바꾸므로 build 때 항상 걸어둔다.

    사용법:
        patch_ipc_vertex_attach()          # import genesis 이후, scene.build() 이전
        ...
        scene.build()
        slot = scene.sim.coupler.cloth_slots[(bag, 0)]

    docs/GenesisPatch.md 참조.
    """
    import genesis as gs
    from genesis.engine.couplers.ipc_coupler.coupler import IPCCoupler
    from genesis.engine.materials.FEM.cloth import Cloth
    from uipc.constitution import SoftPositionConstraint

    if getattr(IPCCoupler, "_vertex_attach_patched", False):
        return

    _orig = IPCCoupler._add_fem_entities_to_ipc

    def _wrapped(self):
        _orig(self)   # Genesis 가 천 메시를 씬에 올린다(슬롯은 버려진다)

        spc = SoftPositionConstraint()
        self._ipc_constitution_tabular.insert(spc)
        self.cloth_slots = {}

        for i_e, entity in enumerate(self.fem_solver.entities):
            if not isinstance(entity.material, Cloth):
                continue
            for env_idx in range(self.sim._B):
                objs = self._ipc_scene.objects().find(f"cloth_{i_e}_{env_idx}")
                if not objs:
                    gs.logger.warning(f"[vertex_attach] cloth_{i_e}_{env_idx} 못 찾음")
                    continue
                gids = list(objs[0].geometries().ids())
                slot = self._ipc_scene.geometries().find(int(gids[0]))[0]
                spc.apply_to(slot.geometry(), strength_rate)
                self.cloth_slots[(entity, env_idx)] = slot

        n = len(self.cloth_slots)
        print(f"[fem_ipc_workarounds] SoftPositionConstraint 배선 완료 — 천 {n}개 "
              f"(strength_rate={strength_rate}, 정점은 전부 꺼진 상태)")

    IPCCoupler._add_fem_entities_to_ipc = _wrapped
    IPCCoupler._vertex_attach_patched = True
