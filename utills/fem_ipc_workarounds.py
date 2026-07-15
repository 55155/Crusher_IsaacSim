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
