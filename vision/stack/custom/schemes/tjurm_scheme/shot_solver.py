# custom/schemes/tjurm_scheme/shot_solver.py
"""一发解：迭代 model → select → traj，输出 fire / mode / yaw / pitch + 前馈 + fire_key。

迭代变量是 fly_time。每一轮：
    model   —— 按 fly 外推，拿候选板/灯臂和车心（各模型内部按类型分）
    select  —— 按 target 类型 + mode 挑瞄点（ArmorSelector 提供三种几何）
    traj    —— 从瞄准点解 (yaw, pitch, fly_new)
    fly_new 反馈给下一轮，直到收敛或迭代次数用尽。

fire 判定在循环外做一次——需要收敛后的 fly。

前馈：瞄点视线的角速度。分两类算——
    真实板 / 灯臂（车 auto、前哨、符）：v = aim_velocity（含自转分量）
    虚拟点（车 single）：v = 车心平移速度（虚拟点不随车自转）

差分窗口太小看不见提速/切板——那是 MPC 的事。但"瞄点在以什么角速度转"
是当前状态，EKF 一直在维护，不依赖窗口——用它做前馈是对的。

输出 ShotSolution（发给电控）+ pred_center / pred_armor（供 tracker 构造 Prediction）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from config import cfg
from core.bus.dataflow.control import ShotSolution
from core.entities.property.entity_types import EntityType


@dataclass
class ShotBundle:
    """一发的完整解 + 中间结果。"""
    solution: Optional[ShotSolution]
    fly: float
    pred_center: Optional[np.ndarray]
    pred_armor: Optional[np.ndarray]


class ShotSolver:
    """一发解。"""

    ITERS = 5
    FLY_EPS = 1e-4
    FLY_INIT = 0.2

    def __init__(self, shoot_decider, traj_solver, armor_selector, frame):
        self.shoot_decider = shoot_decider
        self.traj_solver = traj_solver
        self.armor_selector = armor_selector
        self.frame = frame
        self._last_fly = self.FLY_INIT

    def reset(self) -> None:
        self._last_fly = self.FLY_INIT
        self.traj_solver.reset()

    def solve(self, target, self_state, timestamp) -> ShotBundle:
        mode = self.shoot_decider.decide_mode(target)
        kind = getattr(target, "robot_type", None)

        gun_rel_pos = self._muzzle_rel(self_state)
        cur_dir = self._cur_dir(self_state)

        fly = self._last_fly
        aim_point = None
        yaw = pitch = 0.0
        for _ in range(self.ITERS):
            aim_point = self._select(target, kind, mode, fly, gun_rel_pos, cur_dir)
            if aim_point is None:
                return ShotBundle(None, self._last_fly, None, None)
            result = self.traj_solver.solve_aim(aim_point, self_state)
            if result is None:
                return ShotBundle(None, self._last_fly, None, None)
            yaw, pitch, fly_new = result
            if abs(fly_new - fly) < self.FLY_EPS:
                fly = fly_new
                break
            fly = fly_new

        self._last_fly = fly

        fire = self.shoot_decider.decide_fire(target, fly, mode, gun_rel_pos)
        yaw_rate, pitch_rate = self._calc_rate(
            target, kind, mode, aim_point, gun_rel_pos, fly)

        fire_key = getattr(self.armor_selector, "selected_armor_id", None)
        cooldown_hint = (float(cfg.sim.aim.rune_cooldown)
                         if kind == EntityType.Rune else 0.0)

        solution = ShotSolution(
            yaw=float(yaw), pitch=float(pitch),
            yaw_rate=yaw_rate, pitch_rate=pitch_rate,
            fire=bool(fire), fire_mode=str(mode),
            fire_key=fire_key,
            cooldown=cooldown_hint,
            timestamp=float(timestamp),
        )

        x = target.ekf.ekf.x
        pred_center = np.array([x[0] + x[3] * fly,
                                x[1] + x[4] * fly,
                                x[2] + x[5] * fly])
        pred_armor = np.asarray(aim_point, float).copy()
        return ShotBundle(solution, fly, pred_center, pred_armor)

    # ------------------------------------------------------------ 内部
    def _select(self, target, kind, mode, fly, gun_rel_pos, cur_dir):
        if kind == EntityType.Rune:
            return self.armor_selector.select_lit_blade(target, gun_rel_pos, fly)
        if kind == EntityType.Outpost or mode == "auto":
            _id, pos = self.armor_selector.select_armor(
                target, gun_rel_pos, fly, cur_dir)
            return pos
        return self.armor_selector.select_virtual(target, gun_rel_pos, fly)

    def _muzzle_rel(self, self_state):
        muzzle = np.asarray(self_state.muzzle.pos, float)
        if self.frame == "G":
            muzzle = muzzle - np.asarray(self_state.gimbal.pos, float)
        return muzzle

    def _cur_dir(self, self_state):
        yaw = float(self_state.gimbal.rpy[2])
        pitch = -float(self_state.gimbal.rpy[1])
        return np.array([np.cos(yaw) * np.cos(pitch),
                         np.sin(yaw) * np.cos(pitch),
                         np.sin(pitch)])

    @staticmethod
    def _calc_rate(target, kind, mode, aim_point, gun_rel_pos, fly):
        """瞄点视线的角速度（rad/s），给电控做前馈。

        分两类：
            - 真实板 / 灯臂（车 auto、前哨、符）：v = aim_velocity，含自转分量。
            - 虚拟点（车 single）：v = 车心平移速度，虚拟点不随车自转。

        之前的版本对虚拟点也用 aim_velocity——那会算出一个虚假的自转分量，
        把云台带偏（且 ω 越大偏得越多）。
        """
        d = np.asarray(aim_point, float) - np.asarray(gun_rel_pos, float)
        r_xy = float(np.hypot(d[0], d[1]))
        if r_xy < 1e-3:
            return 0.0, 0.0

        if kind == EntityType.Rune or kind == EntityType.Outpost or mode == "auto":
            v = target.aim_velocity(aim_point, fly)
        else:
            # 车 single（虚拟点）：虚拟点只随车心平移
            x = target.ekf.ekf.x
            v = np.array([x[3], x[4], x[5]], float)

        yaw_rate = (d[0] * v[1] - d[1] * v[0]) / (r_xy * r_xy)
        pitch_rate = (r_xy * v[2] - d[2] * (d[0] * v[0] + d[1] * v[1]) / r_xy) / \
            (r_xy * r_xy + d[2] * d[2])
        return float(yaw_rate), float(pitch_rate)