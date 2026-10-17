# custom/components/shooters/tjurm_shooter.py
"""射击决策：三支策略的判据。

    车 ω < 阈     auto     追板     板法向朝向自车
    车 ω ≥ 阈     single   虚拟点   板接近虚拟点
    前哨          auto     追板     板法向朝向自车（同车 auto）
    符            auto     点亮灯臂  有灯臂点亮（灯臂亮着就持续打）

fire 输出与开火模式的关系：
    fire 一律是"电平"——不作脉冲。
    - auto：电控按冷却节流——灯臂/板亮着就一遍遍打，直到打中或条件消失。
      符和前哨都走这条：符的"打中"由 Rune.register_hit 反馈（灯臂灭 → fire 落回 False）。
    - single + fire_key 非 None：电控按 fire_key 变化判"新一发"。
    - single + fire_key 为 None（车虚拟点）：电控按 fire 上升沿判"新一发"。

本类不做几何选择（归 ArmorSelector）、不做弹道（归 TjurmTrajSolver）、
不做伺服（归 ControlManager）。它只判"该不该打这一发"。
"""
from __future__ import annotations

import math

import numpy as np

from core.entities.property.entity_types import EntityType


class ShootDecider:
    """三支策略。"""

    def __init__(self, spin_thresh=3.0,
                 armor_fire_cos=0.8, virtual_fire_thresh=0.3):
        self.spin_thresh = float(spin_thresh)
        self.armor_fire_cos = float(armor_fire_cos)
        self.virtual_fire_thresh = float(virtual_fire_thresh)

    # ------------------------------------------------------------ mode
    def decide_mode(self, target) -> str:
        kind = getattr(target, "robot_type", None)
        if kind == EntityType.Rune:
            # 符：灯臂亮着就按冷却持续打——打中由 Rune 侧反馈（灯臂灭 → fire 落回）。
            # 打不中时灯臂不会切换，需要继续打——single 的"切换才打一发"会让符永远卡死。
            return "auto"
        if kind == EntityType.Outpost:
            return "auto"          # 前哨 2.5 rad/s，不够快，永远追板
        x = target.ekf.ekf.x
        omega = abs(float(x[7])) if len(x) > 7 else 0.0
        return "single" if omega > self.spin_thresh else "auto"

    # ------------------------------------------------------------ fire
    def decide_fire(self, target, fly, mode, self_rel_pos) -> bool:
        kind = getattr(target, "robot_type", None)
        if kind == EntityType.Rune:
            return self._fire_rune(target, fly)
        if kind == EntityType.Outpost or mode == "auto":
            return self._fire_normal(target, fly, self_rel_pos)
        return self._fire_virtual(target, fly, self_rel_pos)

    def _fire_normal(self, target, fly, self_rel_pos) -> bool:
        """板法向朝向自车。"""
        cands = target.ekf.armor_candidates(fly)
        for pos, normal, _lit in cands:
            sight = self_rel_pos - np.asarray(pos, float)
            dist = float(np.linalg.norm(sight))
            if dist < 1e-6:
                continue
            cos_alpha = float(np.dot(normal, sight / dist))
            if cos_alpha > self.armor_fire_cos:
                return True
        return False

    def _fire_virtual(self, target, fly, self_rel_pos) -> bool:
        """板接近虚拟点（= 板方位接近指向自车方向）。"""
        x = target.ekf.ekf.x
        center = np.array([x[0] + x[3] * fly,
                           x[1] + x[4] * fly,
                           x[2] + x[5] * fly])
        d_self = self_rel_pos - center
        n1 = float(np.linalg.norm(d_self))
        if n1 < 1e-6:
            return False
        d_self = d_self / n1
        for pos, _normal, _lit in target.ekf.armor_candidates(fly):
            d_armor = np.asarray(pos, float) - center
            n2 = float(np.linalg.norm(d_armor))
            if n2 < 1e-6:
                continue
            d_armor = d_armor / n2
            cos = float(np.clip(np.dot(d_self, d_armor), -1.0, 1.0))
            if math.acos(cos) < self.virtual_fire_thresh:
                return True
        return False

    def _fire_rune(self, target, fly) -> bool:
        """符：有灯臂点亮就 fire（电平）。

        auto 模式下，电控按冷却节流持续打——直到打中、灯臂灭、fire 落回 False。
        """
        cands = target.ekf.armor_candidates(fly)
        return any(c[2] for c in cands)