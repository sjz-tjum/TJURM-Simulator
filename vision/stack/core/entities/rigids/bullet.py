# core/entities/bullet.py
"""弹丸：重力 + 二次阻力（欧拉积分）。时间由调用方给定。"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from config import cfg
from core.entities.property.rigid import Rigid


_GRAVITY = np.array([0.0, 0.0, -float(cfg.sim.gravity)])


class Bullet(Rigid):
    """一发弹丸。弹种参数（mass / caliber / drag_cd / damage）由调用方从配置取。"""

    def __init__(self, pos, vel, mass, caliber, drag_cd, damage, shooter=None,
                 origin=None):
        super().__init__(world_pos=pos, world_vel=vel)
        # 发射点：射程判据要的是"飞了多远"，不是"离场地原点多远"。
        self.origin = (self.world_pos.copy() if origin is None
                       else np.asarray(origin, float).copy())
        self.mass = float(mass)
        self.caliber = float(caliber)
        self.drag_cd = float(drag_cd)
        self.rho_air = float(cfg.sim.air_density)
        self.area = math.pi * (self.caliber / 2.0) ** 2
        self.damage = float(damage)
        self.shooter = shooter
        self.age = 0.0
        self.active = True
        self.hit = False
        self.hit_pos = None
        self.hit_target = None
        self.hit_time: Optional[float] = None

    def update(self, dt: float) -> None:
        """一步欧拉积分：阻力反向、重力向下。"""
        v = float(np.linalg.norm(self.world_vel))
        if v > 1e-6:
            drag = 0.5 * self.rho_air * self.area * self.drag_cd * v * v
            acc = -drag / self.mass * (self.world_vel / v)
        else:
            acc = np.zeros(3)
        self.world_vel += (acc + _GRAVITY) * dt
        self.world_pos += self.world_vel * dt
        self.age += dt