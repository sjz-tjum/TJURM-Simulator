# core/entities/rigids/outpost.py
"""前哨站：固定位置，恒速旋转，三块装甲板。"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from config import cfg
from core.algorithms.math.angle import limit_rad
from core.algorithms.math.transform import rpy_to_R
from core.entities.property.entity_types import EntityType
from core.entities.property.health import Health
from core.entities.rigids.armor import Armor
from core.entities.property.rigid import Rigid


class Outpost(Rigid):
    def __init__(self, world_pos: Optional[np.ndarray] = None, **kwargs):
        c = cfg.outpost
        default_pos = np.array([0.0, 0.0, float(c.base_height) + float(c.height_step)])
        if world_pos is None:
            world_pos = default_pos.copy()
        super().__init__(world_pos=world_pos, **kwargs)

        self.radius = float(c.radius)
        self.armor_count = int(c.armor_count)
        self.base_height = float(c.base_height)
        self.height_step = float(c.height_step)
        self.rotate_speed = float(c.rotate_speed)
        self.armor_tilt = math.radians(float(cfg.outpost.armor_tilt))
        self.entity_type = EntityType.Outpost
        self.health = Health(c.hp)

        self.armors = []
        self._create_armors()
        self.update_children()

    # ---------------------------------------------------------- 创建
    def _create_armors(self) -> None:
        center_z = self.world_pos[2]
        for i in range(self.armor_count):
            armor = Armor(armor_id=i, entity_type=EntityType.Outpost)
            h = self.base_height + i * self.height_step
            angle = i * 2 * math.pi / self.armor_count
            rel_pos = np.array([self.radius * math.cos(angle),
                                self.radius * math.sin(angle),
                                h - center_z])
            rel_rpy = np.array([0, self.armor_tilt, angle])
            armor.mount_pos = rel_pos
            armor.mount_R = rpy_to_R(rel_rpy)
            armor.mount_pitch = float(rel_rpy[1])
            self.armors.append(armor)

    # ---------------------------------------------------------- 子部件
    def update_children(self) -> None:
        """把塔体状态传给三块装甲板。"""
        R = rpy_to_R(self.world_rpy)
        for i, armor in enumerate(self.armors):
            armor.world_pos = self.world_pos + R @ armor.mount_pos
            armor_world_R = R @ armor.mount_R
            armor.world_R = armor_world_R

            pitch = math.asin(float(np.clip(-armor_world_R[2, 0], -1, 1)))
            armor.world_rpy = self.world_rpy.copy()
            armor.world_rpy[1] = pitch
            armor.world_rpy[2] = limit_rad(
                self.world_rpy[2] + i * 2 * math.pi / self.armor_count)

            r_vec = armor.world_pos - self.world_pos
            armor.world_vel = self.world_vel + np.cross(self.world_omg, r_vec)
            armor.world_omg = self.world_omg.copy()

            for j, pt in enumerate(armor.init_light_corners):
                armor.light_corners[j] = armor.world_pos + armor_world_R @ pt

    # ---------------------------------------------------------- 运动学步进
    def update(self, dt: float, t: float) -> None:
        """运动学步进：恒速旋转。t 是仿真时刻（秒）。"""
        self.world_omg[2] = self.rotate_speed
        self.world_rpy[2] += self.world_omg[2] * dt
        self.world_rpy[2] = (self.world_rpy[2] + math.pi) % (2 * math.pi) - math.pi
        self.update_children()
