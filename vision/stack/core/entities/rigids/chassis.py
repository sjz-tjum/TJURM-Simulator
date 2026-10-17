# core/entities/rigids/chassis.py
"""底盘：带装甲板，位置变化时把状态推给装甲板与云台。"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from config import cfg
from core.algorithms.math.angle import limit_rad
from core.algorithms.math.transform import rpy_to_R
from core.entities.property.entity_types import EntityType
from core.entities.rigids.armor import Armor
from core.entities.property.rigid import Rigid


class Chassis(Rigid):
    def __init__(self, entity_type, world_pos: Optional[np.ndarray] = None, **kwargs):
        entity_type = EntityType.coerce(entity_type)
        given_z = (None if world_pos is None
                   else float(np.asarray(world_pos, float).reshape(-1)[2]))
        super().__init__(world_pos=world_pos, **kwargs)

        name = EntityType.get_name(entity_type)
        c = cfg.robot[name]
        self.entity_type = entity_type
        self.priority = c.priority
        self.armor_count = int(c.armor_count)
        self.armor_size = c.armor_size
        self.length = float(c.length)
        self.width = float(c.width)
        self.high_height = float(c.high_height)
        self.low_height = float(c.low_height)
        self.radius = self.length / 2.0
        self.light_bar_interval = c.light_bar_interval
        self.light_bar_length = c.light_bar_length
        self.armor_tilt = math.radians(float(cfg.robot.armor_tilt))

        # 没给高度（或给了 0）时按车体高度取默认值
        if self.armor_count == 4:
            default_z = (self.high_height + self.low_height) / 2.0
        else:
            default_z = self.low_height
        if given_z is None or abs(given_z) < 1e-12:
            self.world_pos[2] = default_z

        self.armors = []
        self.gimbal = None
        self._create_armors()
        self.update_children()

    # ---------------------------------------------------------- 创建
    def _create_armors(self) -> None:
        """按 armor_count 均布在水平面上；偶数号在前/后轴、奇数号在左/右轴。

        位置 = 对应轴的半宽 × 单位方向；高度偶低奇高（RM 规则的高低交替）。
        """
        ref_z = self.world_pos[2]
        for i in range(self.armor_count):
            armor = Armor(armor_id=i, entity_type=self.entity_type)
            angle = i * 2.0 * math.pi / self.armor_count
            half_dim = self.length / 2.0 if i % 2 == 0 else self.width / 2.0
            rel_pos = np.array([math.cos(angle) * half_dim,
                                math.sin(angle) * half_dim,
                                (self.low_height if i % 2 == 0
                                 else self.high_height) - ref_z])
            rel_rpy = np.array([0.0, self.armor_tilt, angle])
            armor.mount_pos = rel_pos
            armor.mount_R = rpy_to_R(rel_rpy)
            armor.mount_pitch = float(rel_rpy[1])
            self.armors.append(armor)

    # ---------------------------------------------------------- 子部件
    def update_children(self) -> None:
        """把底盘状态推给每块装甲板，再推给云台。"""
        R = rpy_to_R(self.world_rpy)
        for armor in self.armors:
            armor.world_pos = self.world_pos + R @ armor.mount_pos
            armor_world_R = R @ armor.mount_R
            armor.world_R = armor_world_R

            pitch = math.asin(float(np.clip(-armor_world_R[2, 0], -1, 1)))
            offset = armor.armor_id * 2 * math.pi / self.armor_count
            armor.world_rpy = self.world_rpy.copy()
            armor.world_rpy[1] = pitch
            armor.world_rpy[2] = limit_rad(self.world_rpy[2] + offset)

            r_vec = armor.world_pos - self.world_pos
            armor.world_vel = self.world_vel + np.cross(self.world_omg, r_vec)
            armor.world_omg = self.world_omg.copy()

            for j, pt in enumerate(armor.init_light_corners):
                armor.light_corners[j] = armor.world_pos + armor_world_R @ pt

        if self.gimbal is not None:
            self.gimbal.world_pos = self.world_pos + R @ self.gimbal.mount_pos
            r_vec = self.gimbal.world_pos - self.world_pos
            self.gimbal.world_vel = self.world_vel + np.cross(self.world_omg, r_vec)