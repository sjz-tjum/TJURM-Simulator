# core/entities/rigids/armor.py
"""装甲板：车 / 前哨站的矩形板。圆靶（能量机关）不走这里。"""
from __future__ import annotations

import copy

import numpy as np

from config import cfg
from core.entities.property.entity_types import EntityType
from core.entities.property.rigid import Rigid


class Armor(Rigid):
    def __init__(self, armor_id: int, entity_type, **kwargs):
        super().__init__(**kwargs)
        entity_type = EntityType.coerce(entity_type)
        if entity_type is EntityType.Rune:
            raise ValueError("能量机关是圆靶，不是装甲板；用 RuneBlade。")
        self.armor_id = armor_id
        self.entity_type = entity_type

        name = EntityType.get_name(entity_type)
        if entity_type is EntityType.Outpost:
            self.armor_size = cfg.outpost.armor_size
            self.light_bar_interval = cfg.outpost.light_bar_interval
            self.light_bar_length = cfg.outpost.light_bar_length
        else:
            self.armor_size = cfg.robot[name].armor_size
            self.light_bar_interval = cfg.robot[name].light_bar_interval
            self.light_bar_length = cfg.robot[name].light_bar_length

        w = self.light_bar_interval
        h = self.light_bar_length
        # 灯条角点（本体坐标），顺序：左上 LT / 右上 RT / 右下 RB / 左下 LB
        self.init_light_corners = [
            np.array([0, w / 2.0, h / 2.0]),
            np.array([0, -w / 2.0, h / 2.0]),
            np.array([0, -w / 2.0, -h / 2.0]),
            np.array([0, w / 2.0, -h / 2.0]),
        ]
        self.light_corners = copy.deepcopy(self.init_light_corners)

        self.mount_pos = np.zeros(3)
        self.mount_R = np.eye(3)
        self.mount_pitch = 0.0
        self.world_R = np.eye(3)