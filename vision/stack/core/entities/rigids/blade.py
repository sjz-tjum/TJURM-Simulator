# core/entities/rigids/blade.py
"""能量机关的灯臂：圆靶，靶心由 Rune 推。

它不是装甲板 —— 是十环靶。现在的形态与既有消费者（sensor / bullet /
visualization）的读法一致；等 sensor 那轮改成直接给靶心后可以再简化。
"""
from __future__ import annotations

import copy

import numpy as np

from config import cfg
from core.entities.property.rigid import Rigid


class Blade(Rigid):
    def __init__(self, blade_id: int, **kwargs):
        super().__init__(**kwargs)
        c = cfg.rune
        self.blade_id = int(blade_id)
        self.armor_id = self.blade_id           # sensor / rune 里沿用旧名
        self.armor_size = c.armor_size
        self.hit_shape = "disc"
        self.hit_radius = float(c.target_diameter) / 2.0

        w = c.light_bar_interval
        h = c.light_bar_length
        # 灯条角点（本体坐标），顺序：左上 LT / 右上 RT / 右下 RB / 左下 LB
        self.init_light_corners = [
            np.array([0, w / 2.0, h / 2.0]),
            np.array([0, -w / 2.0, h / 2.0]),
            np.array([0, -w / 2.0, -h / 2.0]),
            np.array([0, w / 2.0, -h / 2.0]),
        ]
        self.light_corners = copy.deepcopy(self.init_light_corners)
        self.world_R = np.eye(3)