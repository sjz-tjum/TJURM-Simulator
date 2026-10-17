# core/entities/rigids/rigids.py
"""刚体基类：只存状态，不做物理。"""
from __future__ import annotations

from typing import Optional

import numpy as np


def _vec3(v) -> np.ndarray:
    if v is None: return np.zeros(3)

    a = np.array(v, dtype=float).reshape(-1)
    if a.size != 3:
        raise ValueError(f"期望 3 维向量，实得 shape={np.shape(v)}")
    return a.copy()


class Rigid:
    def __init__(self,
                 world_pos: Optional[np.ndarray] = None,
                 world_vel: Optional[np.ndarray] = None,
                 world_acc: Optional[np.ndarray] = None,
                 world_rpy: Optional[np.ndarray] = None,
                 world_omg: Optional[np.ndarray] = None,
                 world_alp: Optional[np.ndarray] = None):
        self.world_pos = _vec3(world_pos)
        self.world_vel = _vec3(world_vel)
        self.world_acc = _vec3(world_acc)
        self.world_rpy = _vec3(world_rpy)
        self.world_omg = _vec3(world_omg)
        self.world_alp = _vec3(world_alp)
