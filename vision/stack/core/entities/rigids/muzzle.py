# core/entities/rigids/muzzle.py
"""枪口：云台上的一个固定点。"""
from __future__ import annotations

from typing import Optional

import numpy as np

from core.entities.property.rigid import Rigid


class Muzzle(Rigid):
    def __init__(self,
                 mount_pos: Optional[np.ndarray] = None,
                 mount_rpy: Optional[np.ndarray] = None,
                 **kwargs):
        super().__init__(**kwargs)
        self.mount_pos = (np.zeros(3) if mount_pos is None
                          else np.asarray(mount_pos, float).copy())
        self.mount_rpy = (np.zeros(3) if mount_rpy is None
                          else np.asarray(mount_rpy, float).copy())
