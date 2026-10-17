# sim/render/helpers.py
"""render 包内的共享辅助。"""
from __future__ import annotations

import numpy as np

from core.algorithms.math.transform import gimbal_to_world


def rel_to_world(rel_pos, frame, controlled):
    """观测/预测的 rel_pos -> 世界坐标。

    frame = "W"：rel_pos 已是世界坐标。
    frame = "G"：rel_pos 相对云台中心（G 系原点），加 gimbal.world_pos。
    """
    if frame == "W":
        return np.asarray(rel_pos, float).reshape(3)
    pos, _ = gimbal_to_world(rel_pos, np.eye(3),
                             gimbal_pos_world=controlled.gimbal.world_pos)
    return pos