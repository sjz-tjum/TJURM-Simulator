# core/algorithms/math/distance.py
"""距离计算。"""
import math

import numpy as np

__all__ = [
    "distance", "get_distance", "distance_squared",
    "point_to_segment_distance",
]


def distance(pose1, pose2):
    """两个 3D 点的欧氏距离。"""
    dx = pose1[0] - pose2[0]
    dy = pose1[1] - pose2[1]
    dz = pose1[2] - pose2[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


get_distance = distance


def distance_squared(pose1, pose2):
    """两个 3D 点的欧氏距离平方（避免开方）。"""
    dx = pose1[0] - pose2[0]
    dy = pose1[1] - pose2[1]
    dz = pose1[2] - pose2[2]
    return dx * dx + dy * dy + dz * dz


def point_to_segment_distance(point, seg_a, seg_b):
    """点到线段的最短距离（3D）。"""
    p = np.asarray(point, float)
    a = np.asarray(seg_a, float)
    b = np.asarray(seg_b, float)
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return float(np.linalg.norm(p - a))
    t = float((p - a) @ ab) / denom
    t = max(0.0, min(1.0, t))
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))