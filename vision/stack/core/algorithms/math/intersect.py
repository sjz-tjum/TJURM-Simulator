# core/algorithms/math/intersect.py
"""线段与平面图形的相交判定。

无状态纯几何：不认识弹丸、装甲板或任何管理器，只吃基元（点 / 边向量 / 法线 / 半径）。
放在 math 层而不是 sim 层，是因为 custom 只能依赖 core——将来选板、遮挡判断要复用时够得着。

约定：所有点与向量都是世界系下的 (3,) numpy 数组，单位米。
"""
from __future__ import annotations

import math

import numpy as np

__all__ = ["segment_rect_hit", "segment_disc_hit"]


def _unit(v):
    """归一化；零向量返回 None。"""
    n = math.sqrt(float(v @ v))
    if n < 1e-12:
        return None
    return v / n


def segment_rect_hit(p0, p1, corner0, edge1, edge2, normal=None):
    """线段 p0 -> p1 与矩形面的相交判定。

    参数：
        corner0 —— 矩形的一个角点
        edge1 / edge2 —— 从 corner0 出发的两条邻边向量
        normal —— 面法线；不给就用 edge1 x edge2 现算。法线方向定义了"正面"

    返回 (hit, point)。要求：
        1. 线段穿过该平面（两端法向分量异号或为零）
        2. 交点落在矩形内（用 edge1/edge2 的斜坐标判 [0,1] x [0,1]）
        3. 子弹从正面来（线段方向与法线夹角 > 90 度）
    """
    v1 = np.asarray(edge1, float)
    v2 = np.asarray(edge2, float)
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    rel = np.asarray(corner0, float)

    if normal is None:
        normal = np.cross(v1, v2)
    n = _unit(np.asarray(normal, float))
    if n is None:
        return False, None

    d0 = float((p0 - rel) @ n)
    d1 = float((p1 - rel) @ n)
    if d0 * d1 > 0.0:
        return False, None
    denom = d1 - d0
    if abs(denom) < 1e-15:
        return False, None
    t = -d0 / denom
    if t < 0.0 or t > 1.0:
        return False, None

    point = p0 + t * (p1 - p0)
    w = point - rel
    a11 = float(v1 @ v1)
    a12 = float(v1 @ v2)
    a22 = float(v2 @ v2)
    det = a11 * a22 - a12 * a12
    if det < 1e-18:
        return False, None
    b1 = float(v1 @ w)
    b2 = float(v2 @ w)
    u = (a22 * b1 - a12 * b2) / det
    v = (a11 * b2 - a12 * b1) / det
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return False, None

    seg_dir = _unit(p1 - p0)
    if seg_dir is None or float(seg_dir @ n) >= 0.0:
        return False, None
    return True, point


def segment_disc_hit(p0, p1, center, normal, radius):
    """线段 p0 -> p1 与圆盘（靶面）的相交判定。

    三种情形：
        1. 穿面 —— 两端法向分量异号，交点落在半径内（要求从正面来）
        2. 共面 —— 两端法向分量都约等于 0（射手与靶面同竖直平面，对能量机关是常态）
        3. 平行不共面 —— 直接判不中

    返回 (hit, point)。
    """
    r = float(radius)
    if r <= 0.0:
        return False, None
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    c = np.asarray(center, float)
    n = _unit(np.asarray(normal, float))
    if n is None:
        return False, None

    d0 = float((p0 - c) @ n)
    d1 = float((p1 - c) @ n)
    seg = p1 - p0

    if abs(d1 - d0) > 1e-12 and d0 * d1 <= 0.0:
        # 穿面：子弹必须从法线正面来（与 segment_rect_hit 同口径）
        seg_dir = _unit(seg)
        if seg_dir is None or float(seg_dir @ n) >= 0.0:
            return False, None
        t = -d0 / (d1 - d0)
        if 0.0 <= t <= 1.0:
            point = p0 + t * seg
            if float(np.linalg.norm(point - c)) <= r:
                return True, point
        return False, None

    # 共面：两端贴面（|d| 极小），子弹在平面内走 —— 无正反之分，不检查方向
    if abs(d0) > 1e-6 or abs(d1) > 1e-6:
        return False, None

    w = p0 - c
    a = float(seg @ seg)
    if a < 1e-12:
        return False, None
    b = 2.0 * float(w @ seg)
    cc = float(w @ w) - r * r
    disc = b * b - 4.0 * a * cc
    if disc < 0.0:
        return False, None
    root = math.sqrt(disc)
    for t in ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)):
        if 0.0 <= t <= 1.0:
            return True, p0 + t * seg
    return False, None

