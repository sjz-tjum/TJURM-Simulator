# core/algorithms/math/angle.py
"""角度域数学工具。"""
import math

import numpy as np

__all__ = [
    "TWO_PI",
    "limit_rad", "safe_angle_sub", "unwrap_near", "unwrap_angle",
    "is_angle_step", "is_angle_trans", "angle_step_index", "trans_near",
    "get_angle_trans", "get_toggle", "nearest_armor_angle", "get_angle_min",
    "get_weight_by_theta",
]

TWO_PI = 2.0 * math.pi


# ---------------------------------------------------------------- wrapping
def limit_rad(angle):
    """包裹到 [-pi, pi)。支持标量、列表、ndarray。"""
    if isinstance(angle, np.ndarray):
        return (angle + np.pi) % TWO_PI - np.pi
    if isinstance(angle, (list, tuple)):
        return np.asarray([limit_rad(a) for a in angle])
    return (angle + math.pi) % TWO_PI - math.pi


def safe_angle_sub(angle1, angle2):
    """最短方向差：limit_rad(angle1 - angle2)，结果在 [-pi, pi)。"""
    return limit_rad(angle1 - angle2)


def unwrap_near(angle, ref):
    """把 angle 解包到离 ref 最近的等价角。"""
    return ref + safe_angle_sub(angle, ref)


unwrap_angle = unwrap_near


# ------------------------------------------------------- armor-count helpers
def is_angle_step(armor_count, target_angle, src_angle):
    """判断 target 与 src 的角差是否超过半个板间隔。"""
    if armor_count <= 0:
        return False
    return abs(safe_angle_sub(target_angle, src_angle)) > (math.pi / armor_count)


is_angle_trans = is_angle_step


def angle_step_index(armor_count, target_angle, src_angle, refer_angle=None):
    """需要平移多少个板间隔才能让 src 最接近 target。"""
    if armor_count <= 0:
        return 0
    step = TWO_PI / armor_count
    if refer_angle is None:
        refer_angle = src_angle
    k = 0
    while safe_angle_sub(refer_angle, target_angle) > step / 2.0:
        refer_angle -= step
        k -= 1
    while safe_angle_sub(target_angle, refer_angle) > step / 2.0:
        refer_angle += step
        k += 1
    return k


def trans_near(armor_count, target_angle, src_angle, refer_angle=None):
    """把 src_angle 平移 2*pi/armor_count 的整数倍，使其最接近 target。"""
    step = TWO_PI / armor_count if armor_count > 0 else 0.0
    return limit_rad(src_angle + angle_step_index(
        armor_count, target_angle, src_angle, refer_angle) * step)


get_angle_trans = trans_near


def get_toggle(armor_count, toggle, target_angle, src_angle):
    """偶数装甲板的乒乓标签。"""
    if armor_count < 4:
        return 0
    differ_angle = abs(safe_angle_sub(target_angle, src_angle))
    differ_toggle = int(round(2 * differ_angle / math.pi)) % 2
    return differ_toggle ^ toggle


def nearest_armor_angle(armor_angle, x, y, armor_count):
    """把 (x, y) 指向的方位角折算到离 armor_angle 最近的一块板上。"""
    center_angle = math.atan2(y, x)
    return trans_near(armor_count, center_angle, armor_angle, None)


get_angle_min = nearest_armor_angle


# ---------------------------------------------------------------- weights
def get_weight_by_theta(theta, k=400.0):
    """按角差给权重：exp(-k*theta^2)。"""
    w = math.exp(-theta * theta * k)
    if math.isfinite(w) and w > 0.0:
        return w
    return float(np.finfo(np.float64).tiny)