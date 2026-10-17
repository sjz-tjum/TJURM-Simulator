# core/algorithms/math/quaternion.py
"""四元数：构造、运算、与旋转矩阵互转、插值。

约定：四元数表示为 ``np.ndarray``，形状 (4,)，顺序 ``[w, x, y, z]``，
单位四元数表示旋转。
"""
import math
from typing import Optional

import numpy as np

__all__ = [
    "Quat",
    "quat_identity", "quat_from_rpy", "quat_to_rpy",
    "quat_from_axis_angle", "quat_from_rotation_matrix",
    "quat_to_rotation_matrix",
    "quat_multiply", "quat_conjugate", "quat_inverse",
    "quat_normalize", "quat_norm", "quat_dot",
    "quat_rotate", "quat_slerp", "quat_nlerp",
    "quat_is_unit", "quat_almost_equal",
]

Quat = np.ndarray


# ---------------------------------------------------------------- 构造
def quat_identity() -> Quat:
    """单位四元数。"""
    return np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_rpy(rpy) -> Quat:
    """欧拉角 (roll, pitch, yaw) -> 四元数（ZYX 顺序，与 Rz @ Ry @ Rx 一致）。"""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def quat_to_rpy(q) -> np.ndarray:
    """四元数 -> 欧拉角 (roll, pitch, yaw)。"""
    w, x, y, z = (float(v) for v in q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def quat_from_axis_angle(axis, angle: float) -> Quat:
    """绕单位轴 axis 旋转 angle 弧度。"""
    axis = np.asarray(axis, float).reshape(3)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return quat_identity()
    axis = axis / n
    half = angle * 0.5
    s = math.sin(half)
    return np.array([math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_from_rotation_matrix(R) -> Quat:
    """旋转矩阵 -> 四元数（Shepperd 方法，按最大分量分支）。"""
    R = np.asarray(R, float)
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([w, x, y, z])
    return quat_normalize(q)


def quat_to_rotation_matrix(q) -> np.ndarray:
    """四元数 -> 旋转矩阵。"""
    w, x, y, z = quat_normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ])


# ---------------------------------------------------------------- 运算
def quat_multiply(q1, q2) -> Quat:
    """四元数乘法 q1 * q2。"""
    w1, x1, y1, z1 = (float(v) for v in q1)
    w2, x2, y2, z2 = (float(v) for v in q2)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conjugate(q) -> Quat:
    """四元数共轭（单位四元数的逆）。"""
    w, x, y, z = (float(v) for v in q)
    return np.array([w, -x, -y, -z])


def quat_inverse(q) -> Quat:
    """四元数逆。"""
    q = np.asarray(q, float)
    n2 = float(q @ q)
    if n2 < 1e-24:
        return quat_identity()
    return quat_conjugate(q) / n2


def quat_norm(q) -> float:
    """四元数模长。"""
    return float(np.linalg.norm(np.asarray(q, float)))


def quat_normalize(q) -> Quat:
    """归一化；模长过小时返回单位四元数。"""
    q = np.asarray(q, float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return quat_identity()
    return q / n


def quat_dot(q1, q2) -> float:
    """四元数点积。"""
    return float(np.asarray(q1, float) @ np.asarray(q2, float))


def quat_rotate(q, v) -> np.ndarray:
    """用四元数旋转 3D 向量 v：q * [0, v] * q^-1。"""
    q = quat_normalize(q)
    v = np.asarray(v, float).reshape(3)
    qv = np.array([0.0, v[0], v[1], v[2]])
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[1:]


# ---------------------------------------------------------------- 插值
def quat_nlerp(q0, q1, t: float) -> Quat:
    """归一化线性插值（快，但角速度不均匀）。"""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    if quat_dot(q0, q1) < 0.0:
        q1 = -q1
    return quat_normalize((1.0 - t) * q0 + t * q1)


def quat_slerp(q0, q1, t: float) -> Quat:
    """球面线性插值（角速度均匀）。"""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = quat_dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        return quat_nlerp(q0, q1, t)

    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * t
    sin_theta = math.sin(theta)
    sin_theta0 = math.sin(theta0)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0
    return quat_normalize(s0 * q0 + s1 * q1)


# ---------------------------------------------------------------- 检查
def quat_is_unit(q, tol: float = 1e-6) -> bool:
    """是否为单位四元数。"""
    return abs(quat_norm(q) - 1.0) <= tol


def quat_almost_equal(q1, q2, tol: float = 1e-6) -> bool:
    """近似相等，考虑 q 与 -q 表示同一旋转。"""
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    if quat_dot(q1, q2) < 0.0:
        q2 = -q2
    return bool(np.allclose(q1, q2, atol=tol))