# core/algorithms/math/transform.py
"""坐标系转换：所有坐标系之间的变换。

坐标系总览：

    物理链（由机械结构、电机角、相机投影决定）：
        world ↔ gimbal ↔ camera / muzzle
        world ↔ camera（用 camera 的绝对位姿一步到位，与 world ↔ gimbal 同构）
        camera ↔ pixel ↔ normalized

    PnP 数学链（像素 ↔ C' 的换算归单板求解器，见 custom/components/armor_solvers/）：
        abs_camera ↔ gimbal（方向一致，只差平移）
        abs_camera ↔ abs_armor（A'，方向与 world 一致、原点在板中心）
        abs_armor ↔ armor

约定：
    - 命名 <from>_to_<to>(pos, R, *, ...) -> (pos', R')
    - 参数名带坐标系后缀：gimbal_pos_world 是"云台在世界下的位置"
    - 轴序后缀：涉及 camera 系或归一化的函数，名字里带 FLU / RDF 说清轴序
    - 正交矩阵的逆 = 转置；角度逆 = 取负
    - 不提供复合变换：需要多段的，自己组合
    - vec2d_ 派生自 vec3d_：轴序公式只在 vec3d_ 一处
"""
from __future__ import annotations

import math

import numpy as np

__all__ = [
    "rpy_to_R", "R_to_rpy", "rpyt_to_T",
    "FLU2RDF", "RDF2FLU",
    "vec3d_FLU2RDF", "vec3d_RDF2FLU",
    "vec2d_FLU2RDF", "vec2d_RDF2FLU",
    "world_to_gimbal", "gimbal_to_world",
    "world_to_camera_FLU", "camera_FLU_to_world",
    "gimbal_to_camera", "camera_to_gimbal",
    "gimbal_to_muzzle", "muzzle_to_gimbal",
    "camera_RDF_to_normalized",
    "pixel_to_normalized_RDF", "normalized_RDF_to_pixel",
    "abs_camera_to_gimbal", "gimbal_to_abs_camera",
    "abs_camera_to_abs_armor", "abs_armor_to_abs_camera",
    "armor_to_abs_armor", "abs_armor_to_armor",
]


# ---------------------------------------------------------------- 通用数学

def rpy_to_R(rpy):
    """欧拉角 [roll, pitch, yaw] -> R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    """R -> [roll, pitch, yaw]（ZYX 顺序）。"""
    R = np.asarray(R, float)
    sp = -R[2, 0]
    if sp >= 1.0 - 1e-9:
        pitch = math.pi / 2
        roll = 0.0
        yaw = math.atan2(R[0, 1], R[0, 2])
    elif sp <= -1.0 + 1e-9:
        pitch = -math.pi / 2
        roll = 0.0
        yaw = -math.atan2(R[0, 1], R[0, 2])
    else:
        pitch = math.asin(sp)
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw], float)


def rpyt_to_T(rpy, t):
    """欧拉角 + 平移 -> 4x4 齐次变换矩阵。"""
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rpy)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


# ---------------------------------------------------------------- 轴序常量

_FLU2RDF = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
], float)


def FLU2RDF():
    """FLU（前-左-上）轴序 -> RDF（右-下-前）轴序的旋转矩阵。"""
    return _FLU2RDF.copy()


def RDF2FLU():
    """RDF 轴序 -> FLU 轴序的旋转矩阵（FLU2RDF 的转置）。"""
    return _FLU2RDF.T.copy()


def vec3d_FLU2RDF(v):
    """3D 向量（FLU 轴序）-> 3D 向量（RDF 轴序）。单点 (3,) 或批量 (N,3)。"""
    return np.asarray(v, float) @ _FLU2RDF.T


def vec3d_RDF2FLU(v):
    """3D 向量（RDF 轴序）-> 3D 向量（FLU 轴序）。"""
    return np.asarray(v, float) @ _FLU2RDF


def vec2d_FLU2RDF(v):
    """归一化坐标（FLU）-> 归一化坐标（RDF）。

    FLU 归一化 = (y/x, z/x)，补 x=1 升维，用 vec3d_FLU2RDF 换轴序，
    再除以 RDF 深度 z。轴序公式只在 vec3d_FLU2RDF 一处。
    """
    v = np.asarray(v, float)
    rays = np.concatenate([np.ones(v.shape[:-1] + (1,)), v], axis=-1)
    rdf = vec3d_FLU2RDF(rays)
    return rdf[..., :2] / rdf[..., 2:3]


def vec2d_RDF2FLU(v):
    """归一化坐标（RDF）-> 归一化坐标（FLU）。"""
    v = np.asarray(v, float)
    rays = np.concatenate([v, np.ones(v.shape[:-1] + (1,))], axis=-1)
    flu = vec3d_RDF2FLU(rays)
    return flu[..., 1:] / flu[..., 0:1]


# ---------------------------------------------------------------- world ↔ gimbal

def world_to_gimbal(pos, R, *, gimbal_pos_world):
    """world -> gimbal。两系方向一致，只差平移。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    gimbal_pos_world = np.asarray(gimbal_pos_world, float).reshape(3)
    return pos - gimbal_pos_world, R.copy()


def gimbal_to_world(pos, R, *, gimbal_pos_world):
    """gimbal -> world。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    gimbal_pos_world = np.asarray(gimbal_pos_world, float).reshape(3)
    return pos + gimbal_pos_world, R.copy()


# ---------------------------------------------------------------- world ↔ camera

def world_to_camera_FLU(pos, R, *, camera_pos_world, camera_rpy_world):
    """world -> camera（FLU 轴序）。

    用 camera 的绝对位姿一步到位——camera 自己知道它在世界里的姿态，
    不必经过 gimbal；与 world_to_gimbal 同构，不是复合变换。
    """
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    camera_pos_world = np.asarray(camera_pos_world, float).reshape(3)
    R_c = rpy_to_R(camera_rpy_world)
    return R_c.T @ (pos - camera_pos_world), R_c.T @ R


def camera_FLU_to_world(pos, R, *, camera_pos_world, camera_rpy_world):
    """camera（FLU）-> world。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    camera_pos_world = np.asarray(camera_pos_world, float).reshape(3)
    R_c = rpy_to_R(camera_rpy_world)
    return R_c @ pos + camera_pos_world, R_c @ R


# ---------------------------------------------------------------- gimbal ↔ camera

def _gimbal_to_camera_pose(yaw, pitch, pitch_offset,
                            camera_mount_pos, camera_mount_rpy):
    """算出 camera 在 gimbal 下的位姿 (pos_c2g, R_c2g)。"""
    yaw = float(yaw)
    pitch = float(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    cp, sp = math.cos(pitch), math.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)

    pitch_offset = np.asarray(pitch_offset, float).reshape(3)
    camera_mount_pos = np.asarray(camera_mount_pos, float).reshape(3)
    camera_mount_rpy = np.asarray(camera_mount_rpy, float).reshape(3)

    R_pitch_frame = Rz @ Ry
    R_c2g = R_pitch_frame @ rpy_to_R(camera_mount_rpy)
    pos_c2g = Rz @ pitch_offset + R_pitch_frame @ camera_mount_pos
    return pos_c2g, R_c2g


def gimbal_to_camera(pos, R, *, yaw, pitch, pitch_offset,
                     camera_mount_pos, camera_mount_rpy):
    """gimbal -> camera。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    pos_c2g, R_c2g = _gimbal_to_camera_pose(
        yaw, pitch, pitch_offset, camera_mount_pos, camera_mount_rpy)
    pos_camera = R_c2g.T @ (pos - pos_c2g)
    R_camera = R_c2g.T @ R
    return pos_camera, R_camera


def camera_to_gimbal(pos, R, *, yaw, pitch, pitch_offset,
                     camera_mount_pos, camera_mount_rpy):
    """camera -> gimbal（gimbal_to_camera 的逆）。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    pos_c2g, R_c2g = _gimbal_to_camera_pose(
        yaw, pitch, pitch_offset, camera_mount_pos, camera_mount_rpy)
    pos_gimbal = pos_c2g + R_c2g @ pos
    R_gimbal = R_c2g @ R
    return pos_gimbal, R_gimbal


# ---------------------------------------------------------------- gimbal ↔ muzzle

def _gimbal_to_muzzle_pose(yaw, pitch, pitch_offset, muzzle_mount_pos):
    """算出 muzzle 在 gimbal 下的位姿 (pos_m2g, R_m2g)。"""
    yaw = float(yaw)
    pitch = float(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    cp, sp = math.cos(pitch), math.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)

    pitch_offset = np.asarray(pitch_offset, float).reshape(3)
    muzzle_mount_pos = np.asarray(muzzle_mount_pos, float).reshape(3)

    R_m2g = Rz @ Ry
    pos_m2g = Rz @ pitch_offset + R_m2g @ muzzle_mount_pos
    return pos_m2g, R_m2g


def gimbal_to_muzzle(pos, R, *, yaw, pitch, pitch_offset, muzzle_mount_pos):
    """gimbal -> muzzle。链同 camera，只是没有安装旋转。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    pos_m2g, R_m2g = _gimbal_to_muzzle_pose(
        yaw, pitch, pitch_offset, muzzle_mount_pos)
    pos_muzzle = R_m2g.T @ (pos - pos_m2g)
    R_muzzle = R_m2g.T @ R
    return pos_muzzle, R_muzzle


def muzzle_to_gimbal(pos, R, *, yaw, pitch, pitch_offset, muzzle_mount_pos):
    """muzzle -> gimbal（gimbal_to_muzzle 的逆）。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    pos_m2g, R_m2g = _gimbal_to_muzzle_pose(
        yaw, pitch, pitch_offset, muzzle_mount_pos)
    pos_gimbal = pos_m2g + R_m2g @ pos
    R_gimbal = R_m2g @ R
    return pos_gimbal, R_gimbal


# ---------------------------------------------------------------- camera ↔ normalized

def camera_RDF_to_normalized(pos):
    """camera（RDF）3D 点 -> 归一化（RDF）。"""
    pos = np.asarray(pos, float)
    return pos[..., :2] / pos[..., 2:3]


# ---------------------------------------------------------------- pixel ↔ normalized

def pixel_to_normalized_RDF(pixel, *, K):
    """像素 -> 归一化（RDF）。"""
    K = np.asarray(K, float)
    f = float(K[0, 0])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    p = np.asarray(pixel, float)
    return np.stack([(p[..., 0] - cx) / f, (p[..., 1] - cy) / f], axis=-1)


def normalized_RDF_to_pixel(normalized, *, K):
    """归一化（RDF）-> 像素。"""
    K = np.asarray(K, float)
    f = float(K[0, 0])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    n = np.asarray(normalized, float)
    return np.stack([n[..., 0] * f + cx, n[..., 1] * f + cy], axis=-1)


# ---------------------------------------------------------------- C' ↔ gimbal

def abs_camera_to_gimbal(pos, R, *, camera_pos_gimbal):
    """C' -> gimbal。两系方向一致，只差平移。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    camera_pos_gimbal = np.asarray(camera_pos_gimbal, float).reshape(3)
    return pos + camera_pos_gimbal, R.copy()


def gimbal_to_abs_camera(pos, R, *, camera_pos_gimbal):
    """gimbal -> C'。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    camera_pos_gimbal = np.asarray(camera_pos_gimbal, float).reshape(3)
    return pos - camera_pos_gimbal, R.copy()


# ---------------------------------------------------------------- C' ↔ A'

def abs_camera_to_abs_armor(pos, R, *, armor_pos_abs_camera):
    """C' -> A'。两系方向一致，只差平移。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    armor_pos_abs_camera = np.asarray(armor_pos_abs_camera, float).reshape(3)
    return pos - armor_pos_abs_camera, R.copy()


def abs_armor_to_abs_camera(pos, R, *, armor_pos_abs_camera):
    """A' -> C'。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    armor_pos_abs_camera = np.asarray(armor_pos_abs_camera, float).reshape(3)
    return pos + armor_pos_abs_camera, R.copy()


# ---------------------------------------------------------------- A' ↔ armor

def armor_to_abs_armor(pos, R, *, psi, theta_0):
    """armor -> A'。同原点（板中心），方向由 psi 与 theta_0 决定。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    psi = float(psi)
    theta_0 = float(theta_0)
    cz, sz = math.cos(psi), math.sin(psi)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], float)
    cy, sy = math.cos(theta_0), math.sin(theta_0)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], float)
    R_a2a = Rz @ Ry
    return pos.copy(), R_a2a @ R


def abs_armor_to_armor(pos, R, *, psi, theta_0):
    """A' -> armor（armor_to_abs_armor 的逆）。"""
    pos = np.asarray(pos, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    psi = float(psi)
    theta_0 = float(theta_0)
    cz, sz = math.cos(psi), math.sin(psi)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], float)
    cy, sy = math.cos(theta_0), math.sin(theta_0)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], float)
    R_a2a = Rz @ Ry
    return pos.copy(), R_a2a.T @ R