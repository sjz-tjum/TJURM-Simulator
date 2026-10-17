# custom/components/armor_solvers/ippe.py
"""非增强 PnP：原始像素 -> 单应分解 + GN 精化 -> C' 系板位姿。

理论见 docs/design/ippe_derivation.md。投影的两段（camera（RDF）-> 归一化、
归一化 -> 像素）都调 transform 的函数；雅可比是导数推导，留在本文件。
"""
from __future__ import annotations

import math

import numpy as np

from core.algorithms.math.transform import (
    RDF2FLU,
    camera_RDF_to_normalized,
    normalized_RDF_to_pixel,
)
from custom.components.armor_solvers import plate_size


def object_points(armor_size):
    """板系四角点，顺序 **左上, 右上, 左下, 右下**（与图像点顺序严格一致）。

    ★★ 2026-10-13 按用户指出的真实工程修正顺序 ★★
      TJURM-Aiming/src/threads/pipeline/baseline/tracker/locater.cpp:27-39
          // 设置装甲板3D坐标，顺序为左上-右上-左下-右下，即矩阵行优先输出顺序
          (-w/2,-h/2,0);  (+w/2,-h/2,0);  (-w/2,+h/2,0);  (+w/2,+h/2,0)
      OpenRM/src/pointer/setter.cpp:93-107 setArmorFourPointsRelative()
          图像点同样是 [左.上, 右.上, 左.下, 右.下]，且**左右按灯条中心 x 强制分离**。

    本文件用板系 FLU 轴序（+x = 板面法线朝外, +y = 左, +z = 上）⇒ 左 = +y, 上 = +z：
        左上 = (0, +w/2, +h/2)   右上 = (0, -w/2, +h/2)
        左下 = (0, +w/2, -h/2)   右下 = (0, -w/2, -h/2)
    旧版是 [左上, 右上, 右下, 左下]（绕圈），与官方及配套图像点顺序都不一致 ✗
    """
    w, h = plate_size(armor_size)
    return np.array([
        [0.0, +w / 2.0, +h / 2.0],     # 左上 LT
        [0.0, -w / 2.0, +h / 2.0],     # 右上 RT
        [0.0, +w / 2.0, -h / 2.0],     # 左下 LB
        [0.0, -w / 2.0, -h / 2.0],     # 右下 RB
    ])


def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _rotvec_to_R(rvec):
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3)
    Kx = _skew(rvec / theta)
    return np.eye(3) + math.sin(theta) * Kx + (1.0 - math.cos(theta)) * (Kx @ Kx)


def _R_to_rotvec(R):
    cos_t = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    theta = math.acos(cos_t)
    if theta < 1e-9:
        return np.zeros(3)
    if abs(theta - math.pi) < 1e-6:
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        k = int(np.argmax(axis))
        axis = axis / max(axis[k], 1e-12)
        if R[k, (k + 1) % 3] < 0:
            axis[(k + 1) % 3] *= -1.0
        return axis * theta
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * math.sin(theta)) * theta


def _reprojection(rvec, t, obj, img, K):
    """返回 (代价, 残差, 雅可比)；角点落到相机背后返回 (None, None, None)。

    投影两段调 transform：camera（RDF）-> 归一化（RDF）-> 像素；
    雅可比是这两段的导数（链式展开），必须与投影同模型，故就地手写。
    """
    R = _rotvec_to_R(rvec)
    P = (R @ obj.T).T + t          # camera（RDF）3D 点
    z = P[:, 2]
    if np.any(z <= 1e-6):
        return None, None, None
    n = camera_RDF_to_normalized(P)
    proj = normalized_RDF_to_pixel(n, K=K)
    res = (proj - img).reshape(-1)
    if not np.all(np.isfinite(res)):
        return None, None, None
    f = float(K[0, 0])
    J = np.zeros((res.size, 6))
    for i in range(obj.shape[0]):
        x, y, zc = P[i]
        dproj_dP = np.array([[f / zc, 0.0, -f * x / (zc * zc)],
                             [0.0, f / zc, -f * y / (zc * zc)]])
        J[2 * i:2 * i + 2, :3] = dproj_dP @ (-_skew(R @ obj[i]))
        J[2 * i:2 * i + 2, 3:] = dproj_dP
    return float(res @ res), res, J


def solve_homography_pnp(image_points, obj, K, refine_iters=20):
    """原始像素 + 板系点 -> (R_pnp, t_pnp)，C_rdf 轴序；失败返回 (None, None)。"""
    img = np.asarray(image_points, float).reshape(-1, 2)
    K = np.asarray(K, float)
    if img.shape[0] != 4:
        return None, None
    K_inv = np.linalg.inv(K)

    xy = obj[:, 1:3]
    o_c = xy.mean(axis=0)
    scale_o = math.sqrt(2.0) / max(float(np.mean(np.linalg.norm(xy - o_c, axis=1))), 1e-9)
    o_n = (xy - o_c) * scale_o
    i_c = img.mean(axis=0)
    scale_i = math.sqrt(2.0) / max(float(np.mean(np.linalg.norm(img - i_c, axis=1))), 1e-9)
    i_n = (img - i_c) * scale_i

    A = []
    for (X, Y), (u, v) in zip(o_n, i_n):
        A.append([-X, -Y, -1.0, 0.0, 0.0, 0.0, u * X, u * Y, u])
        A.append([0.0, 0.0, 0.0, -X, -Y, -1.0, v * X, v * Y, v])
    _u, _s, Vt = np.linalg.svd(np.asarray(A, float))
    Hn = Vt[-1].reshape(3, 3)
    T_obj = np.array([[scale_o, 0.0, -scale_o * o_c[0]],
                      [0.0, scale_o, -scale_o * o_c[1]],
                      [0.0, 0.0, 1.0]])
    T_img_inv = np.array([[1.0 / scale_i, 0.0, i_c[0]],
                          [0.0, 1.0 / scale_i, i_c[1]],
                          [0.0, 0.0, 1.0]])
    H = T_img_inv @ Hn @ T_obj
    H = H / max(float(np.linalg.norm(H[:, 1])), 1e-12)

    h1 = K_inv @ H[:, 0]
    h2 = K_inv @ H[:, 1]
    h3 = K_inv @ H[:, 2]
    n1, n2 = float(np.linalg.norm(h1)), float(np.linalg.norm(h2))
    if n1 < 1e-12 or n2 < 1e-12:
        return None, None
    scale = 2.0 / (n1 + n2)
    ry = h1 / n1
    rz = h2 / n2
    rx = np.cross(ry, rz)
    if float(np.linalg.norm(rx)) < 1e-9:
        return None, None
    rx /= np.linalg.norm(rx)
    U, _S, Vt2 = np.linalg.svd(np.column_stack([rx, ry, rz]))
    R = U @ Vt2
    if np.linalg.det(R) < 0.0:
        U[:, -1] *= -1.0
        R = U @ Vt2
    t = scale * h3

    rvec = _R_to_rotvec(R)
    t = np.asarray(t, float).reshape(3)
    cost, res, J = _reprojection(rvec, t, obj, img, K)
    if cost is None:
        return None, None
    mu = 1e-6
    for _ in range(int(refine_iters)):
        if cost <= 1e-24:
            break
        accepted = False
        delta = None
        for _try in range(8):
            try:
                delta = np.linalg.solve(J.T @ J + mu * np.eye(6), -J.T @ res)
            except np.linalg.LinAlgError:
                mu *= 10.0
                continue
            if not np.all(np.isfinite(delta)):
                mu *= 10.0
                continue
            nc, nr, nJ = _reprojection(rvec + delta[:3], t + delta[3:], obj, img, K)
            if nc is not None and nc < cost:
                rvec = rvec + delta[:3]
                t = t + delta[3:]
                cost, res, J = nc, nr, nJ
                mu = max(mu * 0.3, 1e-12)
                accepted = True
                break
            mu *= 10.0
        if not accepted:
            break
        if delta is not None and float(np.linalg.norm(delta)) < 1e-12:
            break
    R = _rotvec_to_R(rvec)
    if not np.all(np.isfinite(R)) or not np.all(np.isfinite(t)):
        return None, None
    return R, t


class IppeSolver:
    """非增强 PnP：全 6 自由度，不依赖任何解算系前提。"""

    name = "ippe"
    description = "非增强 PnP：单应分解 + GN，全 6 自由度"

    def estimate(self, *, camera_k, armor_size, image_points_2d,
                 rot_camera_to_abs_camera, mount_pitch=0.0):
        img = np.asarray(image_points_2d, float).reshape(-1, 2)
        if img.shape[0] != 4:
            return None, None
        obj = object_points(armor_size)

        R_pnp, t_pnp = solve_homography_pnp(img, obj, camera_k)
        if R_pnp is None:
            return None, None

        B = RDF2FLU()
        R_c1 = B @ R_pnp
        t_c1 = B @ np.asarray(t_pnp, float).reshape(3)

        P_c1 = (R_c1 @ obj.T).T + t_c1
        if np.any(P_c1[:, 0] <= 0.0):
            return None, None

        M = np.asarray(rot_camera_to_abs_camera, float).reshape(3, 3)
        R_abs = M @ R_c1
        t_abs = M @ t_c1

        psi = math.atan2(float(R_abs[1, 0]), float(R_abs[0, 0]))
        return t_abs, np.array([0.0, float(mount_pitch), psi], float)