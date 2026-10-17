# custom/components/armor_solvers/qcqp.py
"""方向约束的闭式解（docs/design/qcqp_derivation.md）。

每块板的四条边各给一条方向约束：法向换到模型系 C' (4.5) 后只剩 psi 的 (c, s)
与已知的 theta_0，t 被完全消掉 —— 剩下的是单位圆上的二次问题：

    (5.1)(5.4)  u_k = R_y(theta_0) v_k, a_k = (n0*u0 + n1*u1, n1*u0 - n0*u1), c_k = n2*u2
    (P')        min ||diag(sqrt w) (A x - b)||^2  s.t. ||x||^2 = 1，行归一化 w_k
    (6.1)(6.2)  f(psi) = alpha + beta*cos2p + gamma*sin2p - 2(delta*cosp + epsilon*sinp)
    (6.3)       驻点方程用 t = tan(psi/2) 有理化成四次多项式，取 f 最小者 = 全局解

theta_0 != 0 时竖直边的 c_k 自带判别信息 (6.6)，psi 与 psi+pi 由代价自己分开；
theta_0 = 0 时方向信息退化，改用带标签点的重投影代价（复用 diff.py 的工具）。
t 由 (4.2)(4.4) 线性消去，再 GN 抛光（§9 D3）。
"""
from __future__ import annotations

import math

import numpy as np

from custom.components.armor_solvers.diff import (build_t_system, costs_at,
                                                  flu_normalized, line_normals,
                                                  model_basis, refine_t, solve_t)


def unit_circle_terms(normals_abs, v_rot):
    """(5.1)(5.3)(5.4)：a_k、c_k，以及 (P') 的行归一化权重 w_k。"""
    a = np.empty((4, 2), float)
    c = np.empty(4, float)
    for k in range(4):
        n0, n1, n2 = (float(x) for x in normals_abs[k])
        u0, u1, u2 = (float(x) for x in v_rot[k])
        a[k, 0] = n0 * u0 + n1 * u1
        a[k, 1] = n1 * u0 - n0 * u1
        c[k] = n2 * u2
    w = 1.0 / np.maximum(np.sum(a * a, axis=1) + c * c, 1e-18)
    return a, c, w


def direction_costs(psis, a, c, w):
    """(6.1) 的 f(psi)；常数项省略（不影响 argmin）。"""
    psis = np.asarray(psis, float).reshape(-1)
    x = np.column_stack([np.cos(psis), np.sin(psis)])
    r = x @ np.asarray(a, float).T + np.asarray(c, float)[None, :]
    return np.sum(r * r * np.asarray(w, float)[None, :], axis=1)


def quartic_coeffs(beta, gamma, delta, epsilon):
    """(6.3) 用 t = tan(psi/2) 有理化后的降幂系数（按 (6.2) 的记号展开）。"""
    return np.array([gamma + epsilon,
                     2.0 * (2.0 * beta + delta),
                     -6.0 * gamma,
                     2.0 * (delta - 2.0 * beta),
                     gamma - epsilon], float)


def solve_unit_circle(a, c, w):
    """(P')(6.1)-(6.3) 的全局解；返回 (psi, f_min)。"""
    A = np.asarray(a, float)
    cc = np.asarray(c, float)
    ww = np.asarray(w, float)
    M = A.T @ (ww[:, None] * A)
    g = -(A.T @ (ww * cc))
    beta = 0.5 * (M[0, 0] - M[1, 1])
    gamma = float(M[0, 1])
    delta, epsilon = float(g[0]), float(g[1])
    coeffs = quartic_coeffs(beta, gamma, delta, epsilon)
    cand = [math.pi]
    if np.any(np.abs(coeffs) > 1e-15):
        for root in np.roots(coeffs):
            if abs(float(root.imag)) < 1e-9:
                cand.append(2.0 * math.atan(float(root.real)))
    cand = np.asarray([p for p in cand if np.isfinite(p)], float)
    if cand.size == 0:
        return None, np.inf
    vals = direction_costs(cand, a, c, w)
    k = int(np.argmin(vals))
    return float(cand[k]), float(vals[k])


class QcqpSolver:
    """方向约束的闭式解。"""

    name = "qcqp"
    description = "方向约束：四次方程闭式求 psi，t 由线性方程消去"

    def estimate(self, *, camera_k, armor_size, image_points_2d,
                 rot_camera_to_abs_camera, mount_pitch=0.0):
        img = np.asarray(image_points_2d, float).reshape(-1, 2)
        if img.shape[0] != 4:
            return None, None
        M = np.asarray(rot_camera_to_abs_camera, float).reshape(3, 3)

        obs = flu_normalized(img, camera_k)                    # (0.1)
        X, Y, Z, q_rot, v_rot = model_basis(armor_size, mount_pitch)
        normals_abs = (M @ line_normals(obs).T).T              # (4.5)
        a, c, w = unit_circle_terms(normals_abs, v_rot)
        psi, _f = solve_unit_circle(a, c, w)
        if psi is None:
            return None, None

        t_system = build_t_system(obs, M, X, Y, Z)
        # (6.4)：theta_0 != 0 时两支本来就分得开，theta_0 = 0 时靠带标签点判
        cand = np.array([psi, psi + math.pi])
        rep = costs_at(cand, obs, M, t_system, X, Y, Z, kind="ml")
        if not np.any(np.isfinite(rep)):
            return None, None
        psi = float(cand[int(np.argmin(rep))])

        t = solve_t(psi, t_system)
        t = refine_t(psi, t, obs, M, q_rot)
        if t is None or not np.all(np.isfinite(t)):
            return None, None
        return t, np.array([0.0, float(mount_pitch), psi])
