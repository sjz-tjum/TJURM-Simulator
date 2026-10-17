# custom/components/armor_solvers/diff.py
"""差分代价单板解算（docs/design/diff_derivation.md）。

观测留在 C1（原始像素），模型放在 C'（绝对方向），两者靠 R^{C1}_{C'} 换架 (2.4)；
平移 t 由 (4.2)(4.4) 的线性方程整帧消去，剩下的只有 psi 一维：

    (3.3)  J_df = sum |Delta e_j|^2，e_j 是四个角点的归一化残差
    (5.1)  前方性闸门查的是换到 C1 之后的第 0 分量
    (6.1)  全周网格 + 逐级细化，分辨率 2*pi/N / (m-1)^k

psi 与 psi+pi 两支都在搜索区间里，由代价自己分辨（theta_0=0 时方向型判别信号
消失，但带标签点的残差还在）。本模块同时给出 qcqp.py 复用的观测、投影、消去
工具 —— 同一套理论，不另设基类、也不另开文件。
"""
from __future__ import annotations

import math

import numpy as np

from core.algorithms.math.transform import (
    pixel_to_normalized_RDF, vec2d_RDF2FLU,
)
from custom.components.armor_solvers import plate_size

EDGES = ((0, 1), (1, 2), (2, 3), (3, 0))


# --------------------------------------------------------------------- 观测

def flu_normalized(image_points_2d, camera_k):
    """原始像素 -> C1 的 FLU 排列归一化坐标 (upsilon, zeta)。

    像素先归一到 RDF（transform 的统一口径），再用 vec2d_RDF2FLU 换到 FLU。
    轴序公式只在 transform 一处，本函数不重写。
    """
    n = pixel_to_normalized_RDF(image_points_2d, K=camera_k)
    return vec2d_RDF2FLU(n).reshape(-1, 2)


def line_normals(obs):
    """(4.2)(4.3)：四条边的反投影平面法向，表达在 C1 里。"""
    n = np.empty((4, 3), float)
    for k, (a, b) in enumerate(EDGES):
        ua, za = float(obs[a][0]), float(obs[a][1])
        ub, zb = float(obs[b][0]), float(obs[b][1])
        n[k] = (ub * za - ua * zb, zb - za, ua - ub)
    return n


# --------------------------------------------------------------------- 模型

def model_basis(armor_size, mount_pitch):
    """(2.3) 把模型写成对 x = (cos psi, sin psi) 的仿射式。"""
    w, h = plate_size(armor_size)
    q = np.array([
        [0.0, +w / 2.0, +h / 2.0],
        [0.0, -w / 2.0, +h / 2.0],
        [0.0, -w / 2.0, -h / 2.0],
        [0.0, +w / 2.0, -h / 2.0],
    ])
    v = np.array([
        [0.0, -w, 0.0],
        [0.0, 0.0, -h],
        [0.0, +w, 0.0],
        [0.0, 0.0, +h],
    ])
    th = float(mount_pitch)
    ct, st = math.cos(th), math.sin(th)
    Ry = np.array([[ct, 0.0, st],
                   [0.0, 1.0, 0.0],
                   [-st, 0.0, ct]])
    q_rot = (Ry @ q.T).T
    v_rot = (Ry @ v.T).T
    X = np.column_stack([q_rot[:, 0], q_rot[:, 1], np.zeros(4)])
    Y = np.column_stack([-q_rot[:, 1], q_rot[:, 0], np.zeros(4)])
    Z = np.column_stack([np.zeros(4), np.zeros(4), q_rot[:, 2]])
    return X, Y, Z, q_rot, v_rot


def build_t_system(obs, rot_camera_to_abs_camera, X, Y, Z):
    """(4.2)(4.3)(4.4)：A t = b(psi)，b 对 x = (cos psi, sin psi) 线性。"""
    Rt = np.asarray(rot_camera_to_abs_camera, float).reshape(3, 3).T
    r0, r1, r2 = Rt[0], Rt[1], Rt[2]
    A = np.zeros((8, 3), float)
    Bx = np.zeros(8, float)
    Bs = np.zeros(8, float)
    Bc = np.zeros(8, float)
    for i in range(4):
        ups, zet = float(obs[i][0]), float(obs[i][1])
        for j, row in enumerate((ups * r0 - r1, zet * r0 - r2)):
            k = 2 * i + j
            A[k] = row
            Bx[k] = -float(row @ X[i])
            Bs[k] = -float(row @ Y[i])
            Bc[k] = -float(row @ Z[i])
    P = np.linalg.pinv(A)
    return P @ np.column_stack([Bx, Bs]), P @ Bc, A


def solve_t(psi, t_system):
    """(4.4) 在给定 psi 上的代数解。"""
    t_lin, t_const, _A = t_system
    return t_lin @ np.array([math.cos(psi), math.sin(psi)]) + t_const


def predict_norm(x, rot_camera_to_abs_camera, t_system, X, Y, Z):
    """候选 x = (c, s)，返回 C1 里的预测归一化坐标 (2.4)(2.5) 与闸门 (5.1)。"""
    x = np.asarray(x, float).reshape(-1, 2)
    t_lin, t_const, _A = t_system
    model = (x[:, 0:1, None] * X[None, :, :]
             + x[:, 1:2, None] * Y[None, :, :] + Z[None, :, :])
    t = x @ t_lin.T + t_const
    P_abs = model + t[:, None, :]
    P_c1 = P_abs @ np.asarray(rot_camera_to_abs_camera, float).reshape(3, 3)
    ok = np.all(P_c1[:, :, 0] > 1e-9, axis=1)
    depth = np.where(ok[:, None], P_c1[:, :, 0], 1.0)
    pred = np.stack([P_c1[:, :, 1] / depth, P_c1[:, :, 2] / depth], axis=-1)
    return pred, ok


# --------------------------------------------------------------------- 代价

def cost_ml(residual):
    """(3.2) 点残差平方和。"""
    return np.einsum("kij,kij->k", residual, residual)


def cost_df(residual):
    """(3.3) 相邻角点差分残差平方和（对整体平移不敏感）。"""
    delta = residual[:, (1, 2, 3, 0), :] - residual
    return np.einsum("kij,kij->k", delta, delta)


def costs_at(psis, obs, rot_camera_to_abs_camera, t_system, X, Y, Z, kind="df"):
    """一组候选 psi 的代价 (3.3)；闸门 (5.1) 不过的候选记为 inf。"""
    psis = np.asarray(psis, float).reshape(-1)
    x = np.column_stack([np.cos(psis), np.sin(psis)])
    pred, ok = predict_norm(x, rot_camera_to_abs_camera, t_system, X, Y, Z)
    residual = np.asarray(obs, float)[None, :, :] - pred
    cost = cost_df(residual) if kind == "df" else cost_ml(residual)
    return np.where(ok, cost, np.inf)


def search_psi(obs, rot_camera_to_abs_camera, t_system, X, Y, Z,
               kind="df", first=180, rounds=4, points=16):
    """(6.1) 全周网格 + 逐级细化；全被闸门挡掉时返回 (None, inf)。"""
    psis = np.linspace(-np.pi, np.pi, int(first), endpoint=False)
    costs = costs_at(psis, obs, rot_camera_to_abs_camera, t_system, X, Y, Z, kind)
    k = int(np.argmin(costs))
    best_psi, best_cost = float(psis[k]), float(costs[k])
    if not np.isfinite(best_cost):
        return None, np.inf
    step = 2.0 * np.pi / float(first)
    for _ in range(int(rounds)):
        cand = best_psi + np.linspace(-step, step, int(points))
        costs = costs_at(cand, obs, rot_camera_to_abs_camera, t_system, X, Y, Z, kind)
        k = int(np.argmin(costs))
        if not np.isfinite(costs[k]):
            return None, np.inf
        best_psi, best_cost = float(cand[k]), float(costs[k])
        step = 2.0 * step / float(points - 1)
    return best_psi, best_cost


def refine_t(psi, t, obs, rot_camera_to_abs_camera, q_rot, iters=8):
    """对重投影误差做 GN 抛光；失败返回 None。"""
    M = np.asarray(rot_camera_to_abs_camera, float).reshape(3, 3)
    ct, st = math.cos(float(psi)), math.sin(float(psi))
    Rz = np.array([[ct, -st, 0.0], [st, ct, 0.0], [0.0, 0.0, 1.0]])
    model = (Rz @ np.asarray(q_rot, float).T).T
    target = np.asarray(obs, float).reshape(-1, 2)
    t = np.asarray(t, float).reshape(3).copy()

    def residuals(tt):
        P = model + tt
        p = P @ M
        if np.any(p[:, 0] <= 1e-9):
            return None, None
        pred = np.column_stack([p[:, 1] / p[:, 0], p[:, 2] / p[:, 0]])
        r = (pred - target).reshape(-1)
        J = np.zeros((8, 3), float)
        for i in range(4):
            x0, x1, x2 = p[i]
            dproj = np.array([[-x1 / (x0 * x0), 1.0 / x0, 0.0],
                              [-x2 / (x0 * x0), 0.0, 1.0 / x0]])
            J[2 * i:2 * i + 2, :] = dproj @ M
        return r, J

    r, J = residuals(t)
    if r is None:
        return None
    cost = float(r @ r)
    lam = 1e-9
    for _ in range(int(iters)):
        if cost <= 1e-24:
            break
        moved = False
        for _try in range(6):
            try:
                delta = np.linalg.solve(J.T @ J + lam * np.eye(3), -J.T @ r)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            if not np.all(np.isfinite(delta)):
                lam *= 10.0
                continue
            nr, nJ = residuals(t + delta)
            if nr is not None and float(nr @ nr) < cost:
                t = t + delta
                r, J = nr, nJ
                cost = float(nr @ nr)
                lam = max(lam * 0.3, 1e-12)
                moved = True
                break
            lam *= 10.0
        if not moved:
            break
    return t


# --------------------------------------------------------------------- 实现

class DiffSolver:
    """差分代价：psi 一维搜索，t 由线性方程消去。"""

    name = "diff"
    description = "差分代价：全周网格 + 逐级细化，t 由线性方程消去"

    def __init__(self, first=180, rounds=6, points=16, kind="df"):
        # 默认分辨率：初始网格 2pi/180，每轮缩到 2/15，6 轮后 ≈ 7e-8 rad
        self.first = int(first)
        self.rounds = int(rounds)
        self.points = int(points)
        self.kind = str(kind)

    def estimate(self, *, camera_k, armor_size, image_points_2d,
                 rot_camera_to_abs_camera, mount_pitch=0.0):
        img = np.asarray(image_points_2d, float).reshape(-1, 2)
        if img.shape[0] != 4:
            return None, None
        M = np.asarray(rot_camera_to_abs_camera, float).reshape(3, 3)

        obs = flu_normalized(img, camera_k)
        X, Y, Z, q_rot, _v_rot = model_basis(armor_size, mount_pitch)
        t_system = build_t_system(obs, M, X, Y, Z)

        psi, cost = search_psi(obs, M, t_system, X, Y, Z,
                               kind=self.kind, first=self.first,
                               rounds=self.rounds, points=self.points)
        if psi is None or not np.isfinite(cost):
            return None, None

        t = solve_t(psi, t_system)
        t = refine_t(psi, t, obs, M, q_rot)
        if t is None or not np.all(np.isfinite(t)):
            return None, None
        return t, np.array([0.0, float(mount_pitch), psi])