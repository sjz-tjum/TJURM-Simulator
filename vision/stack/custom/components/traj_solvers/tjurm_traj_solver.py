# custom/components/traj_solvers/tjurm_traj_solver.py
"""弹道解算：给定初速和目标位置，解俯仰角、飞行时间、云台角度。

两层结构：
    TrajectorySolver   —— 底层：给定 (v0, target_pos, pitch_guess)，
                          数值迭代解 (fly_time, pitch)。带空气阻力。
    TjurmTrajSolver    —— 上层：从"瞄准点 + SelfState"出发，
                          调底层解角度，输出 (yaw, pitch, fly_time)。

drag_k_from_bullet 是"弹丸物理量 -> 弹道阻力系数 k"的换算——放在这里，
弹丸参数和 k 的推导在同一文件，不会和 core/entities/bullet.py 漂开。
"""
from __future__ import annotations

import math

import numpy as np

from config import cfg

__all__ = ["TrajectorySolver", "TjurmTrajSolver", "drag_k_from_bullet"]

PITCH_GUESS_INIT = 0.1
MIN_HORIZONTAL = 1e-3


# ------------------------------------------------------------------ 物理量换算

def drag_k_from_bullet(mass, caliber, drag_cd, air_density=1.2):
    """由弹丸的物理参数推出弹道模型的阻力系数 k。

    弹丸模型（core/entities/bullet.py）是  |a_drag| = 0.5·rho·A·Cd·v² / m ，
    本模块的模型是  |a_drag| = k·v²  ，所以

        k = 0.5 · rho · A · Cd / m ,   A = pi·(caliber/2)²

    放在这里算而不是写进 YAML：同一个 k 写两处，迟早会和弹丸那边漂开，
    而漂开的表现是"打远处总差一点"——很难往回追到配置。
    """
    area = math.pi * (float(caliber) / 2.0) ** 2
    return 0.5 * float(air_density) * area * float(drag_cd) / float(mass)


# ------------------------------------------------------------------ 底层

class TrajectorySolver:
    """带空气阻力的弹道数值解（两点边值问题的打靶法）。

    给定初速 v0、目标相对位置 target_pos、初始俯仰猜测 pitch_guess，
    迭代解出 (fly_time, pitch)。不收敛返回 (None, None)。

    上层 TjurmTrajSolver 负责准备参数；本类只做数学。
    """

    def __init__(self, g=9.8, k=0.001, N=100):
        self.g = float(g)
        self.k = float(k)
        self.N = int(N)
        self._mid = np.arange(self.N) + 0.5

    def solve(self, v0, target_pos, pitch_guess, max_iter=20, tol=1e-7):
        x0 = np.sqrt(target_pos[0] ** 2 + target_pos[1] ** 2)
        y0 = target_pos[2]
        r1 = np.tan(pitch_guess)
        t_est = (x0 / (v0 * np.cos(pitch_guess))
                 if abs(np.cos(pitch_guess)) > 1e-6 else 0.0)
        r0 = ((v0 * np.sin(pitch_guess) - self.g * t_est) / (v0 * np.cos(pitch_guess))
              if abs(np.cos(pitch_guess)) > 1e-6 else 0.0)
        R = np.array([r0, r1])
        for _it in range(max_iter):
            r0, r1 = R[0], R[1]
            c = (self.g * (1 + r1 ** 2) / (self.k * v0 ** 2)
                 + r1 * np.sqrt(1 + r1 ** 2)
                 + np.log(r1 + np.sqrt(1 + r1 ** 2)))
            X, Y, _ = self._integrals(r0, r1, c)
            D = np.array([x0 - X, y0 - Y])
            if np.linalg.norm(D) < tol:
                pitch = np.arctan(r1)
                _, _, fly_time = self._integrals(r0, r1, c)
                return fly_time, pitch

            eps = 1e-6
            Xe, Ye, _ = self._integrals(r0 + eps, r1, c)
            X_r0 = (Xe - X) / eps
            Y_r0 = (Ye - Y) / eps
            r1_eps = r1 + eps
            c1 = (self.g * (1 + r1_eps ** 2) / (self.k * v0 ** 2)
                  + r1_eps * np.sqrt(1 + r1_eps ** 2)
                  + np.log(r1_eps + np.sqrt(1 + r1_eps ** 2)))
            X1, Y1, _ = self._integrals(r0, r1_eps, c1)
            X_r1 = (X1 - X) / eps
            Y_r1 = (Y1 - Y) / eps
            J = np.array([[X_r0, X_r1], [Y_r0, Y_r1]])

            cond = np.linalg.cond(J)
            if cond > 1e12:
                dR = np.linalg.pinv(J) @ D
            else:
                try:
                    dR = np.linalg.solve(J, D)
                except np.linalg.LinAlgError:
                    dR = np.linalg.pinv(J) @ D

            if np.linalg.norm(dR) > 1.0:
                dR = dR / np.linalg.norm(dR) * 1.0
            R += dR
            if abs(R[1]) > 10:
                R[1] = np.clip(R[1], -10, 10)

        return None, None

    def _integrals(self, r0, r1, c):
        dr = (r0 - r1) / self.N
        r = r1 + self._mid * dr
        denom = self.k * (c - r * np.sqrt(1.0 + r * r) - np.arcsinh(r))
        return ((-1.0 / denom).sum() * dr,
                (-r / denom).sum() * dr,
                (-1.0 / np.sqrt(self.g * denom)).sum() * dr)


# ------------------------------------------------------------------ 上层

class TjurmTrajSolver:
    """从"瞄准点 + SelfState"到"云台角度 + 飞行时间"。

    职责只有"解一颗弹"——不含开火判据、不含伺服。
    """

    name = "tjurm"

    def __init__(self, caliber="17mm", *, frame=None):
        self.frame = str(cfg.sim.frame if frame is None else frame).upper()
        if self.frame not in ("G", "W"):
            raise ValueError(f"Sim.frame 必须是 'G' 或 'W'，实得 {self.frame!r}")
        spec = cfg.bullet._17mm
        self.speed = float(spec.speed)
        self.traj_solver = TrajectorySolver(
            g=float(cfg.sim.gravity),
            k=drag_k_from_bullet(spec.mass, spec.caliber, spec.drag_cd,
                                 float(cfg.sim.air_density)))
        self._last_pitch = PITCH_GUESS_INIT

    def reset(self) -> None:
        self._last_pitch = PITCH_GUESS_INIT

    def solve_aim(self, aim_point, self_state):
        """从瞄准点解 (yaw, pitch, fly_time)；无解返回 None。"""
        aim = np.asarray(aim_point, float)
        muzzle = np.asarray(self_state.muzzle.pos, float)
        if self.frame == "G":
            muzzle = muzzle - np.asarray(self_state.gimbal.pos, float)
        d = aim - muzzle
        if math.hypot(float(d[0]), float(d[1])) < MIN_HORIZONTAL:
            return None

        yaw = math.atan2(float(d[1]), float(d[0]))
        fly, pitch_up = self.traj_solver.solve(self.speed, d, self._last_pitch)
        if pitch_up is None:
            return None
        self._last_pitch = float(pitch_up)
        return yaw, float(pitch_up), float(fly)