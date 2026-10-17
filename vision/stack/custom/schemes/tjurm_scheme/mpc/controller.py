# custom/schemes/tjurm_scheme/mpc/controller.py
"""向后兼容层：把旧的 MPCController 接口映射到 core.algorithms.control.mpc。"""
from __future__ import annotations

import numpy as np

from core.algorithms.control.mpc import LinearGimbalMPC, MPCConfig

__all__ = ["MPCController", "MPCConfig", "LinearGimbalMPC"]


class MPCController:
    """旧接口适配器。"""

    def __init__(self, dt, N, q_theta=100., q_dtheta=10., r_alpha=0.01,
                 alpha_max=30, theta_min=-1e9, theta_max=1e9,
                 q_alpha=0.05, iters=400, warm_start=True):
        self.cfg = MPCConfig(
            dt=float(dt), horizon=int(N),
            q_theta=float(q_theta), q_omega=float(q_dtheta),
            q_alpha=float(q_alpha), r_u=float(r_alpha),
            alpha_max=float(alpha_max),
            theta_min=float(theta_min), theta_max=float(theta_max),
            iters=int(iters), warm_start=bool(warm_start))
        self.mpc = LinearGimbalMPC(self.cfg)

    def solve(self, x0, theta_ref, omega_ref=None, theta_min=None,
              theta_max=None):
        """返回 (theta_des, omega_des, alpha_des)，与旧接口一致。"""
        x0 = np.asarray(x0, dtype=float).reshape(2)
        theta_ref = np.asarray(theta_ref, dtype=float).reshape(-1)
        if theta_ref.size == 0:
            return float(x0[0]), float(x0[1]), 0.0

        omega_ref_arr = None
        if omega_ref is not None:
            omega_ref_arr = np.asarray(omega_ref, dtype=float).reshape(-1)

        out = self.mpc.solve(x0, theta_ref, omega_ref_arr, return_plan=True)
        theta_des, omega_des, alpha_des, plan = out

        if theta_min is not None and theta_max is not None:
            lo = np.asarray(theta_min, dtype=float).reshape(-1)
            hi = np.asarray(theta_max, dtype=float).reshape(-1)
            n = min(lo.size, hi.size, plan["theta_pred"].size)
            clipped = np.clip(plan["theta_pred"][:n], lo[:n], hi[:n])
            plan["theta_pred_clipped"] = clipped
        self.last_plan = plan
        return theta_des, omega_des, alpha_des