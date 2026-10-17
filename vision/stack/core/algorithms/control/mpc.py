# core/algorithms/control/mpc.py
"""滚动时域 MPC：纯 NumPy 投影梯度 QP，无 osqp 依赖。"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

__all__ = ["MPCConfig", "ProjectedGradientQP", "LinearGimbalMPC"]


@dataclass
class MPCConfig:
    dt: float = 0.01
    horizon: int = 10
    q_theta: float = 100.0
    q_omega: float = 10.0
    q_alpha: float = 0.05
    r_u: float = 0.01
    alpha_max: float = 35.0
    theta_min: float = -1e9
    theta_max: float = 1e9
    iters: int = 400
    tol: float = 1e-10
    warm_start: bool = True

    def __post_init__(self):
        if self.dt <= 0:
            raise ValueError("dt 必须 > 0")
        if self.horizon < 1:
            raise ValueError("horizon 必须 >= 1")
        if self.alpha_max <= 0:
            raise ValueError("alpha_max 必须 > 0")


class ProjectedGradientQP:
    """盒约束凸 QP 求解器：主动集法 + 投影梯度兜底。"""

    def __init__(self, H, l, h):
        H = np.asarray(H, float)
        self.n = H.shape[0]
        self.H = 0.5 * (H + H.T)
        self.l = np.asarray(l, float).copy()
        self.h = np.asarray(h, float).copy()
        if np.any(self.l > self.h):
            raise ValueError("存在 l > h 的约束")
        spec = np.linalg.eigvalsh(self.H)
        self._lambda_max = max(float(spec[-1]), 1e-12)
        self._lambda_min = max(float(spec[0]), 0.0)
        self._step = 1.0 / self._lambda_max
        self._reg = 1e-12 * max(1.0, self._lambda_max)

    def objective(self, u, g):
        return 0.5 * float(u @ self.H @ u) + float(g @ u)

    def _solve_active_set(self, g, itmax=200):
        n = self.n
        u = np.zeros(n)
        try:
            u_unc = np.linalg.solve(self.H + self._reg * np.eye(n), -g)
        except np.linalg.LinAlgError:
            u_unc = np.zeros(n)
        if np.all(u_unc >= self.l - 1e-12) and np.all(u_unc <= self.h + 1e-12):
            return np.clip(u_unc, self.l, self.h), 0, True

        u = np.clip(u_unc, self.l, self.h)
        for it in range(itmax):
            active = (u <= self.l + 1e-12) | (u >= self.h - 1e-12)
            free = ~active
            if not np.any(free):
                return u, it, True

            u_new = u.copy()
            Hf = self.H[np.ix_(free, free)]
            gf = g[free].copy()
            if np.any(active):
                gf = gf + self.H[np.ix_(free, active)] @ u[active]
            m = Hf.shape[0]
            try:
                uf = np.linalg.solve(Hf + self._reg * np.eye(m), -gf)
            except np.linalg.LinAlgError:
                uf = np.linalg.lstsq(Hf + self._reg * np.eye(m), -gf, rcond=None)[0]

            viol_hi = uf > self.h[free] + 1e-12
            viol_lo = uf < self.l[free] - 1e-12
            if not (np.any(viol_hi) or np.any(viol_lo)):
                u_new[free] = uf
                u = np.clip(u_new, self.l, self.h)
                return u, it, True

            uf_clipped = np.clip(uf, self.l[free], self.h[free])
            u_new[free] = uf_clipped
            prev = u.copy()
            u = u_new
            if np.allclose(prev, u, atol=1e-15):
                return u, it, True
        return u, itmax, False

    def solve(self, g, u0=None, iters=400, tol=1e-10, return_history=False):
        g = np.asarray(g, float)
        u, _n_it, ok = self._solve_active_set(g)
        if not ok:
            u = self._projected_gradient(g, u0, iters, tol)

        if return_history:
            _u2, hist = self._projected_gradient(g, u, iters=60, tol=0.0,
                                                  return_history=True)
            hist = list(hist) + [self.objective(u, g)]
            return u, hist
        return u

    def _projected_gradient(self, g, u0=None, iters=400, tol=1e-10,
                            return_history=False):
        u = (np.zeros(self.n) if u0 is None
             else np.clip(np.asarray(u0, float).copy(), self.l, self.h))
        hist = [] if return_history else None
        f_prev = self.objective(u, g)
        step = self._step
        for _ in range(iters):
            u = u - step * (self.H @ u + g)
            np.clip(u, self.l, self.h, out=u)
            f = self.objective(u, g)
            if return_history:
                hist.append(f)
            if abs(f_prev - f) <= tol * max(1.0, abs(f_prev)):
                f_prev = f
                break
            f_prev = f
        if return_history:
            return u, hist
        return u


class LinearGimbalMPC:
    """单轴双积分器 MPC。"""

    def __init__(self, config=None, **kwargs):
        if config is None:
            config = MPCConfig(**kwargs)
        elif kwargs:
            raise TypeError("同时给 config 与关键字参数有歧义")
        self.cfg = config
        self._u_warm = None
        self._build()

    def _build(self):
        cfg = self.cfg
        N, dt = cfg.horizon, cfg.dt

        Gth = np.zeros((N, N))
        Gw = np.zeros((N, N))
        for i in range(1, N + 1):
            for j in range(i):
                Gth[i - 1, j] = (i - j - 0.5) * dt ** 2
                Gw[i - 1, j] = dt
        self.Gth, self.Gw = Gth, Gw

        H = (cfg.q_theta * (Gth.T @ Gth)
             + cfg.q_omega * (Gw.T @ Gw)
             + cfg.r_u * np.eye(N))
        if cfg.q_alpha > 0 and N > 1:
            D = np.zeros((N - 1, N))
            for i in range(N - 1):
                D[i, i] = -1.0
                D[i, i + 1] = 1.0
            H = H + cfg.q_alpha * (D.T @ D)
        self._H = H

        self._qp = ProjectedGradientQP(
            H,
            np.full(N, -cfg.alpha_max),
            np.full(N, cfg.alpha_max))

    @property
    def QP(self):
        return self._qp

    def solve(self, x0, theta_ref, omega_ref=None,
              ref_is_delta=False, return_plan=False):
        cfg = self.cfg
        N, dt = cfg.horizon, cfg.dt
        x0 = np.asarray(x0, float).reshape(2)
        theta_ref = np.asarray(theta_ref, float).reshape(-1)
        if theta_ref.size < N:
            theta_ref = (np.concatenate([theta_ref, np.full(N - theta_ref.size, theta_ref[-1])])
                         if theta_ref.size else np.zeros(N))
        theta_ref = theta_ref[:N]

        if omega_ref is None:
            omega_ref = np.gradient(theta_ref, dt)
        else:
            omega_ref = np.asarray(omega_ref, float).reshape(-1)[:N]
            if omega_ref.size < N:
                omega_ref = np.concatenate(
                    [omega_ref, np.full(N - omega_ref.size, omega_ref[-1])])

        th0 = x0[0]
        w0 = x0[1]
        rp = th0 + theta_ref if ref_is_delta else theta_ref.copy()

        steps = np.arange(1, N + 1)
        th_base = th0 + steps * dt * w0
        w_base = np.full(N, w0)

        g = (cfg.q_theta * (self.Gth.T @ (th_base - rp))
             + cfg.q_omega * (self.Gw.T @ (w_base - omega_ref)))

        u0 = None
        if cfg.warm_start and self._u_warm is not None:
            u0 = np.concatenate([self._u_warm[1:], self._u_warm[-1:]])
        u, hist = self._qp.solve(g, u0=u0, iters=cfg.iters, tol=cfg.tol,
                                 return_history=True)
        self._u_warm = u

        th_pred = th_base + self.Gth @ u
        w_pred = w_base + self.Gw @ u

        theta_des = float(rp[0])
        omega_des = float(omega_ref[0])
        alpha0 = float(u[0])

        if theta_ref.size and ref_is_delta:
            theta_des = float(th0 + theta_ref[0])

        if not return_plan:
            return theta_des, omega_des, alpha0
        plan = {
            "u": u,
            "theta_pred": th_pred,
            "omega_pred": w_pred,
            "theta_ref": rp,
            "omega_ref": omega_ref,
            "obj": self._qp.objective(u, g),
            "obj_history": hist,
            "iters": len(hist),
        }
        return theta_des, omega_des, alpha0, plan

    def simulate(self, theta0, omega0, ref_fn, n_steps,
                 alpha_prev=None, dt=None):
        dt = dt if dt is not None else self.cfg.dt
        N = self.cfg.horizon
        th, w = float(theta0), float(omega0)
        ts, ths, ws, als, refs, errs = [], [], [], [], [], []
        self._u_warm = None
        for k in range(n_steps):
            t = k * dt
            theta_ref = np.array([ref_fn(t + (i + 1) * dt)[0] for i in range(N)])
            omega_ref = np.array([ref_fn(t + (i + 1) * dt)[1] for i in range(N)])
            _th, _w, alpha, _plan = self.solve([th, w], theta_ref, omega_ref,
                                               return_plan=True)
            th_new = th + w * dt + 0.5 * dt ** 2 * alpha
            w_new = w + dt * alpha
            ts.append(t)
            ths.append(th)
            ws.append(w)
            als.append(alpha)
            refs.append(ref_fn(t)[0])
            errs.append(ref_fn(t)[0] - th)
            th, w = th_new, w_new
        return {
            "t": np.array(ts), "theta": np.array(ths), "omega": np.array(ws),
            "alpha": np.array(als), "theta_ref": np.array(refs),
            "theta_err": np.array(errs),
        }