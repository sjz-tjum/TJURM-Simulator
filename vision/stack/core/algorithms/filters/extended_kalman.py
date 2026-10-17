# core/algorithms/filters/extended_kalman.py
"""扩展卡尔曼滤波器。"""
from collections import deque
from typing import Callable, Optional

import numpy as np

__all__ = ["ExtendedKalmanFilter"]


class ExtendedKalmanFilter:
    def __init__(self,
                 x0=None,
                 P0=None,
                 x_add_func: Optional[Callable] = None,
                 state_dim: Optional[int] = None):
        self.state_dim = None

        if state_dim is not None:
            self.state_dim = int(state_dim)
            self.x = np.zeros(self.state_dim)

        if x0 is not None:
            self.state_dim = len(x0)
            self.x = np.asarray(x0, float).copy().flatten()
        elif P0 is not None:
            self.state_dim = P0.shape[0]
            self.x = np.zeros(self.state_dim)

        if self.state_dim is None:
            raise ValueError("ExtendedKalmanFilter: 必须给出 state_dim / x0 / P0 之一")

        if P0 is not None:
            self.P = np.asarray(P0, float).copy()
            if self.P.shape != (self.state_dim, self.state_dim):
                raise ValueError("ExtendedKalmanFilter: P0 与 x0 维度不一致")
        else:
            self.P = np.eye(self.state_dim)

        self.x_add_func = x_add_func if x_add_func is not None else (lambda a, b: a + b)

        self.window_size = 50
        self.recent_nis_failures = deque(maxlen=self.window_size)
        self.recent_nees_values = deque(maxlen=self.window_size)
        self.recent_nees_failures = deque(maxlen=self.window_size)

        self.nis_threshold = 9.488
        self.nees_threshold = 19.675
        self.nis = None
        self.nees = None

    def set_chi2_thresholds(self, nis_dof: int, nees_dof: Optional[int] = None,
                            conf: float = 0.95):
        def chi2_ppf(dof, p):
            z = _norm_ppf(p)
            return dof * (1.0 - 2.0 / (9.0 * dof)
                          + z * np.sqrt(2.0 / (9.0 * dof))) ** 3

        self.nis_threshold = float(chi2_ppf(nis_dof, conf))
        if nees_dof is None:
            nees_dof = self.state_dim
        self.nees_threshold = float(chi2_ppf(nees_dof, conf))
        return self.nis_threshold, self.nees_threshold

    def restart(self):
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim)

    def predict(self, F=None, Q=None, f_func=None, F_jacobian=None,
                f_delta_func=None):
        if Q is None:
            raise ValueError("predict 需要 Q")

        # F 是状态转移在 x_k 处的雅可比，必须在状态推进之前求值
        if f_delta_func is not None:
            if F_jacobian is not None:
                F = F_jacobian(self.x)
            delta = np.asarray(f_delta_func(self.x), float)
            self.x = self.x_add_func(self.x, delta)
            if F is None:
                raise ValueError("f_delta_func 模式需要 F_jacobian 或 F")
        elif f_func is not None:
            if F_jacobian is not None:
                F = F_jacobian(self.x)
            x_next = np.asarray(f_func(self.x), float)
            delta = x_next - self.x
            self.x = self.x_add_func(self.x, delta)
            if F is None:
                raise ValueError("f_func 模式需要 F_jacobian 或 F")
        else:
            if F is None:
                raise ValueError("线性模式需要 F")
            self.x = self.x_add_func(self.x, F @ self.x - self.x)

        self.P = F @ self.P @ F.T + Q
        return self.x

    def update(self, z, H=None, R=None, z_sub_func=None, h_func=None,
               H_jacob=None, x_true=None):
        if R is None:
            raise ValueError("update 需要 R")

        if h_func is not None:
            z_pred = np.asarray(h_func(self.x), float)
            if H_jacob is not None:
                H = H_jacob(self.x)
            if H is None:
                raise ValueError("h_func 模式需要 H_jacob 或 H")
        else:
            if H is None:
                raise ValueError("线性模式需要 H")
            z_pred = H @ self.x

        if z_sub_func is not None:
            y = z_sub_func(np.asarray(z, float), z_pred)
        else:
            y = np.asarray(z, float) - z_pred

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.pinv(S)

        self.x = self.x_add_func(self.x, K @ y)
        I = np.eye(self.state_dim)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        self._eval(y, S, x_true)
        return self.x

    def _eval(self, y, S, x_true=None):
        y = np.asarray(y, float).reshape(-1)
        nis = float(y @ np.linalg.pinv(S) @ y)
        self.nis = nis
        self.recent_nis_failures.append(bool(nis > self.nis_threshold))

        if x_true is not None:
            err = np.asarray(x_true, float).reshape(-1) - self.x
            try:
                nees = float(err @ np.linalg.inv(self.P) @ err)
            except np.linalg.LinAlgError:
                nees = float(err @ np.linalg.pinv(self.P) @ err)
            self.nees = nees
            self.recent_nees_values.append(nees)
            self.recent_nees_failures.append(bool(nees > self.nees_threshold))

    def get_nis_fail_rate(self):
        if len(self.recent_nis_failures) == 0:
            return 0.0
        return sum(self.recent_nis_failures) / len(self.recent_nis_failures)

    def get_nees_fail_rate(self):
        if len(self.recent_nees_failures) == 0:
            return 0.0
        return sum(self.recent_nees_failures) / len(self.recent_nees_failures)

    def get_avg_nees(self):
        if len(self.recent_nees_values) == 0:
            return 0.0
        return float(np.mean(self.recent_nees_values))


def _norm_ppf(p: float) -> float:
    """标准正态分布分位数的有理逼近（Acklam）。"""
    if not (0.0 < p < 1.0):
        raise ValueError("p 必须在 (0,1)")
    a = [-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
         1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00]
    b = [-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
         6.680131188771972e+01, -1.328068155288572e+01]
    c = [-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
         -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00]
    d = [7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
         3.754408661907416e+00]
    plow, phigh = 0.02425, 1 - 0.02425
    if p < plow:
        q = np.sqrt(-2 * np.log(p))
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)
    if p > phigh:
        q = np.sqrt(-2 * np.log(1 - p))
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
                ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)
    q = p - 0.5
    r = q * q
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / \
           (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1)