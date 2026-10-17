# core/algorithms/filters/kalman.py
"""线性卡尔曼滤波器。"""
from typing import Callable, Optional

import numpy as np

__all__ = ["KalmanFilter"]


class KalmanFilter:
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
            raise ValueError("KalmanFilter: 必须给出 state_dim / x0 / P0 之一")

        if P0 is not None:
            self.P = np.asarray(P0, float).copy()
            if self.P.shape != (self.state_dim, self.state_dim):
                raise ValueError("KalmanFilter: P0 与 x0 维度不一致")
        else:
            self.P = np.eye(self.state_dim)

        self.x_add_func = x_add_func if x_add_func is not None else (lambda a, b: a + b)
        self.y = None
        self.S = None

    def predict(self, F, Q):
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z, H, R, z_sub_func=None):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.pinv(S)

        if z_sub_func is not None:
            y = z_sub_func(z, H @ self.x)
        else:
            y = z - H @ self.x

        self.x = self.x_add_func(self.x, K @ y)
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

        self.y = y
        self.S = S
        return self.x

    def restart(self):
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim)