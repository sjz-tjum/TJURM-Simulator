# core/algorithms/filters/unscented_kalman.py
"""无迹卡尔曼滤波器。"""
from typing import Callable, Optional

import numpy as np
import scipy.linalg as spla

__all__ = ["UnscentedKalmanFilter"]


class UnscentedKalmanFilter:
    def __init__(self,
                 state_dim: Optional[int] = None,
                 x0=None,
                 P0=None,
                 alpha: float = 0.001,
                 beta: float = 2.0,
                 kappa: float = 0.0,
                 x_add_func: Optional[Callable] = None,
                 x_sub_func: Optional[Callable] = None,
                 x_mean_func: Optional[Callable] = None,
                 z_sub_func: Optional[Callable] = None,
                 z_mean_func: Optional[Callable] = None):
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
            raise ValueError("UnscentedKalmanFilter: 必须给出 state_dim / x0 / P0 之一")

        if P0 is not None:
            self.P = np.asarray(P0, float).copy()
            if self.P.shape != (self.state_dim, self.state_dim):
                raise ValueError("UnscentedKalmanFilter: P0 与 x0 维度不一致")
        else:
            self.P = np.eye(self.state_dim)

        self.x_add_func = x_add_func if x_add_func is not None else np.add
        self.x_sub_func = x_sub_func if x_sub_func is not None else np.subtract
        self.x_mean_func = (x_mean_func if x_mean_func is not None
                            else (lambda a, b: np.dot(b, a)))
        self.z_sub_func = z_sub_func if z_sub_func is not None else np.subtract
        self.z_mean_func = (z_mean_func if z_mean_func is not None
                            else (lambda a, b: np.dot(b, a)))

        self.alpha = float(alpha)
        self.beta = float(beta)
        self.kappa = float(kappa)
        self.lambda_plus_n = 0.0

        self.mean_weights = None
        self.cov_weights = None
        self.y = None
        self.S = None

        self._get_weights()

    def restart(self):
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim)

    def _get_weights(self):
        n = self.state_dim
        lambda_ = self.alpha ** 2 * (n + self.kappa) - n

        self.mean_weights = np.full(2 * n + 1, 0.5 / (n + lambda_))
        self.mean_weights[0] = lambda_ / (n + lambda_)

        self.cov_weights = np.full(2 * n + 1, 0.5 / (n + lambda_))
        self.cov_weights[0] = lambda_ / (n + lambda_) + (1 - self.alpha ** 2 + self.beta)

        self.lambda_plus_n = lambda_ + n

    def _get_sigma_points(self, x, P):
        n = self.state_dim
        sigma_points = np.zeros((2 * n + 1, n))

        P = (P + P.T) / 2.0
        try:
            U = spla.cholesky(self.lambda_plus_n * P)
        except np.linalg.LinAlgError:
            u, s, _vh = np.linalg.svd(P)
            U = (np.sqrt(s * self.lambda_plus_n) * u).T

        sigma_points[0] = x
        for k in range(n):
            sigma_points[k + 1] = self.x_add_func(x, U[k])
            sigma_points[n + k + 1] = self.x_sub_func(x, U[k])

        return sigma_points

    def _unscented_transform(self, sigma_points, mean_weights, cov_weights,
                             cov_noise, mean_func, sub_func):
        mean = mean_func(sigma_points, mean_weights)

        n = len(mean)
        cov = np.zeros((n, n))
        for k in range(len(sigma_points)):
            diff = sub_func(sigma_points[k], mean)
            cov += cov_weights[k] * np.outer(diff, diff)
        cov += cov_noise

        return mean, cov

    def predict(self, Q, f_func):
        sigma_points = self._get_sigma_points(self.x, self.P)
        sigma_points_f = np.array([f_func(s) for s in sigma_points])

        self.x, self.P = self._unscented_transform(
            sigma_points_f,
            self.mean_weights,
            self.cov_weights,
            Q,
            self.x_mean_func,
            self.x_sub_func)

    def update(self, z, R, h_func, x_true=None):
        sigma_points = self._get_sigma_points(self.x, self.P)
        sigma_points_h = np.array([h_func(s) for s in sigma_points])

        zp, S = self._unscented_transform(
            sigma_points_h,
            self.mean_weights,
            self.cov_weights,
            R,
            self.z_mean_func,
            self.z_sub_func)

        T = np.zeros((self.state_dim, len(z)))
        for i in range(2 * self.state_dim + 1):
            diff_x = self.x_sub_func(sigma_points[i], self.x)
            diff_z = self.z_sub_func(sigma_points_h[i], zp)
            T += self.cov_weights[i] * np.outer(diff_x, diff_z)

        K = T @ np.linalg.pinv(S)
        y = self.z_sub_func(z, zp)
        self.x = self.x_add_func(self.x, K @ y)
        self.P = self.P - K @ T.T

        self.P = (self.P + self.P.T) / 2.0
        self.P += np.eye(self.state_dim) * 1e-6

        self.y = y
        self.S = S