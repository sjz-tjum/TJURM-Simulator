# core/algorithms/math/quintic.py
"""五次多项式轨迹：起终点 (p,v,a) 与总时间 T -> 唯一五次多项式。"""
import numpy as np

__all__ = ["Quintic"]


class Quintic:
    """给定起终点状态 (p, v, a) 和总时间 T，解五次多项式。"""

    def __init__(self, p0, v0, a0, p1, v1, a1, T):
        self.T = float(T)
        self.coeffs = self._compute_coeffs(p0, v0, a0, p1, v1, a1, self.T)

    @staticmethod
    def _compute_coeffs(p0, v0, a0, p1, v1, a1, T):
        A = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, T, T ** 2, T ** 3, T ** 4, T ** 5],
            [0, 1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
            [0, 0, 2, 6 * T, 12 * T ** 2, 20 * T ** 3],
        ])
        b = np.array([p0, v0, a0, p1, v1, a1])
        return np.linalg.solve(A, b)

    def evaluate(self, t):
        if t < 0:
            t = 0.0
        elif t > self.T:
            t = self.T
        c = self.coeffs
        pos = c[0] + c[1] * t + c[2] * t ** 2 + c[3] * t ** 3 + c[4] * t ** 4 + c[5] * t ** 5
        vel = c[1] + 2 * c[2] * t + 3 * c[3] * t ** 2 + 4 * c[4] * t ** 3 + 5 * c[5] * t ** 4
        acc = 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t ** 2 + 20 * c[5] * t ** 3
        return pos, vel, acc