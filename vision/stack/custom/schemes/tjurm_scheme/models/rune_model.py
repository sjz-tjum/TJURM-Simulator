# custom/schemes/tjurm_scheme/models/rune_model.py
"""能量机关模型：圆心 + 转速的 EKF。"""
from __future__ import annotations

import numpy as np
import math

from core.entities.property.entity_types import EntityType
from core.algorithms.filters.extended_kalman import ExtendedKalmanFilter
from core.algorithms.math.angle import limit_rad, safe_angle_sub
from config import cfg


class _RuneRotation:
    """符的转速滤波器：state = [φ, p, a, w, b]，量测是灯臂参考相位。"""

    def __init__(self, phi0: float = 0.0):
        self.x = np.array([float(phi0), 0.0, 0.0, 1.94, float(cfg.rune.small_speed)])
        self.P = np.diag([1e-3, 4.0, 0.05, 0.01, 0.25])
        self.Q = np.diag([1e-5, 1e-3, 1e-4, 1e-5, 1e-4])
        self.R = 1e-4

    def _step(self, dt: float) -> np.ndarray:
        phi, p, a, w, b = self.x
        wp = w * dt
        dphi = b * dt
        if abs(w) > 1e-6:
            dphi += (a / w) * (np.cos(p) - np.cos(p + wp))
        else:
            dphi += a * np.sin(p) * dt
        return np.array([phi + dphi, p + wp, a, w, b])

    def _jac(self, dt: float) -> np.ndarray:
        phi, p, a, w, b = self.x
        wp = w * dt
        F = np.eye(5)
        F[0, 1] = (a / w) * (np.sin(p + wp) - np.sin(p)) if abs(w) > 1e-6 else a * np.cos(p) * dt
        F[0, 2] = (np.cos(p) - np.cos(p + wp)) / w if abs(w) > 1e-6 else np.sin(p) * dt
        F[0, 4] = dt
        F[1, 3] = dt
        return F

    def _clamp(self):
        self.x[2] = float(np.clip(self.x[2], 0.0, 2.5))
        self.x[3] = float(np.clip(self.x[3], 0.3, 3.5))
        self.x[4] = float(np.clip(self.x[4], -2.5, 2.5))

    def predict(self, dt: float):
        if dt <= 0.0:
            return
        self.predict_cov(dt)
        self.x = self._step(dt)
        self._clamp()

    def predict_cov(self, dt: float):
        F = self._jac(dt)
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, phi_obs: float):
        H = np.zeros((1, 5))
        H[0, 0] = 1.0
        y = np.array([safe_angle_sub(phi_obs, self.x[0])])
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).ravel()
        self.P = (np.eye(5) - K @ H) @ self.P
        self._clamp()

    def phase_delta(self, tau: float) -> float:
        if abs(tau) < 1e-12:
            return 0.0
        return float(self._step(tau)[0] - self.x[0])

    def speed(self, tau: float = 0.0) -> float:
        phi, p, a, w, b = self.x
        return float(b + a * np.sin(p + w * tau))

    @property
    def phase(self) -> float:
        return float(self.x[0])


class RuneModel:
    """state = [cx, cy, cz, vx, vy, vz, φ, ω]，半径取配置值。"""

    def __init__(self, rune_cfg=None):
        rune_cfg = rune_cfg or cfg.rune
        self.cfg = rune_cfg
        self.robot_type = EntityType.Rune
        self.armor_count = int(rune_cfg.leaf_count)
        self.radius = float(rune_cfg.radius)
        self.state_dim = 9
        self.is_init = False
        self.match_id = 0
        self.lit_marks = []
        self.lit_hold = 1.0
        self._now = 0.0
        self.engaged = {}
        self.engaged_hold = 1.5
        self.velocity_tau = 0.3        # 与 OutpostModel 同：固定设备，速度快速衰减

        self.ekf = ExtendedKalmanFilter(state_dim=self.state_dim,
                                        x_add_func=lambda a, b: a + b)
        self.ekf.P = np.eye(self.state_dim)
        self.ekf.P[6, 6] = 0.5
        self.ekf.P[7, 7] = 20.0
        self.ekf.P[8, 8] = 4.0
        self.ekf.set_chi2_thresholds(nis_dof=4, nees_dof=self.state_dim)

        self.q_xy = 0.01
        self.q_z = 0.02
        self.q_v = 0.5
        self.q_phase = 0.0
        self.q_omg = 0.0
        self.q_alpha = 0.0
        self.Q_cont = np.diag([self.q_xy, self.q_xy, self.q_z,
                                self.q_v, self.q_v, self.q_v,
                                self.q_phase, self.q_omg, self.q_alpha])

        self.r_xy = 0.1
        self.r_z = 0.1
        self.r_phase = 0.1
        self.R = np.diag([self.r_xy, self.r_xy, self.r_z, self.r_phase])

        self.rot = _RuneRotation()

    def obs_phase(self, obs) -> float:
        phase = getattr(obs, "phase", float("nan"))
        if phase == phase:
            return float(phase)
        pos = np.asarray(obs.rel_pos, dtype=float)
        x = self.ekf.x
        return float(np.arctan2(pos[2] - x[2], pos[1] - x[1]))

    def phase_at(self, dt: float, phase: float = None) -> float:
        if phase is None:
            return self.rot.phase + self.rot.phase_delta(dt)
        return float(phase) + self.rot.phase_delta(dt)

    def _sync_rotation(self):
        self.ekf.x[6] = self.rot.phase
        self.ekf.x[7] = self.rot.speed()
        self.ekf.x[8] = 0.0
        for idx in (6, 7, 8):
            self.ekf.P[idx, :] = 0.0
            self.ekf.P[:, idx] = 0.0
        self.ekf.P[6, 6] = self.ekf.P[7, 7] = self.ekf.P[8, 8] = 1e-6

    def blade_phase(self, k: int, phase: float = None) -> float:
        phase = self.ekf.x[6] if phase is None else phase
        return phase + k * (2.0 * np.pi) / self.armor_count

    def blade_position(self, k: int, phase: float = None, radius: float = None) -> np.ndarray:
        phase = self.blade_phase(k, phase)
        radius = self.radius if radius is None else radius
        x = self.ekf.x
        return np.array([x[0],
                         x[1] + radius * np.cos(phase),
                         x[2] + radius * np.sin(phase)])

    def get_all_armor_positions_at_time(self, t_future: float):
        x = self.ekf.x
        phase = self.phase_at(t_future)
        center = x[:3] + x[3:6] * t_future
        step = 2.0 * np.pi / self.armor_count
        return [center + self.radius * np.array([0.0, np.cos(phase + k * step),
                                                 np.sin(phase + k * step)])
                for k in range(self.armor_count)]

    def armor_candidates(self, t_future: float):
        x = self.ekf.x
        phase = self.phase_at(t_future)
        center = x[:3] + x[3:6] * t_future
        step = 2.0 * np.pi / self.armor_count
        out = []
        marks = self.lit_phases(t_future)
        keys = self.selectable_keys()
        for k in range(self.armor_count):
            a = phase + k * step
            normal = np.array([0.0, np.cos(a), np.sin(a)])
            is_lit = any(abs(safe_angle_sub(a, p)) < step / 2.0
                         for p, key in zip(marks, self.lit_keys()) if key in keys)
            out.append((center + self.radius * normal, normal, is_lit))
        return out

    def lit_phases(self, t_future: float = 0.0):
        if not self.lit_marks:
            return []
        now = self._now
        return [self.phase_at(t_future + (now - seen), phase)
                for phase, seen, _key in self.lit_marks if now - seen <= self.lit_hold]

    def lit_keys(self):
        return [key for _p, seen, key in self.lit_marks if self._now - seen <= self.lit_hold]

    def mark_engaged(self, key: int, hold: float = None):
        if key is None:
            return
        self.engaged[int(key)] = self._now + (self.engaged_hold if hold is None else float(hold))

    def selectable_keys(self) -> list:
        lit = self.lit_keys()
        live = [k for k in lit if self.engaged.get(k, -1.0) > self._now]
        self.engaged = {k: v for k, v in self.engaged.items() if v > self._now or k in lit}
        rest = [k for k in lit if k not in live]
        return rest if rest else lit

    def predict_observation(self, armor_id: int):
        x = self.ekf.x
        r = self.radius
        phase = self.blade_phase(armor_id)
        c, s = np.cos(phase), np.sin(phase)
        z_pred = np.array([x[0], x[1] + r * c, x[2] + r * s, phase])
        H = np.zeros((4, self.state_dim))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[1, 6] = -r * s
        H[2, 2] = 1.0
        H[2, 6] = r * c
        H[3, 6] = 1.0
        return z_pred, H

    def init_model(self, obs):
        self.is_init = True
        phase = self.obs_phase(obs)
        self.rot = _RuneRotation(phi0=phase)

        pos = np.asarray(obs.rel_pos, dtype=float)
        x0 = np.zeros(self.state_dim)
        x0[0] = pos[0]
        x0[1] = pos[1] - self.radius * np.cos(phase)
        x0[2] = pos[2] - self.radius * np.sin(phase)
        x0[6] = phase
        x0[7] = self.rot.speed()
        self.ekf.x = x0
        self.ekf.P = np.eye(self.state_dim) * 0.1
        self.ekf.P[6, 6] = 0.01
        self.ekf.P[7, 7] = 0.01
        self.ekf.P[8, 8] = 0.01
        self.match_id = 0

    def predict(self, dt):
        if dt <= 0:
            return
        self._now += dt
        self.rot.predict(dt)

        def f_func(x):
            nx = x.copy()
            nx[0] += x[3] * dt
            nx[1] += x[4] * dt
            nx[2] += x[5] * dt
            # 符圆心固定不动——速度衰减，防止观测噪声驱动的虚假速度累积漂移。
            # 与前哨站模型同：exp(-dt / velocity_tau)。
            decay = math.exp(-dt / self.velocity_tau)
            nx[3] = x[3] * decay
            nx[4] = x[4] * decay
            nx[5] = x[5] * decay
            return nx

        def F_jacob(x):
            F = np.eye(self.state_dim)
            F[0, 3] = F[1, 4] = F[2, 5] = dt
            decay = math.exp(-dt / self.velocity_tau)
            F[3, 3] = F[4, 4] = F[5, 5] = decay
            return F

        self.ekf.predict(F=None, Q=self.Q_cont * dt, f_func=f_func, F_jacobian=F_jacob)
        self._sync_rotation()

    def update(self, obs):
        if not self.is_init or obs.entity_type != self.robot_type:
            self.init_model(obs)
            return

        pos = np.asarray(obs.rel_pos, dtype=float)
        phi_meas = self.obs_phase(obs)
        step = 2.0 * np.pi / self.armor_count

        x = self.ekf.x
        key_obs = int(round(safe_angle_sub(phi_meas, x[6]) / step)) % self.armor_count
        self.match_id = key_obs

        phi_ref_obs = phi_meas - key_obs * step
        self.rot.update(phi_ref_obs)

        z_meas = np.array([pos[0], pos[1], pos[2], phi_meas])
        r = self.radius
        step_m = self.match_id

        def h_func(x):
            phase = x[6] + step_m * step
            return np.array([x[0], x[1] + r * np.cos(phase),
                             x[2] + r * np.sin(phase), phase])

        def H_jacob(x):
            phase = x[6] + step_m * step
            c, s = np.cos(phase), np.sin(phase)
            H = np.zeros((4, self.state_dim))
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[1, 6] = -r * s
            H[2, 2] = 1.0
            H[2, 6] = r * c
            H[3, 6] = 1.0
            return H

        self.ekf.update(z=z_meas, H=None, R=self.R,
                        z_sub_func=self._meas_sub,
                        h_func=h_func, H_jacob=H_jacob)

        self._sync_rotation()

        # 按 key 维护 lit_marks：
        #   key 是灯臂的稳定编号（0-4），不随相位变——"同一块灯臂"用 key 识别，
        #   而不是相位。之前按相位匹配时，灯臂转动让旧记录的相位与新观测不匹配，
        #   lit=False 的观测清不掉旧条目——旧灯臂会在 lit_marks 里挂满 lit_hold 秒，
        #   期间 select_lit_blade 可能挑到已打掉的灯臂，fire_key 也卡在旧值。
        hit = None
        for i, (_phase_m, _seen, key_m) in enumerate(self.lit_marks):
            if key_m == key_obs:
                hit = i
                break
        if hit is not None:
            if getattr(obs, "lit", False):
                self.lit_marks[hit] = (phi_meas, self._now, key_obs)
            else:
                self.lit_marks.pop(hit)
        else:
            if getattr(obs, "lit", False):
                self.lit_marks.append((phi_meas, self._now, key_obs))
        self.lit_marks = [m for m in self.lit_marks if self._now - m[1] <= self.lit_hold]

    @staticmethod
    def _meas_sub(z, z_pred):
        diff = z - z_pred
        diff[3] = limit_rad(diff[3])
        return diff

    def mahalanobis_to(self, obs):
        if not self.is_init:
            return float("inf"), -1
        pos = np.asarray(obs.rel_pos, dtype=float)
        phase_obs = self.obs_phase(obs)
        z_meas = np.array([pos[0], pos[1], pos[2], phase_obs])
        best_d2, best_id, best_key = float("inf"), -1, None
        for k in range(self.armor_count):
            z_pred, H = self.predict_observation(k)
            y = self._meas_sub(z_meas, z_pred)
            S = H @ self.ekf.P @ H.T + self.R
            try:
                d2 = float(y @ np.linalg.solve(S, y))
            except np.linalg.LinAlgError:
                d2 = float(y @ np.linalg.pinv(S) @ y)
            geo = float(np.linalg.norm(pos - self.blade_position(k)))
            key = (d2, geo)
            if best_key is None or key < best_key:
                best_key, best_d2, best_id = key, d2, k
        return best_d2, best_id

    def get_geometric_distance(self, obs_pos) -> float:
        obs_pos = np.asarray(obs_pos, dtype=float)
        return float(min(np.linalg.norm(obs_pos - self.blade_position(k))
                         for k in range(self.armor_count)))

    def diverged(self) -> bool:
        if not self.is_init:
            return True
        x = self.ekf.x
        if not np.all(np.isfinite(x)):
            return True
        if self.ekf.get_nis_fail_rate() > 0.8 and \
                self.ekf.recent_nis_failures.maxlen == len(self.ekf.recent_nis_failures):
            return True
        return bool(abs(x[7]) > 20.0 or np.linalg.norm(x[3:6]) > 5.0)
