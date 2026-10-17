# custom/schemes/tjurm_scheme/models/outpost_model.py
"""前哨站模型：车辆模型（AntitopV5 12-state）加前哨站通道。"""
import numpy as np
import math
from collections import deque

from core.entities.property.entity_types import EntityType
from core.algorithms.filters.extended_kalman import ExtendedKalmanFilter
from core.algorithms.filters.kalman import KalmanFilter
from core.algorithms.math.angle import limit_rad, safe_angle_sub
from config import cfg


class _SlidingAvg:
    """固定长度滑窗平均。"""

    def __init__(self, size: int = 30):
        self.buf = deque(maxlen=int(size))

    def push(self, value):
        self.buf.append(np.asarray(value, float).copy())

    def get(self, fallback):
        if not self.buf:
            return np.asarray(fallback, float)
        return np.mean(np.stack(self.buf), axis=0)

    def clear(self):
        self.buf.clear()


class OutpostModel:
    """前哨站模型。"""

    def __init__(self):
        outpost = cfg.outpost
        self.cfg = outpost
        self.outpost_radius = float(outpost.radius)
        self.height_step = float(outpost.height_step)
        self.robot_type = None
        self.state_dim = 12
        self.ekf = ExtendedKalmanFilter(
            state_dim=self.state_dim,
            x_add_func=lambda x, d: x + d
        )
        self.ekf.P = np.eye(self.state_dim)
        self.ekf.P[6, 6] = 1.0
        self.ekf.P[7, 7] = 200.0
        self.ekf.P[8, 8] = 0.01
        self.ekf.P[9, 9] = 0.01

        self.q_xy   = 0.01
        self.q_z    = 0.1
        self.q_vxy  = 0.5
        self.q_vz   = 0.5
        self.q_yaw  = 1e-5
        self.q_omg  = 0.2
        self.q_r    = 1e-7
        self.q_dz   = 1e-7
        self.Q_cont = np.diag([
            self.q_xy, self.q_xy, self.q_z,
            self.q_vxy, self.q_vxy, self.q_vz,
            self.q_yaw, self.q_omg,
            self.q_r, self.q_r,
            self.q_dz, self.q_dz
        ])

        self.r_xy  = 0.1
        self.r_z   = 0.1
        self.r_yaw = 0.1
        self.R = np.diag([self.r_xy, self.r_xy, self.r_z, self.r_yaw])

        self.angle_kf = KalmanFilter(state_dim=2, x_add_func=lambda a, b: a + b)
        self.angle_kf.P = np.eye(2) * 0.1
        self.angle_Q_cont = np.diag([0.01, 0.1])
        self.angle_R = np.eye(1) * 0.1

        self.cont_yaw = 0.0
        self.last_obs_yaw = 0.0

        self.default_radius = 0.2
        self.is_init = False
        self.velocity_tau = 0.3
        self.center_win = _SlidingAvg(30)

        self.armor_count = 4
        self.id_counts = []
        self.id_lost = []
        self.stable_id = 0
        self.match_id = 0

        self.dist_thresh = 9.49
        self.confirm_thresh = 3
        self.max_lost = 10

        self.keep_threshold = 0.6
        self.hysteresis_bias = 0.45

        self.id_mode = "switch"
        self.switch_threshold_div = 1.5
        self.last_switched = False
        self.last_switch_dir = 0

    @staticmethod
    def _meas_sub(z, z_pred):
        diff = z - z_pred
        diff[3] = limit_rad(diff[3])
        return diff

    def init_model(self, obs):
        self.robot_type = obs.entity_type
        self.is_init = True
        obs_pos = obs.rel_pos
        obs_yaw = obs.rel_rpy[2]
        obs_id = 0

        self.cont_yaw = obs_yaw
        self.last_obs_yaw = obs_yaw
        self.angle_kf.x = np.array([obs_yaw, 0.0])
        self.angle_kf.P = np.eye(2) * 0.1

        x0 = np.zeros(self.state_dim)
        x0[0] = obs_pos[0] - self.default_radius * np.cos(obs_yaw)
        x0[1] = obs_pos[1] - self.default_radius * np.sin(obs_yaw)
        x0[2] = obs_pos[2]
        if self.robot_type == EntityType.Outpost:
            x0[6] = obs_yaw - obs_id % self._get_armor_count() * \
                (2.0 * np.pi / self._get_armor_count())
        else:
            x0[6] = obs_yaw
        if self.robot_type == EntityType.Outpost:
            k0 = obs_id % self._get_armor_count()
            x0[2] = obs_pos[2] - k0 * self.height_step
            x0[8] = self.outpost_radius
            x0[10] = obs_pos[2] + (1 - k0) * self.height_step
            x0[11] = obs_pos[2] + (2 - k0) * self.height_step
        else:
            x0[8] = self.default_radius
            x0[9] = self.default_radius
            x0[10] = 0.0
            x0[11] = 0.0
        self.ekf.x = x0

        self.ekf.P = np.eye(self.state_dim) * 0.1
        self.ekf.P[7, 7] = 50.0

        self.armor_count = self._get_armor_count()
        self.id_counts = [0] * self.armor_count
        self.id_lost = [0] * self.armor_count
        self.stable_id = 0
        self.match_id = obs_id % self.armor_count

    def predict(self, dt):
        if dt <= 0:
            return

        F_angle = np.array([[1, dt], [0, 1]])
        Q_angle = self.angle_Q_cont * dt
        self.angle_kf.predict(F_angle, Q_angle)

        Q_step = self.Q_cont * dt

        def f_func(x):
            nx = x.copy()
            nx[0] += x[3] * dt
            nx[1] += x[4] * dt
            nx[2] += x[5] * dt
            nx[6] += x[7] * dt
            if self.robot_type == EntityType.Outpost:
                decay = math.exp(-dt / self.velocity_tau)
                nx[3] = x[3] * decay
                nx[4] = x[4] * decay
                nx[5] = x[5] * decay
            return nx

        def F_jacob(x):
            F = np.eye(self.state_dim)
            F[0, 3] = dt
            F[1, 4] = dt
            F[2, 5] = dt
            F[6, 7] = dt
            if self.robot_type == EntityType.Outpost:
                decay = math.exp(-dt / self.velocity_tau)
                F[3, 3] = F[4, 4] = F[5, 5] = decay
            return F

        self.ekf.predict(F=None, Q=Q_step, f_func=f_func, F_jacobian=F_jacob)

    def update(self, obs):
        if not self.is_init or obs.entity_type != self.robot_type:
            self.init_model(obs)
            return

        obs_pos = obs.rel_pos
        obs_yaw = obs.rel_rpy[2]

        angle_diff = safe_angle_sub(obs_yaw, self.last_obs_yaw)
        interval = (2.0 * np.pi) / self.armor_count
        threshold = interval / self.switch_threshold_div
        switched = False
        switch_dir = 0
        if abs(angle_diff) > threshold:
            switched = True
            switch_dir = 1 if angle_diff > 0 else -1
            angle_diff += -interval if angle_diff > 0 else interval
        self.cont_yaw += angle_diff
        self.last_obs_yaw = obs_yaw

        z_angle = np.array([self.cont_yaw])
        H_angle = np.array([[1, 0]])

        def z_sub_angle(z, zp):
            diff = z - zp
            diff[0] = limit_rad(diff[0])
            return diff

        self.angle_kf.update(z_angle, H_angle, self.angle_R, z_sub_func=z_sub_angle)
        omega_kf = self.angle_kf.x[1]
        P_omega_kf = self.angle_kf.P[1, 1]

        if self.id_mode == "search":
            best_id, aligned_yaw = self._solve_match_id_search(
                obs_pos, obs_yaw, self.match_id)
        else:
            best_id, aligned_yaw = self._solve_match_id(
                obs_pos, obs_yaw, self.match_id, switched, switch_dir)
        self.match_id = best_id
        self.last_switched = switched
        self.last_switch_dir = switch_dir

        z_meas = np.array([obs_pos[0], obs_pos[1], obs_pos[2], aligned_yaw])

        def h_func(x):
            psi = x[6]
            r, _dz, _, _ = self._get_armor_params(best_id)
            armor_yaw = psi + best_id * (2.0 * np.pi / self.armor_count)
            return np.array([
                x[0] + r * np.cos(armor_yaw),
                x[1] + r * np.sin(armor_yaw),
                self._plate_z(x, best_id),
                armor_yaw
            ])

        def H_jacob(x):
            H = np.zeros((4, self.state_dim))
            psi = x[6]
            r, _dz, r_idx, dz_idx = self._get_armor_params(best_id)
            armor_yaw = psi + best_id * (2.0 * np.pi / self.armor_count)
            c, s = np.cos(armor_yaw), np.sin(armor_yaw)

            H[0, 0] = 1.0
            H[0, 6] = -r * s
            if r_idx is not None:
                H[0, r_idx] = c
            H[1, 1] = 1.0
            H[1, 6] = r * c
            if r_idx is not None:
                H[1, r_idx] = s
            H[2, 2] = 0.0 if (self.robot_type == EntityType.Outpost and best_id > 0) else 1.0
            if dz_idx is not None:
                H[2, dz_idx] = 1.0
            H[3, 6] = 1.0
            return H

        self.ekf.update(
            z=z_meas, H=None, R=self.R,
            z_sub_func=self._meas_sub,
            h_func=h_func, H_jacob=H_jacob
        )
        self.center_win.push(self.ekf.x[:3])

        if self.robot_type == EntityType.Outpost:
            self.ekf.x[7] = omega_kf
            self.ekf.P[:, 7] = 0.0
            self.ekf.P[7, :] = 0.0
            self.ekf.P[7, 7] = P_omega_kf

            R_MIN = 0.265
            R_MAX = 0.275
            if self.ekf.x[8] < R_MIN:
                self.ekf.x[8] = R_MIN
            elif self.ekf.x[8] > R_MAX:
                self.ekf.x[8] = R_MAX
        else:
            MIN_RADIUS = 0.1
            MAX_RADIUS = 0.40
            if self.ekf.x[8] < MIN_RADIUS:
                self.ekf.x[8] = MIN_RADIUS
            elif self.ekf.x[8] > MAX_RADIUS:
                self.ekf.x[8] = MAX_RADIUS
            if self.ekf.x[9] < MIN_RADIUS:
                self.ekf.x[9] = MIN_RADIUS
            elif self.ekf.x[9] > MAX_RADIUS:
                self.ekf.x[9] = MAX_RADIUS

    def _get_armor_count(self):
        if self.robot_type == EntityType.Outpost:
            return 3
        return 4

    def _get_armor_params(self, k):
        x = self.ekf.x
        if self.robot_type == EntityType.Outpost:
            if k == 0:
                return x[8], 0.0, 8, None
            elif k == 1:
                return x[8], x[10], 8, 10
            else:
                return x[8], x[11], 8, 11
        else:
            if k % 2 == 0:
                return x[8], 0.0, 8, None
            else:
                return x[9], x[10], 9, 10

    def _plate_z(self, x, k: int) -> float:
        _r, dz, _ri, _di = self._get_armor_params(k)
        if self.robot_type == EntityType.Outpost and k > 0:
            return float(dz)
        return float(x[2]) + float(dz)

    def _solve_match_id(self, obs_pos, obs_yaw, last_id,
                        switched=False, switch_dir=0):
        center_yaw = self.ekf.x[6]
        n = self.armor_count
        step = (2.0 * np.pi) / n

        if self.is_init and 0 <= last_id < n:
            if not switched:
                best_id = last_id
            else:
                best_id = int((last_id + switch_dir) % n)
        else:
            diffs = [abs(safe_angle_sub(obs_yaw, center_yaw + i * step))
                     for i in range(n)]
            best_id = int(np.argmin(diffs))

        pred_best_yaw = center_yaw + best_id * step
        yaw_residual = safe_angle_sub(obs_yaw, pred_best_yaw)
        aligned_yaw = pred_best_yaw + yaw_residual

        return best_id, aligned_yaw

    def _solve_match_id_search(self, obs_pos, obs_yaw, last_id):
        center_yaw = self.ekf.x[6]
        n = self.armor_count
        step = (2.0 * np.pi) / n

        angle_diffs = [abs(safe_angle_sub(obs_yaw, center_yaw + i * step))
                       for i in range(n)]
        best_id = int(np.argmin(angle_diffs))

        if 0 <= last_id < n:
            if (angle_diffs[last_id] < self.keep_threshold or
                    angle_diffs[last_id] < angle_diffs[best_id] + self.hysteresis_bias):
                best_id = last_id

        pred_best_yaw = center_yaw + best_id * step
        aligned_yaw = pred_best_yaw + safe_angle_sub(obs_yaw, pred_best_yaw)
        return best_id, aligned_yaw

    def armor_candidates(self, t_future: float):
        x = self.ekf.x
        psi = x[6] + x[7] * t_future
        center = self.center_win.get(x[:3]) + x[3:6] * t_future
        step = (2.0 * np.pi) / self.armor_count
        out = []
        for k in range(self.armor_count):
            r, _dz, _, _ = self._get_armor_params(k)
            a = psi + k * step
            out.append((center + np.array([r * np.cos(a), r * np.sin(a), 0.0])
                        + np.array([0.0, 0.0, self._plate_z(x, k) - x[2] + x[5] * t_future]),
                        np.array([np.cos(a), np.sin(a), 0.0]), False))
        return out

    def predict_observation(self, armor_id):
        x = self.ekf.x
        r, dz, r_idx, dz_idx = self._get_armor_params(armor_id)
        step = (2.0 * np.pi) / self.armor_count
        armor_yaw = x[6] + armor_id * step
        c, s = np.cos(armor_yaw), np.sin(armor_yaw)

        z_pred = np.array([x[0] + r * c, x[1] + r * s, self._plate_z(x, armor_id), armor_yaw])

        H = np.zeros((4, self.state_dim))
        H[0, 0] = 1.0
        H[0, 6] = -r * s
        if r_idx is not None:
            H[0, r_idx] = c
        H[1, 1] = 1.0
        H[1, 6] = r * c
        if r_idx is not None:
            H[1, r_idx] = s
        H[2, 2] = 1.0
        if dz_idx is not None:
            H[2, dz_idx] = 1.0
        H[3, 6] = 1.0
        return z_pred, H

    def mahalanobis_to(self, obs):
        if not self.is_init:
            return float("inf"), -1
        obs_pos = np.asarray(obs.rel_pos, dtype=float)[:3]
        obs_yaw = float(obs.rel_rpy[2])
        z_meas = np.array([obs_pos[0], obs_pos[1], obs_pos[2], obs_yaw])

        best_d2, best_id = float("inf"), -1
        best_key = None
        for k in range(self.armor_count):
            z_pred, H = self.predict_observation(k)
            y = z_meas - z_pred
            y[3] = safe_angle_sub(z_meas[3], z_pred[3])
            S = H @ self.ekf.P @ H.T + self.R
            try:
                d2 = float(y @ np.linalg.solve(S, y))
            except np.linalg.LinAlgError:
                d2 = float(y @ np.linalg.pinv(S) @ y)
            r, dz, _, _ = self._get_armor_params(k)
            step = (2.0 * np.pi) / self.armor_count
            a_yaw = self.ekf.x[6] + k * step
            geo = float(np.linalg.norm([
                obs_pos[0] - (self.ekf.x[0] + r * np.cos(a_yaw)),
                obs_pos[1] - (self.ekf.x[1] + r * np.sin(a_yaw)),
                obs_pos[2] - self._plate_z(self.ekf.x, k),
            ]))
            key = (d2, geo)
            if best_key is None or key < best_key:
                best_key = key
                best_d2, best_id = d2, k
        return best_d2, best_id

    def get_all_armor_positions_at_time(self, dt):
        if not self.is_init:
            return [np.zeros(3)] * self.armor_count
        x = self.ekf.x
        pred_cx = x[0] + x[3] * dt
        pred_cy = x[1] + x[4] * dt
        pred_cz = x[2] + x[5] * dt
        pred_psi = x[6] + x[7] * dt
        armors = []
        for k in range(self.armor_count):
            r, dz, _, _ = self._get_armor_params(k)
            armor_yaw = pred_psi + k * (2.0 * np.pi / self.armor_count)
            ax = pred_cx + r * np.cos(armor_yaw)
            ay = pred_cy + r * np.sin(armor_yaw)
            az = self._plate_z(x, k) + x[5] * dt
            armors.append(np.array([ax, ay, az]))
        return armors

    def get_geometric_distance(self, obs_pos):
        xc, yc, zc = self.ekf.x[0], self.ekf.x[1], self.ekf.x[2]
        psi = self.ekf.x[6]
        min_dist = float('inf')
        for k in range(self.armor_count):
            r, dz, _, _ = self._get_armor_params(k)
            armor_yaw = psi + k * (2.0 * np.pi / self.armor_count)
            pred_ax = xc + r * np.cos(armor_yaw)
            pred_ay = yc + r * np.sin(armor_yaw)
            pred_az = self._plate_z(self.ekf.x, k)
            dist = np.linalg.norm([obs_pos[0] - pred_ax,
                                    obs_pos[1] - pred_ay,
                                    obs_pos[2] - pred_az])
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def diverged(self):
        if not self.is_init:
            return True
        x = self.ekf.x
        if x[8] > 0.6 or x[9] > 0.6:
            return True
        if abs(x[3]) > 10 or abs(x[4]) > 10:
            return True
        return False