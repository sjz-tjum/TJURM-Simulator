# custom/schemes/tjurm_scheme/mpc/reference.py
"""参考轨迹生成：为 MPC 提供 theta_ref / omega_ref。

整条 MPC 链路默认不启用（enable_mpc=False）。
"""
import numpy as np

from custom.components.armor_selectors.tjurm_armor_selector import ArmorSelector


class ReferenceGenerator:
    def __init__(self, robot, selector: ArmorSelector, dt=0.01, N=50, yaw_offset=0.0):
        self.robot = robot
        self.dt = dt
        self.N = N
        self.yaw_offset = yaw_offset
        self.selector = selector

    def generate(self, target, t0, delay):
        theta_ref = np.zeros(self.N)
        omega_ref = np.zeros(self.N)

        robot = self.robot
        if robot is None:
            return theta_ref, omega_ref

        cam_rel_pos = robot.get_camera_rel_pos()
        x = target.ekf.ekf.x

        for i in range(self.N):
            t_future = i * self.dt + delay
            armor_id, armor_pos = self.selector.select_armor(
                target, cam_rel_pos, t_future)

            dx = armor_pos[0] - cam_rel_pos[0]
            dy = armor_pos[1] - cam_rel_pos[1]
            theta_des = np.arctan2(dy, dx) + self.yaw_offset
            theta_ref[i] = theta_des

            pred_vx = x[3]
            pred_vy = x[4]
            r2 = dx * dx + dy * dy
            if r2 > 1e-6:
                omega_des = (dx * pred_vy - dy * pred_vx) / r2
            else:
                omega_des = 0.0
            omega_ref[i] = omega_des

        theta_ref = self._unwrap_angle(theta_ref)
        return theta_ref, omega_ref

    @staticmethod
    def _unwrap_angle(angle_seq):
        unwrapped = np.zeros_like(angle_seq)
        unwrapped[0] = angle_seq[0]
        for i in range(1, len(angle_seq)):
            diff = angle_seq[i] - unwrapped[i - 1]
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            unwrapped[i] = unwrapped[i - 1] + diff
        return unwrapped