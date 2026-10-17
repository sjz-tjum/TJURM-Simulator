# core/entities/rigids/gimbal.py
"""云台：带相机和枪口，安装在底盘上，有相对旋转。

位置由底盘推（Chassis.update_children 写 world_pos / world_vel），
本类只负责把状态继续推给相机与枪口。
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from config import cfg
from core.algorithms.control import CascadedPID, PIDParams
from core.algorithms.math.angle import limit_rad
from core.algorithms.math.transform import rpy_to_R
from core.entities.rigids.camera import Camera
from core.entities.rigids.muzzle import Muzzle
from core.entities.property.rigid import Rigid


class Gimbal(Rigid):
    def __init__(self,
                 mount_pos: Optional[np.ndarray] = None,
                 mount_rpy: Optional[np.ndarray] = None,
                 pitch_offset: Optional[np.ndarray] = None,
                 camera_mount_pos: Optional[np.ndarray] = None,
                 muzzle_mount_pos: Optional[np.ndarray] = None,
                 **kwargs):
        super().__init__(**kwargs)
        self.mount_pos = (np.zeros(3) if mount_pos is None
                          else np.asarray(mount_pos, float).copy())
        self.mount_rpy = (np.zeros(3) if mount_rpy is None
                          else np.asarray(mount_rpy, float).copy())
        # 偏航轴系到俯仰轴的偏移：相机/枪口挂在俯仰轴之后
        self.pitch_offset = (np.zeros(3) if pitch_offset is None
                             else np.asarray(pitch_offset, float).copy())

        g = cfg.sim.gimbal
        self.alpha_max_yaw = float(g.alpha_max_yaw)
        self.alpha_max_pitch = float(g.alpha_max_pitch)

        self.camera = Camera(mount_pos=camera_mount_pos)
        self.muzzle = Muzzle(mount_pos=muzzle_mount_pos)

        self.auto_aiming = False
        self.auto_fire = False
        self.target_theta = 0.0
        self.target_omega = 0.0
        self.target_alpha = 0.0
        self.target_pitch = 0.0
        self.target_pitch_omega = 0.0
        self.target_pitch_alpha = 0.0

        self.yaw_pid = CascadedPID(
            PIDParams(kp=float(g.yaw_outer.kp),
                      ki=float(g.yaw_outer.ki),
                      kd=float(g.yaw_outer.kd),
                      max_output=float(g.yaw_outer.max_output)),
            PIDParams(kp=float(g.yaw_inner.kp),
                      ki=float(g.yaw_inner.ki),
                      kd=float(g.yaw_inner.kd),
                      max_output=float(g.yaw_inner.max_output)))
        self.pitch_pid = CascadedPID(
            PIDParams(kp=float(g.pitch_outer.kp),
                      ki=float(g.pitch_outer.ki),
                      kd=float(g.pitch_outer.kd),
                      max_output=float(g.pitch_outer.max_output)),
            PIDParams(kp=float(g.pitch_inner.kp),
                      ki=float(g.pitch_inner.ki),
                      kd=float(g.pitch_inner.kd),
                      max_output=float(g.pitch_inner.max_output)))

        self.update_children()

    # ---------------------------------------------------------- 接口
    def set_target(self, theta, omega, alpha, pitch, pitch_omega, pitch_alpha) -> None:
        self.target_theta = theta
        self.target_omega = omega
        self.target_alpha = alpha
        self.target_pitch = pitch
        self.target_pitch_omega = pitch_omega
        self.target_pitch_alpha = pitch_alpha

    def switch_auto_aiming(self) -> None:
        self.set_auto_aiming(not self.auto_aiming)

    def set_auto_aiming(self, on: bool) -> None:
        self.auto_aiming = bool(on)

    def switch_auto_fire(self) -> None:
        self.set_auto_fire(not self.auto_fire)

    def set_auto_fire(self, on: bool) -> None:
        self.auto_fire = bool(on)

    # ---------------------------------------------------------- 子部件
    def update_children(self) -> None:
        """把云台状态传给相机和枪口（都挂在俯仰轴之后）。"""
        R = rpy_to_R(self.world_rpy)
        # pitch_offset 是理论链 G →(转ψ)→ Y1 →(平移 d)→ P0 里的 d，属于 Y1 系：
        # 只受偏航影响，俯仰不该把它转走（用全姿态 R 会让俯仰角把相机/枪口带偏
        # —— Hero 在 pitch 20 度时约 3.4 cm、45 度时约 7.1 cm）。
        R_yaw = rpy_to_R(np.array([0.0, 0.0, self.world_rpy[2]]))
        p1_world = self.world_pos + R_yaw @ self.pitch_offset

        self.camera.world_pos = p1_world + R @ self.camera.mount_pos
        self.camera.world_rpy = self.world_rpy + self.camera.mount_rpy
        self.camera.world_vel = self.world_vel + np.cross(
            self.world_omg, self.camera.world_pos - self.world_pos)
        self.camera.world_omg = self.world_omg.copy()

        self.muzzle.world_pos = p1_world + R @ self.muzzle.mount_pos
        self.muzzle.world_rpy = self.world_rpy + self.muzzle.mount_rpy
        self.muzzle.world_vel = self.world_vel + np.cross(
            self.world_omg, self.muzzle.world_pos - self.world_pos)
        self.muzzle.world_omg = self.world_omg.copy()

    # ---------------------------------------------------------- 控制
    def apply_control(self, dt: float) -> None:
        if not self.auto_aiming:
            return

        err_theta = limit_rad(self.target_theta - self.world_rpy[2])
        sp_theta = self.world_rpy[2] + err_theta
        alpha_cmd = self.yaw_pid.update(
            sp_theta, self.world_rpy[2], self.world_omg[2],
            self.target_omega, self.target_alpha, dt)
        alpha_cmd = float(np.clip(alpha_cmd, -self.alpha_max_yaw, self.alpha_max_yaw))
        self.world_alp[2] = alpha_cmd
        self.world_omg[2] += alpha_cmd * dt
        self.world_rpy[2] += self.world_omg[2] * dt

        p_alpha = self.pitch_pid.update(
            self.target_pitch, self.world_rpy[1], self.world_omg[1],
            self.target_pitch_omega, self.target_pitch_alpha, dt)
        p_alpha = float(np.clip(p_alpha, -self.alpha_max_pitch, self.alpha_max_pitch))
        self.world_alp[1] = p_alpha
        self.world_omg[1] += p_alpha * dt
        self.world_rpy[1] += self.world_omg[1] * dt
        self.world_rpy[1] = float(np.clip(self.world_rpy[1], -np.pi / 2, np.pi / 2))
