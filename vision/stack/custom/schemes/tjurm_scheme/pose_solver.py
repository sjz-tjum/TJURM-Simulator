# custom/schemes/tjurm_scheme/pose_solver.py
"""tjurm 帧级位姿解算：遍历检测 -> 单板 PnP -> C' -> G/W -> 打包 Observation。"""
from __future__ import annotations

import math

import numpy as np

from config import cfg
from core.algorithms.math.transform import (abs_camera_to_gimbal,
                                            gimbal_to_world,
                                            rpy_to_R)
from core.bus.dataflow import ArmorObservation, Observation
from core.entities.property.entity_types import EntityType
from core.interfaces.pose_solver import PoseSolver, register_pose_solver

from custom.components.armor_solvers import get_solver


@register_pose_solver("tjurm")
class TjurmPoseSolver(PoseSolver):
    """tjurm 方案默认的帧级解算：遍历 + 单板 PnP + 打包。"""

    description = "遍历检测 → 单板 PnP → C' → G/W"

    def __init__(self, armor_solver_name=None, *, frame=None):
        self.frame = str(cfg.sim.frame if frame is None else frame).upper()
        if self.frame not in ("G", "W"):
            raise ValueError(f"Sim.frame 必须是 'G' 或 'W'，实得 {self.frame!r}")
        self.armor_solver = get_solver(
            armor_solver_name or str(cfg.sim.pnp_method))

    def reset(self) -> None:
        pass

    def solve(self, frame, self_state):
        cam = self_state.camera
        R_camera_to_abs_camera = rpy_to_R(cam.rpy)
        gimbal_pos_world = np.asarray(self_state.gimbal.pos, float)
        camera_pos_gimbal = np.asarray(cam.pos, float) - gimbal_pos_world

        armors = []
        for d in frame.detections:
            if d.entity_type is EntityType.Rune:
                pos = np.asarray(d.center, float)
                if self.frame != "W":
                    pos = pos - gimbal_pos_world
                armors.append(ArmorObservation(
                    entity_type=d.entity_type,
                    rel_pos=pos,
                    rel_rpy=np.array([0.0, 0.0, math.pi]),  # 圆靶无板面方向，先固定 -x 朝观测者
                    armor_size=d.armor_size,
                    image_points_2d=d.image_points_2d,
                    lit=d.lit,
                ))
                continue

            pos_c, rpy_c = self.armor_solver.estimate(
                camera_k=cam.K,
                armor_size=d.armor_size,
                image_points_2d=d.image_points_2d,
                rot_camera_to_abs_camera=R_camera_to_abs_camera,
                mount_pitch=self._mount_pitch(d.entity_type))
            if pos_c is None:
                continue

            pos, _ = abs_camera_to_gimbal(pos_c, np.eye(3),
                                          camera_pos_gimbal=camera_pos_gimbal)
            if self.frame == "W":
                pos, _ = gimbal_to_world(pos, np.eye(3),
                                         gimbal_pos_world=gimbal_pos_world)

            armors.append(ArmorObservation(
                entity_type=d.entity_type,
                rel_pos=np.asarray(pos, float),
                rel_rpy=np.asarray(rpy_c, float),
                armor_size=d.armor_size,
                image_points_2d=d.image_points_2d,
                lit=d.lit,
            ))
        return Observation(armors, float(frame.timestamp))

    @staticmethod
    def _mount_pitch(entity_type):
        tilt = float(cfg.outpost.armor_tilt if entity_type is EntityType.Outpost
                     else cfg.robot.armor_tilt)
        return math.radians(tilt)