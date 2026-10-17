# core/entities/rigids/robot.py
"""完整机器人：底盘 + 云台 + 血量。"""
from __future__ import annotations

from typing import Optional

import numpy as np

from config import cfg
from core.algorithms.math.transform import world_to_gimbal
from core.entities.property.entity_types import EntityType
from core.entities.property.health import Health
from core.entities.rigids.chassis import Chassis
from core.entities.rigids.gimbal import Gimbal


class Robot:
    def __init__(self, entity_type, chassis_pos=None, chassis_rpy=None,
                 gimbal_mount_pos: Optional[np.ndarray] = None,
                 gimbal_mount_rpy: Optional[np.ndarray] = None):
        entity_type = EntityType.coerce(entity_type)
        self.entity_type = entity_type
        name = EntityType.get_name(entity_type)
        c = cfg.robot[name]
        self.health = Health(c.hp)

        if gimbal_mount_pos is None:
            # 云台相对底盘的安装偏置（理论链 Chassis -> G），与 pitch_offset 一样按车型配；
            # 显式传参时以传参为准。
            gimbal_mount_pos = np.asarray(
                getattr(c, "gimbal_mount_pos", [0.0, 0.0, 0.0]), float)
        if gimbal_mount_rpy is None:
            gimbal_mount_rpy = np.zeros(3)
        # 云台偏航轴之后的偏移与两个挂点，全部来自车型配置
        pitch_offset = np.asarray(getattr(c, "pitch_offset", [0.0, 0.0, 0.0]), float)
        camera_mount_pos = np.asarray(getattr(c, "camera_mount_pos", [0.0, 0.0, 0.0]), float)
        muzzle_mount_pos = np.asarray(getattr(c, "muzzle_mount_pos", [0.0, 0.0, 0.0]), float)

        self.chassis = Chassis(entity_type, world_pos=chassis_pos, world_rpy=chassis_rpy)
        self.gimbal = Gimbal(mount_pos=gimbal_mount_pos,
                             mount_rpy=gimbal_mount_rpy,
                             pitch_offset=pitch_offset,
                             camera_mount_pos=camera_mount_pos,
                             muzzle_mount_pos=muzzle_mount_pos)
        # 父推子：底盘先推云台位置，云台再推相机与枪口
        self.chassis.gimbal = self.gimbal
        self.gimbal.world_rpy = self.chassis.world_rpy + gimbal_mount_rpy
        self.chassis.update_children()
        self.gimbal.update_children()

    # ---------------------------------------------------------- 查询
    @property
    def name(self) -> str:
        return EntityType.get_name(self.entity_type)

    def get_armors(self):
        return self.chassis.armors

    def get_camera(self):
        return self.gimbal.camera

    def get_muzzle(self):
        return self.gimbal.muzzle

    def get_camera_rel_pos(self) -> np.ndarray:
        """相机在 G 系（云台系）下的位置。"""
        pos, _ = world_to_gimbal(self.gimbal.camera.world_pos, np.eye(3),
                                 gimbal_pos_world=self.gimbal.world_pos)
        return pos

    def get_muzzle_rel_pos(self) -> np.ndarray:
        """枪口在 G 系（云台系）下的位置。"""
        pos, _ = world_to_gimbal(self.gimbal.muzzle.world_pos, np.eye(3),
                                 gimbal_pos_world=self.gimbal.world_pos)
        return pos
