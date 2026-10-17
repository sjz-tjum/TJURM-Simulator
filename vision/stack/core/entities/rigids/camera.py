# core/entities/rigids/camera.py
"""相机：针孔投影模型，内参从 cfg.sim.camera 读。

world → 像素这条链的每一段都调 transform 的函数；本类不重写小孔成像公式。
world_to_camera_RDF 不提供——RDF 由调用者 vec3d_FLU2RDF(world_to_camera_FLU(...))
自己拼。
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from config import cfg
from core.algorithms.math.transform import (
    camera_RDF_to_normalized,
    normalized_RDF_to_pixel,
    rpy_to_R,
    vec3d_FLU2RDF,
    world_to_camera_FLU,
)
from core.entities.property.rigid import Rigid


class Camera(Rigid):
    def __init__(self,
                 mount_pos: Optional[np.ndarray] = None,
                 mount_rpy: Optional[np.ndarray] = None,
                 **kwargs):
        super().__init__(**kwargs)
        c = cfg.sim.camera
        self.mount_pos = (np.zeros(3) if mount_pos is None
                          else np.asarray(mount_pos, float).copy())
        self.mount_rpy = (np.zeros(3) if mount_rpy is None
                          else np.asarray(mount_rpy, float).copy())
        self.max_range = float(c.max_range)
        self.image_size = (int(c.image_width), int(c.image_height))
        self.focal_len = float(c.focal_len)
        self.pixel_decimals = int(c.pixel_decimals)
        self.fov = 2.0 * math.atan(self.image_size[0] / (2.0 * self.focal_len))

    # ---------------------------------------------------------- 世界 -> camera
    def world_to_camera_FLU(self, world_pos) -> Optional[np.ndarray]:
        """世界 -> camera（FLU 轴序）。相机后方返回 None。"""
        pos_c, _ = world_to_camera_FLU(
            world_pos, np.eye(3),
            camera_pos_world=self.world_pos,
            camera_rpy_world=self.world_rpy)
        if pos_c[0] <= 0:
            return None
        return pos_c

    # ---------------------------------------------------------- camera -> 像素
    def camera_RDF_to_pixel(self, camera_rdf_pos) -> np.ndarray:
        """camera（RDF 轴序）-> 像素。带像素量化。"""
        n = camera_RDF_to_normalized(camera_rdf_pos)
        pixel = normalized_RDF_to_pixel(n, K=self.get_intrinsic_matrix())
        return np.array([self._quantize(float(pixel[0])),
                         self._quantize(float(pixel[1]))])

    # ---------------------------------------------------------- 世界 -> 像素
    def world_to_pixel(self, world_pos) -> Optional[np.ndarray]:
        """世界 -> 像素。不可见返回 None。"""
        flu = self.world_to_camera_FLU(world_pos)
        if flu is None:
            return None
        rdf = vec3d_FLU2RDF(flu)
        if not self._in_fov(rdf):
            return None
        return self.camera_RDF_to_pixel(rdf)

    def _in_fov(self, camera_rdf_pos) -> bool:
        """camera（RDF）点是否在视场与量程内。"""
        if camera_rdf_pos[2] <= 0:
            return False
        if float(np.linalg.norm(camera_rdf_pos)) > self.max_range:
            return False
        pixel = self.camera_RDF_to_pixel(camera_rdf_pos)
        u, v = pixel
        return (abs(u) <= self.image_size[0] / 2
                and abs(v) <= self.image_size[1] / 2)

    # ---------------------------------------------------------- 其他
    def _quantize(self, value: float) -> float:
        if self.pixel_decimals < 0:
            return float(value)
        return round(value, self.pixel_decimals)

    def get_forward_vec(self) -> np.ndarray:
        R = rpy_to_R(self.world_rpy)
        return R @ np.array([1.0, 0.0, 0.0])

    def faces_camera(self, normal_world, pos_world) -> bool:
        """面的外法线是否朝向相机 —— 背面看不到正面。

        法线点乘"面心 -> 相机"的向量，只判符号；不需要单位向量。
        装甲板与灯臂共用这一个语义。
        """
        normal = np.asarray(normal_world, float)
        pos = np.asarray(pos_world, float)
        return float(normal @ (self.world_pos - pos)) > 0.0

    def get_intrinsic_matrix(self) -> np.ndarray:
        return np.array([
            [self.focal_len, 0, 0],
            [0, self.focal_len, 0],
            [0, 0, 1],
        ])