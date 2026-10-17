# sim/managers/sensor_manager.py
"""传感器管理器：真值 -> 可见性 / 角点投影 / 像素噪声 -> 检测消息。

模拟"图像处理之后"的检测头：**只产出 2D 图像点，不做任何解算**。
解算与跟踪是 custom 侧的事，由入口侧的 keeper 接着往下送。

像素噪声是这里唯一注入的误差：它在解算里会自然传播成位姿误差，
不需要再单独加位姿噪声。

可见性只有一处判据：`Camera.faces_camera` —— 面的法线是否朝向相机。
装甲板与灯臂共用。

发帧规则（两类"不发"要区分）：
    - 没到 sensor_fps 采样点：返回 None，不发消息（下游保持上一帧）。
    - 采样了但没看到目标：发空列表的 DetectionFrame（下游知道"这一拍是空的"）。
"""
from __future__ import annotations

import numpy as np

from config import cfg
from core.bus.dataflow import Detection, DetectionFrame
from core.entities.property.entity_types import EntityType


class SensorManager:
    def __init__(self, robot_manager, device_manager, *, rng=None):
        self.robot_manager = robot_manager
        self.device_manager = device_manager
        self._rng = rng if rng is not None else np.random.default_rng()

        s = cfg.sim
        self.pixel_noise_sigma = float(s.pixel_noise_sigma)
        # 相邻角点最短边长（px）——低于它就说明板在像面上退化成一条线，
        # 深度必然不可解。安全网，不是筛选器。
        self.min_edge_px = float(s.camera.min_edge_px)

        # 相机采样率：仿真步进 200 Hz，相机不是每帧都出图。0 或负数 = 不降频。
        self.sensor_fps = float(s.sensor_fps or 0.0)
        self._last_emit_t = float("-inf")

        self.last_frame = None
        self._t = 0.0

    # ---------------------------------------------------------- 随机源
    def set_rng(self, rng):
        self._rng = rng

    def reset(self):
        self.last_frame = None
        self._t = 0.0
        self._last_emit_t = float("-inf")

    # ---------------------------------------------------------- 主入口
    def get_obs(self, timestamp=None):
        """采集一帧检测；没到采样点返回 None，采样到但没看到东西发空帧。"""
        observer = self.robot_manager.controlled_robot
        if observer is None or not observer.health.alive:
            return None

        t = self._t if timestamp is None else float(timestamp)
        self._t = t

        if self.sensor_fps > 0.0 and t - self._last_emit_t < 1.0 / self.sensor_fps:
            return None                       # 没到采样点：不发

        camera = observer.gimbal.camera
        detections = []

        for robot in self.robot_manager.robots:
            if robot is observer or not robot.health.alive:
                continue                      # 不打自己，也不看死人
            for armor in robot.get_armors():
                d = self._detect_armor(armor, camera)
                if d is not None:
                    detections.append(d)

        outpost = self.device_manager.outpost
        if outpost is not None and outpost.health.alive:
            for armor in outpost.armors:
                d = self._detect_armor(armor, camera)
                if d is not None:
                    detections.append(d)

        rune = self.device_manager.rune
        if rune is not None:
            detections.extend(self._detect_rune(rune, camera))

        # 空列表也发 —— "这一拍看了，没看到东西"。
        # 上面那个 return None 是"这一拍没到采样点"，两者语义不同。
        frame = DetectionFrame(detections, t)
        self.last_frame = frame
        self._last_emit_t = t
        return frame

    # ---------------------------------------------------------- 单块装甲板
    def _detect_armor(self, armor, camera):
        """一块装甲板 -> Detection；任何一步不成立就返回 None。"""
        if not camera.faces_camera(armor.world_R[:, 0], armor.world_pos):
            return None
        pixels = self._project_corners(armor, camera)
        if pixels is None:
            return None
        return Detection(
            entity_type=armor.entity_type,
            image_points_2d=pixels,
            armor_size=armor.armor_size,
        )

    # ---------------------------------------------------------- 四角点
    def _project_corners(self, armor, camera):
        """一块板/灯臂 -> 四个角点像素；任何一步不成立就返回 None。

        装甲板与灯臂共用：两者在本体系里都是 x 恒为 0 的 w×h 矩形，
        顺序 LT/RT/RB/LB，与 PnP 的对象点顺序严格一致（错一个就是镜像的板）。
        """
        R_armor = np.asarray(armor.world_R, float)
        local = np.asarray(armor.init_light_corners, float)
        world_corners = (R_armor @ local.T).T + np.asarray(armor.world_pos, float)

        pixels = []
        for corner in world_corners:
            px = camera.world_to_pixel(corner)
            if px is None:
                # 出像面的角点真实相机拍不到。整块丢弃，不做"截断到边界"——
                # 那种假点会把 PnP 往错误方向拽。
                return None
            pixels.append(px)
        pixels = np.asarray(pixels, float)

        # 像素噪声：u/v 各向独立。它是这里唯一注入的误差，会自然传播成位姿误差。
        if self.pixel_noise_sigma > 0.0:
            pixels = pixels + self._rng.normal(
                0.0, self.pixel_noise_sigma, size=pixels.shape)

        # 退化剔除：检测器在这种形态下本来就给不出可信的角点
        edges = pixels[[1, 2, 3, 0]] - pixels
        if float(np.min(np.linalg.norm(edges, axis=1))) < self.min_edge_px:
            return None
        return pixels

    # ---------------------------------------------------------- 能量机关
    def _detect_rune(self, rune, camera):
        """能量机关：每块灯臂给四个角点 —— 解算端靠它拿靶心（圆靶没有板面方向）。

        看见几块待激活的灯臂就产出几条检测：多条就是多个靶心。
        """
        out = []
        for blade in rune.blades:
            if not camera.faces_camera(blade.world_R[:, 0], blade.world_pos):
                continue
            pixels = self._project_corners(blade, camera)
            if pixels is None:
                continue
            # 靶心就是四角点的中心（本体系里角点绕板心对称）。圆靶没有板面方向可解，
            # 所以不走 PnP：中心直接放进去，解算端原样透传。
            R_armor = np.asarray(blade.world_R, float)
            local = np.asarray(blade.init_light_corners, float)
            corners = (R_armor @ local.T).T + np.asarray(blade.world_pos, float)
            out.append(Detection(
                entity_type=EntityType.Rune,
                image_points_2d=pixels,
                armor_size=blade.armor_size,
                lit=blade.armor_id in rune.lit,
                center=corners.mean(axis=0),
            ))
        return out