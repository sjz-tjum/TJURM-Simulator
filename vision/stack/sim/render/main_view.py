# sim/render/main_view.py
"""MainView：俯视主视图。

跟随受控车；显示真值（车 / 前哨站 / 符 / 弹丸）+ 观测箭头 + 预测点 + 相机 FOV。
"""
from __future__ import annotations

import math

import numpy as np
import pygame as pg

from core.algorithms.math.transform import rpy_to_R
from sim.render.helpers import rel_to_world

_BG = (20, 20, 20)
_ORIGIN_FG = (255, 220, 0)
_ORIGIN_BG = (140, 110, 0)
_RUNE_BASE_LINE = (140, 146, 156)
_RUNE_SPOKE = (120, 126, 134)
_RUNE_HIT_BAND = (255, 60, 60)


class MainView:
    def __init__(self, viewport, resources, world_scale, frame):
        self.viewport = viewport
        self.resources = resources
        self.world_scale = float(world_scale)
        self.frame = frame
        self._center_screen = None

    def reset(self):
        self._center_screen = None

    # ---------------------------------------------------------------- 主入口
    def draw(self, robots, controlled, outpost, rune, bullets, obs, pred):
        vp = self.viewport
        c = self.resources.c
        rect = vp.main_rect

        self._center_screen = self._map_center(controlled)

        vp.set_clip(rect)
        pg.draw.rect(vp.screen, _BG, rect)
        pg.draw.rect(vp.screen, c.white, rect, 2)

        self._draw_origin()
        self._draw_robots(robots, controlled)
        self._draw_outpost(outpost)
        self._draw_rune(rune)
        self._draw_observations(obs, controlled)
        self._draw_predictions(pred, controlled)
        self._draw_bullets(bullets)

        selected_camera = controlled.get_camera() if controlled is not None else None
        if selected_camera is not None:
            self._draw_camera_fov(selected_camera)

        vp.clear_clip()

    # ---------------------------------------------------------------- 投影
    def _map_center(self, controlled):
        """俯视图中心：跟随受控车（车恒在视图中心）。"""
        if controlled is None:
            return self.viewport.main_center
        pos = np.asarray(controlled.chassis.world_pos, float)
        # 屏幕 x 对应世界 -y、屏幕 y 对应世界 -x（见 _to_screen）
        return np.array([self.viewport.main_center[0] + pos[1] * self.world_scale,
                         self.viewport.main_center[1] + pos[0] * self.world_scale])

    def _to_screen(self, world_pos):
        u = self._center_screen[0] - world_pos[1] * self.world_scale
        v = self._center_screen[1] - world_pos[0] * self.world_scale
        return np.array([int(u), int(v)])

    def _draw_point(self, world_pos, color, radius):
        screen_pos = self._to_screen(world_pos)
        self.viewport.draw_point(screen_pos, color, radius)
        return screen_pos

    # ---------------------------------------------------------------- 内容
    def _draw_origin(self):
        vp = self.viewport
        origin = self._to_screen(np.zeros(3))
        pg.draw.circle(vp.screen, _ORIGIN_FG, origin, 9)
        pg.draw.circle(vp.screen, _ORIGIN_BG, origin, 9, 2)

    def _draw_robots(self, robots, controlled):
        c = self.resources.c
        selected_camera = controlled.get_camera() if controlled is not None else None
        # 相机位置（悬挂点）
        for robot in robots:
            camera = robot.get_camera()
            color = c.camera if camera == selected_camera else c.camera_other
            self._draw_point(camera.world_pos, color, 6)
        # 车体 + 装甲板
        for robot in robots:
            alive = robot.health.alive
            self._draw_point(robot.chassis.world_pos,
                             c.robot_alive if alive else c.robot_dead, 6)
            armor_color = c.armor if alive else c.robot_dead
            for armor in robot.get_armors():
                self._draw_point(armor.world_pos,
                                 c.white if alive else c.robot_dead, 4)
                self._draw_armor_box(armor, armor_color)
                for corner in armor.light_corners:
                    self._draw_point(corner, armor_color, 4)

    def _draw_outpost(self, outpost):
        if outpost is None:
            return
        c = self.resources.c
        self._draw_point(outpost.world_pos, c.robot_alive, 8)
        for armor in outpost.armors:
            self._draw_point(armor.world_pos, c.white, 4)
            self._draw_armor_box(armor, c.armor)
            for corner in armor.light_corners:
                self._draw_point(corner, c.armor, 3)

    def _draw_rune(self, rune):
        if rune is None:
            return
        c = self.resources.c
        vp = self.viewport
        center_screen = self._draw_point(rune.world_pos, c.white, 8)
        base_world = np.asarray(rune.world_pos, float) - np.array(
            [0.0, 0.0, rune.cfg.height])
        base_screen = self._to_screen(base_world)
        pg.draw.line(vp.screen, _RUNE_BASE_LINE, base_screen, center_screen, 3)

        for i, blade in enumerate(rune.blades):
            if rune.activatable[i]:
                blade_color = c.observation
            elif rune.state == "activated":
                blade_color = c.rune_activated
            elif rune.hit_rings(i):
                blade_color = _RUNE_HIT_BAND
            else:
                blade_color = c.armor
            blade_screen = self._to_screen(blade.world_pos)
            pg.draw.line(vp.screen, _RUNE_SPOKE, center_screen, blade_screen, 2)
            radius_px = int(max(blade.hit_radius * self.world_scale, 4))
            pg.draw.circle(vp.screen, blade_color, blade_screen, radius_px)
            pg.draw.circle(vp.screen, blade_color, blade_screen, radius_px, 2)
            if rune.activatable[i]:
                pg.draw.circle(vp.screen, c.rune_activatable,
                               blade_screen, radius_px + 5, 2)
                font = self.resources.info_font(14)
                vp.screen.blit(
                    font.render("可激活", True, c.rune_activatable),
                    (int(blade_screen[0]) + radius_px + 8,
                     int(blade_screen[1]) - 8))
            self._draw_arrow(blade.world_pos,
                             np.asarray(blade.world_R)[:, 0],
                             blade_color, length=0.35, width=2)

    def _draw_observations(self, obs, controlled):
        if obs is None or controlled is None:
            return
        for armor in obs.obs_armors:
            world_pos = rel_to_world(armor.rel_pos, self.frame, controlled)
            normal = rpy_to_R(armor.rel_rpy)[:, 0]
            self._draw_arrow(world_pos, normal, self.resources.c.observation,
                             length=0.2, width=2)

    def _draw_predictions(self, pred, controlled):
        if pred is None or not pred.is_tracking or controlled is None:
            return
        if pred.center is not None:
            world_pos = rel_to_world(pred.center, self.frame, controlled)
            self._draw_point(world_pos, self.resources.c.pred_center, 6)
        for armor_pos in pred.armors:
            world_pos = rel_to_world(armor_pos, self.frame, controlled)
            self._draw_point(world_pos, self.resources.c.prediction, 4)

    def _draw_bullets(self, bullets):
        if not bullets:
            return
        c = self.resources.c
        for bullet in bullets:
            if bullet.hit:
                if bullet.hit_pos is None:
                    continue
                center = self._draw_point(bullet.hit_pos, c.hit, 0)
                self.viewport.draw_hit_cross(center, c.hit)
            elif bullet.active:
                self._draw_point(bullet.world_pos, c.bullet, 3)

    def _draw_camera_fov(self, camera):
        c = self.resources.c
        vp = self.viewport
        cam_screen = self._draw_point(camera.world_pos, c.camera, 8)
        forward = camera.get_forward_vec()
        forward_screen = self._to_screen(
            np.asarray(camera.world_pos, float) + forward)
        pg.draw.line(vp.screen, c.camera, cam_screen, forward_screen, 3)

        fov = camera.fov
        max_range = camera.max_range
        forward_yaw = math.atan2(forward[1], forward[0])

        fan_segments = 20
        vertexes = [cam_screen]
        for i in range(fan_segments + 1):
            angle = forward_yaw - fov / 2 + (fov / fan_segments) * i
            wp = np.asarray(camera.world_pos, float).copy()
            wp[0] += math.cos(angle) * max_range
            wp[1] += math.sin(angle) * max_range
            vertexes.append(self._to_screen(wp))

        if len(vertexes) >= 3:
            rel_points = [(p[0] - vp.main_rect.left, p[1] - vp.main_rect.top)
                          for p in vertexes]
            vp.fill_polygon_alpha(vp.main_alpha, vp.main_rect, rel_points,
                                  (0, 255, 255, 30))
            pg.draw.lines(vp.screen, c.camera, True, vertexes[1:], 2)
            mid_wp = np.asarray(camera.world_pos, float).copy()
            mid_wp[0] += math.cos(forward_yaw) * max_range
            mid_wp[1] += math.sin(forward_yaw) * max_range
            pg.draw.line(vp.screen, c.camera, cam_screen,
                         self._to_screen(mid_wp), 1)

    # ---------------------------------------------------------------- 原语
    def _draw_armor_box(self, armor, color):
        vp = self.viewport
        screen_points = [self._to_screen(corner) for corner in armor.light_corners]
        if len(screen_points) != 4:
            return
        rel_points = [(p[0] - vp.main_rect.left, p[1] - vp.main_rect.top)
                      for p in screen_points]
        vp.fill_polygon_alpha(vp.main_alpha, vp.main_rect, rel_points,
                              (*tuple(color)[:3], 80))
        pg.draw.polygon(vp.screen, color, screen_points, 2)

    def _draw_arrow(self, world_pos, world_dir, color, length=0.28, width=3):
        if world_pos is None:
            return
        world_dir = np.asarray(world_dir, float)
        norm = float(np.linalg.norm(world_dir))
        if norm < 1e-9:
            return
        start = self._to_screen(world_pos)
        end_world = np.asarray(world_pos, float) + world_dir / norm * length
        end = self._to_screen(end_world)
        self.viewport.draw_arrow_head(start, end, color, width)