# sim/render/camera_view.py
"""CameraView：相机针孔投影画面。

针孔投影 + 像面 letterbox 居中。显示车 / 前哨站 / 符靶面 / 观测 / 预测 / 弹丸。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pygame as pg

from core.algorithms.math.transform import (
    camera_RDF_to_normalized,
    normalized_RDF_to_pixel,
    rpy_to_R,
    vec3d_FLU2RDF,
)
from sim.render.helpers import rel_to_world

_BG = (5, 5, 5)
_IMAGE_BG = (40, 44, 50)
_IMAGE_BORDER = (110, 116, 126)
_LABEL_FG = (150, 158, 168)

_RUNE_RIM = (120, 126, 134)
_RUNE_RING = (74, 80, 90)
_RUNE_HIT_BAND = (255, 60, 60)
_RUNE_MARK_DIM = (96, 44, 44)
_RUNE_FAR_DOT = (150, 156, 166)


@dataclass(frozen=True)
class CameraFrame:
    """一帧的相机不变量：世界位姿 + 内参 + 面板缩放。

    符靶面一帧要投上百个点，这些量每帧只变一次 —— 提前算好、传引用，
    避免在批量投影里反复重算。
    """
    R_body: np.ndarray
    world_pos: np.ndarray
    K: np.ndarray
    scale: float


class CameraView:
    def __init__(self, viewport, resources, frame):
        self.viewport = viewport
        self.resources = resources
        self.frame = frame

    def reset(self):
        pass

    # ---------------------------------------------------------------- 主入口
    def draw(self, robots, controlled, outpost, rune, bullets, obs, pred):
        vp = self.viewport
        rect = vp.camera_rect
        c = self.resources.c

        pg.draw.rect(vp.screen, _BG, rect)
        pg.draw.rect(vp.screen, c.white, rect, 2)

        if controlled is None:
            return
        camera = controlled.get_camera()
        if camera is None:
            return

        img_rect = self._image_rect(camera)
        pg.draw.rect(vp.screen, _IMAGE_BG, img_rect)
        pg.draw.rect(vp.screen, _IMAGE_BORDER, img_rect, 1)
        w_img, h_img = camera.image_size
        font = self.resources.info_font(13)
        vp.screen.blit(font.render(f"camera {w_img}x{h_img}", True, _LABEL_FG),
                       (img_rect.left + 2, img_rect.bottom + 1))

        vp.set_clip(img_rect)
        self._draw_robots(robots, camera)
        self._draw_outpost(outpost, camera)
        self._draw_rune(rune, camera)
        self._draw_observations(obs, controlled, camera)
        self._draw_predictions(pred, controlled, camera)
        self._draw_bullets(bullets, camera)
        vp.clear_clip()

    # ---------------------------------------------------------------- 投影
    def _image_rect(self, camera):
        vp = self.viewport
        rect = vp.camera_rect
        w_img, h_img = camera.image_size
        scale = min(rect.width / float(w_img), rect.height / float(h_img))
        img_w, img_h = int(w_img * scale), int(h_img * scale)
        return pg.Rect(rect.centerx - img_w // 2, rect.centery - img_h // 2,
                       img_w, img_h)

    def _camera_scale(self, camera):
        rect = self.viewport.camera_rect
        w_img, h_img = camera.image_size
        return min(rect.width / float(w_img), rect.height / float(h_img))

    def _make_camera_frame(self, camera) -> CameraFrame:
        return CameraFrame(
            R_body=rpy_to_R(camera.world_rpy),
            world_pos=np.asarray(camera.world_pos, float),
            K=camera.get_intrinsic_matrix(),
            scale=self._camera_scale(camera),
        )

    def _to_screen(self, world_pos, camera):
        """世界点 -> 相机面板坐标：整幅像面等比缩放进面板（越界返回 None）。"""
        uv = camera.world_to_pixel(world_pos)
        if uv is None:
            return None
        scale = self._camera_scale(camera)
        u = float(uv[0]) * scale + self.viewport.camera_center[0]
        v = float(uv[1]) * scale + self.viewport.camera_center[1]
        cam_rect = self.viewport.camera_rect
        if not (cam_rect.left <= u <= cam_rect.right
                and cam_rect.top <= v <= cam_rect.bottom):
            return None
        return np.array([int(u), int(v)])

    def _draw_point(self, world_pos, camera, color, radius):
        screen_pos = self._to_screen(world_pos, camera)
        if screen_pos is not None:
            self.viewport.draw_point(screen_pos, color, radius)
        return screen_pos

    # ---------------------------------------------------------------- 内容
    def _draw_robots(self, robots, camera):
        c = self.resources.c
        for robot in robots:
            self._draw_point(robot.chassis.world_pos, camera, c.robot_alive, 6)
            for armor in robot.get_armors():
                self._draw_point(armor.world_pos, camera, c.white, 4)
                self._draw_armor_box(armor, camera, c.armor)
                for corner in armor.light_corners:
                    self._draw_point(corner, camera, c.armor, 4)

    def _draw_outpost(self, outpost, camera):
        if outpost is None:
            return
        c = self.resources.c
        self._draw_point(outpost.world_pos, camera, c.robot_alive, 6)
        for armor in outpost.armors:
            self._draw_point(armor.world_pos, camera, c.white, 4)
            self._draw_armor_box(armor, camera, c.armor)
            for corner in armor.light_corners:
                self._draw_point(corner, camera, c.armor, 3)

    def _draw_rune(self, rune, camera):
        if rune is None:
            return
        c = self.resources.c
        cf = self._make_camera_frame(camera)
        self._draw_point(rune.world_pos, camera, c.white, 6)
        radius_m = rune.cfg.target_diameter / 2.0
        for i, blade in enumerate(rune.blades):
            self._draw_rune_blade(rune, blade, cf, radius_m,
                                  rune.activatable[i], rune.hit_rings(i))

    def _draw_observations(self, obs, controlled, camera):
        if obs is None:
            return
        for armor in obs.obs_armors:
            world_pos = rel_to_world(armor.rel_pos, self.frame, controlled)
            normal = rpy_to_R(armor.rel_rpy)[:, 0]
            self._draw_arrow(world_pos, normal, camera,
                             self.resources.c.observation, length=0.2, width=2)

    def _draw_predictions(self, pred, controlled, camera):
        if pred is None or not pred.is_tracking:
            return
        if pred.center is not None:
            world_pos = rel_to_world(pred.center, self.frame, controlled)
            self._draw_point(world_pos, camera, self.resources.c.pred_center, 6)
        for armor_pos in pred.armors:
            world_pos = rel_to_world(armor_pos, self.frame, controlled)
            self._draw_point(world_pos, camera, self.resources.c.prediction, 4)

    def _draw_bullets(self, bullets, camera):
        if not bullets:
            return
        c = self.resources.c
        for bullet in bullets:
            if bullet.hit:
                if bullet.hit_pos is None:
                    continue
                center = self._draw_point(bullet.hit_pos, camera, c.hit, 0)
                if center is not None:
                    self.viewport.draw_hit_cross(center, c.hit)
            elif bullet.active:
                self._draw_point(bullet.world_pos, camera, c.bullet, 3)

    # ---------------------------------------------------------------- 原语
    def _draw_armor_box(self, armor, camera, color):
        vp = self.viewport
        screen_points = []
        for corner in armor.light_corners:
            pos = self._to_screen(corner, camera)
            if pos is None:
                return
            screen_points.append(pos)
        if len(screen_points) == 4:
            rel_points = [(p[0] - vp.camera_rect.left, p[1] - vp.camera_rect.top)
                          for p in screen_points]
            vp.fill_polygon_alpha(vp.camera_alpha, vp.camera_rect, rel_points,
                                  (*tuple(color)[:3], 80))
            pg.draw.polygon(vp.screen, color, screen_points, 2)

    def _draw_arrow(self, world_pos, world_dir, camera, color,
                    length=0.28, width=3):
        if world_pos is None:
            return
        world_dir = np.asarray(world_dir, float)
        norm = float(np.linalg.norm(world_dir))
        if norm < 1e-9:
            return
        start = self._to_screen(world_pos, camera)
        end = self._to_screen(np.asarray(world_pos, float) + world_dir / norm * length,
                              camera)
        if start is None or end is None:
            return
        self.viewport.draw_arrow_head(start, end, color, width)

    # ---------------------------------------------------------------- 符靶面
    def _blade_points_screen(self, blade, locals_, cf: CameraFrame):
        """把灯臂平面内的一组点（板系坐标）批量投影到面板。

        world -> camera（FLU）-> RDF -> 归一化 -> 像素 都调 transform 的函数；
        批量是行向量形式，逐点调单点函数会慢一个量级，故就地展开。
        cf 提供这一帧的相机不变量 —— 一次投影上百个点，不重复算它们。
        """
        R_blade = np.asarray(blade.world_R, float)
        world = np.asarray(blade.world_pos, float) + np.asarray(locals_, float) @ R_blade.T

        rel = world - cf.world_pos
        flu = rel @ cf.R_body
        rdf = vec3d_FLU2RDF(flu)

        z = rdf[:, 2]
        if np.any(z <= 1e-6):
            return None
        n = camera_RDF_to_normalized(rdf)
        uv = normalized_RDF_to_pixel(n, K=cf.K)

        cam_rect = self.viewport.camera_rect
        center = self.viewport.camera_center
        xs = uv[:, 0] * cf.scale + center[0]
        ys = uv[:, 1] * cf.scale + center[1]
        if np.any(xs < cam_rect.left) or np.any(xs > cam_rect.right):
            return None
        if np.any(ys < cam_rect.top) or np.any(ys > cam_rect.bottom):
            return None
        return [(int(x), int(y)) for x, y in zip(xs, ys)]

    def _blade_circle_screen(self, blade, radius, cf: CameraFrame, segments=10):
        angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
        locals_ = np.column_stack([np.zeros(segments),
                                   radius * np.cos(angles),
                                   radius * np.sin(angles)])
        return self._blade_points_screen(blade, locals_, cf)

    def _draw_rune_blade(self, rune, blade, cf: CameraFrame, radius_m,
                         activatable, hit_rings):
        vp = self.viewport
        c = self.resources.c
        full = radius_m
        rim = self._blade_circle_screen(blade, full, cf)
        if rim is None:
            return
        span = max(max(p[0] for p in rim) - min(p[0] for p in rim),
                   max(p[1] for p in rim) - min(p[1] for p in rim))
        if span < 6:
            pg.draw.circle(vp.screen,
                           c.rune_activatable if activatable else _RUNE_FAR_DOT,
                           rim[0], 2)
            return
        pg.draw.polygon(vp.screen, _RUNE_RIM, rim, 1)

        ring_count = int(rune.cfg.ring_count)
        r_in = rune.cfg.ring_inner_diameter / 2.0
        step = (full - r_in) / max(ring_count - 1, 1)
        px_per_m = max(span, 1.0) / (2.0 * full)
        hit_set = set(hit_rings or ())

        for ring in range(1, ring_count + 1):
            r_outer = r_in + (ring_count - ring) * step
            r_inner = r_in + (ring_count - ring - 1) * step if ring < ring_count else 0.0
            if ring in hit_set:
                band = max(r_outer - r_inner, 3.0 / px_per_m)
                band = min(band, r_outer)
                seg = 16
                outer_pts = self._blade_circle_screen(blade, r_outer, cf, segments=seg)
                inner_pts = self._blade_circle_screen(blade, max(r_outer - band, 0.0),
                                                      cf, segments=seg)
                if outer_pts is not None and inner_pts is not None:
                    for k in range(len(outer_pts)):
                        k2 = (k + 1) % len(outer_pts)
                        pg.draw.polygon(vp.screen, _RUNE_HIT_BAND,
                                        [outer_pts[k], outer_pts[k2],
                                         inner_pts[k2], inner_pts[k]], 0)
                elif outer_pts is not None:
                    pg.draw.polygon(vp.screen, _RUNE_HIT_BAND, outer_pts, 0)
                continue
            arc = self._blade_circle_screen(blade, r_outer, cf, segments=8)
            if arc is not None:
                pg.draw.lines(vp.screen, _RUNE_RING, True, arc, 1)

        mark_color = _RUNE_HIT_BAND if activatable else _RUNE_MARK_DIM
        for d_out, d_in in rune.cfg.mark_bands:
            for diameter in (d_out, d_in):
                band_rim = self._blade_circle_screen(blade, diameter / 2000.0, cf,
                                                     segments=8)
                if band_rim is not None:
                    pg.draw.lines(vp.screen, mark_color, True, band_rim, 1)
        r_in_cross = rune.cfg.mark_cross_inner_diameter / 2000.0
        half = math.radians(rune.cfg.mark_cross_half_angle)
        locals_cross = []
        for k in range(4):
            base = k * math.pi / 2.0
            for radius, angle in ((full, base - half), (full, base + half),
                                  (r_in_cross, base + half), (r_in_cross, base - half)):
                locals_cross.append([0.0, radius * math.cos(angle),
                                     radius * math.sin(angle)])
        quads = self._blade_points_screen(blade, locals_cross, cf)
        if quads is not None:
            for k in range(4):
                pg.draw.polygon(vp.screen, mark_color, quads[4 * k:4 * k + 4],
                                0 if activatable else 1)
        if activatable:
            halo = self._blade_circle_screen(blade, full * 1.2, cf, segments=8)
            if halo is not None:
                pg.draw.lines(vp.screen, c.rune_activatable, True, halo, 2)