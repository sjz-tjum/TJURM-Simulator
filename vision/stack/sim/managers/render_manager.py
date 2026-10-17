# sim/managers/render_manager.py
"""RenderManager：渲染协调器。

持有 viewport / bus_cache / resources 与四个视图，show() 时依次调用。
自己不做渲染、不做投影、不做布局。

内部视图住在 sim/render/ —— 那些是它的实现，不对外暴露。
"""
from __future__ import annotations

from config import cfg
from sim.render.bus_cache import BusCache
from sim.render.camera_view import CameraView
from sim.render.curve_panel import CurvePanel
from sim.render.info_panel import InfoPanel
from sim.render.main_view import MainView
from sim.render.resources import Resources
from sim.render.viewport import Viewport


class RenderManager:
    def __init__(self, robot_manager, device_manager, bullet_manager):
        ui = cfg.ui
        self.robot_manager = robot_manager
        self.device_manager = device_manager
        self.bullet_manager = bullet_manager

        frame = str(cfg.sim.frame).upper()
        world_scale = float(cfg.sim.world_scale)

        self.viewport = Viewport(ui)
        self.bus_cache = BusCache()
        self.resources = Resources(ui.colors)

        self.main_view = MainView(self.viewport, self.resources, world_scale, frame)
        self.camera_view = CameraView(self.viewport, self.resources, frame)
        self.info_panel = InfoPanel(self.viewport, self.resources)
        self.curve_panel = CurvePanel(self.viewport, self.resources)

        self._probe = {"snapshot": 0.0, "fill": 0.0, "main": 0.0,
                       "camera": 0.0, "info": 0.0, "curve": 0.0}
        self._probe_n = 0

    # ---------------------------------------------------------------- 主循环
    def show(self):
        """画一帧。Runner 负责 pg.display.flip()，这里只画不翻页。"""
        obs, pred, stats, plot, texts = self.bus_cache.snapshot()
        controlled = self.robot_manager.controlled_robot
        self.curve_panel.sample(plot, controlled)

        self.viewport.screen.fill((0, 0, 0))

        robots = list(self.robot_manager.robots)
        camera = controlled.get_camera() if controlled is not None else None
        bullets = self.bullet_manager.bullets
        outpost = self.device_manager.outpost
        rune = self.device_manager.rune

        self.main_view.draw(robots, controlled, outpost, rune, bullets, obs, pred)
        self.camera_view.draw(robots, controlled, outpost, rune, bullets, obs, pred)
        self.info_panel.draw(robots, outpost, rune, obs, pred, stats, texts)
        if self.curve_panel.visible:
            self.curve_panel.draw()

    # ---------------------------------------------------------------- 生命周期
    def reset(self):
        self.bus_cache.reset()
        self.main_view.reset()
        self.camera_view.reset()
        self.info_panel.reset()
        self.curve_panel.reset()

    def switch_curve_panel(self):
        return self.curve_panel.switch_visible()

    def cycle_curve_preset(self):
        return self.curve_panel.cycle_preset()