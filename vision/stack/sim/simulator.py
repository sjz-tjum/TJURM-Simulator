# sim/simulator.py
"""仿真器：装配所有 manager，驱动主循环。"""
import numpy as np

from config import cfg
from core.bus import sys_bus
from core.bus.messages import SysMessages
from pygame.time import Clock as WallClock
from sim.clock import SimClock
from sim.comm import SimComm
from sim.managers.registry_manager import RegistryManager
from sim.managers.motion_manager import MotionManager
from sim.managers.robot_manager import RobotManager
from sim.managers.device_manager import DeviceManager
from sim.managers.bullet_manager import BulletManager
from sim.managers.sensor_manager import SensorManager
from sim.managers.control_manager import ControlManager
from sim.managers.render_manager import RenderManager


class Simulator:
    def __init__(self, comm=None):
        self.comm = comm if comm is not None else SimComm()

        self.wall_clock = WallClock()
        self.clock = SimClock()
        self.sim_fps = int(cfg.sim.sim_fps)

        self._root_seed = cfg.sim.seed
        self._reseed()

        self.registry = RegistryManager()
        self.motion_manager = MotionManager(self.registry)
        self.robot_manager = RobotManager(self.registry)
        self.device_manager = DeviceManager(self.registry, rng=self.rune_rng)
        self.motion_manager.set_controlled_getter(
            lambda: (self.robot_manager.controlled_robot.chassis
                     if self.robot_manager.controlled_robot else None))

        self.bullet_manager = BulletManager(self.robot_manager, self.device_manager)
        self.sensor_manager = SensorManager(self.robot_manager, self.device_manager,
                                            rng=self.sensor_rng)
        self.control_manager = ControlManager(self.robot_manager, self.bullet_manager)
        self.render_manager = RenderManager(
            self.robot_manager, self.device_manager, self.bullet_manager)

    def _reseed(self):
        ss = np.random.SeedSequence(self._root_seed)
        sensor_ss, rune_ss = ss.spawn(2)
        self.sensor_rng = np.random.default_rng(sensor_ss)
        self.rune_rng = np.random.default_rng(rune_ss)

    @property
    def sim_time(self):
        return self.clock.sim_time

    @property
    def dt(self):
        return self.clock.dt

    def collect_self_state(self):
        from core.bus.dataflow import RigidState, CameraState, SelfState
        robot = self.robot_manager.controlled_robot
        if robot is None:
            zeros = np.zeros(3)
            eye = np.eye(3)
            return SelfState(
                chassis=RigidState(zeros, zeros, zeros, zeros),
                gimbal=RigidState(zeros, zeros, zeros, zeros),
                muzzle=RigidState(zeros, zeros, zeros, zeros),
                camera=CameraState(zeros, zeros, eye))
        chassis = robot.chassis
        gimbal = robot.gimbal
        muzzle = robot.get_muzzle()
        camera = robot.get_camera()
        return SelfState(
            chassis=RigidState(chassis.world_pos.copy(), chassis.world_rpy.copy(),
                               chassis.world_vel.copy(), chassis.world_omg.copy()),
            gimbal=RigidState(gimbal.world_pos.copy(), gimbal.world_rpy.copy(),
                              gimbal.world_vel.copy(), gimbal.world_omg.copy()),
            muzzle=RigidState(muzzle.world_pos.copy(), muzzle.world_rpy.copy(),
                              muzzle.world_vel.copy(), muzzle.world_omg.copy()),
            camera=CameraState(camera.world_pos.copy(), camera.world_rpy.copy(),
                               camera.get_intrinsic_matrix()))

    def run(self):
        self.update()
        self.render_manager.show()

    def update(self):
        elapsed_ms = self.wall_clock.tick(self.sim_fps)
        dt = self.clock.update_from_ms(elapsed_ms)
        self._advance(dt)

    def _advance(self, dt):
        t = self.sim_time
        self.motion_manager.update(dt, t)

        # 电控：从 comm 拿 custom 发来的最新控制指令
        solution = self.comm.latest_control()
        self.control_manager.update(t, solution)

        self.bullet_manager.update(dt, t)

        # 传感器出帧时，把检测 + 同刻自车状态发给 custom
        frame = self.sensor_manager.get_obs(self.sim_time)
        if frame is not None:
            self.comm.publish_frame(frame, self.collect_self_state())

    def reset(self):
        self._reseed()
        self.sensor_manager.set_rng(self.sensor_rng)
        self.device_manager.set_rng(self.rune_rng)

        self.clock.reset()
        self.motion_manager.reset()
        self.robot_manager.reset()
        self.device_manager.reset()
        self.bullet_manager.reset()
        self.sensor_manager.reset()
        self.control_manager.reset()
        self.render_manager.reset()
        self.comm.reset()

        # keeper 不归 Simulator 持有，复位靠总线通知
        sys_bus().publish(SysMessages.RESET, None)