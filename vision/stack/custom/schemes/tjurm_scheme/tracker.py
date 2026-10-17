# custom/schemes/tjurm_scheme/tracker.py
"""多目标跟踪器：马氏门控分配 + 装甲板 EKF + 一发解。

I/O 协议（core 契约）：
    输入   Observation + SelfState
    输出   List[Prediction]，第一条是"该打谁"
    失败   不产出 Prediction

一发解（model→select→traj 迭代 + fire/mode 判定 + 前馈）封装在 ShotSolver。
tracker.step 只调 ShotSolver.solve 一次，拿回 solution + 瞄点。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from config import cfg
from core.algorithms.math.angle import limit_rad
from core.bus import public_bus
from core.bus.dataflow import GimbalTarget, Prediction
from core.bus.messages import PublicMessages
from core.bus.public_bus import debug_plot, debug_text
from core.entities.property.entity_types import EntityType, TargetType
from core.interfaces.tracker import Tracker, register_tracker

from custom.components.armor_selectors.tjurm_armor_selector import ArmorSelector
from custom.components.shooters.tjurm_shooter import ShootDecider
from custom.components.traj_solvers.tjurm_traj_solver import TjurmTrajSolver
from custom.schemes.tjurm_scheme.mpc.controller import MPCController
from custom.schemes.tjurm_scheme.mpc.reference import ReferenceGenerator
from custom.schemes.tjurm_scheme.pool import MultiTargetTracker
from custom.schemes.tjurm_scheme.shot_solver import ShotSolver


@dataclass
class MultiTargetParams:
    # 决策参数
    spin_thresh: float = 3.0              # 车 auto / single 模式判定的 ω 阈值
    lock_bias: float = 0.2                # 选板的锁定偏置
    armor_fire_cos: float = 0.8           # auto / 前哨："板朝向自车"的 cos 阈值
    virtual_fire_thresh: float = 0.3      # 车 single："板接近虚拟点"的角阈值（rad）
    # 时序与 MPC 参数
    dt: float = 0.01
    horizon: int = 10
    yaw_offset: float = 0.0
    q_theta: float = 100.0
    q_dtheta: float = 10.0
    r_alpha: float = 0.01
    alpha_max: float = 35.0
    theta_min: float = -1e5
    theta_max: float = 1e5
    enable_mpc: bool = False
    publish_plot: bool = True
    center_bucket_px: float = 45.0


class _GimbalView:
    def __init__(self, gimbal_state):
        self.world_rpy = np.asarray(gimbal_state.rpy, float)
        self.world_omg = np.asarray(gimbal_state.omg, float)


class _SelfView:
    """把 core 的 SelfState 适配成自瞄认的自车口径。"""

    def __init__(self, self_state, frame):
        self.state = self_state
        self.frame = frame
        self.gimbal = _GimbalView(self_state.gimbal)

    def _rel(self, pos):
        pos = np.asarray(pos, float)
        if self.frame == "W":
            return pos
        return pos - np.asarray(self.state.gimbal.pos, float)

    def get_muzzle_rel_pos(self) -> np.ndarray:
        return self._rel(self.state.muzzle.pos)

    def get_gimbal_rel_pos(self) -> np.ndarray:
        return self._rel(self.state.gimbal.pos)

    def get_camera_rel_pos(self) -> np.ndarray:
        return self._rel(self.state.camera.pos)


@register_tracker("multi_target", aliases=["demo5", "default"])
class MultiTargetTrackerNode(Tracker):
    description = "多目标跟踪（马氏门控 + 装甲板 EKF + 射击决策；MPC 默认不启用）"

    def __init__(self, params: Optional[MultiTargetParams] = None):
        self.params = params or MultiTargetParams()
        self.frame = str(cfg.sim.frame).upper()
        if self.frame not in ("G", "W"):
            raise ValueError(f"Sim.frame 必须是 'G' 或 'W'，实得 {self.frame!r}")

        self.total_delay = 0.2
        self.multi_tracker = MultiTargetTracker(
            center_bucket_px=self.params.center_bucket_px)

        self.armor_selector = self._new_selector()
        self.robot = None

        self.shoot_decider = ShootDecider(
            spin_thresh=self.params.spin_thresh,
            armor_fire_cos=self.params.armor_fire_cos,
            virtual_fire_thresh=self.params.virtual_fire_thresh,
        )
        self.traj_solver = TjurmTrajSolver()
        self.shot_solver = ShotSolver(
            shoot_decider=self.shoot_decider,
            traj_solver=self.traj_solver,
            armor_selector=self.armor_selector,
            frame=self.frame,
        )
        self.ref_gen = ReferenceGenerator(
            None,
            self._new_selector(),
            dt=self.params.dt,
            N=self.params.horizon,
            yaw_offset=self.params.yaw_offset,
        )
        self.mpc_yaw = MPCController(
            dt=self.params.dt,
            N=self.params.horizon,
            q_theta=self.params.q_theta,
            q_dtheta=self.params.q_dtheta,
            r_alpha=self.params.r_alpha,
            alpha_max=self.params.alpha_max,
            theta_min=self.params.theta_min,
            theta_max=self.params.theta_max,
        )

        self.obs_armors = None
        self.timestamp = 0.

    def _new_selector(self) -> ArmorSelector:
        return ArmorSelector(spin_branch_thresh=self.params.spin_thresh,
                             lock_bias=self.params.lock_bias)

    @staticmethod
    def _clear_selector(sel: ArmorSelector) -> None:
        sel.locked_armor_id = None
        sel.selected_armor_id = None

    def reset(self) -> None:
        self.multi_tracker = MultiTargetTracker(
            center_bucket_px=self.params.center_bucket_px)
        self._clear_selector(self.armor_selector)
        self._clear_selector(self.ref_gen.selector)
        self.obs_armors = None
        self.timestamp = 0.
        self.total_delay = 0.2
        self.shot_solver.reset()

    # ------------------------------------------------------------------ 主流程
    def step(self, obs, self_state) -> List[Prediction]:
        if obs is None:
            obs_list = []
        else:
            obs_list = list(obs.obs_armors or [])
            self.timestamp = float(obs.timestamp)

        self.robot = _SelfView(self_state, self.frame)
        self.ref_gen.robot = self.robot
        self.obs_armors = obs_list
        timestamp = self.timestamp

        self.multi_tracker.push_observation(self.obs_armors, timestamp)
        best_target = self.multi_tracker.get_best_target()
        if best_target:
            debug_text(f'jumped: {best_target.jumped}')

        gimbal_rel_pos = self.robot.get_gimbal_rel_pos()

        if best_target is None or not best_target.ekf.is_init:
            if self.params.publish_plot:
                debug_plot(timestamp=timestamp, obs_yaw=None, obs_pitch=None,
                           pred_yaw=None, pred_pitch=None, mpc_yaw=None,
                           actual_yaw=None, actual_pitch=None, ballistic_pitch=None)
            return []

        bundle = self.shot_solver.solve(best_target, self_state, timestamp)
        self.total_delay = bundle.fly

        if bundle.solution is None:
            if self.params.publish_plot:
                debug_plot(timestamp=timestamp, obs_yaw=None, obs_pitch=None,
                           pred_yaw=None, pred_pitch=None, mpc_yaw=None,
                           actual_yaw=None, actual_pitch=None, ballistic_pitch=None)
            return []

        public_bus().publish(PublicMessages.CONTROL, bundle.solution)

        predictions = self._build_predictions(bundle, best_target, timestamp)

        # MPC（可选，默认关）
        gimbal_yaw = float(self.robot.gimbal.world_rpy[2])
        gimbal_omg = float(self.robot.gimbal.world_omg[2])
        x0_yaw = np.array([gimbal_yaw, gimbal_omg])
        omega_des, alpha_yaw = None, None
        if self.params.enable_mpc:
            theta_ref, omega_ref = self.ref_gen.generate(
                best_target, timestamp, self.total_delay)
            if len(theta_ref) > 0:
                k = int(np.round((theta_ref[0] - gimbal_yaw) / (2 * np.pi)))
                adjusted = theta_ref[0] - 2 * np.pi * k
                if abs(adjusted - gimbal_yaw) > np.pi:
                    k += 1 if adjusted > gimbal_yaw else -1
                theta_ref = theta_ref - 2 * np.pi * k
            theta_des, omega_des, alpha_yaw = self.mpc_yaw.solve(
                x0=x0_yaw, theta_ref=theta_ref, omega_ref=omega_ref)

        obs_yaw, obs_pitch = self._calc_obs_angle()
        pred_yaw, pred_pitch = self._calc_pred_angle(bundle, gimbal_rel_pos)

        if self.params.enable_mpc and alpha_yaw is not None and predictions:
            predictions[0].gimbal_target = GimbalTarget(
                yaw=float(pred_yaw), yaw_rate=float(omega_des),
                yaw_acc=float(alpha_yaw), pitch=float(bundle.solution.pitch),
                pitch_rate=0.0, pitch_acc=0.0)

        if self.params.publish_plot and self.obs_armors:
            debug_plot(
                timestamp=timestamp,
                obs_yaw=self.obs_armors[0].rel_rpy[2], obs_pitch=obs_pitch,
                pred_yaw=pred_yaw, pred_pitch=pred_pitch,
                mpc_yaw=limit_rad(pred_yaw) if pred_yaw is not None else None,
                actual_yaw=limit_rad(gimbal_yaw),
                actual_pitch=float(self.robot.gimbal.world_rpy[1]),
                ballistic_pitch=float(bundle.solution.pitch))

        return predictions

    def _calc_obs_angle(self):
        if not self.obs_armors:
            return None, None
        obs = self.obs_armors[0]
        gimbal_rel_pos = self.robot.get_gimbal_rel_pos()
        dx = obs.rel_pos[0] - gimbal_rel_pos[0]
        dy = obs.rel_pos[1] - gimbal_rel_pos[1]
        dz = obs.rel_pos[2] - gimbal_rel_pos[2]
        yaw = np.arctan2(dy, dx)
        pitch = -np.arctan2(dz, np.sqrt(dx * dx + dy * dy))
        return yaw, pitch

    def _calc_pred_angle(self, bundle, gimbal_rel_pos):
        if bundle.pred_armor is None:
            return None, None
        pred_armor = bundle.pred_armor
        dx = pred_armor[0] - gimbal_rel_pos[0]
        dy = pred_armor[1] - gimbal_rel_pos[1]
        dz = pred_armor[2] - gimbal_rel_pos[2]
        yaw = np.arctan2(dy, dx)
        pitch = -np.arctan2(dz, np.sqrt(dx * dx + dy * dy))
        return yaw, pitch

    @staticmethod
    def _target_type(target):
        kind = getattr(target, "robot_type", None)
        if kind == EntityType.Rune:
            return TargetType.RUNE
        if kind == EntityType.Outpost:
            return TargetType.OUTPOST
        return TargetType.ROBOT

    def _build_predictions(self, bundle, target, timestamp) -> List[Prediction]:
        if bundle.pred_armor is None:
            return []
        sol = bundle.solution
        pred = Prediction(
            target_type=self._target_type(target),
            center=bundle.pred_center,
            armors=[bundle.pred_armor],
            timestamp=timestamp,
            is_tracking=True,
            aim_rate=np.array([sol.yaw_rate, sol.pitch_rate]),
        )
        return [pred]