# custom/schemes/tjurm_scheme/pool.py
"""多目标池：目标生命周期管理 + 数据关联。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from config import cfg
from core.entities.property.entity_types import EntityType
from custom.schemes.tjurm_scheme.models.armor_model import DemoModel5
from custom.schemes.tjurm_scheme.models.outpost_model import OutpostModel
from custom.schemes.tjurm_scheme.models.rune_model import RuneModel


def target_priority(entity_type) -> int:
    """车型权重（越小越优先）。"""
    try:
        name = EntityType.get_name(entity_type)
    except ValueError:
        return 99
    if entity_type is EntityType.Outpost:
        return int(cfg.outpost.priority)
    if entity_type is EntityType.Rune:
        return int(cfg.rune.priority)
    node = cfg.robot[name]
    return int(getattr(node, "priority", 99)) if node is not None else 99


def obs_center_px(obs) -> float:
    """观测板心到画面中心的像素距离（本项目主点即像面原点）。"""
    pts = getattr(obs, "image_points_2d", None)
    if pts is None:
        return float("inf")
    pts = np.asarray(pts, float).reshape(-1, 2)
    if pts.size == 0:
        return float("inf")
    u, v = pts.mean(axis=0)
    return float(np.hypot(u, v))


class TrackedTarget:
    """一个被跟踪目标（含装甲板 EKF + 状态机）。"""

    def __init__(self, init_obs, t, *, min_detect_count: int = 3,
                 max_lost_count: int = 10, mahal_thresh: float = 9.49):
        if init_obs.entity_type == EntityType.Rune:
            self.ekf = RuneModel()
        elif init_obs.entity_type == EntityType.Outpost:
            self.ekf = OutpostModel()
        else:
            self.ekf = DemoModel5()
        self.ekf.init_model(init_obs)
        self.last_update = t
        self.update_count = 1
        self.robot_type = init_obs.entity_type
        self.priority = target_priority(init_obs.entity_type)
        self.center_px = obs_center_px(init_obs)
        self.last_match_id = 0
        self.jumped = False

        self.state = 'detecting'
        self.detect_count = 0
        self.lost_count = 0
        self.min_detect_count = int(min_detect_count)
        self.max_lost_count = int(max_lost_count)

        self.mahal_thresh = float(mahal_thresh)
        self.last_cost = float("nan")

    def predict(self, dt):
        if dt > 0:
            self.ekf.predict(dt)

    def update(self, obs, t, cost: float = float("nan")):
        if self.ekf.diverged():
            self._reset()
            return

        self.ekf.update(obs)
        self.jumped = (self.ekf.match_id != self.last_match_id)
        self.last_match_id = self.ekf.match_id
        self.last_update = t
        self.update_count += 1
        self.last_cost = float(cost)
        self.center_px = obs_center_px(obs)

        if self.state in ('lost', 'temp_lost'):
            self.state = 'detecting'
            self.detect_count = 1
            self.lost_count = 0
        elif self.state == 'detecting':
            self.detect_count += 1
            if self.detect_count >= self.min_detect_count:
                self.state = 'tracking'
                self.lost_count = 0
        elif self.state == 'tracking':
            self.lost_count = 0

    def on_missing(self):
        """本帧没有匹配到观测。"""
        self.lost_count += 1
        if self.lost_count > self.max_lost_count:
            self.state = 'lost'
        elif self.state == 'tracking':
            self.state = 'temp_lost'
        elif self.state == 'detecting':
            self.state = 'lost'

    def is_active(self):
        return self.state in ('detecting', 'tracking', 'temp_lost')

    def _reset(self):
        self.state = 'lost'
        self.detect_count = 0
        self.lost_count = 0
        self.last_match_id = 0
        self.ekf.is_init = False

    def association_cost(self, obs) -> float:
        d2, _ = self.ekf.mahalanobis_to(obs)
        return d2

    def geometric_distance(self, obs_pos) -> float:
        return self.ekf.get_geometric_distance(obs_pos)

    def get_pred_pos(self, dt, self_rel_pos, armor_selector, cur_dir=None):
        if not self.ekf.is_init:
            return np.zeros(3), np.zeros(3)
        x = self.ekf.ekf.x
        pred_cx = x[0] + x[3] * dt
        pred_cy = x[1] + x[4] * dt
        pred_cz = x[2] + x[5] * dt
        center = np.array([pred_cx, pred_cy, pred_cz])

        armor_id, armor_pos = armor_selector.select_armor(
            self, self_rel_pos, dt, cur_dir)
        return center, armor_pos

    def aim_velocity(self, armor_pos, dt: float) -> np.ndarray:
        """瞄准点的世界速度：模型自转（符绕 x、车绕 z）+ 平移。"""
        x = self.ekf.ekf.x
        omega = np.array([float(x[7]), 0.0, 0.0]) if self.robot_type == EntityType.Rune \
            else np.array([0.0, 0.0, float(x[7])])
        center = x[:3] + x[3:6] * dt
        return x[3:6] + np.cross(omega, np.asarray(armor_pos, float) - center)

    def snapshot(self) -> Dict:
        ekf = self.ekf
        return {
            "state": self.state,
            "robot_type": getattr(self.robot_type, "name", str(self.robot_type)),
            "update_count": self.update_count,
            "lost_count": self.lost_count,
            "match_id": ekf.match_id,
            "jumped": self.jumped,
            "last_cost": self.last_cost,
            "omega": float(ekf.ekf.x[7]) if ekf.is_init else 0.0,
            "radius": float(ekf.ekf.x[8]) if ekf.is_init and len(ekf.ekf.x) > 8 else 0.0,
            "nis_fail_rate": ekf.ekf.get_nis_fail_rate() if ekf.is_init else 0.0,
        }


@dataclass
class AssociationResult:
    matches: List[Tuple[int, int, float]] = field(default_factory=list)
    unmatched_obs: List[int] = field(default_factory=list)
    unmatched_targets: List[int] = field(default_factory=list)
    new_targets: List[int] = field(default_factory=list)

    def summary(self) -> str:
        return (f"matches={len(self.matches)} new={len(self.new_targets)} "
                f"miss={len(self.unmatched_targets)}")


class MultiTargetTracker:
    """多目标跟踪池：预测 -> 关联 -> 更新/新建 -> 淘汰。"""

    def __init__(self, max_lost_time: float = 0.5, mahal_thresh: float = 9.49,
                 geo_fallback: bool = True, geo_thresh: float = 0.45,
                 min_detect_count: int = 3, max_lost_count: int = 10,
                 center_bucket_px: float = 45.0):
        self.targets: List[TrackedTarget] = []
        self.max_lost_time = max_lost_time
        self.mahal_thresh = float(mahal_thresh)
        self.geo_fallback = bool(geo_fallback)
        self.geo_thresh = float(geo_thresh)
        self.min_detect_count = int(min_detect_count)
        self.max_lost_count = int(max_lost_count)
        self.center_bucket_px = float(center_bucket_px)
        self.last_association: Optional[AssociationResult] = None

    def _build_cost_matrix(self, obs_list, targets) -> np.ndarray:
        C = np.full((len(obs_list), len(targets)), np.inf)
        for i, obs in enumerate(obs_list):
            for j, tar in enumerate(targets):
                if tar.robot_type != obs.entity_type:
                    continue
                d2 = tar.association_cost(obs)
                if d2 <= self.mahal_thresh:
                    C[i, j] = d2
                elif self.geo_fallback:
                    geo = tar.geometric_distance(obs.rel_pos)
                    if geo <= self.geo_thresh:
                        C[i, j] = 2.0 * self.mahal_thresh + geo
        return C

    @staticmethod
    def _greedy_one_to_one(C: np.ndarray):
        m, n = C.shape
        pairs = [(C[i, j], i, j) for i in range(m) for j in range(n)
                 if np.isfinite(C[i, j])]
        pairs.sort(key=lambda x: x[0])
        used_i, used_j, out = set(), set(), []
        for cost, i, j in pairs:
            if i in used_i or j in used_j:
                continue
            used_i.add(i); used_j.add(j)
            out.append((i, j, cost))
        return out

    def push_observation(self, obs_list, t):
        obs_list = list(obs_list or [])

        for target in self.targets:
            target.predict(t - target.last_update)

        res = AssociationResult()

        if self.targets and obs_list:
            C = self._build_cost_matrix(obs_list, self.targets)
            for i, j, cost in self._greedy_one_to_one(C):
                self.targets[j].update(obs_list[i], t, cost=cost)
                res.matches.append((i, j, cost))

            matched_obs = {i for i, _, _ in res.matches}
            matched_tar = {j for _, j, _ in res.matches}
            res.unmatched_obs = [i for i in range(len(obs_list)) if i not in matched_obs]
            res.unmatched_targets = [j for j in range(len(self.targets))
                                     if j not in matched_tar]
        else:
            res.unmatched_obs = list(range(len(obs_list)))
            res.unmatched_targets = list(range(len(self.targets)))

        for j in res.unmatched_targets:
            self.targets[j].last_update = t
            self.targets[j].on_missing()

        for i in res.unmatched_obs:
            obs = obs_list[i]
            best_j, best_d2 = None, float("inf")
            for j, tar in enumerate(self.targets):
                if tar.robot_type != obs.entity_type:
                    continue
                d2 = tar.association_cost(obs)
                if d2 <= self.mahal_thresh and d2 < best_d2:
                    best_d2, best_j = d2, j
            if best_j is not None:
                self.targets[best_j].update(obs, t, cost=best_d2)
                res.matches.append((i, best_j, best_d2))
            else:
                self._create_target(obs, t)
                res.new_targets.append(i)

        self.targets = [tt for tt in self.targets if tt.state != 'lost']

        self.last_association = res
        return res

    def _create_target(self, obs, t):
        new_target = TrackedTarget(
            obs, t,
            min_detect_count=self.min_detect_count,
            max_lost_count=self.max_lost_count,
            mahal_thresh=self.mahal_thresh)
        self.targets.append(new_target)
        return new_target

    def get_best_target(self):
        """选出当前"最该打"的目标。"""
        state_priority = {'tracking': 0, 'detecting': 1, 'temp_lost': 2, 'lost': 3}
        active = [tt for tt in self.targets if tt.is_active()]
        if not active:
            return None

        def key(tt):
            return (state_priority.get(tt.state, 9),
                    int(tt.center_px / self.center_bucket_px),
                    tt.priority,
                    -tt.update_count)

        return min(active, key=key)

    def reset(self):
        self.targets = []
        self.last_association = None

    def snapshots(self) -> List[Dict]:
        return [tt.snapshot() for tt in self.targets]