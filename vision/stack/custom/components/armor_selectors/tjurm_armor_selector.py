# custom/components/armor_selectors/tjurm_armor_selector.py
"""选板：给三类目标各提供一个"选瞄点"的几何操作。

    select_armor       —— 车 / 前哨：从候选里挑一块正对自车的真实板（带锁定偏置）。
    select_virtual     —— 车大 ω：车心沿指向自车方向外推一个半径的虚拟点。
    select_lit_blade   —— 符：从点亮的灯臂里挑最朝向自车的一块靶心。

三个方法的语义互不相同，只是同住一个文件（车、前哨、符都要"挑一个瞄点"）。

命名说明：`spin_branch_thresh` 是"spin 兜底分支启用的角速度阈值"——当没有板
朝向自车、且车转得够快时，选一块方位最接近自车方向的板。它和 ShootDecider 的
`spin_thresh`（auto/single 模式判定）是两个不同的阈值，此处用不同名字避免混淆。
"""
import numpy as np

from core.algorithms.math.angle import limit_rad


class ArmorSelector:
    """几何全部交给模型（`armor_candidates`）。"""

    def __init__(self, spin_branch_thresh=12.0, lock_bias=0.3):
        self.spin_branch_thresh = spin_branch_thresh
        self.lock_bias = lock_bias
        self.locked_armor_id = None
        self.selected_armor_id = None

    # ------------------------------------------------------------ 车 / 前哨：追板
    def select_armor(self, target, self_rel_pos, t_future, cur_dir=None):
        if target.jumped:
            self.locked_armor_id = None

        cands = target.ekf.armor_candidates(t_future)
        x = target.ekf.ekf.x
        omega = float(x[7]) if len(x) > 7 else 0.0

        lit_ids = [i for i, c in enumerate(cands) if len(c) > 2 and c[2]]
        index_pool = lit_ids if lit_ids else list(range(len(cands)))

        best_score, best_id, best_pos = -np.inf, None, None
        for i in index_pool:
            pos, normal = cands[i][0], cands[i][1]
            sight = np.asarray(self_rel_pos, float) - pos
            dist = float(np.linalg.norm(sight))
            if dist < 1e-6:
                continue
            cos_alpha = float(np.dot(normal, sight / dist))
            if cos_alpha <= 0.0:
                continue
            score = cos_alpha / (dist * dist)
            if self.locked_armor_id is not None and i == self.locked_armor_id:
                score += self.lock_bias
            if score > best_score:
                best_score, best_id, best_pos = score, i, pos

        if best_id is not None:
            self.locked_armor_id = best_id
            self.selected_armor_id = best_id
            return best_id, best_pos

        if len(index_pool) > 1 and cur_dir is not None:
            best_diff, best_id = float("inf"), None
            for i in index_pool:
                sight = np.asarray(cands[i][0], float) - np.asarray(self_rel_pos, float)
                n = float(np.linalg.norm(sight))
                if n < 1e-6:
                    continue
                cos_a = float(np.clip(np.dot(sight / n, cur_dir), -1.0, 1.0))
                if np.arccos(cos_a) < best_diff:
                    best_diff, best_id = np.arccos(cos_a), i
            if best_id is not None:
                self.locked_armor_id = best_id
                self.selected_armor_id = best_id
                return best_id, cands[best_id][0]

        if abs(omega) >= self.spin_branch_thresh and len(cands) > 1:
            center = np.asarray(x[:3], float) + np.asarray(x[3:6], float) * t_future
            yaw_to_self = np.arctan2(self_rel_pos[1] - center[1], self_rel_pos[0] - center[0])
            best_diff, best_id = float("inf"), index_pool[0]
            for i in index_pool:
                pos = cands[i][0]
                armor_yaw = np.arctan2(pos[1] - center[1], pos[0] - center[0])
                diff = abs(limit_rad(armor_yaw - yaw_to_self))
                if diff < best_diff:
                    best_diff, best_id = diff, i
            self.locked_armor_id = best_id
            self.selected_armor_id = best_id
            return best_id, cands[best_id][0]

        self.locked_armor_id = None
        self.selected_armor_id = index_pool[0]
        return index_pool[0], cands[index_pool[0]][0]

    # ------------------------------------------------------------ 车大 ω：虚拟点
    def select_virtual(self, target, self_rel_pos, t_future):
        """车大 ω：车心 + radius × 指向自车方向。

        G 系原点就是本车，"指向自车"= "指向原点"。
        虚拟点模式下没有"板 id"——selected_armor_id 置 None，让电控走"上升沿"判据。
        """
        x = target.ekf.ekf.x
        center = np.array([x[0] + x[3] * t_future,
                           x[1] + x[4] * t_future,
                           x[2] + x[5] * t_future])
        radius = float(x[8])  # 偶数板半径（车模型 x[8]）
        direction = np.asarray(self_rel_pos, float) - center
        norm = float(np.linalg.norm(direction))
        self.selected_armor_id = None
        if norm < 1e-6:
            return center
        return center + radius * direction / norm

    # ------------------------------------------------------------ 符：点亮灯臂
    def select_lit_blade(self, target, self_rel_pos, t_future):
        """符：从点亮的灯臂里挑最朝向自车的一块靶心。返回位置或 None。

        符的灯臂在 y-z 竖直平面内绕 x 轴旋转。判"朝向自车"= 灯臂在旋转平面
        内的相位与"圆心指向自车"的相位最接近。与车辆 spin 分支同思路。
        """
        cands = target.ekf.armor_candidates(t_future)
        lit_ids = [i for i, c in enumerate(cands) if c[2]]
        if not lit_ids:
            self.selected_armor_id = None
            return None

        x = target.ekf.ekf.x
        center = np.array([x[0] + x[3] * t_future,
                           x[1] + x[4] * t_future,
                           x[2] + x[5] * t_future])
        # 圆心指向自车 —— 投影到 y-z 平面
        d_self = np.asarray(self_rel_pos, float) - center
        proj_self = np.array([d_self[1], d_self[2]])
        n_self = float(np.linalg.norm(proj_self))
        if n_self < 1e-6:
            # 自车在符的旋转轴上——所有灯臂对称，取第一块
            best_id = lit_ids[0]
        else:
            phi_self = np.arctan2(proj_self[1], proj_self[0])
            best_id, best_diff = lit_ids[0], float("inf")
            for i in lit_ids:
                pos = np.asarray(cands[i][0], float)
                proj_armor = np.array([pos[1] - center[1], pos[2] - center[2]])
                if float(np.linalg.norm(proj_armor)) < 1e-6:
                    continue
                phi_armor = np.arctan2(proj_armor[1], proj_armor[0])
                diff = abs(limit_rad(phi_armor - phi_self))
                if diff < best_diff:
                    best_diff, best_id = diff, i

        self.selected_armor_id = best_id
        return np.asarray(cands[best_id][0], float)