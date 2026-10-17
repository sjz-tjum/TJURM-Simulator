# core/interfaces/pose_solver.py
"""PoseSolver 接口：帧级位姿解算。

输入 DetectionFrame + SelfState，输出 Observation。
实现在 custom/schemes 下；pipeline 通过这个接口调用它。

内部通常遍历检测、对每块板调单板 PnP、换架到 G/W、打包。
单板 PnP 由 custom/components/armor_solvers/ 提供，本接口不规定用它。
"""
from __future__ import annotations

from core.interfaces.registry import Registry, register

__all__ = ["PoseSolver", "register_pose_solver"]


class PoseSolver(Registry):
    kind = "帧级位姿解算器"

    def solve(self, frame, self_state):
        """DetectionFrame + SelfState -> Observation。"""
        raise NotImplementedError

    def reset(self) -> None:
        pass


def register_pose_solver(name, aliases=None):
    return register(PoseSolver, name, aliases)