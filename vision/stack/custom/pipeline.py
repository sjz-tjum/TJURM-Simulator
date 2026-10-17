# custom/pipeline.py
"""custom 的整条链：一帧检测进，预测发回 public_bus。

流程：
    DetectionFrame + SelfState
        ↓ pipeline
    pose_solver.solve(frame, self_state)
        ↓ 内部遍历检测、调 components/armor_solvers、打包
    Observation
        ↓ tracker.step(obs, self_state)
        ↓ 内部调 components/{armor_selectors, traj_solvers, shooters}
        ↓ tracker 内部自己发 CONTROL（通过 public_bus）
    List[Prediction]
        ↓ 发 public_bus.pred/<type>

它**不订阅 DETECTION / SELF_STATE**——那些由 CustomComm 收。
什么时候跑不由它决定：keeper 按设备帧率叫它 tick()。

本类同时是 custom 循环的度量点：tick 的滑动窗口平均 fps 经 PIPELINE_STATS
发到 public_bus，供渲染层显示。
"""
from __future__ import annotations

import time
from collections import deque

from core.bus import public_bus
from core.bus.dataflow import PipelineStats
from core.bus.messages import PublicMessages


class Pipeline:
    def __init__(self, tracker, pose_solver, comm):
        self.tracker = tracker
        self.pose_solver = pose_solver
        self.comm = comm
        self._tick_times = deque(maxlen=30)

    def tick(self) -> None:
        """从 comm 取最新输入，跑方案，结果发回 public_bus。没有输入就什么都不做。"""
        self._publish_fps()

        frame, self_state = self.comm.latest_inputs()
        if frame is None or self_state is None:
            return

        obs = self.pose_solver.solve(frame, self_state)
        public_bus().publish(PublicMessages.OBS, obs)

        preds = self.tracker.step(obs, self_state) or ()
        for pred in preds:
            public_bus().publish(PublicMessages.pred(pred.target_type), pred)

    def _publish_fps(self) -> None:
        """滑动窗口平均 fps：窗口内首末 tick 的时间差除间隔数。

        先发 fps 再决定要不要跑方案：这样即使某帧没输入，fps 曲线也连续。
        """
        now = time.perf_counter()
        self._tick_times.append(now)
        if len(self._tick_times) < 2:
            return
        span = self._tick_times[-1] - self._tick_times[0]
        if span <= 0.0:
            return
        fps = (len(self._tick_times) - 1) / span
        public_bus().publish(PublicMessages.PIPELINE_STATS, PipelineStats(fps=fps))

    def reset(self) -> None:
        self._tick_times.clear()
        self.pose_solver.reset()
        self.tracker.reset()