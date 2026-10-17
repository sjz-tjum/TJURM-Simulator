# core/bus/dataflow/stats.py
"""pipeline 循环的运行时统计。"""
from dataclasses import dataclass


@dataclass
class PipelineStats:
    fps: float = 0.0