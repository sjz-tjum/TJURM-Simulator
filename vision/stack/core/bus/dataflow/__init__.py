# core/bus/dataflow/__init__.py
"""跨层传递的消息契约：检测、观测位姿、自车状态、预测、调试消息。"""
from core.bus.dataflow.detection import Detection, DetectionFrame
from core.bus.dataflow.observation import ArmorObservation, Observation
from core.bus.dataflow.self_state import RigidState, CameraState, SelfState
from core.bus.dataflow.prediction import Prediction, GimbalTarget
from core.bus.dataflow.control import ShotSolution
from core.bus.dataflow.draw import DrawText
from core.bus.dataflow.stats import PipelineStats
from core.entities.property.entity_types import TargetType

__all__ = [
    "Detection", "DetectionFrame",
    "ArmorObservation", "Observation",
    "RigidState", "CameraState", "SelfState",
    "Prediction", "GimbalTarget",
    "ShotSolution",
    "DrawText",
    "PipelineStats",
    "TargetType",
]