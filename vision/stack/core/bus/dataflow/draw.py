# core/bus/dataflow/draw.py
from dataclasses import dataclass
from typing import Tuple


@dataclass
class DrawText:
    text: str
    color: Tuple[int, int, int] = (255, 255, 255)
    view: str = "info"