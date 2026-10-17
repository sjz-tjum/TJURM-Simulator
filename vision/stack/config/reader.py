# config/reader.py
"""读 data/data.yaml —— loader 与 generator 共用的唯一入口。"""
from __future__ import annotations

import os

import yaml

_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "data.yaml")


class ConfigError(Exception):
    """配置出问题：文件读不到、YAML 语法错、和 schema 对不上。"""


def read(path: str | None = None) -> dict:
    """读 YAML 成 dict；文件错、语法错都包成 ConfigError 并带上位置。"""
    p = path or _PATH
    try:
        with open(p, encoding="utf-8") as f:
            text = f.read()
    except OSError as e:
        raise ConfigError(f"{p}：{e.strerror or e}") from None
    try:
        return yaml.safe_load(text) or {}
    except yaml.YAMLError as e:
        mark = getattr(e, "problem_mark", None)
        where = f"{p}:{mark.line + 1}:{mark.column + 1}" if mark else p
        problem = getattr(e, "problem", str(e))
        raise ConfigError(f"{where}：{problem}") from None