# config/loader.py
"""data.yaml -> 配置对象：按 schema 的类型注解递归构造。

配置和 schema 对不上就报错：缺键、多键都在这里拦下，带字段路径。

前置：schema.py 是生成物（config/generator.py 产出）。正常入口
（entrance.cli）会在任何 cfg 访问之前先跑 generate()；从别的入口进时
要先跑 `python -m config.generator`。
"""
from __future__ import annotations

from dataclasses import fields, is_dataclass
from typing import Any, get_args, get_origin, get_type_hints

from config.reader import ConfigError, read

try:
    from config.schema import Config
except ModuleNotFoundError as e:
    if e.name != "config.schema":
        raise
    raise ConfigError(
        "config/schema.py 不存在——它由 config/generator.py 生成。\n"
        "正常入口（entrance.cli）会先跑 generate()；"
        "从别的入口进时，先跑 `python -m config.generator`。"
    ) from None


def build(cls, data: dict, path: str = ""):
    """按类型注解递归构造。缺键、多键都在这里报错，路径跟着递归走。"""
    here = path or cls.__name__
    if not isinstance(data, dict):
        raise ConfigError(f"{here}：需要映射，实得 {type(data).__name__}")
    hints = get_type_hints(cls)
    known = [f.name for f in fields(cls)]
    missing = [k for k in known if k not in data]
    extra = [k for k in data if k not in known]
    if missing or extra:
        raise ConfigError(f"{here}：缺 {missing}，多 {extra}")
    return cls(**{k: _coerce(hints[k], data[k], f"{path}.{k}" if path else k)
                  for k in known})


def _coerce(annotation, value, path: str):
    if is_dataclass(annotation) and isinstance(value, dict):
        return build(annotation, value, path)
    origin = get_origin(annotation)
    if origin is dict and isinstance(value, dict):
        args = get_args(annotation)
        item = args[1] if len(args) == 2 else Any
        return {k: _coerce(item, v, f"{path}.{k}") for k, v in value.items()}
    if origin is list and isinstance(value, list):
        args = get_args(annotation)
        item = args[0] if args else Any
        return [_coerce(item, v, f"{path}[{i}]") for i, v in enumerate(value)]
    return value


def load_config(path: str | None = None) -> Config:
    return build(Config, read(path))