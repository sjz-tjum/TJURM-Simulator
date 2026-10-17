# config/generator.py
"""从 data.yaml 生成 schema.py：一个 dict 一个类，名字由路径推出。

生成物只服务 IDE 补全；数值的唯一来源永远是 YAML。
"""
from __future__ import annotations

import os
from typing import Any, List, Tuple

from config.reader import ConfigError, read

_SCHEMA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "schema.py")

#: 需要动态取键的节点：子键运行时才知道（车型名等）。
DYNAMIC = {("robot",)}

_HEADER = '''# schema.py — 由 config/generator.py 生成，勿手改。
from dataclasses import dataclass
from typing import Any, List, Literal
'''


def _pascal(text: str) -> str:
    return "".join(w[:1].upper() + w[1:] for w in str(text).split("_"))


def _class_name(path: Tuple[str, ...]) -> str:
    return "".join(_pascal(p) for p in path) + "Config"


def _scalar_type(value: Any) -> str:
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        if not value:
            return "List[Any]"
        inner = {_scalar_type(v) for v in value}
        if inner == {"int", "float"}:
            inner = {"float"}
        return "List[%s]" % next(iter(inner)) if len(inner) == 1 else "List[Any]"
    return "Any"


def _render(name: str, fields: List[Tuple[str, str]],
            keys: List[str], spec: str | None) -> str:
    body = "\n".join(f"    {k}: {t}" for k, t in fields) or "    pass"
    if spec is not None:
        lit = ", ".join(f'"{k}"' for k in keys)
        body += (f"\n\n    def __getitem__(self, key: Literal[{lit}]) -> {spec}:\n"
                 f"        return getattr(self, key)")
    return f"@dataclass\nclass {name}:\n{body}\n"

def _emit(node: dict, path: Tuple[str, ...], chunks: List[str]) -> str:
    """为一个 dict 节点生成类，返回类名。"""
    name = _class_name(path)
    dynamic = path in DYNAMIC
    fields: List[Tuple[str, str]] = []
    keys: List[str] = []
    spec: str | None = None
    ref: dict | None = None          # 第一个 dict 子节点，用作同构参照

    for key, value in node.items():
        if not isinstance(value, dict):
            fields.append((key, _scalar_type(value)))
            continue
        if not dynamic:
            fields.append((key, _emit(value, path + (key,), chunks)))
            continue
        if spec is None:
            spec = _emit(value, path + ("spec",), chunks)
            ref = value
        elif set(value) != set(ref):
            raise ConfigError(
                f"{'.'.join(path)}：子节点不同构，__getitem__ 需要统一返回类型")
        fields.append((key, spec))
        keys.append(key)

    chunks.append(_render(name, fields, keys, spec))
    return name

def render(data: dict) -> str:
    chunks: List[str] = []
    root = [(k, _emit(v, (k,), chunks) if isinstance(v, dict) else _scalar_type(v))
            for k, v in data.items()]
    chunks.append(_render("Config", root, [], None))
    return _HEADER + "\n\n" + "\n\n".join(chunks).rstrip("\n") + "\n"


def generate(force: bool = False) -> bool:
    """生成 schema.py；内容没变就不写盘。返回是否真的写了盘。"""
    text = render(read())
    if not force and os.path.exists(_SCHEMA):
        with open(_SCHEMA, encoding="utf-8") as f:
            if f.read() == text:
                return False
    with open(_SCHEMA, "w", encoding="utf-8") as f:
        f.write(text)
    return True


if __name__ == "__main__":
    generate(force=True)