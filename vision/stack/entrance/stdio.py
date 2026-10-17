# entrance/stdio.py
"""把标准输出/错误流切到 UTF-8（Windows 中文控制台友好）。"""
from __future__ import annotations

import io
import sys


def enable_utf8_stdio():
    """尽力把 stdout/stderr 切到 UTF-8；不做失败上报。"""
    for name in ("stdout", "stderr"):
        stream = getattr(sys, name, None)
        if stream is None:
            continue
        rc = getattr(stream, "reconfigure", None)
        if callable(rc):
            try:
                rc(encoding="utf-8", errors="replace")
                continue
            except (ValueError, OSError, AttributeError):
                pass
        buf = getattr(stream, "buffer", None)
        if buf is None:
            continue
        try:
            wrapped = io.TextIOWrapper(buf, encoding="utf-8",
                                       errors="replace", line_buffering=True)
        except (ValueError, OSError):
            continue
        setattr(sys, name, wrapped)