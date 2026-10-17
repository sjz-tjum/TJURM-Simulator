# entrance/__init__.py
"""进程入口：装配层。唯一同时认识 sim 与 custom 的地方。"""
from entrance.cli import main

__all__ = ["main"]