# config/__init__.py
"""配置：data.yaml 是唯一真相，运行时是一棵真的配置对象树。

schema.py 由 config/generator.py 生成；loader 按它的注解把 YAML 构造成对象。
对外只暴露一个入口：from config import cfg
"""
from typing import TYPE_CHECKING

from config.reader import ConfigError

if TYPE_CHECKING:                       # 只给 IDE 看，运行时不执行
    from config.loader import load_config
    from config.schema import Config
    cfg: Config

__all__ = ["cfg", "load_config", "ConfigError"]


def __getattr__(name):
    """惰性加载：cfg / load_config 只在真正被取用时才触发 loader。

    这样 import config 本身不碰 schema —— 启动时 generate 才有时机先跑。
    """
    if name == "cfg":
        from config.loader import load_config
        value = load_config()
        globals()["cfg"] = value
        return value
    if name == "load_config":
        from config.loader import load_config
        return load_config
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")