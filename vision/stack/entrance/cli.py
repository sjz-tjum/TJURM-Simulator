# entrance/cli.py
"""启动准备与分发：stdio、schema、宿主循环。

没有命令行参数 —— 选实现走启动界面。要改仿真参数就改 data.yaml。
"""
def _enable_utf8_stdio():
    from entrance.stdio import enable_utf8_stdio
    enable_utf8_stdio()


def _generate_schema():
    from config.generator import generate
    generate()


def main():
    _enable_utf8_stdio()
    _generate_schema()

    from entrance.host import run
    return run()