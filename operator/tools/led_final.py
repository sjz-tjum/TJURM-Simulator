# led_final.py - MAT_LED_Blue: Unlit + 强自发光，并且**真正存到磁盘**。
#
# 前情：unreal.EditorAssetLibrary.save_asset / save_directory 在这个工程的命令行 Python 里都返回
# False（uasset 时间戳不变，等于没存）。这次换 UE 的 "Save All" 路径
# (EditorLoadingAndSavingUtils.save_packages / save_dirty_packages)，三个都试，哪个成功一目了然。
#
# 数值按"给神经网络做视觉识别"来定（用户要求：光圈会很暗，灯条必须够亮）：
#   * 纯青 (0, 6, 14) —— R 通道为 0，最利于做颜色阈值分割
#   * 乘 40 —— 暗光圈下也顶到饱和
#   * Unlit —— 不吃场景光照，曝光之外没有别的变量
#
# Run: UE4Editor.exe <RM2019Sim.uproject> -run=pythonscript -script=<ascii path>

import unreal

MAT = "/Game/RM2019/MAT_LED_Blue"
ML = unreal.MaterialEditingLibrary
EAL = unreal.EditorAssetLibrary
ELS = unreal.EditorLoadingAndSavingUtils

CYAN = unreal.LinearColor(0.0, 6.0, 14.0, 1.0)
GAIN = 40.0


def log(msg):
    unreal.log("WLLED3|" + msg)


def main():
    mat = EAL.load_asset(MAT)
    if mat is None:
        log("load failed: %s" % MAT)
        return
    log("material %s" % mat.get_name())

    # 1) 自发光：原有发光节点（如果有）乘 GAIN，否则用一个纯青常量
    emis = None
    try:
        emis = ML.get_material_property_input_node(mat, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    except Exception as exc:                                  # noqa: BLE001
        log("emissive node read failed: %s" % exc)

    const = ML.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -1100, 0)
    const.set_editor_property("constant", CYAN)
    src = emis if emis else const
    mul = ML.create_material_expression(mat, unreal.MaterialExpressionMultiply, -600, 0)
    mul.set_editor_property("const_b", GAIN)
    ML.connect_material_expressions(src, "", mul, "A")
    ML.connect_material_property(mul, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    log("emissive = %s * %.0f" % ("existing node" if emis else "cyan constant", GAIN))

    # 2) Unlit
    try:
        mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
        log("shading_model -> UNLIT")
    except Exception as exc:                                  # noqa: BLE001
        log("shading_model failed: %s" % exc)
    try:
        ML.recompile_material(mat)
        log("recompiled")
    except Exception as exc:                                  # noqa: BLE001
        log("recompile failed: %s" % exc)

    # 3) 三种保存路径都试一遍
    pkg = mat.get_outermost()
    try:
        log("save_packages -> %s" % ELS.save_packages([pkg], only_dirty=False))
    except Exception as exc:                                  # noqa: BLE001
        log("save_packages raised: %s" % exc)
    try:
        log("save_dirty_packages -> %s" % ELS.save_dirty_packages(True, True))
    except Exception as exc:                                  # noqa: BLE001
        log("save_dirty_packages raised: %s" % exc)
    try:
        log("save_asset -> %s" % EAL.save_asset(MAT, only_if_is_dirty=False))
    except Exception as exc:                                  # noqa: BLE001
        log("save_asset raised: %s" % exc)

    # 4) 自己再看一眼：从内存里读回来的状态
    log("final shading_model=%s" % mat.get_editor_property("shading_model"))
    log("package dirty=%s" % (pkg.is_dirty() if hasattr(pkg, "is_dirty") else "?"))
    log("done")


main()
