# led_emissive.py - make the robot's light bars strongly self-illuminated.
#
# User's requirement (2026-10-09): "后面想接入神经网络做视觉识别灯条，会把相机光圈调整的很暗，
# 所以灯条自发光必须亮一点".  Machine vision needs the bars to be bright no matter how dark the
# exposure is, so the right answer is not "a bit more emissive" but:
#   * switch the dedicated light-bar material(s) to UNLIT  -> not affected by scene lighting at all
#   * drive their emissive to a strong, saturated colour -> survives a very dark aperture
#
# The script first REPORTS every material slot of the car mesh (so the mapping is on record), then
# boosts only the slots that look like the light bars (LED / light / emissive / glow / neon + a few
# Chinese spellings), and saves the assets.
#
# Run: UE4Editor.exe <RM2019Sim.uproject> -run=pythonscript -script=<ascii path> -unattended -nosplash -nullrhi

import unreal

MESH = "/Game/RM2019/rm2019_ue"
ML = unreal.MaterialEditingLibrary
EAL = unreal.EditorAssetLibrary
KEYWORDS = ["led", "light", "lamp", "emissive", "glow", "neon", "\u706f", "\u53d1\u5149", "\u4eae"]


def log(msg):
    unreal.log("WLLED|" + msg)


def looks_like_led(name):
    low = name.lower()
    for k in KEYWORDS:
        if k in low:
            return True
    return False


def boost(material):
    """Unlit + 强自发光。返回一句说明。"""
    if material is None:
        return "no material"
    notes = []
    try:
        material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
        notes.append("unlit")
    except Exception as exc:                                  # noqa: BLE001
        notes.append("shading_model failed: %s" % exc)

    # 1) instance 参数（最省事也最常见）：把任何像发光强度的标量拉满，把发光颜色的向量拉亮
    try:
        scalars = ML.get_scalar_parameter_names(material)
    except Exception:                                         # noqa: BLE001
        scalars = []
    try:
        vectors = ML.get_vector_parameter_names(material)
    except Exception:                                         # noqa: BLE001
        vectors = []
    for s in scalars:
        low = s.lower()
        if any(k in low for k in ("emissive", "glow", "intensity", "bright", "power", "led", "\u4eae")):
            ML.set_material_instance_scalar_parameter_value(material, s, 40.0)
            notes.append("scalar %s=40" % s)
    for v in vectors:
        low = v.lower()
        if any(k in low for k in ("emissive", "glow", "color", "led", "\u8272", "\u4eae")):
            ML.set_material_instance_vector_parameter_value(material, v, unreal.LinearColor(0.05, 2.5, 9.0, 1.0))
            notes.append("vector %s=(bright cyan)" % v)

    # 2) 基础材质：直接改发光引脚
    try:
        is_base = isinstance(material, unreal.Material)
    except Exception:                                         # noqa: BLE001
        is_base = False
    if is_base:
        try:
            emis = ML.get_material_property_input_node(material, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
            mul = ML.create_material_expression(material, unreal.MaterialExpressionMultiply, -350, 0)
            mul.set_editor_property("const_b", 30.0)
            if emis:
                ML.connect_material_expressions(emis, "", mul, "A")
            else:
                const = ML.create_material_expression(material, unreal.MaterialExpressionConstant3Vector, -600, 0)
                const.set_editor_property("constant", unreal.LinearColor(0.05, 2.5, 9.0, 1.0))
                ML.connect_material_expressions(const, "", mul, "A")
            ML.connect_material_property(mul, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
            notes.append("emissive x30 wired on the base material")
            ML.recompile_material(material)
        except Exception as exc:                              # noqa: BLE001
            notes.append("base-material wiring failed: %s" % exc)

    try:
        EAL.save_loaded_asset(material)
        notes.append("saved")
    except Exception as exc:                                  # noqa: BLE001
        notes.append("save failed: %s" % exc)
    return ", ".join(notes)


def main():
    mesh = EAL.load_asset(MESH)
    if mesh is None:
        log("could not load %s" % MESH)
        return
    log("mesh %s" % mesh.get_name())
    try:
        slots = mesh.get_editor_property("materials")
    except Exception as exc:                                  # noqa: BLE001
        log("could not read material slots: %s" % exc)
        return

    n = 0
    for sm in slots:
        try:
            slot_name = str(sm.get_editor_property("material_slot_name"))
            mat = sm.get_editor_property("material_interface")
        except Exception as exc:                              # noqa: BLE001
            log("  slot %d unreadable: %s" % (n, exc))
            n += 1
            continue
        mat_name = mat.get_name() if mat else "<none>"
        cls = mat.get_class().get_name() if mat else "-"
        hit = looks_like_led(slot_name) or looks_like_led(mat_name)
        log("  slot[%d] '%s' -> %s (%s)%s" % (n, slot_name, mat_name, cls, "   <== LED" if hit else ""))
        if hit and mat:
            log("      boost: %s" % boost(mat))
        n += 1
    log("done (%d slot(s))" % n)


main()
