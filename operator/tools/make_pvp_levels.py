# make_pvp_levels.py —— 建/补 PVP demo 需要的两张关卡。
#
# 为什么用脚本：关卡本身几乎是空的（竞技场由 C++ 在 Play 时程序化生成），手点"New Level"反而
# 容易漏东西。但"空"是指**逻辑**为空，不是**画面上**为空 —— 用户实测的"进去全是黑的"就是因为
# 新建的空关卡在编辑器视口里没有灯也没有地面。所以这个脚本会把"看得见的舞台"写进关卡资产：
#   * 20 x 20 m 地台（编辑器视口与菜单背景都能看见东西）
#   * 平行光 + 天光
#   * RM_Menu 里放一个 PlayerStart，位置就是菜单相机的机位（保证 Play 的第一帧就在正确视角，
#     不依赖 Pawn 与 PlayerController 的生成先后）
#
# 运行：
#   UE4Editor.exe <project>.uproject -run=pythonscript -script=<这个文件的路径> -unattended -nosplash
#   （路径不能带空格，否则命令行会把参数截断 —— 所以用的时候先拷到无空格目录）

import unreal

CUBE = "/Engine/BasicShapes/Cube"


def log(msg):
    unreal.log("WLLEVEL|" + msg)


def spawn(actor_class, loc, rot):
    try:
        return unreal.EditorLevelLibrary.spawn_actor_from_class(actor_class, loc, rot)
    except Exception as exc:                      # noqa: BLE001
        unreal.log_warning("WLLEVEL|could not spawn %s (%s)" % (actor_class, exc))
        return None


def ensure_stage(menu_mode, camera_loc, camera_rot):
    """把关卡布置成"看得见"的样子（幂等：已存在的同名 actor 不再重复生成）。"""
    existing = set()
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        existing.add(actor.get_actor_label())

    if "PVP_Stage_Floor" not in existing:
        floor = spawn(unreal.StaticMeshActor, unreal.Vector(0, 0, -10), unreal.Rotator(0, 0, 0))
        if floor:
            floor.set_actor_label("PVP_Stage_Floor")
            mesh = unreal.EditorAssetLibrary.load_asset(CUBE)
            comp = floor.get_editor_property("static_mesh_component")
            comp.set_editor_property("static_mesh", mesh)
            comp.set_world_scale3d(unreal.Vector(20.0, 20.0, 0.2))
            log("  floor added")

    if "PVP_Stage_Sun" not in existing:
        sun = spawn(unreal.DirectionalLight, unreal.Vector(0, 0, 800), unreal.Rotator(-42, 130, 0))
        if sun:
            sun.set_actor_label("PVP_Stage_Sun")
            log("  sun added")

    if "PVP_Stage_Sky" not in existing:
        sky = spawn(unreal.SkyLight, unreal.Vector(0, 0, 300), unreal.Rotator(0, 0, 0))
        if sky:
            sky.set_actor_label("PVP_Stage_Sky")
            log("  sky light added")

    if menu_mode and "PVP_Menu_Camera" not in existing:
        start = spawn(unreal.PlayerStart, camera_loc, camera_rot)
        if start:
            start.set_actor_label("PVP_Menu_Camera")
            log("  menu PlayerStart added at %s" % (camera_loc,))


def set_game_mode(game_mode_path):
    """WorldSettings 的 DefaultGameMode。4.27 的 Python 接口不一定成功，
    失败也没关系：Config/DefaultEngine.ini 里的 GameModeMapPrefixes 兜着（优先级更低但同样有效）。"""
    klass = None
    try:
        klass = unreal.load_class(None, game_mode_path)
    except Exception as exc:                      # noqa: BLE001
        unreal.log_warning("WLLEVEL|could not load %s (%s)" % (game_mode_path, exc))
    if not klass:
        return False
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        if "WorldSettings" in actor.get_class().get_name():
            try:
                actor.set_editor_property("default_game_mode", klass)
                return True
            except Exception as exc:              # noqa: BLE001
                unreal.log_warning("WLLEVEL|set default_game_mode failed (%s)" % exc)
                return False
    return False


def make_level(asset_path, game_mode_path, title, menu_mode, cam_loc, cam_rot):
    if unreal.EditorAssetLibrary.does_asset_exist(asset_path):
        unreal.EditorLevelLibrary.load_level(asset_path)      # 已存在：打开它补舞台
        log("%s exists (%s) - topping up the stage" % (asset_path, title))
    else:
        unreal.EditorLevelLibrary.new_level(asset_path)
        log("created %s (%s)" % (asset_path, title))
    ensure_stage(menu_mode, cam_loc, cam_rot)
    ok = set_game_mode(game_mode_path)
    unreal.EditorLevelLibrary.save_current_level()
    log("%s saved (default_game_mode set=%s)" % (asset_path, ok))


make_level("/Game/Maps/RM_Menu", "/Script/RM2019Sim.WLMenuGameMode",
           "main menu / level & character select", True,
           unreal.Vector(-620.0, -880.0, 330.0), unreal.Rotator(-14.0, 35.0, 0.0))
make_level("/Game/Maps/RM_PVP", "/Script/RM2019Sim.WLPVPGameMode",
           "5v5 symmetric arena 26x18 m", False,
           unreal.Vector(0.0, 0.0, 200.0), unreal.Rotator(0.0, 0.0, 0.0))
log("done")
