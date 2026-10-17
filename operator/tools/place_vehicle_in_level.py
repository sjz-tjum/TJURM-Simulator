# place_vehicle_in_level.py  (task-21, defect 1)
#
# "Without pressing Play you cannot see the car model."
#
# Structural cause: the vehicle only ever existed as the GameMode's DEFAULT PAWN, so RM_Arena
# contained no vehicle actor at all - the arena, the lights and a PlayerStart, and nothing to look
# at in the editor viewport.  This script puts ONE AWLVehiclePawn instance into the level at the
# exact pose the GameMode spawns it at (the rig origin, 29.67 cm above the Arena_Floor top face),
# so the map and Play agree.
#
# The GameMode's SpawnDefaultPawnAtTransform is redirected onto this actor, so Play possesses it
# instead of spawning a second car - see AWLGameMode::SpawnDefaultPawnAtTransform_Implementation.
#
# Run it headless through the FULL editor (not -run=PythonScript: that commandlet has no level
# editor, and EditorLevelLibrary.load_level() dereferences it and crashes):
#
#   UE4Editor.exe <project>.uproject -nullrhi -unattended -nosplash
#        -ExecCmds=py <path-to-this-file>
#
# The editor loads EditorStartupMap (= /Game/Maps/RM_Arena) itself, so this script never loads a
# level; it edits the one that is already open and saves it.  Idempotent: existing
# AWLVehiclePawn actors are removed first.

import unreal

SPAWN_LOC = unreal.Vector(0.0, 0.0, 29.67)   # rig origin = Arena_Floor top (Z=0) + 29.67 cm
SPAWN_ROT = unreal.Rotator(0.0, 0.0, 0.0)


def log(msg):
    unreal.log("[place_vehicle] " + str(msg))


def main():
    les = unreal.EditorLevelLibrary

    actors = les.get_all_level_actors()
    if not actors:
        log("FAIL: no level is open - is EditorStartupMap set to /Game/Maps/RM_Arena?")
        return

    # ---- 1. clear any previous placement (idempotent) ----------------------
    removed = 0
    for actor in actors:
        if actor.get_class().get_name() == "WLVehiclePawn":
            les.destroy_actor(actor)
            removed += 1
    log("removed %d existing WLVehiclePawn actor(s)" % removed)

    # ---- 2. place one ------------------------------------------------------
    cls = unreal.load_class(None, "/Script/RM2019Sim.WLVehiclePawn")
    if cls is None:
        log("FAIL: /Script/RM2019Sim.WLVehiclePawn not found - is the C++ module built?")
        return

    car = les.spawn_actor_from_class(cls, SPAWN_LOC, SPAWN_ROT, False)
    if car is None:
        log("FAIL: spawn_actor_from_class returned None")
        return

    car.set_actor_label("WL_Vehicle")
    loc = car.get_actor_location()
    log("PLACED %s at (%.2f, %.2f, %.2f)" % (car.get_class().get_name(), loc.x, loc.y, loc.z))

    # ---- 3. report what the editor will actually draw ----------------------
    try:
        for c in car.get_components_by_class(unreal.SkeletalMeshComponent):
            mesh = c.get_editor_property("skeletal_mesh")
            log("  component %s: mesh=%s visible=%s cast_shadow=%s"
                % (c.get_name(), mesh.get_name() if mesh else "NONE",
                   c.is_visible(), c.get_editor_property("cast_shadow")))
    except Exception as exc:                                   # noqa: BLE001
        log("note: component inspection failed (%s)" % exc)

    names = sorted(a.get_actor_label() for a in les.get_all_level_actors())
    log("level now has %d actors: %s" % (len(names), ", ".join(names)))

    # ---- 4. save -----------------------------------------------------------
    log("SAVED %s" % les.save_current_level())


main()
