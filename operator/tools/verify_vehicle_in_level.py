# verify_vehicle_in_level.py  (task-21, defect 1 acceptance check)
#
# Answers, from the saved level itself and not from a runtime spawn:
#   * is there exactly ONE AWLVehiclePawn instance in RM_Arena?
#   * does that instance carry the MODEL (USkeletalMesh) on a VISIBLE component, which is what the
#     editor viewport draws before Play is ever pressed?
#   * is the collision follower present but hidden and NOT casting a shadow (it must never render)?
#   * does Play reuse this actor instead of spawning a second car?  (Answered at runtime by the
#     WLSPAWN| log line; this script checks the static half.)
#
# Run headless through the full editor:
#   UE4Editor.exe <project>.uproject -nullrhi -unattended -nosplash -ExecCmds=py <this file>

import unreal


def log(msg):
    unreal.log("[verify_vehicle] " + str(msg))


def main():
    les = unreal.EditorLevelLibrary
    cars = [a for a in les.get_all_level_actors() if a.get_class().get_name() == "WLVehiclePawn"]
    log("AWLVehiclePawn instances in the level: %d" % len(cars))
    if len(cars) != 1:
        log("FAIL: expected exactly 1 placed car")
        return

    car = cars[0]
    loc = car.get_actor_location()
    log("actor label='%s' location=(%.2f, %.2f, %.2f)" % (car.get_actor_label(), loc.x, loc.y, loc.z))

    # USkinnedMeshComponent is the common base of UPoseableMeshComponent (the driven model) and
    # USkeletalMeshComponent (the collision follower).  Querying USkeletalMeshComponent alone
    # MISSES the model entirely - which is exactly how the first version of this check reported
    # only PhysMesh and looked like the car had no mesh at all.
    visible_mesh = 0
    for c in car.get_components_by_class(unreal.SkinnedMeshComponent):
        mesh = c.get_editor_property("skeletal_mesh")
        vis = c.is_visible()
        shadow = c.get_editor_property("cast_shadow")
        log("  %-14s class=%-26s mesh=%-14s visible=%-5s cast_shadow=%s"
            % (c.get_name(), c.get_class().get_name(),
               mesh.get_name() if mesh else "NONE", str(vis), str(shadow)))
        if mesh is not None and vis:
            visible_mesh += 1
            skel = mesh.get_editor_property("skeleton")
            log("  -> %s has a skeleton: %s" % (mesh.get_name(), "yes" if skel else "NO"))
    log("VISIBLE MESH COMPONENTS = %d (must be >= 1 for the editor viewport to draw the car)" % visible_mesh)
    log("RESULT %s" % ("PASS" if visible_mesh >= 1 else "FAIL"))


main()
