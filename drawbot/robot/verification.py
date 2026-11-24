def collision_map_report(rdk, robot, gripper, tool):
    from robodk.robolink import ITEM_TYPE_OBJECT
    pairs = rdk.CollisionActivePairList()
    serialized = [{"item1": a.Name(), "link1": i, "item2": b.Name(), "link2": j}
                  for a, b, i, j in pairs]
    objects = [o.Name() for o in rdk.ItemList(ITEM_TYPE_OBJECT)]
    names = {p[k] for p in serialized for k in ("item1", "item2")}
    def enabled(a, b):
        return any({p["item1"], p["item2"]} == {a, b} for p in serialized)
    from ..pen_stand import off_by_design
    required_pairs = [{"robot": arm, "object": obj, "enabled": enabled(arm, obj)}
                      for arm in (robot.Name(), gripper.Name()) for obj in objects
                      if not off_by_design(arm, obj)]
    coverage_ok = (bool(objects) and all(p["enabled"] for p in required_pairs)
                   and enabled(robot.Name(), gripper.Name()))
    return {
        "coverage_ok": coverage_ok,
        "required_geometry_pairs": required_pairs,
        "active_pair_count": len(pairs), "active_pairs": serialized,
        "object_geometry_names": objects,
        "robot_in_map": robot.Name() in names, "gripper_in_map": gripper.Name() in names,
        "tool_name": tool.Name(), "tool_collision_representation": "attached tool meshes belong to the robot flange link",
        "objects_absent_from_map": sorted(set(objects)-names),
        "check_step_mm": 1.0, "check_step_deg": 1.0,
        "limitations": ["Discrete full-segment sampling is not a continuous swept-volume proof",
                        "Collision checks only cover installed geometry and enabled pairs",
                        "Marker tip/contact compliance, cables and the physical setup require verification",
                        "TCP/frame calibration uncertainty and real braking are not modeled"],
    }


def diagnose_path(program, manifest):
    from robodk.robolink import COLLISION_ON
    try:
        message, matrix, status = program.InstructionListJoints(
            mm_step=1.0, deg_step=1.0, collision_check=COLLISION_ON)
        rows = matrix.Rows()
        failures = []
        if len(rows) >= 10:
            for error, move_id in zip(rows[6], rows[9]):
                if error:
                    index = int(move_id)
                    failures.append({"error_code": error, "move_id": index,
                                     "candidate": manifest[index] if 0 <= index < len(manifest) else None,
                                     "localization": "MOVE_ID mapping candidate; confirm message and GUI"})
        return {"message": message, "status": status, "failed_samples": failures[:50]}
    except Exception as exc:
        return {"diagnostic_error": repr(exc)}
