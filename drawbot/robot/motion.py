from .dataset import is_number


def motion_parameters(settings):
    configured = settings.get("motion", {})
    if not isinstance(configured, dict):
        raise ValueError("motion must be an object")
    result = {}
    for phase in ("draw", "pen_down", "retract", "travel"):
        for suffix, old in (("speed_mm_s", "linear_speed_mm_s"),
                            ("accel_mm_s2", "linear_accel_mm_s2")):
            key = f"{phase}_{suffix}"
            result[key] = configured.get(key, settings[old])
    for key in ("joint_speed_deg_s", "joint_accel_deg_s2"):
        result[key] = configured.get(key, settings[key])
    result["approach_clearance_mm"] = configured.get("approach_clearance_mm", -settings["lift_z_mm"])
    if any(not is_number(v) or v <= 0 for v in result.values()):
        raise ValueError("Speeds, accelerations and approach clearance must be positive finite numbers")
    if settings["draw_z_mm"] != 0:
        raise ValueError("Board surface draw_z_mm must remain 0")
    blend = configured.get("blend_radius_mm", 0.0)
    if not is_number(blend) or blend < 0 or blend > 10:
        raise ValueError("blend_radius_mm must be between 0 and 10")
    strategy = configured.get("travel_strategy", "retract_then_checked_joint")
    if strategy != "retract_then_checked_joint":
        raise ValueError("Only retract_then_checked_joint travel is validated")
    result.update(travel_strategy=strategy, requested_blend_radius_mm=blend,
                      effective_blend_radius_mm=float(blend))
    result["warnings"] = ([f"Blending up to {blend:g} mm on gentle bends; corners, pen down and pen up stop exactly. "
                           "Check line accuracy on the real robot."] if blend else [])
    return result


def phase_for_target(name):
    if name.endswith("_Approach") or name.startswith("Home_"):
        return "travel"
    if name.endswith("_PenDown"):
        return "pen_down"
    if name.endswith("_Retract"):
        return "retract"
    return "draw"
