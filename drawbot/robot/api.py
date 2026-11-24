import copy
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import traceback

from .dataset import validate_drawing, validate_palette
from .json_program import generate, save_report
from .motion import motion_parameters

ROOT = Path(__file__).resolve().parents[2]


def status(settings=None):
    from .connection import connect_existing
    from .station import check_station
    from robodk.robolink import ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET
    settings = settings or json.loads((ROOT / "config/drawing.json").read_text(encoding="utf-8"))
    rdk = None
    try:
        rdk = connect_existing()
        station = check_station(rdk, settings, extra=[(settings["orientation_target"], ITEM_TYPE_TARGET),
                                                      (settings["hardware"]["gripper_name"], ITEM_TYPE_ROBOT),
                                                      (settings["hardware"]["mount_frame_name"], ITEM_TYPE_FRAME)])
        return {"success": True, "ok": True, "station_name": station.Name(), "version": rdk.Version(),
                "message": "RoboDK station is available; no path was checked", "warnings": [], "errors": []}
    except Exception as exc:
        return {"success": False, "ok": False, "station_name": None, "message": str(exc),
                "warnings": [], "errors": [str(exc)]}
    finally:
        if rdk is not None:
            rdk.Disconnect()


def draw_in_robodk(stroke_list, mode="simulate", *, board_width=None, board_height=None,
                   draw_speed=None, lift_speed=None, joint_speed=None, travel_speed=None, approach_mm=None,
                   blend_mm=None, park_upright=None, park_joints=None,
                   motion=None, pen_change_mode=None, preloaded_marker=None, pen_stand=None,
                   program_name=None, settings=None, project_root=None, source_file=None,
                   allow_stub_export_for_testing=False):
    root = Path(project_root or ROOT).resolve()
    report = {"status": "failed", "stage": "input_validation", "warnings": [],
              "data_path": str(source_file) if source_file else "<in-memory StrokeList>",
              "started_at_utc": datetime.now(timezone.utc).isoformat()}
    try:
        if mode == "status":
            return status(settings)
        if mode == "program":
            mode = "export"
        if mode not in ("simulate", "export"):
            raise ValueError("mode must be simulate, export, or status")
        if is_dataclass(stroke_list) and not isinstance(stroke_list, type):
            stroke_list = json.loads(json.dumps(asdict(stroke_list)))
        data = copy.deepcopy(stroke_list)
        config = copy.deepcopy(settings if settings is not None else json.loads((root / "config/drawing.json").read_text(encoding="utf-8")))
        if not isinstance(config, dict):
            raise ValueError("Settings must be an object")
        config["_project_root"] = str(root)
        if not isinstance(data, dict):
            raise ValueError("Pass the complete StrokeList/to_dict(), not only list[Stroke]")
        for key, value in (("board_width", board_width), ("board_height", board_height)):
            if value is not None and value != data.get(key):
                raise ValueError(f"{key} must match StrokeList; the program generator never rescales points")
        config.setdefault("motion", {}).update(motion or {})
        if park_upright is not None and config.get("home"):
            config["home"]["park_upright"] = bool(park_upright)
        if park_joints is not None and config.get("home"):
            config["home"]["park_joints_deg"] = [float(v) for v in park_joints]
        for key, value in (("draw_speed_mm_s", draw_speed), ("travel_speed_mm_s", travel_speed),
                           ("pen_down_speed_mm_s", lift_speed), ("retract_speed_mm_s", lift_speed),
                           ("joint_speed_deg_s", joint_speed), ("approach_clearance_mm", approach_mm),
                           ("blend_radius_mm", blend_mm)):
            if value is not None:
                config["motion"][key] = value
        if pen_change_mode is not None:
            config["pen_change_mode"] = pen_change_mode
        if preloaded_marker is not None:
            config["preloaded_marker"] = preloaded_marker
        if pen_stand is not None:
            config["pen_stand"] = dict(pen_stand)
        if not isinstance(allow_stub_export_for_testing, bool):
            raise ValueError("allow_stub_export_for_testing must be a boolean")
        config["allow_stub_export_for_testing"] = allow_stub_export_for_testing
        if program_name is not None:
            if not isinstance(program_name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,60}", program_name):
                raise ValueError("program_name must be a safe identifier of at most 61 characters")
            if program_name != config["program_name"]:
                config["target_frame_name"] = program_name + "_Targets"
            config["program_name"] = program_name
        report["program"] = config["program_name"]
        palette = validate_palette(config)
        strokes = validate_drawing(data, palette)
        motion_parameters(config)
        report["input_sha256"] = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
        generate(config, strokes, palette, report)
        if mode == "export":
            report["stage"] = "export"
            from .export import export_program
            export_program(config, report, root)
        report["stage"] = "complete"
    except Exception as exc:
        report.update(status="failed", error=str(exc), error_type=type(exc).__name__, traceback=traceback.format_exc())
    finally:
        if mode != "status":
            report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            save_report(root, report)
    success = report["status"] == "success"
    return {"success": success, "ok": success, "program_name": report.get("program"),
            "stroke_count": report.get("stroke_count"), "drawing_point_count": report.get("path_point_count"),
            "target_count": report.get("target_count"), "motion_count": report.get("motion_count"),
            "valid_ratio": report.get("valid_ratio"), "simulation_time_s": report.get("time_s"),
            "tcp_distance_mm": report.get("distance_mm"), "exported_files": report.get("exported_files", []),
            "warnings": report.get("warnings", []), "errors": [] if success else [report.get("error", "Unknown error")],
            "message": ("Program created and path checked; no motion started" if success else report.get("error")),
            "report_path": report.get("report_path"), "report": report}


def station_board_size():
    """Board width and height in mm measured by the last calibration, or None."""
    from .connection import connect_existing
    from ..calibration import station_board_size as read_size
    try:
        rdk = connect_existing()
    except Exception:
        return None
    try:
        return read_size(rdk)
    finally:
        rdk.Disconnect()


def robodk_status():
    """The active station name, or None."""
    return status().get("station_name")
