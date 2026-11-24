from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
try:
    import fcntl
except ImportError:
    fcntl = None
import hashlib
import json
import math
from pathlib import Path
import uuid

from .robot import api as robot_api

ROOT = Path(__file__).resolve().parents[1]
BASELINE = {"approach_mm": 80.0, "draw_speed": 150.0, "lift_speed": 150.0, "joint_speed": 120.0, "blend_mm": 1.0,
            "park_upright": True, "park_joints": None}
PROGRAM_NAME = "DrawBot_Drawing"
EXPORT_MODES = {
    "Pen stand: the robot changes pens itself": "stand",
    "Single marker — manually preloaded": "manual_preloaded",
    "STUB test export — no physical pen changes": "stub",
    "External — validated real ChangePen programs": "external",
}


def drawing_dict(stroke_list):
    """Copy full precision fields; never call the rounding serializer."""
    if not is_dataclass(stroke_list) or isinstance(stroke_list, type):
        raise ValueError("UI must pass a complete StrokeList dataclass")
    return json.loads(json.dumps(asdict(stroke_list), allow_nan=False))


def strokes_json(stroke_list):
    return json.dumps(drawing_dict(stroke_list), ensure_ascii=False, indent=2, allow_nan=False)


def request_fingerprint(stroke_list, settings, svg_bytes, options=None):
    value = {"drawing": drawing_dict(stroke_list), "settings": settings,
             "svg_sha256": hashlib.sha256(svg_bytes).hexdigest(), "options": options or {}}
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def external_preflight(approach_mm):
    """Read-only offline checks; never move or edit programs DrawBot did not create."""
    from .robot.connection import connect_existing
    from robodk.robolink import (ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME,
                                ITEM_TYPE_TOOL, PROGRAM_RUN_ON_SIMULATOR, COLLISION_ON)
    from robodk.robomath import invH
    config = json.loads((ROOT / "config/drawing.json").read_text(encoding="utf-8"))
    rdk = connect_existing()
    try:
        from .robot.station import check_station
        station = check_station(rdk, config)
        checks = {}
        for pen in config["palette"]:
            candidates = [p for p in rdk.ItemList(ITEM_TYPE_PROGRAM) if p.Name() == pen["program"]]
            if len(candidates) != 1:
                raise ValueError(f"One real program required: {pen['program']}")
            program = candidates[0]
            parent = program.Parent()
            while parent.Valid() and parent != station:
                parent = parent.Parent()
            if parent != station or program.getParam("DrawBotStub") == b"1":
                raise ValueError(f"Real non-STUB ChangePen program required: {pen['program']}")
            if program.RunType() != PROGRAM_RUN_ON_SIMULATOR or program.Busy():
                raise ValueError(f"ChangePen must be idle and set to simulator: {pen['program']}")
            count = program.InstructionCount()
            valid, seconds, distance, ratio, message = program.Update(
                COLLISION_ON, timeout_sec=120, mm_step=1.0, deg_step=1.0)
            if (count <= 0 or not all(math.isfinite(v) for v in (valid, seconds, distance, ratio))
                    or valid < count or ratio != 1.0):
                raise ValueError(f"ChangePen offline validation failed: {pen['program']}: {message}")
            checks[pen["program"]] = {"valid_instructions": valid, "instruction_count": count,
                                       "valid_ratio": ratio, "message": message}
        robot = rdk.Item(config["robot_name"], ITEM_TYPE_ROBOT)
        board = rdk.Item(config["board_frame"], ITEM_TYPE_FRAME)
        tool = rdk.Item(config["tool_name"], ITEM_TYPE_TOOL)
        tcp = invH(board.PoseAbs()) * robot.PoseAbs() * robot.SolveFK(robot.Joints()) * tool.PoseTool()
        if tcp.Pos()[2] > -approach_mm + 1e-6:
            raise ValueError("Initial TCP is not retracted to the requested clearance; external pen change refused")
        return checks
    finally:
        rdk.Disconnect()


def submit_drawing(stroke_list, mode, settings, *, svg_bytes, source_name,
                   program_name, export_mode="manual_preloaded", preloaded_marker="black",
                   stub_confirmed=False, external_confirmed=False, project_root=None, pen_stand=None):
    """Serialize UI requests and retain the actual uploaded input as evidence."""
    root = Path(project_root or ROOT)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    directory = root / "logs/ui_requests" / (stamp + "_" + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    metadata = {"source_name": source_name, "svg_sha256": hashlib.sha256(svg_bytes).hexdigest(),
                "mode": mode, "settings": settings, "export_mode": export_mode,
                "stub_confirmed": stub_confirmed, "external_confirmed": external_confirmed,
                "program_name": program_name, "physical_robot_used": False, "pen_stand": pen_stand}
    result = None
    try:
        data = drawing_dict(stroke_list)
        (directory / "input.svg").write_bytes(svg_bytes)
        snapshot = directory / "strokes.json"
        snapshot.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        metadata["stroke_list_sha256"] = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
        if mode not in ("simulate", "export"):
            raise ValueError("Only offline simulate/export are available")
        if set(settings) != set(BASELINE):
            raise ValueError("UI motion settings must contain " + ", ".join(BASELINE))
        markers = {s["marker"] for s in data["strokes"]}
        single = export_mode == "manual_preloaded" and markers == {preloaded_marker}
        if export_mode == "stand":
            pen_mode = "stand"
        else:
            pen_mode = "manual_preloaded" if mode == "simulate" and single else "stub" if mode == "simulate" else export_mode
        if mode == "export":
            if pen_mode == "manual_preloaded" and markers != {preloaded_marker}:
                raise ValueError("Manual preload export requires every stroke to use the selected marker")
            if pen_mode == "stub" and stub_confirmed is not True:
                raise ValueError("Explicit STUB test-export confirmation is required")
            if pen_mode == "external" and external_confirmed is not True:
                raise ValueError("Confirm the real ChangePen synchronous-return/retracted-TCP contract first")
            if pen_mode not in ("manual_preloaded", "stub", "external", "stand"):
                raise ValueError("Unknown export pen mode")
        with (root / "logs/ui_job.lock").open("a") as lock:
            try:
                if fcntl is not None:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("Another Web UI RoboDK request is running; retry when it finishes") from None
            if pen_mode == "external":
                metadata["external_program_checks"] = external_preflight(settings["approach_mm"])
            result = robot_api.draw_in_robodk(
                stroke_list, mode=mode, **settings, program_name=program_name,
                pen_change_mode=pen_mode, preloaded_marker=preloaded_marker,
                pen_stand=pen_stand if pen_mode == "stand" else None,
                allow_stub_export_for_testing=(mode == "export" and pen_mode == "stub" and stub_confirmed is True),
                project_root=root, source_file=snapshot)
        if result["ok"] and result["report"].get("input_sha256") != metadata["stroke_list_sha256"]:
            raise RuntimeError("The program result does not match this UI StrokeList; discard this result")
        result["ui_pen_mode"] = pen_mode
        result["animation_played"] = False
        if pen_mode == "stub":
            result.setdefault("warnings", []).append("Pen changes are simulated; any STUB returns immediately. Physical pen motion/time is unverified.")
        if mode == "export" and pen_mode == "stub":
            result["warnings"].append("STUB TEST EXPORT ONLY — not ready for physical robot deployment.")
    except Exception as exc:
        result = {"ok": False, "success": False, "program_name": program_name,
                  "message": str(exc), "errors": [str(exc)], "warnings": [],
                  "exported_files": [], "report_path": str(directory / "result.json"), "report": {}}
    result["ui_request_path"] = str(directory / "request.json")
    result["ui_input_sha256"] = metadata.get("stroke_list_sha256")
    (directory / "request.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    (directory / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def downloadable_files(result):
    """Only this successful request's hash-checked files inside project exports."""
    if not result.get("ok"):
        return []
    records = result.get("report", {}).get("export", {}).get("files", [])
    verified = []
    for filename in result.get("exported_files", []):
        path = Path(filename).resolve()
        if not path.is_relative_to((ROOT / "exports").resolve()) or path.suffix not in (".urp", ".script", ".urscript"):
            raise ValueError("Unexpected export download path")
        record = next((r for r in records if Path(r["path"]).resolve() == path), None)
        content = path.read_bytes()
        if record is None or hashlib.sha256(content).hexdigest() != record["sha256"]:
            raise ValueError("Export file changed since validation; generate again")
        verified.append((path, content))
    return verified


def render_result(st, result):
    (st.success if result["ok"] else st.error)(result.get("message") or "Request failed")
    report = result.get("report", {})
    st.write({"Program name": result.get("program_name"), "Strokes": result.get("stroke_count"),
              "Points": result.get("drawing_point_count"), "Targets": result.get("target_count"),
              "Motions": result.get("motion_count"), "Requested marker changes": report.get("pen_change_count"),
              "Valid ratio": result.get("valid_ratio"), "RoboDK estimated runtime (s)": result.get("simulation_time_s"),
              "TCP distance (mm)": result.get("tcp_distance_mm")})
    st.caption("Simulate creates a program and checks its path. Animation has not been played; no robot execution is started.")
    for message in result.get("warnings", []):
        st.warning(message)
    for message in result.get("errors", []):
        st.error(message)
    if result.get("report_path"):
        st.caption("Diagnostic report: " + result["report_path"])
    st.caption("Current input evidence: " + result["ui_request_path"])
    try:
        for path, content in downloadable_files(result):
            st.code(str(path), language=None)
            st.download_button("Download " + path.name, content, file_name=path.name,
                               mime="application/octet-stream", key="job_file_" + str(path), on_click="ignore")
        if result.get("exported_files"):
            st.info("Keep .urp and its original .script together. .urscript is the verified identical source copy. Controller execution remains unverified.")
    except (OSError, ValueError) as exc:
        st.error(str(exc))


def _pen_stand_panel(st, stroke_list):
    """Pen stand options. Returns (pen_stand settings, problem text or None)."""
    from . import robot_link
    from .pen_stand import check_pickup_points, check_slots, stand_measurement
    from .robot_panel import station_settings
    ss = st.session_state
    order = list(dict.fromkeys(s.marker for s in stroke_list.strokes))
    station = station_settings(ss)
    steps, current = [], station["held_marker"]
    for marker in order:
        if marker != current:
            steps.append(f"{current} → {marker}" if current else f"pick {marker}")
        current = marker
    if current:
        steps.append(f"put {current} back")
    st.caption("Pen changes, each from home and back: " + (", ".join(steps) if steps else "none")
               + ". Slots, speeds and the pen in the gripper are set in Station mode.")
    measurement = stand_measurement()
    problem = None
    try:
        check_slots(station["slots"] or {})
        check_pickup_points(measurement)
    except ValueError as exc:
        problem = str(exc)
        st.warning(problem)
    return {"pickup_data": str(measurement) if measurement else None, **station}, problem


def robot_actions(st, stroke_list, settings, *, svg_bytes, source_name, single_marker=None):
    ss = st.session_state
    markers = {s.marker for s in stroke_list.strokes}
    pen_stand, stand_problem = None, None
    if single_marker:
        pen_mode, preloaded, stub_ack, external_ack = "manual_preloaded", single_marker, False, False
    else:
        with st.expander("Robot export / pen-change mode", expanded=True):
            label = st.selectbox("Export pen mode", list(EXPORT_MODES), key="job_export_mode")
            pen_mode = EXPORT_MODES[label]
            preloaded = st.selectbox("Manually preloaded marker", ["black", "green", "purple", "orange"], key="job_preloaded") if pen_mode == "manual_preloaded" else "black"
            stub_ack = st.checkbox("I understand: STUB export is a software test, with no real pen changes", key="job_stub_ack") if pen_mode == "stub" else False
            external_ack = st.checkbox("Real ChangePen programs synchronously return with TCP retracted; hardware acceptance is separate", key="job_external_ack") if pen_mode == "external" else False
            if pen_mode == "stand":
                pen_stand, stand_problem = _pen_stand_panel(st, stroke_list)
            else:
                st.caption("Simulation permits pen-change STUB interfaces. Export never silently enables STUBs.")
    options = {"pen_mode": pen_mode, "preloaded": preloaded, "stub_ack": stub_ack, "external_ack": external_ack,
               "pen_stand": pen_stand}
    fingerprint = request_fingerprint(stroke_list, settings, svg_bytes, options)
    if ss.get("job_result_fingerprint") != fingerprint:
        ss.pop("job_result", None)
    a1, a2, a3 = st.columns([1, 1, 2], vertical_alignment="center")
    simulate = a1.button("Simulate in RoboDK", type="primary", use_container_width=True,
                         disabled=not stroke_list.strokes or stand_problem is not None)
    blocked = (not stroke_list.strokes or stand_problem is not None
               or (pen_mode == "manual_preloaded" and markers != {preloaded})
               or (pen_mode == "stub" and not stub_ack) or (pen_mode == "external" and not external_ack))
    export = a2.button("Generate robot program", use_container_width=True, disabled=blocked)
    a3.download_button("Download strokes.json", strokes_json(stroke_list), file_name="strokes.json", mime="application/json", on_click="ignore")
    if blocked:
        st.caption("To export: use Single marker, use the pen stand with a valid measurement, "
                   "or acknowledge a test or real pen-change mode above.")
    if simulate or export:
        with st.spinner("Generating and checking the current SVG in RoboDK…"):
            result = submit_drawing(stroke_list, "export" if export else "simulate", settings,
                                    svg_bytes=svg_bytes, source_name=source_name,
                                    program_name=ss.get("job_program_name", PROGRAM_NAME), export_mode=pen_mode,
                                    preloaded_marker=preloaded, stub_confirmed=stub_ack, external_confirmed=external_ack,
                                    pen_stand=pen_stand)
        ss.job_result, ss.job_result_fingerprint = result, fingerprint
        ss.log.insert(0, result.get("message", "Request finished"))
    if "job_result" in ss:
        render_result(st, ss.job_result)
