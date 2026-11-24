from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


def main_script_block(script):
    if "# Main program:" in script:
        return script.split("# Main program:", 1)[1].split("# End of main program", 1)[0]
    return script


def verify_script_semantics(script, report, robot=None):
    from robodk.robomath import Mat, UR_2_Pose, invH
    from .motion import phase_for_target

    def vector(text):
        return [float(x.strip()) for x in text.split(',')]

    def pose(values):
        if len(values) != 6:
            raise RuntimeError("Expected a six-component UR pose")
        return UR_2_Pose([v*1000 if i < 3 else v for i, v in enumerate(values)])

    calibration = report["station_calibration"]
    tcp = re.findall(r"set_tcp\(p\[([^\]]+)\]\)", script)
    frames = re.findall(r"\bref_frame\s*=\s*p\[([^\]]+)\]", script)
    if not tcp or not frames:
        raise RuntimeError("Export lacks explicit TCP/reference frame")
    expected_tool = Mat(calibration["pen_tcp_pose"])
    expected_frame = Mat(calibration["reference_frame_pose_robot"])

    def check_pose(actual, expected, label):
        delta = max(abs(actual[i, j]-expected[i, j]) for i in range(4) for j in range(4))
        if delta > .001:
            raise RuntimeError(f"Exported {label} differs from validated station: {delta}")

    check_pose(pose(vector(tcp[-1])), expected_tool, "TCP")
    check_pose(pose(vector(frames[-1])), expected_frame, "reference frame")
    variables, moves, linear_error, joint_error = {}, [], 0., 0.
    parameters = report["motion_parameters"]
    for line in main_script_block(script).splitlines():
        assignment = re.match(r"\s*(?:global\s+)?(speed_ms|speed_rads|accel_mss|accel_radss)\s*=\s*([-+\d.eE]+)", line)
        if assignment:
            variables[assignment[1]] = float(assignment[2])
        movement = re.match(r"\s*(movej|movel)\(", line)
        if not movement:
            continue
        if len(moves) >= len(report["motion_manifest"]):
            raise RuntimeError("Unexpected extra exported move")
        target = report["motion_manifest"][len(moves)]
        expected_type = "movej" if target["motion"] == "J" else "movel"
        blend = re.search(r"\br=([-+\d.eE]+)\)", line)
        if (movement[1] != expected_type or not blend
                or abs(float(blend[1]) - target.get("blend_mm", 0.0) / 1000) > 1e-4):
            raise RuntimeError(f"Export move type/blending changed at {target['target']}")
        expected_pose = Mat(calibration["drawing_orientation_board"]).copy()
        expected_pose.setPos(target["xyz_mm"])
        if expected_type == "movel":
            found = re.search(r"pose_trans\(ref_frame,p\[([^\]]+)\]\)", line)
            if not found:
                raise RuntimeError(f"Unsupported exported MoveL expression: {line}")
            actual_pose = pose(vector(found[1]))
            check_pose(actual_pose, expected_pose, target["target"])
            linear_error = max(linear_error, math.dist(actual_pose.Pos(), expected_pose.Pos()))
            phase = phase_for_target(target["target"])
            expected_variables = {"speed_ms": parameters[f"{phase}_speed_mm_s"] / 1000,
                                  "accel_mss": parameters[f"{phase}_accel_mm_s2"] / 1000}
        else:
            joints = vector(re.search(r"movej\(\[([^\]]+)\]", line)[1])
            expected_variables = {"speed_rads": math.radians(parameters["joint_speed_deg_s"]),
                                  "accel_radss": math.radians(parameters["joint_accel_deg_s2"])}
            if robot is not None:
                actual_pose = invH(expected_frame) * robot.SolveFK([math.degrees(v) for v in joints]) * expected_tool
                joint_error = max(joint_error, math.dist(actual_pose.Pos(), expected_pose.Pos()))
                if joint_error > .1:
                    raise RuntimeError(f"Exported MoveJ endpoint differs at {target['target']}: {joint_error} mm")
        if any(variables.get(k, 0) <= 0 or abs(variables.get(k, float('inf'))-v) > .000501
               for k, v in expected_variables.items()):
            raise RuntimeError(f"Export speed/acceleration mismatch at {target['target']}")
        moves.append(target["target"])
    if len(moves) != len(report["motion_manifest"]):
        raise RuntimeError("Export omitted movements")
    pen_calls = re.findall(r"^\s*(ChangePen_\w+)\(\)\s*$", main_script_block(script), re.M)
    expected_calls = ([p["program"] for p in report["pen_requests"] if p.get("called", True)]
                      if report["pen_call_count"] else [])
    if pen_calls != expected_calls:
        raise RuntimeError(f"Exported pen interface calls differ: {pen_calls} vs {expected_calls}")
    if report["pen_change_mode"] == "stub":
        for name in set(expected_calls):
            block = re.search(r"def\s+" + re.escape(name) + r"\(\):(.+?)\bend\b", script, re.S)
            if not block or "return 0" not in block[1]:
                raise RuntimeError(f"Test pen interface does not return immediate success: {name}")
    return {"tcp_and_frame_match": True, "move_order_matches": True,
            "pen_interface_calls": pen_calls,
            "speed_acceleration_match": True, "exact_stops_verified": True,
            "max_linear_endpoint_rounding_error_mm": linear_error,
            "max_joint_endpoint_fk_error_mm": joint_error if robot is not None else None,
            "coordinate_tolerance_mm": .001, "joint_endpoint_tolerance_mm": .01}


def inspect_ur_files(folder, expected, report=None, robot=None):
    folder = Path(folder)
    scripts, urps = list(folder.glob("*.script")), list(folder.glob("*.urp"))
    if len(scripts) != 1 or len(urps) != 1:
        raise RuntimeError(f"Expected one main SCRIPT and URP; found {scripts}, {urps}")
    script = scripts[0].read_text(encoding="utf-8")
    calls = re.findall(r"^\s*(movej|movel|movep|movec)\s*\(", main_script_block(script), re.M)
    counts = {kind: calls.count(kind) for kind in ("movej", "movel", "movep", "movec")}
    if (counts["movej"] != expected["joint_motion_count"]
            or counts["movel"] != expected["linear_motion_count"]
            or counts["movep"] or counts["movec"]):
        raise RuntimeError(f"Exported motion counts are incomplete or changed: {counts}")
    if not re.search(r"^\s*def\s+\w+\(", script, re.M) or "set_tcp(" not in script:
        raise RuntimeError("Post output lacks a URScript program definition or TCP configuration")
    raw = urps[0].read_bytes()
    xml = gzip.decompress(raw) if raw.startswith(b'\x1f\x8b') else raw
    tree = ET.fromstring(xml)
    if tree.tag != "URProgram":
        raise RuntimeError(f"Unexpected URP XML root: {tree.tag}")
    urscript = scripts[0].with_suffix(".urscript")
    shutil.copyfile(scripts[0], urscript)
    files = [{"path": str(p.resolve()), "bytes": p.stat().st_size,
              "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
             for p in sorted(folder.iterdir()) if p.is_file()]
    semantics = verify_script_semantics(script, report, robot) if report is not None else None
    return {"motion_counts": counts, "urp_xml_root": tree.tag, "semantics": semantics,
            "urscript_is_identical_verified_source": scripts[0].read_bytes() == urscript.read_bytes(),
            "validation_level": "postprocessor output, XML and static URScript checks; not controller execution",
            "files": files}


EDUCATIONAL_LINE_LIMIT = 1000


def _license_name(rdk):
    try:
        return str(rdk.License()[0])
    except Exception:
        return ""


def check_export_size(license_name, moves, instructions, code=0):
    """Stop before RoboDK's post processor refuses the program. A refused export can close RoboDK."""
    name = license_name.lower()
    if "free" in name and moves > 40:
        raise RuntimeError(f"RoboDK license is '{license_name}': it cannot export programs over 50 lines. "
                           "Activate an Educational or full RoboDK license.")
    lines = 70 + moves + code + 4 * max(0, instructions - moves - code)
    if "educational" in name and lines > EDUCATIONAL_LINE_LIMIT:
        raise RuntimeError(f"This drawing is too big to export: it needs about {lines} lines, and the Educational "
                           f"license allows {EDUCATIONAL_LINE_LIMIT}. It can still be simulated. Use a simpler SVG, "
                           "or the same drawing and pen up/down speed, which needs fewer lines.")


def export_program(settings, report, root):
    if report.get("status") != "success" or report.get("valid_ratio") != 1.0 or not report.get("counts_match"):
        raise RuntimeError("Export refused: program has not passed complete RoboDK validation")
    if settings["pen_change_mode"] == "stub" and not settings.get("allow_stub_export_for_testing", False):
        raise RuntimeError("Export refused with STUB pen changes; use explicit single-marker manual_preloaded or real external programs")
    from .connection import connect_existing
    from robodk.robolink import ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT, RUNMODE_MAKE_ROBOTPROG
    from .json_program import OWNER, OWNER_PARAM
    rdk = connect_existing()
    previous_post = None
    robot = None
    try:
        from .station import check_station
        station = check_station(rdk, settings)
        program = rdk.Item(settings["program_name"], ITEM_TYPE_PROGRAM)
        if not program.Valid() or program.getParam(OWNER_PARAM) != OWNER:
            raise RuntimeError("Validated owned program is missing")
        if program.getParam("DrawBotGeneration") != report.get("generation_id", "").encode():
            raise RuntimeError("Export refused: validation report belongs to a different program generation")
        robot = rdk.Item(settings["robot_name"], ITEM_TYPE_ROBOT)
        previous_post = robot.setParam("PostProcessor")
        post = "Universal_Robots"
        robot.setParam("PostProcessor", post)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        folder = Path(root) / "exports" / f"{settings['program_name']}_{stamp}"
        folder.mkdir(parents=True, exist_ok=False)
        report["export"] = {"postprocessor": post, "output_directory": str(folder),
                            "run_mode": "RUNMODE_MAKE_ROBOTPROG", "upload": False,
                            "pen_change_mode": settings["pen_change_mode"], "deployment_ready": False,
                            "stub_test_only": settings["pen_change_mode"] == "stub"}
        pen_checks = report.get("pen_program_checks", {}).values()
        check_export_size(_license_name(rdk), len(report["motion_manifest"]) + sum(c["moves"] for c in pen_checks),
                          report.get("instruction_count", 0) + sum(c["instructions"] for c in pen_checks),
                          report.get("code_instruction_count", 0) + sum(c["code"] for c in pen_checks))
        rdk.Command("ProgramsShow", "0")
        ok, log, transferred = program.MakeProgram(str(folder), RUNMODE_MAKE_ROBOTPROG)
        report["export"].update(api_success=ok, log=log, transfer_succeeded=transferred)
        if transferred:
            raise RuntimeError("Unexpected transfer flag in offline-only export")
        if not ok:
            raise RuntimeError(f"RoboDK UR export failed: {log or 'no message from RoboDK. Check its license, the post processor and the program size.'}")
        inspection = inspect_ur_files(folder, report["expected_counts"], report, robot)
        report["export"].update(inspection)
        report["exported_files"] = [x["path"] for x in inspection["files"]]
        (folder / "export_manifest.json").write_text(json.dumps(report["export"], indent=2), encoding="utf-8")
        return report["export"]
    finally:
        if robot is not None and previous_post:
            robot.setParam("PostProcessor", previous_post)
        rdk.Command("ProgramsShow", "1")
        rdk.Disconnect()
