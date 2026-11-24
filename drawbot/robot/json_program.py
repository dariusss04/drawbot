import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import traceback

from .dataset import blend_radii, color_changes, stroke_moves, validate_drawing, validate_palette
from .motion import motion_parameters, phase_for_target


OWNER_PARAM = "DrawBotOwner"
OWNER = b"DrawBot-StrokeList-v1"
PEN_PROGRAM_PREFIX = "ChangePen_"


def save_report(root, report):
    root = Path(root)
    directory = root / "logs"
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    destination = directory / f"json_generation_{stamp}.json"
    report["report_path"] = str(destination)
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    destination.write_text(text, encoding="utf-8")
    (directory / "last_json_generation.json").write_text(text, encoding="utf-8")
    if report.get("status") == "success":
        (directory / "last_successful_generation.json").write_text(text, encoding="utf-8")
    return destination


def drawing_statistics(strokes, palette, settings=None):
    """Count the moves without connecting to RoboDK. A configured home adds a MoveJ at each end."""
    points = sum(len(s["points"]) for s in strokes)
    home = settings.get("home") if settings else None
    home_moves = (3 if home.get("park_upright") else 2) if home else 0
    if settings and settings.get("pen_change_mode") == "stand":
        home_moves += max(0, len(list(color_changes(strokes, palette))) - 1)
    return {
        "stroke_count": len(strokes), "path_point_count": points,
        "motion_count": points + 2 * len(strokes) + home_moves,
        "joint_motion_count": len(strokes) + home_moves,
        "linear_motion_count": points + len(strokes),
        "pen_change_count": len(list(color_changes(strokes, palette))),
    }


def generate(settings, strokes, palette, report=None):
    if report is None:
        report = {}
    expected = drawing_statistics(strokes, palette, settings)
    motion_config = motion_parameters(settings)
    if (settings["pen_change_mode"] == "manual_preloaded"
            and {s["marker"] for s in strokes} != {settings.get("preloaded_marker")}):
        raise ValueError("manual_preloaded only supports strokes matching the single preloaded marker")
    stand = settings["pen_change_mode"] == "stand"
    if stand:
        from ..pen_stand import check_pickup_points, pens
        options = settings.get("pen_stand") or {}
        if not settings.get("home"):
            raise ValueError("Pen stand mode needs a home pose in the config")
        check_pickup_points(options.get("pickup_data"))
        held = options.get("held_marker")
        held = None if held in (None, "", "none") else held
        stand_pens = pens(options.get("slots"))
        missing = {s["marker"] for s in strokes} - set(stand_pens)
        if missing or (held is not None and held not in stand_pens):
            raise ValueError(f"The pen stand has no {', '.join(sorted(missing or {held}))} pen")
    report.update(
        status="generating", stage="connect", expected_counts=expected,
        stroke_count=len(strokes), path_point_count=expected["path_point_count"],
        target_count=0, motion_count=0, joint_motion_count=0, linear_motion_count=0,
        pen_change_count=0,
        program=settings["program_name"], pen_change_mode=settings["pen_change_mode"],
        target_frame_name=settings["target_frame_name"],
        hardware_mode="existing_station_read_only", hardware=settings["hardware"],
        pen_requests=[], motion_manifest=[], valid_instructions=None,
        valid_ratio=None, time_s=None, distance_mm=None, message="",
        motion_parameters=motion_config, warnings=list(motion_config["warnings"]),
        pen_call_count=0, deployment_ready=False,
    )
    from robodk.robolink import (
        COLLISION_ON, INSTRUCTION_CALL_PROGRAM, INSTRUCTION_COMMENT, INSTRUCTION_INSERT_CODE,
        INS_TYPE_CODE, INS_TYPE_MOVE, MOVE_TYPE_JOINT, MOVE_TYPE_LINEAR, RUNMODE_SIMULATE,
        ITEM_TYPE_FRAME, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT,
        ITEM_TYPE_TARGET, ITEM_TYPE_TOOL, PROGRAM_RUN_ON_SIMULATOR, Robolink,
    )
    from robodk.robomath import eye, invH

    rdk = None
    try:
        from .connection import connect_existing
        rdk = connect_existing()
        rdk.setRunMode(RUNMODE_SIMULATE)
        rdk.Render(False)
        report["stage"] = "station_preflight"
        from .station import check_station
        station = check_station(rdk, settings)
        root = Path(settings.get("_project_root", Path(__file__).resolve().parents[2]))
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        def save_station(path, key):
            """Record a saved station only if the file really exists. A limited license refuses silently."""
            path.parent.mkdir(parents=True, exist_ok=True)
            path.unlink(missing_ok=True)
            rdk.Save(str(path), station)
            if path.is_file():
                report[key] = str(path)
            else:
                report["warnings"].append(f"RoboDK did not save {path.name}. Check its license.")

        try:
            license_name = str(rdk.License()[0])
        except Exception:
            license_name = ""
        report["license"] = license_name
        if "free" in license_name.lower():
            report["warnings"].append(f"RoboDK license is '{license_name}': saving the station and exporting programs over 50 lines are blocked. Simulation still works.")
        report["stage"] = "station_backup"
        save_station(root / "backups" / "station_before_last_run.rdk", "station_backup")
        report["stage"] = "station_preflight"

        def find(name, kind):
            matches = []
            for item in rdk.ItemList(kind):
                if item.Name() != name:
                    continue
                parent = item.Parent()
                while parent.Valid() and parent != station:
                    parent = parent.Parent()
                if parent == station:
                    matches.append(item)
            if len(matches) > 1:
                raise RuntimeError(f"Ambiguous RoboDK item in active station: {name}")
            return matches[0] if matches else None

        def require(name, kind):
            item = find(name, kind)
            if item is None:
                raise RuntimeError(f"Missing RoboDK item: {name}")
            return item

        robot = require(settings["robot_name"], ITEM_TYPE_ROBOT)
        board = require(settings["board_frame"], ITEM_TYPE_FRAME)
        tool = require(settings["tool_name"], ITEM_TYPE_TOOL)
        reference = require(settings["orientation_target"], ITEM_TYPE_TARGET)
        gripper = require(settings["hardware"]["gripper_name"], ITEM_TYPE_ROBOT)
        mount = require(settings["hardware"]["mount_frame_name"], ITEM_TYPE_FRAME)
        if gripper.Parent() != mount or mount.Parent() != robot or tool.Parent() != robot:
            raise RuntimeError("Existing Hand-E mount and PenTCP must belong to the configured robot")
        report["station_calibration"] = {
            "board_pose_abs": board.PoseAbs().Rows(),
            "pen_tcp_pose": tool.PoseTool().Rows(),
            "gripper_mount_pose": mount.Pose().Rows(),
            "reference_frame_pose_robot": (invH(robot.PoseAbs()) * board.PoseAbs()).Rows(),
        }
        from .verification import collision_map_report
        report["collision_map"] = collision_map_report(rdk, robot, gripper, tool)
        if not report["collision_map"]["coverage_ok"]:
            raise RuntimeError("Collision Map is missing robot, gripper or board geometry coverage; inspect collision_map report")
        orientation = invH(board.PoseAbs()) * reference.PoseAbs()
        report["station_calibration"]["drawing_orientation_board"] = orientation.Rows()

        program_name = settings["program_name"]
        target_frame_name = settings["target_frame_name"]
        if target_frame_name == settings["board_frame"]:
            raise RuntimeError("Generated target frame must not be BoardFrame")
        if program_name in {pen["program"] for pen in palette.values()}:
            raise RuntimeError("Generated program must not be a pen-change program")
        old_outputs = [find(program_name, ITEM_TYPE_PROGRAM),
                       find(target_frame_name, ITEM_TYPE_FRAME)]

        def check_owned(item):
            if item.getParam(OWNER_PARAM) != OWNER:
                raise RuntimeError(
                    f"Refusing to delete unowned item: {item.Name()}. "
                    "Rename the legacy Program/Targets frame in the GUI to keep them, then rerun.")
            for child in item.Childs():
                if child.Parent() == item:
                    check_owned(child)

        report["preserved_legacy_outputs"] = []
        for index, old in enumerate(old_outputs):
            if old is not None:
                try:
                    check_owned(old)
                except RuntimeError as exc:
                    previous_name = old.Name()
                    preserved_name = f"{previous_name}_preserved_{stamp}"
                    old.setName(preserved_name)
                    report["preserved_legacy_outputs"].append(
                        {"original_name": previous_name, "preserved_name": preserved_name,
                         "reason": str(exc)})
                    old_outputs[index] = None

        def own(item):
            item.setParam(OWNER_PARAM, OWNER)
            return item

        def hidden(item):
            """Generated targets and frames would bury the drawing in arrows and labels."""
            item.setVisible(False, False)
            return item

        for marker, pen in palette.items():
            if settings["pen_change_mode"] in ("manual_preloaded", "stand"):
                continue
            change = find(pen["program"], ITEM_TYPE_PROGRAM)
            exists = change is not None
            if settings["pen_change_mode"] == "external":
                if not exists or change.getParam("DrawBotStub") == b"1":
                    raise RuntimeError(f"Real pen-change program required: {pen['program']}")
            elif not exists:
                change = rdk.AddProgram(pen["program"], robot)
                change.setRunType(PROGRAM_RUN_ON_SIMULATOR)
                change.RunInstruction(
                    f"STUB: assume {pen['name']} pen ({pen['hex']}) loaded successfully",
                    INSTRUCTION_COMMENT,
                )
                change.setParam("DrawBotStub", b"1")
            if (settings["pen_change_mode"] == "stub" and change.getParam("DrawBotStub") == b"1"
                    and change.getParam("DrawBotStubReturn") != b"1"):
                change.RunInstruction("return 0", INSTRUCTION_INSERT_CODE)
                change.setParam("DrawBotStubReturn", b"1")

        report["stage"] = "replace_owned_outputs"
        for old in old_outputs:
            if old is not None:
                old.Delete()
        for old in rdk.ItemList(ITEM_TYPE_PROGRAM):
            if old.Name().startswith(PEN_PROGRAM_PREFIX) and old.getParam(OWNER_PARAM) == OWNER:
                old.Delete()

        target_frame = hidden(own(rdk.AddFrame(target_frame_name, board)))
        target_frame.setPose(eye(4))
        program = own(rdk.AddProgram(program_name, robot))
        report["generation_id"] = stamp
        program.setParam("DrawBotGeneration", stamp.encode())
        program.setRunType(PROGRAM_RUN_ON_SIMULATOR)
        program.setPoseFrame(board)
        program.setPoseTool(tool)
        program.setSpeed(settings["linear_speed_mm_s"], settings["joint_speed_deg_s"],
                         settings["linear_accel_mm_s2"], settings["joint_accel_deg_s2"])
        program.setRounding(-1)
        current_speed = (settings["linear_speed_mm_s"], settings["joint_speed_deg_s"],
                         settings["linear_accel_mm_s2"], settings["joint_accel_deg_s2"])

        changes = dict(color_changes(strokes, palette))
        joint_seed = robot.Joints()
        report["initial_robot_joints_deg"] = joint_seed.list()
        board_reference = invH(robot.PoseAbs()) * board.PoseAbs()
        report["approach_joint_cache_refreshed"] = 0

        current_rounding = 0.0

        def add_target(name, xyz, motion, parent, stroke_id, source, joints=None, blend=0.0, joint_target=False):
            nonlocal joint_seed, current_speed, current_rounding
            context = {"stroke": stroke_id, "source": source, "target": name, "xyz_mm": xyz, "motion": motion}
            report["context"] = context
            phase = phase_for_target(name)
            speed = (motion_config[f"{phase}_speed_mm_s"], motion_config["joint_speed_deg_s"],
                     motion_config[f"{phase}_accel_mm_s2"], motion_config["joint_accel_deg_s2"])
            if speed != current_speed:
                program.setSpeed(*speed)
                current_speed = speed
            pose = orientation.copy()
            pose.setPos(xyz)
            target = hidden(own(rdk.AddTarget(name, parent, robot)))
            report["target_count"] += 1
            if joint_target:
                target.setAsJointTarget()
            else:
                target.setAsCartesianTarget()
                target.setPose(pose)
            want = blend if motion == "L" else 0.0
            if want != current_rounding:
                program.setRounding(want if want > 0 else -1)
                current_rounding = want
            context["blend_mm"] = want
            if motion == "J":
                solution = joints if joints is not None else robot.SolveIK(
                    pose, joint_seed, tool=tool.PoseTool(), reference=board_reference)
                if len(solution.list()) < 6:
                    raise RuntimeError(f"No IK solution for {name}. Is it inside the robot's reach?")
                target.setJoints(solution)
                joint_seed = solution
                report["approach_joint_cache_refreshed"] += 1
                program.MoveJ(target)
            else:
                program.MoveL(target)
            report["motion_count"] += 1
            report["joint_motion_count" if motion == "J" else "linear_motion_count"] += 1
            report["motion_manifest"].append({**context, "instruction_index": program.InstructionCount() - 1})
            print(f"Added {motion}: {name}, XYZ={xyz}")

        home_joints = None
        home = settings.get("home")
        if home:
            from robodk.robomath import Mat
            pose = orientation.copy()
            pose.setPos(home["board_xyz_mm"])
            from ..calibration import station_home_seed
            seed = station_home_seed(rdk) or home["seed_joints_deg"]
            report["home_seed_source"] = "calibration" if seed is not home["seed_joints_deg"] else "config"
            home_joints = robot.SolveIK(pose, Mat(seed), tool=tool.PoseTool(), reference=board_reference)
            if len(home_joints.list()) < 6:
                raise RuntimeError("No IK solution for the home pose. Check BoardFrame and the home settings in the config.")
            report["home_joints_deg"] = home_joints.list()
            home_group = hidden(own(rdk.AddFrame("Home", target_frame)))
            home_group.setPose(eye(4))
            add_target("Home_Start", list(home["board_xyz_mm"]), "J", home_group, 0, "home", home_joints)
        pen_programs = []

        def stand_change(current, new):
            """The pen changer's change_pen, from home and back to home, written into its own sub program.
            The main program only calls it, so its moves stay out of the drawing's move checks."""
            nonlocal current_speed, current_rounding
            from ..pen_stand import configure, quiet_collisions
            pc = configure(options["pickup_data"], options.get("grip_force"), options.get("slots"),
                           options.get("speed_mm_s"), options.get("joint_speed_deg_s"))
            sub = own(rdk.AddProgram(f"{PEN_PROGRAM_PREFIX}{len(pen_programs) + 1}_{new or 'none'}", robot))
            sub.setRunType(PROGRAM_RUN_ON_SIMULATOR)
            robot.setPoseFrame(board)
            robot.setPoseTool(tool)
            robot.setJoints(home_joints)
            pc.change_pen(None, current, new, rdk, sub, animate=False)
            quiet_collisions(rdk)
            robot.setJoints(home_joints)
            program.RunInstruction(sub.Name(), INSTRUCTION_CALL_PROGRAM)
            if current_speed is not None:
                linear, joint, linear_accel, joint_accel = current_speed
                for line in (f"speed_ms = {linear / 1000:.6f}", f"speed_rads = {math.radians(joint):.6f}",
                             f"accel_mss = {linear_accel / 1000:.6f}", f"accel_radss = {math.radians(joint_accel):.6f}"):
                    program.RunInstruction(line, INSTRUCTION_INSERT_CODE)
            program.setPoseFrame(board)
            program.setPoseTool(tool)
            current_speed, current_rounding = None, None
            report["pen_call_count"] += 1
            pen_programs.append(sub)
            return sub.Name()

        report["stage"] = "generate_targets"
        for stroke_id, stroke in enumerate(strokes, 1):
            report["context"] = {"stroke": stroke_id, "source": stroke["source"]}
            if stroke_id in changes and stand:
                pen = changes[stroke_id]
                if stroke_id != 1:
                    add_target(f"Home_Pen{report['pen_change_count'] + 1}", list(home["board_xyz_mm"]), "J",
                               home_group, 0, "home", home_joints)
                called = stroke["marker"] != held
                name = stand_change(held, stroke["marker"]) if called else None
                report["pen_change_count"] += 1
                report["pen_requests"].append({"stroke": stroke_id, "marker": stroke["marker"], "from": held,
                                               "program": name, "called": called})
                print(f"S{stroke_id:02d}: pen {held or 'none'} -> {stroke['marker']}" + (f" ({name})" if called else " (already held)"))
                held = stroke["marker"]
            elif stroke_id in changes:
                pen = changes[stroke_id]
                if settings["pen_change_mode"] != "manual_preloaded":
                    program.RunInstruction(pen["program"], INSTRUCTION_CALL_PROGRAM)
                    report["pen_call_count"] += 1
                report["pen_change_count"] += 1
                report["pen_requests"].append({"stroke": stroke_id, "marker": stroke["marker"], **pen})
                action = "manual preloaded" if settings["pen_change_mode"] == "manual_preloaded" else pen["program"]
                print(f"S{stroke_id:02d}: request {pen['name']} ({pen['hex']}) -> {action}")
            group = hidden(own(rdk.AddFrame(f"Stroke {stroke_id}", target_frame)))
            group.setPose(eye(4))
            radii = blend_radii(stroke["points"], motion_config["effective_blend_radius_mm"])
            for index, (name, xyz, motion) in enumerate(stroke_moves(
                    stroke_id, stroke, -motion_config["approach_clearance_mm"], settings["draw_z_mm"])):
                blend = radii[index - 1] if 1 <= index <= len(radii) else 0.0
                add_target(name, xyz, motion, group, stroke_id, stroke["source"], blend=blend)

        if home:
            add_target("Home_End", list(home["board_xyz_mm"]), "J", home_group, 0, "home", home_joints)
            if stand and held is not None:
                name = stand_change(held, None)
                report["pen_requests"].append({"stroke": None, "marker": None, "from": held,
                                               "program": name, "called": True})
                report["held_after"] = None
            if home.get("park_upright"):
                given = home.get("park_joints_deg")
                if given is not None and (len(given) != 6 or not all(math.isfinite(v) for v in given)):
                    raise ValueError("The home pose needs 6 joint angles")
                park = Mat(list(given) if given is not None else [home_joints.list()[0], -90.0, 0.0, -90.0, 0.0, 0.0])
                tip = invH(board_reference) * robot.SolveFK(park) * tool.PoseTool()
                add_target("Home_Park", [round(v, 6) for v in tip.Pos()], "J", home_group, 0, "home", park,
                           joint_target=True)
                report["park_joints_deg"] = park.list()

        report.pop("context", None)
        report["stage"] = "statistics_check"
        instruction_count = program.InstructionCount()
        move_types = [ins[2] for i in range(instruction_count)
                      if (ins := program.Instruction(i))[1] == INS_TYPE_MOVE]
        actual = {
            "motion_count": len(move_types),
            "joint_motion_count": move_types.count(MOVE_TYPE_JOINT),
            "linear_motion_count": move_types.count(MOVE_TYPE_LINEAR),
            "pen_change_count": report["pen_change_count"],
        }
        report.update(instruction_count=instruction_count, actual_counts=actual,
                      code_instruction_count=sum(1 for i in range(instruction_count)
                                                 if program.Instruction(i)[1] == INS_TYPE_CODE))
        if (report["target_count"] != expected["motion_count"]
                or report["motion_count"] != expected["motion_count"]
                or any(v != expected[k] for k, v in actual.items())):
            raise RuntimeError(f"Generated counts do not match input: {actual}, expected {expected}")
        report["counts_match"] = True
        report["stage"] = "path_check"
        check_timeout = 600 if pen_programs else 120
        try:
            valid, time_s, distance, ratio, message = program.Update(
                COLLISION_ON, timeout_sec=check_timeout, mm_step=1.0, deg_step=1.0)
        except TimeoutError:
            raise RuntimeError(f"RoboDK's path check took longer than {check_timeout} s and was stopped. "
                               "Close other RoboDK windows and try again; RoboDK may need a restart.") from None
        metrics = dict(valid_instructions=valid, time_s=time_s, distance_mm=distance, valid_ratio=ratio)
        invalid_metrics = {k: repr(v) for k, v in metrics.items() if not math.isfinite(v)}
        report.update({k: v if math.isfinite(v) else None for k, v in metrics.items()})
        report["message"] = message
        if invalid_metrics:
            report["invalid_update_values"] = invalid_metrics
        print(f"\nPath validity: {ratio:.1%}; motion estimate: {time_s:.2f} s; {distance:.2f} mm")
        print(message)
        if settings["pen_change_mode"] == "stub":
            print("Pen changes use placeholders; their real motion/time is not included.")
            report["warnings"].append("STUB pen changes: simulation only; no real pen motion/time")
        elif settings["pen_change_mode"] == "manual_preloaded":
            report["warnings"].append("Single-marker manual-preloaded test; no automatic pen change")
        if (invalid_metrics or ratio != 1.0 or valid < instruction_count
                or time_s < 0 or distance < 0):
            from .verification import diagnose_path
            report["path_diagnostics"] = diagnose_path(program, report["motion_manifest"])
            if math.isfinite(valid):
                report["diagnostic_candidate"] = next(
                    (m for m in report["motion_manifest"] if m["instruction_index"] >= int(valid)), None)
                report["diagnostic_basis"] = "First motion at/after Update valid-instruction prefix; confirm in GUI"
            raise RuntimeError(f"Path check failed: {message}")
        if pen_programs:
            report["stage"] = "pen_change_check"
            report["pen_program_checks"] = {}
            for sub in pen_programs:
                count = sub.InstructionCount()
                print(f"Checking {sub.Name()} ({count} instructions)")
                valid, seconds, distance, ratio, message = sub.Update(
                    COLLISION_ON, timeout_sec=600, mm_step=5.0, deg_step=5.0)
                kinds = [sub.Instruction(i)[1] for i in range(count)]
                report["pen_program_checks"][sub.Name()] = {
                    "instructions": count, "moves": kinds.count(INS_TYPE_MOVE), "code": kinds.count(INS_TYPE_CODE),
                    "valid_ratio": ratio,
                    "time_s": seconds if math.isfinite(seconds) else None, "message": message}
                if not math.isfinite(ratio) or ratio != 1.0 or valid < count:
                    raise RuntimeError(f"Pen change {sub.Name()} failed the path check: {message}")
            report["pen_change_time_s"] = sum(c["time_s"] or 0 for c in report["pen_program_checks"].values())
        report.update(status="success", stage="complete")
        save_station(root / "stations" / f"{program_name}_validated.rdk", "station_file")
        return report
    except Exception as exc:
        report.update(status="failed", error_type=type(exc).__name__, error=str(exc),
                      traceback=traceback.format_exc())
        raise
    finally:
        if rdk is not None:
            try:
                rdk.Render(True)
                rdk.Disconnect()
            except Exception:
                pass


def main(project_root):
    root = Path(project_root).resolve()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/drawing.json")
    parser.add_argument("--data", help="JSON path relative to DrawBot (or an absolute path)")
    parser.add_argument("--validate-only", action="store_true", help="Validate JSON without RoboDK")
    parser.add_argument("--export", action="store_true", help="After validation, export with official UR post (no upload)")
    parser.add_argument("--pen-mode", choices=["stub", "external", "manual_preloaded"])
    parser.add_argument("--preloaded-marker", choices=["black", "green", "purple", "orange"])
    parser.add_argument("--allow-stub-export-for-testing", action="store_true",
                        help="Keep immediate-success ChangePen interfaces in test-only export")
    args = parser.parse_args()
    if args.validate_only and args.export:
        parser.error("--validate-only cannot be combined with --export")
    report = {"status": "failed", "stage": "input_validation",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "config_path": str((root / args.config).resolve())}
    try:
        settings = json.loads((root / args.config).read_text(encoding="utf-8"))
        settings["_project_root"] = str(root)
        if args.pen_mode:
            settings["pen_change_mode"] = args.pen_mode
        if args.preloaded_marker:
            settings["preloaded_marker"] = args.preloaded_marker
        settings["allow_stub_export_for_testing"] = args.allow_stub_export_for_testing
        palette = validate_palette(settings)
        motion_parameters(settings)
        data_path = (root / (args.data or settings["data_path"])).resolve()
        report["data_path"] = str(data_path)
        raw = data_path.read_bytes()
        report["input_sha256"] = hashlib.sha256(raw).hexdigest()
        strokes = validate_drawing(json.loads(raw), palette)
        stats = drawing_statistics(strokes, palette, settings)
        report.update(stats)
        print(f"Loaded {len(strokes)} strokes, {stats['path_point_count']} points from {data_path}")
        print(f"Expected {stats['motion_count']} motions ({stats['joint_motion_count']} MoveJ, "
              f"{stats['linear_motion_count']} MoveL), {stats['pen_change_count']} pen request(s)")
        if args.validate_only:
            print("JSON and four-color palette validation passed (no RoboDK path check)")
            return
        generate(settings, strokes, palette, report)
        if args.export:
            report["stage"] = "export"
            from .export import export_program
            export_program(settings, report, root)
            report["stage"] = "complete"
        print(f"Program generated and path checked: {settings['program_name']}. No program started.")
    except Exception as exc:
        report.update(status="failed", error_type=type(exc).__name__, error=str(exc),
                      traceback=traceback.format_exc())
        print(f"FAILED at {report['stage']}: {exc}")
        print(report.get("context") or report.get("diagnostic_candidate") or "See RoboDK message and report")
        raise
    finally:
        if not args.validate_only:
            report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            report_path = save_report(root, report)
            print(f"Validation report: {report_path}")
