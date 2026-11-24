import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import traceback

from .api import draw_in_robodk, status
from .circle_check import check_circle
from .circle_fixture import prepare_circle


def run_acceptance(root, tuning=True):
    root = Path(root).resolve()
    summary = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "stages": {},
               "physical_robot_used": False, "controller_execution_verified": False}
    stages = summary["stages"]
    config = json.loads((root / "config/drawing.json").read_text(encoding="utf-8"))
    stages["status"] = status(config)
    circle = None
    try:
        path, provenance = prepare_circle(root)
        circle = json.loads(path.read_text(encoding="utf-8"))
        geometry_config = config.get("circle_acceptance", {})
        geometry = check_circle(circle, **geometry_config)
        stages["circle_geometry"] = {"success": geometry["passed"], **geometry, "provenance": provenance}
        (root / "logs/circle_geometry.json").write_text(json.dumps(geometry, indent=2), encoding="utf-8")
    except Exception as exc:
        stages["circle_geometry"] = {"success": False, "error": str(exc), "traceback": traceback.format_exc()}

    sample_path = root / "data/sample_strokes.json"
    try:
        sample = json.loads(sample_path.read_text(encoding="utf-8"))
    except Exception as exc:
        sample = None
        stages["baseline"] = {"success": False, "error": str(exc)}

    def run(name, data, **kwargs):
        result = draw_in_robodk(data, settings=config, project_root=root, **kwargs)
        stages[name] = result
        (root / "logs" / f"{name}.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"{name}: success={result['success']}, ratio={result.get('valid_ratio')}, "
              f"time={result.get('simulation_time_s')}, errors={result.get('errors')}", flush=True)
        return result

    if sample is not None:
        baseline = run("sample_baseline_full", sample, program_name="SVG_Draw_JSON", source_file=sample_path)
    else:
        baseline = None
    if circle is not None and stages["circle_geometry"]["success"]:
        circle_path = root / "data/circle_100mm_strokes.json"
        run("circle_export_full", circle, mode="export", program_name="Circle_100mm",
            pen_change_mode="manual_preloaded", preloaded_marker="black", source_file=circle_path)
    if tuning and sample is not None:
        if baseline and baseline["success"]:
            from .connection import connect_existing
            from robodk.robolink import ITEM_TYPE_ROBOT, RUNMODE_SIMULATE
            rdk = connect_existing()
            try:
                rdk.setRunMode(RUNMODE_SIMULATE)
                rdk.Item(config["robot_name"], ITEM_TYPE_ROBOT).setJoints(baseline["report"]["initial_robot_joints_deg"])
            finally:
                rdk.Disconnect()
        candidate_motion = {"draw_speed_mm_s": 25, "pen_down_speed_mm_s": 15,
                            "pen_down_accel_mm_s2": 75, "retract_speed_mm_s": 35,
                            "travel_speed_mm_s": 40, "blend_radius_mm": 1.0}
        candidate = run("sample_motion_candidate", sample, program_name="SVG_Draw_JSON_Tuned",
                        motion=candidate_motion, source_file=sample_path)
        improved = (baseline and baseline["success"] and candidate["success"]
                    and candidate["simulation_time_s"] < baseline["simulation_time_s"] - .001)
        stages["motion_comparison"] = {
            "success": bool(baseline and baseline["success"]),
            "baseline_time_s": baseline.get("simulation_time_s") if baseline else None,
            "candidate_time_s": candidate.get("simulation_time_s"),
            "baseline_valid_ratio": baseline.get("valid_ratio") if baseline else None,
            "candidate_valid_ratio": candidate.get("valid_ratio"),
            "candidate_parameters": candidate_motion,
            "selected": "candidate" if improved else "baseline",
            "default_config_changed": False,
            "blend_requested_mm": 1.0,
            "blend_effective_mm": candidate.get("report", {}).get("motion_parameters", {}).get("effective_blend_radius_mm"),
            "same_initial_robot_joints": (bool(baseline and baseline["success"])
                and all(abs(a-b) < 1e-8 for a, b in zip(baseline["report"]["initial_robot_joints_deg"],
                                                     candidate["report"].get("initial_robot_joints_deg", [])))),
        }
    if sample is not None:
        run("sample_test_export_full", sample, mode="export", program_name="SVG_Draw_JSON_TestExport",
            pen_change_mode="stub", allow_stub_export_for_testing=True, source_file=sample_path)
    required = ["status", "sample_baseline_full", "circle_geometry", "circle_export_full", "sample_test_export_full"]
    from .playback import play_in_simulator
    for key in ("sample_baseline_full", "circle_export_full"):
        result = stages.get(key, {})
        if result.get("success"):
            try:
                stages[key + "_playback"] = play_in_simulator(config, result["report"])
            except Exception as exc:
                stages[key + "_playback"] = {"success": False, "error": str(exc), "traceback": traceback.format_exc()}
            required.append(key + "_playback")
    summary["success"] = all(stages.get(key, {}).get("success", False) for key in required)
    if summary["success"]:
        from .connection import connect_existing
        rdk = connect_existing()
        try:
            final = root / "stations/Acceptance_validated.rdk"
            rdk.Save(str(final), rdk.ActiveStation())
            summary["final_station"] = str(final)
        finally:
            rdk.Disconnect()
    summary["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    destination = root / "logs/acceptance.json"
    text = json.dumps(summary, indent=2, ensure_ascii=False)
    if destination.exists():
        previous = json.loads(destination.read_text(encoding="utf-8"))
        previous_stamp = previous.get("started_at_utc", "previous").replace(':', '').replace('.', '').replace('+', '')
        (root / "logs" / f"acceptance_{previous_stamp}.json").write_text(destination.read_text(encoding="utf-8"), encoding="utf-8")
    destination.write_text(text, encoding="utf-8")
    if summary["success"]:
        (root / "logs/last_successful_acceptance.json").write_text(text, encoding="utf-8")
    print(f"Acceptance success={summary['success']}; {destination}", flush=True)
    return summary


def main(root):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-tuning", action="store_true")
    args = parser.parse_args()
    return run_acceptance(root, not args.skip_tuning)
