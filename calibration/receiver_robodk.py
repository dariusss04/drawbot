import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from drawbot import calibration
from drawbot.robot.connection import connect_existing


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=30010, help="must match PORT in teach_whiteboard.script")
    parser.add_argument("--from-json", help="apply a saved ur5e_data.json instead of waiting for the robot")
    parser.add_argument("--press-mm", type=float, default=0.0,
                        help="move the drawing plane along board z. Negative draws in the air, for a dry run")
    parser.add_argument("--keep-board-model", action="store_true", help="do not rebuild the board model")
    args = parser.parse_args(argv)

    if args.from_json:
        records = json.loads(Path(args.from_json).read_text(encoding="utf-8"))
    else:
        print(f"Waiting for the robot on port {args.port} ...")
        raw, records = calibration.receive(args.port)
        folder = calibration.save_records(ROOT, raw, records)
        print(f"Received {len(raw)} characters, saved in {folder}")
    if not records:
        print("No usable lines received. RoboDK NOT changed.")
        return 1
    if len(records) < calibration.EXPECTED:
        print(f"Only {len(records)} of {calibration.EXPECTED} lines. Updating RoboDK with what arrived.")

    settings = json.loads((ROOT / "config/drawing.json").read_text(encoding="utf-8"))
    rdk = connect_existing()
    try:
        report = calibration.apply_to_station(rdk, records, settings, args.press_mm, not args.keep_board_model)
    finally:
        rdk.Disconnect()
    if "board_size_mm" in report:
        print("Board measured: {:.1f} mm wide x {:.1f} mm high".format(*report["board_size_mm"]))
    if "arm_shape_coverage" in report:
        c = report["arm_shape_coverage"]
        print(f"Arm shape for drawing: reaches {c['best']} of {c['points']} board test points "
              f"(the shape used while touching reaches {c['touched_shape']}). Home seed: {report['home_seed_deg']}")
    if "pen_tcp_mm" in report:
        print("Pen TCP:", report["pen_tcp_mm"], "mm")
    print("Updated:", ", ".join(report["updated"]))
    print("Save the station, then simulate again. The web app reads the new board size on its next refresh.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
