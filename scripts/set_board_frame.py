#!/usr/bin/env python3

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from drawbot.calibration import frame_from_3_points  # noqa: E402  (shared with the calibration receiver)


def check_faces_away(pose):
    """A warning when board z points toward the robot base, or None.

    True for most wall and easel boards. A board lying flat near the base's height can trigger it
    although it is correct, so it only warns. Check the pen direction in RoboDK.
    """
    z = [pose[i][2] for i in range(3)]
    origin = [pose[i][3] for i in range(3)]
    if sum(z[i] * origin[i] for i in range(3)) <= 0:
        return ("Board z points toward the robot base. Fine for a flat board near the base's height; otherwise "
                "P2 may be left of P1 or P3 above the top edge. Check that the pen points into the board in RoboDK.")
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for name in ("p1", "p2", "p3"):
        parser.add_argument(f"--{name}", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--tcp", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"), help="pen tip in the flange frame, mm")
    parser.add_argument("--press-mm", type=float, default=0.0,
                        help="move the drawing plane along board z. Positive presses in. Negative draws in the air.")
    args = parser.parse_args(argv)

    pose = frame_from_3_points(args.p1, args.p2, args.p3, args.press_mm)
    warning = check_faces_away(pose)
    if warning:
        print("Warning:", warning)

    from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL
    from drawbot.robot.connection import connect_existing
    from robodk.robomath import Mat, invH, transl

    config = json.loads((ROOT / "config/drawing.json").read_text(encoding="utf-8"))
    rdk = connect_existing()
    robot = rdk.Item(config["robot_name"], ITEM_TYPE_ROBOT)
    board = rdk.Item(config["board_frame"], ITEM_TYPE_FRAME)
    tool = rdk.Item(config["tool_name"], ITEM_TYPE_TOOL)
    if not (robot.Valid() and board.Valid() and tool.Valid()):
        raise SystemExit("Open the station in RoboDK first. It needs the robot, board frame and pen tool named in config/drawing.json.")
    board.setPoseAbs(robot.PoseAbs() * Mat(pose))
    tcp = tool.PoseTool()
    tcp.setPos(args.tcp)
    tool.setPoseTool(tcp)

    for name, p in (("P1", args.p1), ("P2", args.p2), ("P3", args.p3)):
        local = (invH(Mat(pose)) * transl(*p)).Pos()
        print(f"{name} on the board: x={local[0]:.1f} y={local[1]:.1f} z={local[2]:.1f} mm")
    print(f"Board size from the points: top edge {math.dist(args.p1, args.p2):.1f} mm")
    print("BoardFrame and PenTCP are set. Save the station, then generate the program again.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
