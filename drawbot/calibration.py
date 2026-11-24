import json
import math
import re
import socket
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

PICKUP_POINTS = ["1", "2", "3", "4"]
BOARD_POINTS = ["origin", "xaxis", "yaxis"]
EXPECTED = len(PICKUP_POINTS) + len(BOARD_POINTS) + 1
TARGET_NAMES = {"1": "PenPickup_1", "2": "PenPickup_2", "3": "PenPickup_3", "4": "PenPickup_4",
                "origin": "Calib_TopLeft", "xaxis": "Calib_TopRight", "yaxis": "Calib_BottomRight"}
NOPEN_TOOL = "PenTCP_NoPen"
BOARD_THICKNESS_MM = 10.0
SQUARE_WARN_MM = 10.0
HOME_SEED_PARAM = "DRAWBOT_HOME_SEED"


def _unit(v, what):
    length = math.sqrt(sum(c * c for c in v))
    if length < 1.0:
        raise ValueError(f"{what}: the points are too close together or in one line")
    return [c / length for c in v]


def frame_from_3_points(p1, p2, p3, press_mm=0.0):
    """Return a 4x4 pose as nested lists. x along the top edge, y down the board, z into the board."""
    x = _unit([p2[i] - p1[i] for i in range(3)], "P1 to P2")
    d = [p3[i] - p1[i] for i in range(3)]
    z = _unit([x[1] * d[2] - x[2] * d[1], x[2] * d[0] - x[0] * d[2], x[0] * d[1] - x[1] * d[0]], "P1, P2, P3")
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    origin = [p1[i] + press_mm * z[i] for i in range(3)]
    return [[x[i], y[i], z[i], origin[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def rotvec_to_matrix(p):
    """UR pose [x,y,z,rx,ry,rz] (m, axis-angle rad) -> 4x4 nested list in mm."""
    x, y, z, rx, ry, rz = p
    v = np.array([rx, ry, rz])
    th = np.linalg.norm(v)
    T = np.eye(4)
    if th > 1e-9:
        k = v / th
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        T[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
    T[:3, 3] = np.array([x, y, z]) * 1000.0
    return T.tolist()


def board_size(rec_o, rec_x, rec_y):
    """Width along the top edge and height down the board, in mm, and how far from square it is."""
    o = np.array(rec_o["pose"][:3]) * 1000
    tr = np.array(rec_x["pose"][:3]) * 1000
    br = np.array(rec_y["pose"][:3]) * 1000
    xv = tr - o
    width = np.linalg.norm(xv)
    xh = xv / width
    v = br - o
    along = v.dot(xh)
    height = np.linalg.norm(v - along * xh)
    return float(width), float(height), float(along - width)


def box_triangles(w, h, t):
    """Triangles of a w x h x t box: surface at z=0, back face at z=+t."""
    w, h, t = float(w), float(h), float(t)
    c = [[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0],
         [0, 0, t], [w, 0, t], [w, h, t], [0, h, t]]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (3, 7, 6), (3, 6, 2), (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5)]
    return [c[i] for f in faces for i in f]


def _nums(s):
    return [float(n) for n in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", s)]


def _field(s):
    return None if s.strip().lower() == "null" else _nums(s)


def parse_line(line):
    label, tcpname, joints, pose, tcp = line.split(";")
    return {"label": label.strip(), "tcp": tcpname.strip(),
            "joints": _field(joints), "pose": _field(pose), "tcp_offset": _field(tcp)}


def parse_text(text):
    """All usable records in a block of text. Accepts real and literal backslash-n line ends."""
    records = []
    for line in text.replace("\\n", "\n").split("\n"):
        if line.strip():
            try:
                records.append(parse_line(line.strip()))
            except ValueError:
                pass
    return records


def receive(port=30000, expected=EXPECTED, timeout_s=None, on_record=print, on_listen=None):
    """Wait for teach_whiteboard.script and return (raw text, records). on_listen runs once the port is open."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    if on_listen:
        try:
            on_listen()
        except Exception:
            srv.close()
            raise
    srv.settimeout(timeout_s)
    try:
        conn, _ = srv.accept()
    finally:
        srv.close()
    conn.settimeout(timeout_s)
    raw, buf, records = "", "", []
    with conn:
        while len(records) < expected:
            chunk = conn.recv(4096)
            if not chunk:
                break
            text = chunk.decode(errors="ignore")
            raw += text
            buf = (buf + text).replace("\\n", "\n")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                for rec in parse_text(line):
                    records.append(rec)
                    on_record(f"[{len(records)}/{expected}] {rec['label']} ({rec['tcp']})")
    records += parse_text(buf)
    return raw, records


def save_records(root, raw, records, tag=""):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    folder = Path(root) / "logs" / "calibration" / (stamp + (f"-{tag}" if tag else ""))
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "ur5e_raw.txt").write_text(raw, encoding="utf-8")
    (folder / "ur5e_data.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
    return folder


def board_frame_from_records(by_label, press_mm=0.0):
    """Board pose in the robot base frame, in mm. Corners preferred; the robot's own plane as fallback."""
    if all(by_label.get(k) and by_label[k]["pose"] for k in BOARD_POINTS):
        p1, p2, p3 = ([v * 1000 for v in by_label[k]["pose"][:3]] for k in BOARD_POINTS)
        return frame_from_3_points(p1, p2, p3, press_mm)
    if by_label.get("whiteboard") and by_label["whiteboard"]["pose"]:
        pose = rotvec_to_matrix(by_label["whiteboard"]["pose"])
        for i in range(3):
            pose[i][3] += press_mm * pose[i][2]
        return pose
    return None


def pen_direction_warning(pose, by_label):
    """The pen must point into the board while touching it. Otherwise the corners or the TCP are wrong."""
    z = np.array([pose[i][2] for i in range(3)])
    for k in BOARD_POINTS:
        rec = by_label.get(k)
        if rec and rec["pose"]:
            tool_z = np.array(rotvec_to_matrix(rec["pose"]))[:3, 2]
            if tool_z.dot(z) < 0.5:
                return (f"At '{k}' the pen points {math.degrees(math.acos(max(-1, min(1, tool_z.dot(z))))):.0f}° away "
                        "from the board's z axis. Check the corner order (top-left, top-right, bottom-right) and the TCP.")
    return None


def copy_collision_pairs(rdk, source, target):
    """Give a new object the same collision checks as the object it replaces.

    The path check refuses to run unless every object is paired with the robot and the gripper.
    """
    from robodk.robolink import COLLISION_ON
    copied = 0
    for a, b, i, j in rdk.CollisionActivePairList():
        if a == source:
            rdk.setCollisionActivePair(COLLISION_ON, target, b, 0, j)
            copied += 1
        elif b == source:
            rdk.setCollisionActivePair(COLLISION_ON, a, target, i, 0)
            copied += 1
    return copied


def _fit_is_zero(fit):
    from .board_fit import is_zero
    return is_zero(fit)


def apply_to_station(rdk, records, settings, press_mm=0.0, resize_board=True, log=print, fit=None):
    """Update the station from teach records. Returns a short report.

    fit: board corrections from a touch test, {point: mm deeper}, see board_fit. They shift and tilt
    BoardFrame a little; the board model moves with it, so the pen still meets its surface.
    """
    from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET, ITEM_TYPE_TOOL
    from robodk.robomath import Mat, eye

    by_label = {r["label"]: r for r in records}
    report = {"warnings": [], "updated": []}
    robot = rdk.Item(settings["robot_name"], ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{settings['robot_name']}' not found. Open the station in RoboDK first.")
    base = robot.Parent()

    pose = board_frame_from_records(by_label, press_mm)
    frame = rdk.Item(settings["board_frame"], ITEM_TYPE_FRAME)
    if pose is None:
        report["warnings"].append("No board corners or plane received: board frame not changed.")
    elif not frame.Valid():
        raise RuntimeError(f"Frame '{settings['board_frame']}' not found in the station.")
    else:
        warning = pen_direction_warning(pose, by_label)
        if warning:
            report["warnings"].append(warning)
        if fit and not _fit_is_zero(fit):
            if not all(k in by_label for k in BOARD_POINTS):
                raise ValueError("Board corrections need the 3 measured corners.")
            from .board_fit import corrected_pose
            w, h, _ = board_size(*(by_label[k] for k in BOARD_POINTS))
            pose, result = corrected_pose(pose, fit, w, h)
            report["board_fit"] = {"wanted_mm": {k: float(v or 0) for k, v in fit.items()},
                                   "applied_mm": result["applied"], "leftover_mm": result["leftover"]}
        frame.setPoseAbs(robot.PoseAbs() * Mat(pose))
        report["board_pose_base_mm"] = pose
        report["updated"].append(settings["board_frame"])

    if all(k in by_label for k in BOARD_POINTS):
        w, h, skew = board_size(*(by_label[k] for k in BOARD_POINTS))
        report["board_size_mm"] = [round(w, 2), round(h, 2)]
        rdk.setParam("WB_WIDTH_MM", f"{w:.2f}")
        rdk.setParam("WB_HEIGHT_MM", f"{h:.2f}")
        if abs(skew) > SQUARE_WARN_MM:
            report["warnings"].append(f"Bottom-right corner is {skew:+.1f} mm off from straight below top-right. Check the corner touches.")
        if resize_board and frame.Valid():
            name = settings.get("board_object", "Board & image")
            current = rdk.Item(name, ITEM_TYPE_OBJECT)
            board = rdk.AddShape(box_triangles(w, h, BOARD_THICKNESS_MM))
            board.setParent(frame)
            board.setPose(eye(4))
            board.setColor([0.95, 0.95, 0.95, 1.0])
            report["collision_pairs_copied"] = copy_collision_pairs(rdk, current, board) if current.Valid() else 0
            if current.Valid():
                current.Delete()
            board.setName(name)
            if not report["collision_pairs_copied"]:
                report["warnings"].append(f"The new '{name}' has no collision pairs. Enable them in RoboDK's collision map.")
            report["updated"].append(name)

    def tcp_of(name):
        for r in records:
            if r["tcp"] == name and r["tcp_offset"]:
                return Mat(rotvec_to_matrix(r["tcp_offset"]))
        return None

    for data_name, tool_name in (("tcp-pen", settings["tool_name"]), ("tcp-nopen", NOPEN_TOOL)):
        T = tcp_of(data_name)
        if T is None:
            continue
        tool = rdk.Item(tool_name, ITEM_TYPE_TOOL)
        if tool.Valid():
            tool.setPoseTool(T)
        else:
            robot.AddTool(T, tool_name)
        report["updated"].append(tool_name)
        report["pen_tcp_mm" if data_name == "tcp-pen" else "no_pen_tcp_mm"] = [round(v, 2) for v in Mat(T).Pos()]
    pen = rdk.Item(settings["tool_name"], ITEM_TYPE_TOOL)
    if pen.Valid():
        robot.setPoseTool(pen)

    for label, target_name in TARGET_NAMES.items():
        rec = by_label.get(label)
        if not rec or not rec["joints"]:
            continue
        target = rdk.Item(target_name, ITEM_TYPE_TARGET)
        if not target.Valid():
            target = rdk.AddTarget(target_name, base, robot)
        target.setAsJointTarget()
        target.setJoints([math.degrees(j) for j in rec["joints"]])
        target.setVisible(False, False)
        report["updated"].append(target_name)

    if by_label.get("origin") and by_label["origin"]["joints"]:
        touched = [round(math.degrees(j), 3) for j in by_label["origin"]["joints"]]
        seed, coverage = touched, None
        size = report.get("board_size_mm")
        if size and frame.Valid() and hasattr(robot, "SolveIK_All"):
            seed, coverage = best_arm_shape(rdk, robot, settings, size, touched)
            report["arm_shape_coverage"] = coverage
            if coverage["best"] < coverage["points"]:
                report["warnings"].append(f"Only {coverage['best']} of {coverage['points']} board test points are reachable "
                                          "without changing arm shape. Keep drawings away from the unreachable area, or move the board closer.")
            if max(abs(a - b) for a, b in zip(seed, touched)) > 90:
                report["warnings"].append("The best arm shape differs a lot from the touched one: the first MoveJ to home "
                                          "makes a big swing. Jog near home first, or watch it closely at low speed.")
        rdk.setParam(HOME_SEED_PARAM, json.dumps([round(v, 3) for v in seed]))
        report["home_seed_deg"] = [round(v, 3) for v in seed]
        report["touched_joints_deg"] = touched
        report["updated"].append(HOME_SEED_PARAM)

    for line in report["warnings"]:
        log("Warning: " + line)
    return report


def best_arm_shape(rdk, robot, settings, board_size_mm, touched_deg, cols=8, rows=6):
    """Pick the arm shape (IK branch) that reaches the most of the board without changing shape.

    Lines on the board must stay on one IK solution. On a far board one shape can run into the elbow
    singularity halfway across, while another covers everything. Each candidate shape walks a grid row by
    row, on the board and 80 mm above it; a jump of more than 20° counts as a change of shape. Ties go to
    the shape nearest the touched joints, so the first MoveJ stays short.
    """
    from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_TARGET, ITEM_TYPE_TOOL
    from robodk.robomath import Mat, invH
    board = rdk.Item(settings["board_frame"], ITEM_TYPE_FRAME)
    tool = rdk.Item(settings["tool_name"], ITEM_TYPE_TOOL)
    ref = rdk.Item(settings["orientation_target"], ITEM_TYPE_TARGET)
    B = invH(robot.PoseAbs()) * board.PoseAbs()
    O = invH(board.PoseAbs()) * ref.PoseAbs()
    flange = invH(tool.PoseTool())
    w, h = board_size_mm

    def solve(x, y, z, seed):
        pose = Mat(O)
        pose.setPos([x, y, z])
        q = robot.SolveIK(B * pose * flange, Mat(list(seed)))
        return q.list() if len(q.list()) >= 6 else None

    def score(seed):
        reached = 0
        for j in range(rows):
            y = h * (0.05 + 0.9 * j / (rows - 1))
            prev = solve(w * 0.05, y, 0, seed)
            for i in range(cols):
                x = w * (0.05 + 0.9 * i / (cols - 1))
                q = solve(x, y, 0, prev) if prev else None
                if not q or max(abs(a - b) for a, b in zip(q, prev)) > 20:
                    break
                up = solve(x, y, -80, q)
                if not up or max(abs(a - b) for a, b in zip(up, q)) > 40:
                    break
                reached += 1
                prev = q
        return reached

    centre = Mat(O)
    centre.setPos([w / 2, h / 2, -150])
    sols = robot.SolveIK_All(B * centre * flange)
    candidates = [touched_deg] + [[sols[i, k] for i in range(6)] for k in range(sols.size(1))] if sols.size(0) >= 6 else [touched_deg]
    scored = [(score(c), -max(abs(a - b) for a, b in zip(c, touched_deg)), c) for c in candidates]
    best = max(scored, key=lambda s: (s[0], s[1]))
    return best[2], {"points": rows * cols, "best": best[0], "touched_shape": scored[0][0], "candidates": len(candidates)}


def station_home_seed(rdk):
    """Seed for the home pose IK, chosen by the last calibration, or None."""
    try:
        value = rdk.getParam(HOME_SEED_PARAM)
        seed = json.loads(value) if value else None
    except Exception:
        return None
    return seed if isinstance(seed, list) and len(seed) == 6 else None


def station_board_size(rdk):
    try:
        w, h = float(rdk.getParam("WB_WIDTH_MM")), float(rdk.getParam("WB_HEIGHT_MM"))
    except Exception:
        return None
    return (w, h) if w > 0 and h > 0 else None
