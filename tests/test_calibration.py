import json
import math
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from drawbot import calibration as cal


def ur_pose(xyz_mm, R):
    """A UR pose line value (m, rotation vector) for a rotation matrix R."""
    R = np.array(R, float)
    th = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if th < 1e-9:
        v = [0.0, 0.0, 0.0]
    elif abs(th - math.pi) < 1e-6:
        k = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        k[1] = math.copysign(k[1], R[0, 1]); k[2] = math.copysign(k[2], R[0, 2])
        v = list(k * th)
    else:
        v = list(np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th)) * th)
    return [c / 1000 for c in xyz_mm] + v


def fmt(values):
    return "null" if values is None else "[" + ", ".join(f"{v:.6f}" for v in values) + "]"


def line(label, tcp, joints, pose, offset):
    return f"{label};{tcp};{fmt(joints)};{'null' if pose is None else 'p' + fmt(pose)};{'null' if offset is None else 'p' + fmt(offset)}"


CORNERS = {"origin": [-150, 650, 550], "xaxis": [150, 650, 550], "yaxis": [150, 650, 300]}
PEN_R = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
PEN_TCP = [0, 0, 0.275, 0, 0, 0]
JOINTS = [3.0, -1.6, 2.0, 3.5, 4.6, -0.8]


def sample_lines(pen_r=PEN_R, corners=CORNERS):
    out = [line(str(i), "tcp-nopen", JOINTS, ur_pose([300 + 20 * i, -200, 100], [[1, 0, 0], [0, -1, 0], [0, 0, -1]]), [0, 0, 0.155, 0, 0, 0]) for i in range(1, 5)]
    out += [line(k, "tcp-pen", JOINTS, ur_pose(corners[k], pen_r), PEN_TCP) for k in cal.BOARD_POINTS]
    out.append(line("whiteboard", "tcp-pen", None, ur_pose(corners["origin"], PEN_R), None))
    return out


class ParsingTests(unittest.TestCase):
    def test_robot_lines_parse_including_null_fields(self):
        records = cal.parse_text("\n".join(sample_lines()))
        self.assertEqual(len(records), cal.EXPECTED)
        plane = records[-1]
        self.assertEqual((plane["label"], plane["joints"], plane["tcp_offset"]), ("whiteboard", None, None))
        self.assertEqual(len(records[0]["joints"]), 6)
        self.assertAlmostEqual(records[4]["pose"][1], 0.65)

    def test_literal_backslash_n_and_bad_lines(self):
        text = "\\n".join(sample_lines()[:2]) + "\\ngarbage line\\n"
        self.assertEqual([r["label"] for r in cal.parse_text(text)], ["1", "2"])

    def test_numbers_with_exponents(self):
        self.assertEqual(cal.parse_line("a;t;[1e-05, -2.5E+01];null;null")["joints"], [1e-05, -25.0])


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.by = {r["label"]: r for r in cal.parse_text("\n".join(sample_lines()))}

    def test_board_size_and_square_check(self):
        w, h, skew = cal.board_size(*(self.by[k] for k in cal.BOARD_POINTS))
        self.assertAlmostEqual(w, 300, places=3)
        self.assertAlmostEqual(h, 250, places=3)
        self.assertAlmostEqual(skew, 0, places=3)

    def test_frame_from_corners_is_our_contract(self):
        pose = cal.board_frame_from_records(self.by)
        cols = np.array(pose)[:3, :3].T
        np.testing.assert_allclose(cols[0], [1, 0, 0], atol=1e-9)
        np.testing.assert_allclose(cols[1], [0, 0, -1], atol=1e-9)
        np.testing.assert_allclose(cols[2], [0, 1, 0], atol=1e-9)
        np.testing.assert_allclose(np.array(pose)[:3, 3], CORNERS["origin"], atol=1e-6)

    def test_robot_plane_is_the_fallback_and_press_moves_it(self):
        only_plane = {"whiteboard": self.by["whiteboard"]}
        np.testing.assert_allclose(np.array(cal.board_frame_from_records(only_plane)), np.array(cal.board_frame_from_records(self.by)), atol=1e-6)
        pressed = cal.board_frame_from_records(self.by, press_mm=-20)
        self.assertAlmostEqual(pressed[1][3], 630, places=6)

    def test_pen_direction_check(self):
        self.assertIsNone(cal.pen_direction_warning(cal.board_frame_from_records(self.by), self.by))
        flipped = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        by = {r["label"]: r for r in cal.parse_text("\n".join(sample_lines(flipped)))}
        self.assertIn("points", cal.pen_direction_warning(cal.board_frame_from_records(by), by))

    def test_rotvec_round_trip(self):
        for R in (PEN_R, [[0, 1, 0], [1, 0, 0], [0, 0, -1]], np.eye(3).tolist()):
            T = cal.rotvec_to_matrix(ur_pose([10, 20, 30], R))
            np.testing.assert_allclose(np.array(T)[:3, :3], R, atol=1e-6)
            np.testing.assert_allclose(np.array(T)[:3, 3], [10, 20, 30], atol=1e-9)

    def test_board_model_has_surface_at_zero_and_depth_behind(self):
        tris = cal.box_triangles(300, 250, 10)
        self.assertEqual(len(tris), 36)
        self.assertEqual({p[2] for p in tris}, {0.0, 10.0})


class ReceiveTests(unittest.TestCase):
    def test_receives_from_a_robot_over_tcp_and_saves(self):
        probe = socket.socket(); probe.bind(("127.0.0.1", 0)); port = probe.getsockname()[1]; probe.close()
        result = {}
        thread = threading.Thread(target=lambda: result.update(zip(("raw", "records"), cal.receive(port, timeout_s=10, on_record=lambda m: None))))
        thread.start()
        for _ in range(50):
            try:
                robot = socket.create_connection(("127.0.0.1", port), timeout=1); break
            except OSError:
                time.sleep(0.05)
        for text in sample_lines():
            robot.sendall((text + "\n").encode()); time.sleep(0.01)
        robot.close(); thread.join(10)
        self.assertEqual(len(result["records"]), cal.EXPECTED)
        with tempfile.TemporaryDirectory() as tmp:
            folder = cal.save_records(tmp, result["raw"], result["records"])
            self.assertEqual(len(json.loads((folder / "ur5e_data.json").read_text())), cal.EXPECTED)
            self.assertTrue((folder / "ur5e_raw.txt").read_text().startswith("1;tcp-nopen"))


class FakeItem:
    def __init__(self, rdk, name, kind, parent=None):
        self.rdk, self.name, self.kind, self.parent = rdk, name, kind, parent
        self.pose, self.tool, self.visible, self.joints, self.deleted = None, None, True, None, False
    def Valid(self): return not self.deleted
    def Name(self): return self.name
    def Parent(self): return self.parent
    def PoseAbs(self):
        from robodk.robomath import eye
        return self.pose if self.pose is not None else eye(4)
    def setPoseAbs(self, pose): self.pose = pose
    def setPose(self, pose): self.pose = pose
    def setName(self, name):
        self.rdk.items.pop((self.name, self.kind)); self.name = name; self.rdk.items[(name, self.kind)] = self
    def setVisible(self, visible, frame=None): self.visible = visible
    def setParent(self, parent): self.parent = parent
    def setColor(self, color): pass
    def setPoseTool(self, value): self.tool = value
    def PoseTool(self): return self.tool
    def AddTool(self, pose, name):
        from robodk.robolink import ITEM_TYPE_TOOL
        item = self.rdk.add(name, ITEM_TYPE_TOOL, self); item.tool = pose; return item
    def setAsJointTarget(self): pass
    def setJoints(self, joints): self.joints = list(joints)
    def Delete(self): self.deleted = True; self.rdk.items.pop((self.name, self.kind))


class FakeRdk:
    def __init__(self):
        from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL
        self.items, self.params, self.shapes = {}, {}, []
        base = self.add("UR5e Base", ITEM_TYPE_FRAME)
        self.robot = self.add("UR5e", ITEM_TYPE_ROBOT, base)
        self.board = self.add("BoardFrame", ITEM_TYPE_FRAME)
        self.add("Board & image", ITEM_TYPE_OBJECT, self.board)
        self.add("PenTCP", ITEM_TYPE_TOOL, self.robot)
        board = self.items[("Board & image", ITEM_TYPE_OBJECT)]
        self.pairs = [(self.robot, board, link, 0) for link in range(7)]
    def add(self, name, kind, parent=None):
        item = FakeItem(self, name, kind, parent); self.items[(name, kind)] = item; return item
    def Item(self, name, kind):
        return self.items.get((name, kind)) or FakeItem(self, name, kind)._invalid()
    def AddShape(self, triangles):
        from robodk.robolink import ITEM_TYPE_OBJECT
        self.shapes.append(triangles); return self.add("new shape", ITEM_TYPE_OBJECT)
    def AddTarget(self, name, parent, robot):
        from robodk.robolink import ITEM_TYPE_TARGET
        return self.add(name, ITEM_TYPE_TARGET, parent)
    def setParam(self, key, value): self.params[key] = value
    def getParam(self, key): return self.params.get(key)
    def CollisionActivePairList(self):
        return [p for p in self.pairs if p[0].Valid() and p[1].Valid()]
    def setCollisionActivePair(self, state, a, b, i=0, j=0): self.pairs.append((a, b, i, j))


def _invalid(self):
    self.deleted = True
    return self
FakeItem._invalid = _invalid
FakeItem.getLink = lambda self, kind: None


class ApplyTests(unittest.TestCase):
    def setUp(self):
        self.settings = json.loads((ROOT / "config/drawing.json").read_text())
        self.records = cal.parse_text("\n".join(sample_lines()))
        self.rdk = FakeRdk()

    def test_updates_our_station_names_only(self):
        from robodk.robolink import ITEM_TYPE_OBJECT, ITEM_TYPE_TARGET, ITEM_TYPE_TOOL
        report = cal.apply_to_station(self.rdk, self.records, self.settings, log=lambda m: None)
        np.testing.assert_allclose(np.array(self.rdk.board.pose.Rows()),
                                   np.array(cal.board_frame_from_records({r["label"]: r for r in self.records})), atol=1e-6)
        self.assertEqual(report["board_size_mm"], [300.0, 250.0])
        self.assertEqual(len(self.rdk.shapes), 1)
        new_board = self.rdk.Item("Board & image", ITEM_TYPE_OBJECT)
        self.assertIs(new_board.parent, self.rdk.board)
        self.assertFalse(self.rdk.Item("Board & image (original)", ITEM_TYPE_OBJECT).Valid())
        self.assertEqual(report["collision_pairs_copied"], 7)
        self.assertEqual(sorted(p[2] for p in self.rdk.pairs if p[1] is new_board), list(range(7)))
        self.assertEqual([round(v, 3) for v in self.rdk.Item("PenTCP", ITEM_TYPE_TOOL).tool.Pos()], [0, 0, 275])
        self.assertTrue(self.rdk.Item(cal.NOPEN_TOOL, ITEM_TYPE_TOOL).Valid())
        self.assertEqual(report["no_pen_tcp_mm"], [0, 0, 155])
        for name in cal.TARGET_NAMES.values():
            target = self.rdk.Item(name, ITEM_TYPE_TARGET)
            self.assertTrue(target.Valid(), name)
            self.assertFalse(target.visible)
        self.assertEqual(self.rdk.params["WB_WIDTH_MM"], "300.00")
        self.assertEqual(json.loads(self.rdk.params[cal.HOME_SEED_PARAM]), [round(math.degrees(j), 3) for j in JOINTS])
        for theirs in ("Drawing Frame", "Whiteboard 250mm", "Pen_TCP", "tcp-nopen", "1", "origin"):
            self.assertFalse(any(k[0] == theirs for k in self.rdk.items), theirs)

    def test_second_run_still_leaves_one_board_with_collision_pairs(self):
        from robodk.robolink import ITEM_TYPE_OBJECT
        for _ in range(2):
            cal.apply_to_station(self.rdk, self.records, self.settings, log=lambda m: None)
        names = [k[0] for k in self.rdk.items if k[1] == ITEM_TYPE_OBJECT]
        self.assertEqual(names, ["Board & image"])
        board = self.rdk.Item("Board & image", ITEM_TYPE_OBJECT)
        self.assertEqual(sum(1 for p in self.rdk.CollisionActivePairList() if p[1] is board), 7)

    def test_missing_board_frame_is_an_error_not_a_silent_skip(self):
        from robodk.robolink import ITEM_TYPE_FRAME
        self.rdk.items.pop(("BoardFrame", ITEM_TYPE_FRAME))
        with self.assertRaisesRegex(RuntimeError, "BoardFrame"):
            cal.apply_to_station(self.rdk, self.records, self.settings, log=lambda m: None)


class StationParamTests(unittest.TestCase):
    class Rdk:
        def __init__(self, params): self.params = params
        def getParam(self, key): return self.params.get(key)

    def test_home_seed_and_board_size_read_back(self):
        rdk = self.Rdk({cal.HOME_SEED_PARAM: json.dumps([1, 2, 3, 4, 5, 6]), "WB_WIDTH_MM": "300.00", "WB_HEIGHT_MM": "250.00"})
        self.assertEqual(cal.station_home_seed(rdk), [1, 2, 3, 4, 5, 6])
        self.assertEqual(cal.station_board_size(rdk), (300.0, 250.0))

    def test_missing_or_broken_params_mean_none(self):
        for params in ({}, {cal.HOME_SEED_PARAM: "not json", "WB_WIDTH_MM": "abc"}, {cal.HOME_SEED_PARAM: "[1, 2]"}):
            self.assertIsNone(cal.station_home_seed(self.Rdk(params)))
            self.assertIsNone(cal.station_board_size(self.Rdk(params)))


if __name__ == "__main__":
    unittest.main()
