import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drawbot import pen_stand  # noqa: E402
from drawbot.robot.export import check_export_size  # noqa: E402
from drawbot.robot.json_program import drawing_statistics  # noqa: E402
from drawbot.robot.dataset import validate_palette  # noqa: E402
from drawbot.strokes import Stroke  # noqa: E402
from drawbot.svg_pipeline import group_by_marker  # noqa: E402

FIXTURE = ROOT / "tests/fixtures/pen_stand_ur5e_data.json"


def write(records):
    folder = Path(tempfile.mkdtemp())
    path = folder / "ur5e_data.json"
    path.write_text(json.dumps(records))
    return path


class PickupPoints(unittest.TestCase):
    def test_taught_stand_passes(self):
        gaps = pen_stand.check_pickup_points(FIXTURE)
        self.assertEqual(len(gaps), 3)
        self.assertTrue(all(40 < g < 60 for g in gaps))

    def test_points_clicked_through_are_refused(self):
        records = json.loads(FIXTURE.read_text())
        for r in records:
            if r["label"] in pen_stand.PICKUP_LABELS:
                r["pose"] = [0.1, -0.4, 0.5, 0, 3.14, 0]
        with self.assertRaisesRegex(ValueError, "not about 50 mm apart"):
            pen_stand.check_pickup_points(write(records))

    def test_missing_points_and_missing_file(self):
        records = [r for r in json.loads(FIXTURE.read_text()) if r["label"] != "3"]
        with self.assertRaisesRegex(ValueError, "no pen stand point 3"):
            pen_stand.check_pickup_points(write(records))
        with self.assertRaisesRegex(ValueError, "Measure the board first"):
            pen_stand.check_pickup_points(None)

    def test_stand_from_newest_measurement_that_taught_it(self):
        root = Path(tempfile.mkdtemp())
        records = json.loads(FIXTURE.read_text())
        clicked = [dict(r, pose=[0.1, -0.4, 0.5, 0, 3.14, 0]) if r["label"] in pen_stand.PICKUP_LABELS else r
                   for r in records]

        def save(name, data):
            path = root / name / "ur5e_data.json"
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(data))
            return path

        self.assertIsNone(pen_stand.stand_measurement(root))
        fixed = save(pen_stand.STAND_FILE.parent, records)
        self.assertEqual(pen_stand.stand_measurement(root), fixed)
        taught = save("logs/calibration/2026-11-12-a", records)
        save("logs/calibration/2026-11-12-b", clicked)
        self.assertEqual(pen_stand.stand_measurement(root), taught)

    def test_collision_check_skips_only_the_pairs_turned_off_on_purpose(self):
        pc = pen_stand.module()
        self.assertTrue(pen_stand.off_by_design("UR5e", "Pen red"))
        self.assertTrue(pen_stand.off_by_design(pc.GRIPPER_NAME, pc.GRIPPER_ATTACHMENT_NAME))
        self.assertFalse(pen_stand.off_by_design("UR5e", pc.GRIPPER_ATTACHMENT_NAME))
        self.assertFalse(pen_stand.off_by_design(pc.GRIPPER_NAME, "Pen Stand Model"))
        self.assertFalse(pen_stand.off_by_design("UR5e", "Board & image"))

    def test_configure_uses_our_names_and_never_the_sample_data(self):
        pc = pen_stand.configure(FIXTURE, grip_force=300)
        self.assertEqual(pc.PEN_TIP_TOOL_NAME, "PenTCP")
        self.assertEqual(pc.PICKUP_DATA_FILE, str(FIXTURE))
        self.assertFalse(pc.PEN_CHECK_SHOW_POSITION)
        self.assertEqual(pc.GRIPPER_FORCE, 255)
        self.assertEqual(pen_stand.pens(), {"green": 0, "black": 1, "purple": 2, "orange": 3})


class StationSettings(unittest.TestCase):
    def test_slots_and_speeds_reach_the_pen_changer(self):
        slots = {"green": 0, "purple": 1, "orange": 2, "black": 3}
        pc = pen_stand.configure(FIXTURE, slots=slots, speed_mm_s=40, joint_speed_deg_s=20)
        self.assertEqual({c: p["slot"] for c, p in pc.PENS.items()}, slots)
        self.assertEqual((pc.SPEED_LINEAR, pc.SPEED_JOINTS), (40.0, 20.0))
        pc = pen_stand.configure(FIXTURE)
        self.assertEqual(pc.PENS["green"]["slot"], 0)
        self.assertEqual(pc.SPEED_LINEAR, pen_stand.PICK_SPEED_MM_S)

    def test_each_colour_once(self):
        from drawbot import robot_link
        pen_stand.check_slots({"black": 0, "orange": 1, "green": 2, "purple": 3})
        same = robot_link.slot_colors({"slot_1": "orange", "slot_2": "orange", "slot_3": "purple", "slot_4": "green"})
        with self.assertRaisesRegex(ValueError, "different colour"):
            pen_stand.check_slots(same)

    def test_stand_is_taught_with_the_gripper_open_around_seated_pens(self):
        from drawbot import robot_link
        text = robot_link.teach_script(280, True, "10.0.0.1")
        part1 = text[text.index("  if DO_STAND:"):text.index("  if DO_BOARD:")]
        self.assertIn("set_tcp(TCP_NOPEN)", part1)
        self.assertLess(part1.index("grip_open()"), part1.index('record("1", "tcp-nopen"'))
        self.assertNotIn("grip_close()", part1)
        self.assertIn("All 4 pens in their slots", part1)


class CollisionCheck(unittest.TestCase):
    def test_only_the_pairs_switched_off_on_purpose_are_skipped(self):
        pc = pen_stand.module()
        self.assertTrue(pen_stand.off_by_design("UR5e", "Pen purple"))
        self.assertTrue(pen_stand.off_by_design("UR5e", "Pen red"))
        self.assertTrue(pen_stand.off_by_design(pc.GRIPPER_NAME, pc.GRIPPER_ATTACHMENT_NAME))
        self.assertFalse(pen_stand.off_by_design("UR5e", pc.GRIPPER_ATTACHMENT_NAME))
        self.assertFalse(pen_stand.off_by_design(pc.GRIPPER_NAME, "Pen Stand Model"))
        self.assertFalse(pen_stand.off_by_design("UR5e", "Board & image"))


class GripperCode(unittest.TestCase):
    def test_a_pen_change_activates_only_on_an_explicit_act_0(self):
        pc = pen_stand.configure(FIXTURE, grip_force=150)
        lines = pc.gripper_start_code()
        code = "\n".join(lines)
        guard = next(i for i, l in enumerate(lines) if 'str_find(pen_act_reply, "ACT 0") >= 0' in l)
        activate = next(i for i, l in enumerate(lines) if "SET ACT 1" in l)
        self.assertLess(guard, activate)
        self.assertTrue(lines[activate].startswith("  "))
        self.assertIn('while str_find(pen_act_reply, "ACT") < 0', code)
        self.assertNotIn('str_find(pen_gripper_reply, "ACT 1") < 0', code)
        self.assertIn('socket_open("127.0.0.1", 63352, "pen_gripper")', code)
        self.assertIn("SET FOR 150", code)

    def test_pen_check_stops_only_without_a_pen(self):
        pc = pen_stand.configure(FIXTURE)
        held, no_pen, open_ = 199, 207, 0
        self.assertTrue(pc.PEN_CHECK_POS_MIN <= held <= pc.PEN_CHECK_POS_MAX)
        self.assertTrue(no_pen > pc.PEN_CHECK_POS_MAX and open_ < pc.PEN_CHECK_POS_MIN)

    def test_pen_check_only_warns(self):
        pc = pen_stand.configure(FIXTURE)
        lines = []
        program = object()
        add_code, problem = pc.add_code, pc.pen_in_fingers_problem
        pc.add_code = lambda prog, code: lines.extend(code) if prog is program else None
        pc.pen_in_fingers_problem = lambda RDK, color: None
        try:
            pc.check_pen("orange", RDK=object(), program=program)
        finally:
            pc.add_code, pc.pen_in_fingers_problem = add_code, problem
        popup = next(i for i, l in enumerate(lines) if "not between the fingers" in l)
        self.assertIn("warning=True", lines[popup])
        self.assertNotEqual(lines[popup + 1].strip(), "halt")
        self.assertIn("pen_check_pos > 204", "\n".join(lines))


class Grouping(unittest.TestCase):
    def test_each_marker_once_in_first_seen_order(self):
        strokes = [Stroke([(0, 0), (1, 0)], marker="orange"), Stroke([(50, 0), (51, 0)], marker="black"),
                   Stroke([(2, 0), (3, 0)], marker="orange"), Stroke([(52, 0), (53, 0)], marker="black")]
        grouped = group_by_marker(strokes, optimize=False)
        self.assertEqual([s.marker for s in grouped], ["orange", "orange", "black", "black"])
        self.assertEqual(len(group_by_marker(strokes)), 4)


class Counts(unittest.TestCase):
    def setUp(self):
        self.settings = json.loads((ROOT / "config/drawing.json").read_text())
        self.palette = validate_palette(self.settings)
        self.strokes = [{"points": [[0, 0], [1, 1]], "marker": m} for m in ("black", "black", "orange", "purple")]

    def test_stand_adds_a_home_move_per_later_color(self):
        base = drawing_statistics(self.strokes, self.palette, self.settings)
        stand = drawing_statistics(self.strokes, self.palette, {**self.settings, "pen_change_mode": "stand"})
        self.assertEqual(stand["joint_motion_count"] - base["joint_motion_count"], 2)
        self.assertEqual(stand["pen_change_count"], 3)

    def test_code_lines_count_once(self):
        check_export_size("Educational", moves=200, instructions=200 + 320 + 40, code=320)
        with self.assertRaisesRegex(RuntimeError, "too big"):
            check_export_size("Educational", moves=200, instructions=200 + 320 + 40)


if __name__ == "__main__":
    unittest.main()
