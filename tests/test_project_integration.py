import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import venv

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from drawbot.backend import load_svg, svg_to_strokes
from drawbot.robot.circle_check import check_circle
from drawbot.robot.circle_fixture import prepare_circle
from drawbot.robot.dataset import validate_drawing, validate_palette
from drawbot.robot.export import check_export_size
from drawbot.robot.json_program import drawing_statistics
from drawbot.strokes import Placement
from set_board_frame import check_faces_away, frame_from_3_points


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name).resolve()
        self.settings = json.loads((ROOT / "config/drawing.json").read_text())
        self.palette = validate_palette(self.settings)

    def relocated_project(self):
        root = self.base / "relocated project"
        root.mkdir()
        for folder in ("drawbot", "config", "data", "svg"):
            shutil.copytree(ROOT / folder, root / folder, ignore=shutil.ignore_patterns("__pycache__"))
        (root / "scripts").mkdir()
        for name in ("robot_program_json.py", "run_acceptance.py"):
            shutil.copy2(ROOT / "scripts" / name, root / "scripts" / name)
        return root

    def clean_env(self):
        env = os.environ.copy()
        env.pop("DRAWBOT_HOME", None)
        env.pop("PYTHONPATH", None)
        return env

    def test_circle_fixture_matches_the_pipeline_and_is_a_real_circle(self):
        path, provenance = prepare_circle(self.base)
        self.assertEqual(path.read_bytes(), (ROOT / "data/circle_100mm_strokes.json").read_bytes())
        self.assertEqual(provenance["output_json"], str(path))
        data = json.loads(path.read_text())
        strokes = validate_drawing(data, self.palette)
        stats = drawing_statistics(strokes, self.palette, self.settings)
        self.assertEqual((stats["stroke_count"], stats["path_point_count"], stats["motion_count"]), (1, 65, 70))
        geometry = check_circle(data)
        self.assertTrue(geometry["passed"])
        self.assertEqual(geometry["closure_error_mm"], 0)
        with self.assertRaises(FileNotFoundError):
            prepare_circle(self.base, self.base / "missing.svg")

    def test_straight_shapes_get_only_their_corners(self):
        rect = b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100"><rect width="100" height="100" fill="none" stroke="#000"/></svg>'
        strokes = svg_to_strokes(load_svg(rect), Placement(10, 10, 280, 280), 2.0)
        self.assertEqual(len(strokes[0].points), 5)

    def test_root_entry_runs_from_unrelated_cwd(self):
        result = subprocess.run([sys.executable, "-I", str(ROOT / "scripts/robot_program_json.py"),
                                 "--validate-only", "--data", "data/sample_strokes.json"],
                                cwd=self.base, env=self.clean_env(), capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Expected 352 motions", result.stdout)

    def test_copied_entries_find_a_relocated_project(self):
        root = self.relocated_project()
        venv.EnvBuilder(with_pip=False).create(root / ".venv")
        python = root / ".venv/bin/python"
        code = "import runpy,sys; print(runpy.run_path(sys.argv[1],run_name='bootstrap_test')['PROJECT_ROOT'])"
        for name in ("robot_program_json.py", "run_acceptance.py"):
            script = self.base / name
            shutil.copy2(root / "scripts" / name, script)
            result = subprocess.run([str(python), "-I", "-c", code, str(script)], cwd="/tmp",
                                    env=self.clean_env(), capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), str(root))

    def test_home_override_must_be_an_absolute_existing_project(self):
        root = self.relocated_project()
        script = self.base / "robot_program_json.py"
        shutil.copy2(root / "scripts/robot_program_json.py", script)
        code = "import runpy,sys; print(runpy.run_path(sys.argv[1],run_name='bootstrap_test')['PROJECT_ROOT'])"
        for value, good in ((str(root), True), ("relative/path", False), (str(self.base / "missing"), False)):
            env = self.clean_env()
            env["DRAWBOT_HOME"] = value
            result = subprocess.run([sys.executable, "-I", "-c", code, str(script)], cwd="/tmp",
                                    env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode == 0, good, result.stderr)


class BlendPlanTests(unittest.TestCase):
    def test_gentle_bends_blend_and_corners_stop(self):
        from drawbot.robot.dataset import blend_radii
        square = [[0, 0], [100, 0], [100, 100], [0, 100], [0, 0]]
        self.assertEqual(blend_radii(square, 2), [0, 0, 0, 0, 0])
        arc = [[100 * math.cos(math.radians(a)), 100 * math.sin(math.radians(a))] for a in range(0, 91, 10)]
        radii = blend_radii(arc, 2)
        self.assertEqual((radii[0], radii[-1]), (0, 0))
        self.assertTrue(all(r == 2 for r in radii[1:-1]))

    def test_radius_stays_under_the_neighbouring_segments(self):
        from drawbot.robot.dataset import blend_radii
        short = [[0, 0], [1, 0], [2, 0.05], [3, 0.1]]
        self.assertTrue(all(r <= 0.4 for r in blend_radii(short, 5)))
        self.assertEqual(blend_radii(short, 5)[1], 0.0)
        three = [[0, 0], [3, 0], [6, 0.2], [9, 0.4]]
        self.assertEqual(blend_radii(three, 5)[1], 1.0)
        self.assertEqual(blend_radii(short, 0), [0, 0, 0, 0])
        self.assertEqual(blend_radii([[0, 0], [1, 1]], 2), [0, 0])

    def test_blends_are_whole_millimetres_the_ur_script_can_hold(self):
        from drawbot.robot.dataset import blend_radii
        import math
        curve = [[50 * math.cos(a / 40), 50 * math.sin(a / 40)] for a in range(0, 200, 3)]
        radii = blend_radii(curve, 5)
        self.assertTrue(any(radii))
        self.assertTrue(all(r == int(r) for r in radii), radii)


class ExportSizeTests(unittest.TestCase):
    def test_educational_license_stops_big_programs_with_the_numbers(self):
        check_export_size("Educational", 914, 918)
        with self.assertRaisesRegex(RuntimeError, "about 1015 lines"):
            check_export_size("Educational", 929, 933)

    def test_speed_changes_count_as_several_lines(self):
        check_export_size("Educational", 422, 546)
        with self.assertRaises(RuntimeError):
            check_export_size("Educational", 562, 726)

    def test_free_license_keeps_its_own_limit(self):
        check_export_size("Free (Limited)", 40, 44)
        with self.assertRaisesRegex(RuntimeError, "over 50 lines"):
            check_export_size("Free (Limited)", 41, 45)

    def test_other_licenses_and_unknown_are_not_blocked(self):
        check_export_size("Professional", 5000, 5000)
        check_export_size("", 5000, 5000)


class BoardFrameTests(unittest.TestCase):
    P1, P2, P3 = [-150, 400, 300], [150, 400, 300], [-150, 400, 0]

    def test_three_points_give_the_board_contract(self):
        pose = frame_from_3_points(self.P1, self.P2, self.P3)
        self.assertEqual([pose[i][0] for i in range(3)], [1, 0, 0])
        self.assertEqual([pose[i][1] for i in range(3)], [0, 0, -1])
        self.assertEqual([pose[i][2] for i in range(3)], [0, 1, 0])
        self.assertEqual([pose[i][3] for i in range(3)], self.P1)

    def test_negative_press_moves_the_plane_toward_the_robot(self):
        pose = frame_from_3_points(self.P1, self.P2, self.P3, -20)
        self.assertEqual(pose[1][3], 380)

    def test_swapped_points_are_caught(self):
        self.assertIsNone(check_faces_away(frame_from_3_points(self.P1, self.P2, self.P3)))
        self.assertIn("toward the robot", check_faces_away(frame_from_3_points(self.P2, self.P1, self.P3)))
        with self.assertRaises(ValueError):
            frame_from_3_points(self.P1, self.P1, self.P3)


if __name__ == "__main__":
    unittest.main()
