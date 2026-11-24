import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from drawbot.robot.dataset import color_changes, stroke_moves, validate_drawing, validate_palette


class DrawingContractTests(unittest.TestCase):
    def setUp(self):
        self.settings = json.loads((ROOT / "config/drawing.json").read_text())
        self.palette = validate_palette(self.settings)
        self.data = json.loads((ROOT / "data/four_color_strokes.json").read_text())

    def test_names_retain_original_point_indices(self):
        stroke = {"points": [[100, 500], [150, 510], [200, 500]]}
        moves = list(stroke_moves(1, stroke, -80, 0))
        self.assertEqual([m[0] for m in moves], [
            "S01_Approach", "S01_PenDown", "S01_Draw_P002",
            "S01_Draw_P003", "S01_Retract",
        ])
        self.assertEqual(moves[0][1:], ([100, 500, -80], "J"))
        self.assertEqual(moves[-1][1:], ([200, 500, -80], "L"))

    def test_initial_marker_and_transitions_without_reordering(self):
        strokes = [{"marker": m} for m in ["black", "black", "orange", "black"]]
        requests = [(i, p["program"]) for i, p in color_changes(strokes, self.palette)]
        self.assertEqual(requests, [(1, "ChangePen_Black"), (3, "ChangePen_Orange"),
                                    (4, "ChangePen_Black")])

    def test_four_color_sample_is_supported(self):
        strokes = validate_drawing(self.data, self.palette)
        self.assertEqual([(i, p["program"]) for i, p in color_changes(strokes, self.palette)],
                         [(1, "ChangePen_Black"), (2, "ChangePen_Orange"),
                          (3, "ChangePen_Purple"), (4, "ChangePen_Green")])

    def test_reject_invalid_coordinates(self):
        for bad in ([301, 500], [100, -1], [100, float("nan")],
                    [float("inf"), 500], [True, 500], [100, 500, 0],
                    [-1, 500], [100, 601], [100, False], [100, "500"],
                    [10 ** 1000, 500], None, "100,500", {"x": 100, "y": 500}):
            data = copy.deepcopy(self.data)
            data["strokes"][0]["points"][0] = bad
            with self.subTest(point=bad), self.assertRaises(ValueError):
                validate_drawing(data, self.palette)

    def test_reject_unknown_or_invalid_markers(self):
        for marker in ("yellow", "Black", "#000000", "", None, 1, True, [], {}):
            data = copy.deepcopy(self.data)
            data["strokes"][0]["marker"] = marker
            with self.subTest(marker=marker), self.assertRaisesRegex(ValueError, "marker"):
                validate_drawing(data, self.palette)

    def test_reject_empty_or_short_strokes_and_invalid_dimensions(self):
        for key, value in (("strokes", []), ("strokes", {}), ("strokes", [None]),
                           ("strokes", "strokes"), ("units", "pixels"),
                           ("origin", "bottom-left, y up"), ("origin", None)):
            data = copy.deepcopy(self.data)
            data[key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                validate_drawing(data, self.palette)
        for points in ([], [[100, 500]], None, "points", {}):
            data = copy.deepcopy(self.data)
            data["strokes"][0]["points"] = points
            with self.subTest(points=points), self.assertRaises(ValueError):
                validate_drawing(data, self.palette)

    def test_reject_invalid_board_dimensions(self):
        for field in ("board_width", "board_height"):
            for value in (0, -1, True, "300", None, float("nan"), float("inf"), []):
                data = copy.deepcopy(self.data)
                data[field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    validate_drawing(data, self.palette)

    def test_reject_invalid_placement(self):
        for field in ("x", "y", "width", "height"):
            for value in (None, True, "10", float("nan"), float("inf"), -1):
                data = copy.deepcopy(self.data)
                data["placement"][field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    validate_drawing(data, self.palette)
        for placement in (None, [], {}, {"x": 0, "y": 0, "width": 0, "height": 1},
                          {"x": 0, "y": 0, "width": 1, "height": 0},
                          {"x": 250, "y": 0, "width": 100, "height": 1},
                          {"x": 0, "y": 590, "width": 100, "height": 20}):
            data = copy.deepcopy(self.data)
            data["placement"] = placement
            with self.subTest(placement=placement), self.assertRaises(ValueError):
                validate_drawing(data, self.palette)

    def test_require_all_strokelist_fields(self):
        for field in ("board_width", "board_height", "placement", "units", "origin", "strokes"):
            data = copy.deepcopy(self.data)
            del data[field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_drawing(data, self.palette)
        for field in ("points", "marker", "svg_color", "source"):
            data = copy.deepcopy(self.data)
            del data["strokes"][0][field]
            with self.subTest(stroke_field=field), self.assertRaises(ValueError):
                validate_drawing(data, self.palette)

    def test_svg_color_and_source_are_string_metadata(self):
        for field in ("svg_color", "source"):
            for value in (None, 1, True, [], {}):
                data = copy.deepcopy(self.data)
                data["strokes"][0][field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    validate_drawing(data, self.palette)
        self.data["strokes"][0].update(svg_color="rgb(255, 0, 0)", source="")
        strokes = validate_drawing(self.data, self.palette)
        self.assertEqual(next(color_changes(strokes, self.palette))[1]["name"], "Black")

    def test_marker_alone_selects_pen_even_when_svg_color_disagrees(self):
        for stroke in self.data["strokes"]:
            stroke["svg_color"] = "#ffff00"
        strokes = validate_drawing(self.data, self.palette)
        self.assertEqual([p["program"] for _, p in color_changes(strokes, self.palette)],
                         ["ChangePen_Black", "ChangePen_Orange", "ChangePen_Purple", "ChangePen_Green"])
        strokes[1]["marker"] = "black"
        strokes[1]["svg_color"] = "#ff0000"
        self.assertEqual([i for i, _ in color_changes(strokes, self.palette)], [1, 3, 4])

    def test_coordinates_order_and_metadata_are_unchanged(self):
        self.data["strokes"] = list(reversed(self.data["strokes"]))
        self.data["strokes"][0]["points"] = [[200, 530], [153.75, 510.125], [100, 500]]
        before = copy.deepcopy(self.data)
        strokes = validate_drawing(self.data, self.palette)
        self.assertIs(strokes, self.data["strokes"])
        for i, stroke in enumerate(strokes, 1):
            moves = list(stroke_moves(i, stroke, -80, 0))
            self.assertEqual([xyz[:2] for _, xyz, _ in moves[1:-1]], stroke["points"])
            self.assertEqual(moves[0][1][:2], stroke["points"][0])
            self.assertEqual(moves[-1][1][:2], stroke["points"][-1])
            self.assertEqual([m[2] for m in moves], ["J"] + ["L"] * (len(stroke["points"]) + 1))
        self.assertEqual(self.data, before)

    def test_board_edges_are_inclusive_and_placement_is_not_reapplied(self):
        self.data["strokes"][0]["points"] = [[0, 0], [300, 600]]
        strokes = validate_drawing(self.data, self.palette)
        moves = list(stroke_moves(1, strokes[0], -80, 0))
        self.assertEqual(moves[1][1], [0, 0, 0])
        self.assertEqual(moves[2][1], [300, 600, 0])

    def test_legacy_protocol_is_rejected(self):
        legacy = {"units": "mm", "width": 300, "height": 600,
                  "strokes": [{"color": "#000000", "points": [[100, 500], [200, 500]]}]}
        with self.assertRaises(ValueError):
            validate_drawing(legacy, self.palette)
        stroke = self.data["strokes"][0]
        stroke["color"] = stroke.pop("marker")
        with self.assertRaisesRegex(ValueError, "marker"):
            validate_drawing(self.data, self.palette)

    def test_real_sample(self):
        data = json.loads((ROOT / "data/sample_strokes.json").read_text())
        before = copy.deepcopy(data)
        strokes = validate_drawing(data, self.palette)
        self.assertEqual(len(strokes), 5)
        self.assertEqual(sum(len(s["points"]) for s in strokes), 339)
        self.assertEqual({s["marker"] for s in strokes}, {"black"})
        self.assertEqual([s["source"] for s in strokes], ["4.0", "3.0", "1.0", "2.0", "0.0"])
        self.assertEqual([(i, p["program"]) for i, p in color_changes(strokes, self.palette)],
                         [(1, "ChangePen_Black")])
        for i, stroke in enumerate(strokes, 1):
            moves = list(stroke_moves(i, stroke, -80, 0))
            self.assertEqual([xyz[:2] for _, xyz, _ in moves[1:-1]], stroke["points"])
        self.assertEqual(data, before)

    def test_migrated_default_sample_is_supported(self):
        data = json.loads((ROOT / "data/test_strokes.json").read_text())
        strokes = validate_drawing(data, self.palette)
        self.assertEqual([s["points"] for s in strokes],
                         [[[100, 500], [200, 500]], [[160, 520], [200, 520]]])

    def test_validate_only_cli_does_not_import_robodk(self):
        entry = str(ROOT / "scripts/robot_program_json.py")
        code = (
            "import runpy,sys; "
            f"sys.argv=[{entry!r},'--validate-only','--data','data/sample_strokes.json']; "
            f"runpy.run_path({entry!r},run_name='__main__'); "
            "assert not any(m == 'robodk' or m.startswith('robodk.') for m in sys.modules)"
        )
        result = subprocess.run([sys.executable, "-I", "-c", code], cwd=ROOT,
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("5 strokes, 339 points", result.stdout)
        self.assertIn("validation passed", result.stdout)

    def test_palette_programs_must_identify_distinct_pens(self):
        self.settings["palette"][1]["program"] = "ChangePen_Black"
        with self.assertRaisesRegex(ValueError, "unique"):
            validate_palette(self.settings)

    def test_palette_is_keyed_by_marker_names_not_hex(self):
        self.settings["palette"][0]["hex"] = "#123456"
        palette = validate_palette(self.settings)
        self.assertEqual(set(palette), {"black", "orange", "green", "purple"})
        self.assertEqual(palette["black"]["program"], "ChangePen_Black")
        validate_drawing(self.data, palette)

    def test_palette_must_cover_each_available_marker_once(self):
        for name in ("Red", "BLACK"):
            settings = copy.deepcopy(self.settings)
            settings["palette"][1]["name"] = name
            with self.subTest(name=name), self.assertRaises(ValueError):
                validate_palette(settings)

    def test_reject_invalid_tcp_and_out_of_range_gripper_opening(self):
        for field, value in (("pen_tcp_xyz_mm", [0, 200]),
                             ("pen_tcp_xyz_mm", [0, 0, float("nan")]),
                             ("initial_jaw_opening_mm", 51)):
            settings = copy.deepcopy(self.settings)
            settings["hardware"][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                validate_palette(settings)


if __name__ == "__main__":
    unittest.main()
