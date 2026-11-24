from dataclasses import dataclass, field
import fcntl
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from drawbot import ui_bridge as bridge


@dataclass
class Drawing:
    board_width: float = 300
    board_height: float = 300
    placement: dict = field(default_factory=lambda: {"x": 20, "y": 30, "width": 100, "height": 100})
    strokes: list = field(default_factory=lambda: [{"points": [[20.123456789, 30.987654321], [120, 130]],
                                                    "marker": "black", "svg_color": "#111111", "source": "upload-unique"}])
    units: str = "mm"
    origin: str = "board top-left, x right, y down (same as SVG)"

    def to_dict(self):
        raise AssertionError("The rounding serializer must never be called")


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.drawing = Drawing()

    def submit(self, mode="simulate", **options):
        return bridge.submit_drawing(self.drawing, mode, dict(bridge.BASELINE), svg_bytes=b"<svg/>",
                                      source_name="my-upload.svg", program_name="UI_Test", project_root=self.root, **options)

    def successful_api(self, drawing, **kwargs):
        digest = hashlib.sha256(json.dumps(bridge.drawing_dict(drawing), sort_keys=True).encode()).hexdigest()
        return {"ok": True, "success": True, "message": "checked", "warnings": [], "errors": [],
                "report": {"input_sha256": digest}, "exported_files": []}

    def test_complete_dataclass_precision_and_ui_settings_reach_existing_api(self):
        with patch.object(bridge.robot_api, "draw_in_robodk", side_effect=self.successful_api) as api:
            result = self.submit()
        self.assertTrue(result["ok"])
        self.assertIs(api.call_args.args[0], self.drawing)
        kwargs = api.call_args.kwargs
        self.assertEqual({k: kwargs[k] for k in bridge.BASELINE}, bridge.BASELINE)
        self.assertEqual(kwargs["mode"], "simulate")
        self.assertEqual(kwargs["pen_change_mode"], "manual_preloaded")
        self.assertFalse(kwargs["allow_stub_export_for_testing"])
        self.assertNotIn("sample_strokes", str(kwargs["source_file"]))
        data = json.loads(kwargs["source_file"].read_text())
        self.assertEqual(data["strokes"][0]["points"][0], [20.123456789, 30.987654321])

    def test_export_modes_never_silently_allow_stubs(self):
        with patch.object(bridge.robot_api, "draw_in_robodk", side_effect=self.successful_api) as api:
            self.assertFalse(self.submit("export", export_mode="stub")["ok"])
            api.assert_not_called()
            self.assertTrue(self.submit("export", export_mode="stub", stub_confirmed=True)["ok"])
            self.assertTrue(api.call_args.kwargs["allow_stub_export_for_testing"])
            self.assertTrue(self.submit("export", export_mode="manual_preloaded")["ok"])
            self.assertFalse(api.call_args.kwargs["allow_stub_export_for_testing"])
            self.assertEqual(api.call_args.kwargs["pen_change_mode"], "manual_preloaded")

    def test_multicolor_manual_export_and_robot_mode_are_rejected(self):
        self.drawing.strokes.append({**self.drawing.strokes[0], "marker": "orange"})
        with patch.object(bridge.robot_api, "draw_in_robodk") as api:
            self.assertFalse(self.submit("export")["ok"])
            self.assertFalse(self.submit("robot")["ok"])
            api.assert_not_called()

    def test_external_requires_confirmation_and_successful_preflight(self):
        with patch.object(bridge, "external_preflight", side_effect=ValueError("Real non-STUB program required")) as preflight, patch.object(bridge.robot_api, "draw_in_robodk") as api:
            self.assertFalse(self.submit("export", export_mode="external")["ok"])
            preflight.assert_not_called()
            res = self.submit("export", export_mode="external", external_confirmed=True)
            self.assertFalse(res["ok"])
            self.assertIn("Real non-STUB", res["errors"][0])
            api.assert_not_called()

    def test_fingerprints_include_current_svg_points_marker_and_motion(self):
        before = bridge.request_fingerprint(self.drawing, bridge.BASELINE, b"a")
        self.assertNotEqual(before, bridge.request_fingerprint(self.drawing, bridge.BASELINE, b"b"))
        self.assertNotEqual(before, bridge.request_fingerprint(self.drawing, {**bridge.BASELINE, "draw_speed": 25}, b"a"))
        self.drawing.strokes[0]["marker"] = "orange"
        self.assertNotEqual(before, bridge.request_fingerprint(self.drawing, bridge.BASELINE, b"a"))
        self.drawing.strokes[0]["marker"] = "black"
        self.drawing.strokes[0]["points"][0][0] += 1
        self.assertNotEqual(before, bridge.request_fingerprint(self.drawing, bridge.BASELINE, b"a"))

    def test_mismatched_report_hash_discards_success(self):
        bad = {"ok": True, "success": True, "report": {"input_sha256": "old-input"}, "exported_files": ["old.urp"]}
        with patch.object(bridge.robot_api, "draw_in_robodk", return_value=bad):
            result = self.submit()
        self.assertFalse(result["ok"])
        self.assertEqual(result["exported_files"], [])

    def test_lock_prevents_overlapping_ui_calls(self):
        (self.root / "logs").mkdir()
        with (self.root / "logs/ui_job.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.object(bridge.robot_api, "draw_in_robodk") as api:
                result = self.submit()
                self.assertFalse(result["ok"])
                self.assertIn("Another Web UI", result["message"])
                api.assert_not_called()

    def test_downloads_require_current_success_matching_hash_and_export_location(self):
        exports = self.root / "exports"
        exports.mkdir()
        file = exports / "test.urp"
        file.write_bytes(b"urp fixture")
        result = {"ok": True, "exported_files": [str(file)], "report": {"export": {"files": [{"path": str(file), "sha256": hashlib.sha256(file.read_bytes()).hexdigest()}]}}}
        with patch.object(bridge, "ROOT", self.root):
            self.assertEqual(bridge.downloadable_files(result)[0][1], b"urp fixture")
            file.write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "changed"):
                bridge.downloadable_files(result)
            self.assertEqual(bridge.downloadable_files({**result, "ok": False}), [])
            result["exported_files"] = [str(self.root / "outside.urp")]
            with self.assertRaisesRegex(ValueError, "path"):
                bridge.downloadable_files(result)


class ExternalPreflightTests(unittest.TestCase):
    def setUp(self):
        from robodk.robomath import eye, transl
        from robodk.robolink import PROGRAM_RUN_ON_SIMULATOR
        config = json.loads((ROOT / "config/drawing.json").read_text())
        self.rdk = Mock()
        self.station = Mock()
        self.station.Name.return_value = "DrawBot"
        self.rdk.ActiveStation.return_value = self.station
        self.programs = []
        for pen in config["palette"]:
            program = Mock()
            program.Name.return_value = pen["program"]
            program.Parent.return_value = self.station
            program.getParam.return_value = b"0"
            program.RunType.return_value = PROGRAM_RUN_ON_SIMULATOR
            program.Busy.return_value = False
            program.InstructionCount.return_value = 10
            program.Update.return_value = (10, 5, 100, 1, "offline checked")
            self.programs.append(program)
        self.rdk.ItemList.return_value = self.programs
        robot, board, tool = Mock(), Mock(), Mock()
        for item, name in ((robot, config["robot_name"]), (board, config["board_frame"]), (tool, config["tool_name"])):
            item.Name.return_value = name
            item.Parent.return_value = self.station
        robot.PoseAbs.return_value = eye(4)
        robot.SolveFK.return_value = transl(0, 0, -100)
        board.PoseAbs.return_value = eye(4)
        tool.PoseTool.return_value = eye(4)
        items = {config["robot_name"]: robot, config["board_frame"]: board, config["tool_name"]: tool}
        self.rdk.Item.side_effect = lambda name, kind: items[name]
        self.robot = robot

    def check(self):
        with patch("drawbot.robot.connection.connect_existing", return_value=self.rdk):
            return bridge.external_preflight(80)

    def test_all_real_programs_receive_complete_collision_checks(self):
        checks = self.check()
        self.assertEqual(len(checks), 4)
        for program in self.programs:
            self.assertEqual(program.Update.call_args.kwargs, {"timeout_sec":120, "mm_step":1.0, "deg_step":1.0})
            program.RunProgram.assert_not_called()
        self.rdk.Disconnect.assert_called_once()

    def test_stubs_and_incomplete_paths_are_rejected(self):
        self.programs[0].getParam.return_value = b"1"
        with self.assertRaisesRegex(ValueError, "STUB"):
            self.check()
        self.programs[0].getParam.return_value = b"0"
        self.programs[0].Update.return_value = (9, 5, 100, .9, "collision")
        with self.assertRaisesRegex(ValueError, "validation failed"):
            self.check()

    def test_robot_run_type_and_initial_contact_pose_are_rejected(self):
        from robodk.robomath import eye
        self.programs[0].RunType.return_value = 2
        with self.assertRaisesRegex(ValueError, "simulator"):
            self.check()
        self.programs[0].RunType.return_value = 1
        self.robot.SolveFK.return_value = eye(4)
        with self.assertRaisesRegex(ValueError, "Initial TCP"):
            self.check()


class StationCheckTests(unittest.TestCase):
    """Any station works if it has the robot, board frame and pen tool from the config."""

    def setUp(self):
        self.config = json.loads((ROOT / "config/drawing.json").read_text())
        self.station = Mock()
        self.station.Name.return_value = "Someone else's station"
        self.rdk = Mock()
        self.rdk.ActiveStation.return_value = self.station
        self.items = {}
        for key in ("robot_name", "board_frame", "tool_name"):
            item = Mock()
            item.Name.return_value = self.config[key]
            item.Parent.return_value = self.station
            self.items[self.config[key]] = item
        self.rdk.Item.side_effect = lambda name, kind: self.items.get(name, Mock(Valid=Mock(return_value=False)))

    def test_any_station_name_passes(self):
        from drawbot.robot.station import check_station
        self.assertIs(check_station(self.rdk, self.config), self.station)

    def test_missing_board_frame_is_named(self):
        from drawbot.robot.station import check_station
        del self.items[self.config["board_frame"]]
        with self.assertRaisesRegex(RuntimeError, f"Board frame '{self.config['board_frame']}' not found"):
            check_station(self.rdk, self.config)
