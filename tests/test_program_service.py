import copy
from dataclasses import make_dataclass
import gzip
import json
import math
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from drawbot.robot.api import draw_in_robodk, status
from drawbot.robot.circle_check import check_circle
from drawbot.robot.connection import connect_existing
from drawbot.robot.export import export_program, inspect_ur_files, verify_script_semantics
from drawbot.robot.playback import play_in_simulator
from drawbot.robot.motion import motion_parameters, phase_for_target
from drawbot.robot.json_program import save_report
from robodk import robolink as api
from test_json_program import FakeRoboDK


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.settings = json.loads((ROOT / "config/drawing.json").read_text())
        self.settings.pop("home", None)
        self.settings["_project_root"] = str(self.root)
        self.data = json.loads((ROOT / "data/test_strokes.json").read_text())
        self.rdk = FakeRoboDK(self.settings)
        self.connection = patch("drawbot.robot.connection.socket.create_connection")
        self.connection.start()
        self.addCleanup(self.connection.stop)
        self.factory = patch.object(api, "Robolink", return_value=self.rdk)
        self.factory.start()
        self.addCleanup(self.factory.stop)

    def run_service(self, data=None, **kwargs):
        return draw_in_robodk(self.data if data is None else data, settings=self.settings,
                              project_root=self.root, **kwargs)

    def test_service_success_is_from_update_not_only_json_validation(self):
        result = self.run_service()
        self.assertTrue(result["success"])
        self.assertEqual(result["motion_count"], 8)
        self.assertEqual(result["simulation_time_s"], 10)
        self.assertTrue(self.rdk.update_calls)
        self.assertTrue(Path(result["report_path"]).exists())

    def test_service_does_not_mutate_data_or_global_config(self):
        before_data, before_settings = copy.deepcopy(self.data), copy.deepcopy(self.settings)
        result = self.run_service(draw_speed=13, travel_speed=30, approach_mm=90)
        self.assertTrue(result["success"])
        self.assertEqual(self.data, before_data)
        self.assertEqual(self.settings, before_settings)
        self.assertEqual(self.rdk.moves[0][1][-1], -90)

    def test_partial_stroke_list_and_conflicting_board_size_are_rejected(self):
        for data, args in [(self.data["strokes"], {}), (self.data, {"board_width": 999})]:
            with self.subTest(args=args):
                result = self.run_service(data, **args)
                self.assertFalse(result["success"])
                self.assertTrue(result["errors"])
        self.assertFalse(self.rdk.update_calls)

    def test_dataclass_input_keeps_full_precision_without_to_dict_rounding(self):
        cls = make_dataclass("StrokeList", [(k, object) for k in self.data])
        data = copy.deepcopy(self.data)
        data["strokes"][0]["points"][0] = [100.123456789, 500.123456789]
        obj = cls(**data)
        result = self.run_service(obj)
        self.assertTrue(result["success"])
        self.assertEqual(result["report"]["motion_manifest"][1]["xyz_mm"][:2], [100.123456789, 500.123456789])

    def test_status_is_not_path_success(self):
        result = status(self.settings)
        self.assertTrue(result["success"])
        self.assertIn("no path", result["message"])
        self.assertFalse(self.rdk.update_calls)

    def test_status_ignores_station_name_but_needs_the_robot(self):
        self.rdk.station.name = "Any station name"
        self.assertTrue(status(self.settings)["success"])
        self.rdk.robot.name = "Missing UR5e"
        result = status(self.settings)
        self.assertFalse(result["success"])
        self.assertIn("Robot 'UR5e' not found", result["message"])

    def test_api_unavailable_has_clear_failure_and_never_starts_robodk(self):
        with patch("drawbot.robot.connection.socket.create_connection", side_effect=OSError("not running")), patch.object(api, "Robolink") as factory:
            result = self.run_service()
            self.assertFalse(result["success"])
            self.assertIn("not available", result["errors"][0])
            factory.assert_not_called()

    def test_external_missing_real_pen_program_is_failure(self):
        result = self.run_service(pen_change_mode="external")
        self.assertFalse(result["success"])
        self.assertIn("Real pen-change program required", result["errors"][0])

    def test_manual_preloaded_counts_request_without_stub_call(self):
        result = self.run_service(pen_change_mode="manual_preloaded", preloaded_marker="black")
        self.assertTrue(result["success"])
        self.assertEqual(result["report"]["pen_change_count"], 1)
        self.assertEqual(result["report"]["pen_call_count"], 0)
        self.data["strokes"][1]["marker"] = "orange"
        self.assertFalse(self.run_service(pen_change_mode="manual_preloaded", preloaded_marker="black")["success"])

    def test_collision_failure_never_reports_success_or_exports(self):
        self.rdk.update_result = (2, 1., 2., .1, "offline injected collision")
        with patch("drawbot.robot.export.export_program") as export:
            result = self.run_service(mode="export", pen_change_mode="manual_preloaded", preloaded_marker="black")
            self.assertFalse(result["success"])
            export.assert_not_called()

    def test_stub_export_is_blocked_after_path_check(self):
        result = self.run_service(mode="export")
        self.assertFalse(result["success"])
        self.assertIn("STUB", result["errors"][0])
        self.assertEqual(result["valid_ratio"], 1.)

    def test_failed_report_does_not_overwrite_last_passed_result(self):
        success = self.run_service()
        passed = (self.root / "logs/last_successful_generation.json").read_bytes()
        self.rdk.update_error = True
        failed = self.run_service()
        self.assertFalse(failed["success"])
        self.assertEqual((self.root / "logs/last_successful_generation.json").read_bytes(), passed)
        self.assertTrue(Path(success["report"]["station_file"]).exists())

    def test_phase_speeds_are_applied_by_same_generator(self):
        result = self.run_service(motion={"draw_speed_mm_s": 12, "pen_down_speed_mm_s": 5,
                                         "retract_speed_mm_s": 30, "travel_speed_mm_s": 40})
        self.assertTrue(result["success"])
        self.assertTrue({5, 12, 30, 40}.issubset({v[0] for v in self.rdk.speed_history}))
        self.assertEqual(result["motion_count"], 8)

    def test_motion_validation_and_blend_fallback(self):
        for value in [-1, True, float('nan'), float('inf')]:
            config = copy.deepcopy(self.settings)
            config["motion"]["draw_speed_mm_s"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                motion_parameters(config)
        self.settings["motion"]["blend_radius_mm"] = 2
        result = motion_parameters(self.settings)
        self.assertEqual(result["effective_blend_radius_mm"], 2)
        self.assertIn("gentle bends", result["warnings"][0])
        for bad in (-1, 11, float("nan")):
            self.settings["motion"]["blend_radius_mm"] = bad
            with self.subTest(blend=bad), self.assertRaises(ValueError):
                motion_parameters(self.settings)

    def test_phase_classification(self):
        self.assertEqual([phase_for_target(n) for n in ["S01_Approach", "S01_PenDown", "S01_Draw_P002", "S01_Retract"]],
                         ["travel", "pen_down", "draw", "retract"])

    def test_approach_caches_use_official_ik_without_changing_coordinates(self):
        result = self.run_service()
        self.assertTrue(result["success"])
        self.assertEqual(result["report"]["approach_joint_cache_refreshed"], 2)
        self.assertEqual([p.Pos() for p in self.rdk.ik_requests], [[100, 500, -80], [160, 520, -80]])
        self.assertEqual(result["motion_count"], 8)

    def circle_data(self, n=24, r=40):
        data = copy.deepcopy(self.data)
        pts = [[100 + r * math.cos(2 * math.pi * i / n), 100 + r * math.sin(2 * math.pi * i / n)] for i in range(n + 1)]
        data["strokes"] = [{**data["strokes"][0], "points": pts}]
        return data

    def test_blending_rounds_gentle_bends_and_stops_at_the_ends(self):
        result = self.run_service(self.circle_data(), motion={"blend_radius_mm": 2})
        self.assertTrue(result["success"], result.get("message"))
        manifest = result["report"]["motion_manifest"]
        blends = {m["target"]: m["blend_mm"] for m in manifest}
        self.assertEqual(blends["S01_Approach"], 0)
        self.assertEqual(blends["S01_PenDown"], 0)
        self.assertEqual(blends["S01_Draw_P025"], 0)
        self.assertEqual(blends["S01_Retract"], 0)
        middle = [m["blend_mm"] for m in manifest if "_Draw_P0" in m["target"]][:-1]
        self.assertTrue(middle and all(b == 2.0 for b in middle), middle)
        self.assertEqual(self.rdk.rounding_history.count(2.0), 1)
        self.assertEqual(self.rdk.rounding, -1)

    def test_no_blend_keeps_every_stop(self):
        result = self.run_service(self.circle_data(), motion={"blend_radius_mm": 0})
        self.assertTrue(result["success"])
        self.assertTrue(all(m["blend_mm"] == 0 for m in result["report"]["motion_manifest"]))
        self.assertNotIn(2.0, getattr(self.rdk, "rounding_history", []))


class CircleTests(unittest.TestCase):
    def setUp(self):
        self.data = json.loads((ROOT / "data/circle_100mm_strokes.json").read_text())

    def test_real_circle_geometry(self):
        report = check_circle(self.data)
        self.assertTrue(report["passed"])
        self.assertEqual(report["point_count"], 65)
        self.assertEqual(report["center_mm"], [70, 80])
        self.assertEqual(report["measured_width_mm"], 100)
        self.assertEqual(report["measured_height_mm"], 100)
        self.assertEqual(report["closure_error_mm"], 0)
        self.assertLess(report["max_segment_radial_error_mm"], .1)

    def test_segment_error_is_checked_not_only_vertices(self):
        self.data["strokes"][0]["points"] = [[120, 80], [70, 130], [20, 80], [70, 30], [120, 80]]
        report = check_circle(self.data)
        self.assertEqual(report["max_vertex_radial_error_mm"], 0)
        self.assertAlmostEqual(report["max_segment_radial_error_mm"], 50-50/math.sqrt(2))
        self.assertFalse(report["passed"])

    def test_open_wrong_size_and_wrong_center_circles_fail(self):
        self.assertFalse(check_circle(self.data, diameter_mm=90)["passed"])
        self.assertFalse(check_circle(self.data, center=[71, 80])["passed"])
        self.data["strokes"][0]["points"].pop()
        self.assertFalse(check_circle(self.data)["passed"])

    def test_circle_tolerance_is_configurable_and_not_a_teacher_claim(self):
        self.assertFalse(check_circle(self.data, tolerance_mm=.001)["passed"])
        self.assertIn("not a teacher", check_circle(self.data)["tolerance_source"])


class ExportTests(unittest.TestCase):
    def test_failed_validation_never_starts_simulator_playback(self):
        with self.assertRaisesRegex(RuntimeError, "requires successful"):
            play_in_simulator({}, {"status": "failed"})

    def test_stale_joint_cache_is_detected_and_report_pose_is_not_mutated(self):
        from robodk.robomath import eye, transl
        from unittest.mock import Mock
        report = {"station_calibration": {"pen_tcp_pose": eye(4).Rows(),
                  "reference_frame_pose_robot": eye(4).Rows(), "drawing_orientation_board": eye(4).Rows()},
                  "motion_parameters": {"joint_speed_deg_s": 30, "joint_accel_deg_s2": 60},
                  "motion_manifest": [{"target": "S01_Approach", "motion": "J", "xyz_mm": [0, 0, -80]}],
                  "pen_requests": [], "pen_call_count": 0, "pen_change_mode": "manual_preloaded"}
        script = ('def Test():\n # Main program:\n ref_frame=p[0,0,0,0,0,0]\n'
                  ' set_tcp(p[0,0,0,0,0,0])\n accel_radss=1.047\n speed_rads=0.524\n'
                  ' movej([0,0,0,0,0,0],accel_radss,speed_rads,t=0,r=0.000)\n # End of main program\nend\n')
        robot = Mock()
        robot.SolveFK.return_value = transl(100, 0, -80)
        with self.assertRaisesRegex(RuntimeError, "endpoint differs"):
            verify_script_semantics(script, report, robot)
        robot.SolveFK.return_value = transl(0, 0, -80)
        before = copy.deepcopy(report)
        result = verify_script_semantics(script, report, robot)
        self.assertEqual(result["max_joint_endpoint_fk_error_mm"], 0)
        self.assertEqual(report, before)

    def test_export_requires_passed_path_and_explicit_pen_mode(self):
        for report in [{}, {"status": "success", "valid_ratio": .5, "counts_match": True}]:
            with self.assertRaisesRegex(RuntimeError, "not passed"):
                export_program({}, report, ROOT)
        with self.assertRaisesRegex(RuntimeError, "STUB"):
            export_program({"pen_change_mode": "stub"}, {"status": "success", "valid_ratio": 1., "counts_match": True}, ROOT)

    def test_official_file_structure_counts_and_identical_urscript_source(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)
            (p / "test.script").write_text('def test():\n set_tcp(p[0,0,0,0,0,0])\n movej([0,0,0,0,0,0])\n movel(p[0,0,0,0,0,0])\nend\n')
            (p / "test.urp").write_bytes(gzip.compress(b'<URProgram name="test"/>'))
            result = inspect_ur_files(p, {"joint_motion_count": 1, "linear_motion_count": 1})
            self.assertTrue(result["urscript_is_identical_verified_source"])
            self.assertEqual(result["urp_xml_root"], "URProgram")
            with self.assertRaisesRegex(RuntimeError, "incomplete"):
                inspect_ur_files(p, {"joint_motion_count": 1, "linear_motion_count": 2})


if __name__ == "__main__":
    unittest.main()
