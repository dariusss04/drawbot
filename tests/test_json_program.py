import contextlib
import copy
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from drawbot.robot.dataset import validate_drawing, validate_palette
from drawbot.robot.json_program import OWNER, OWNER_PARAM, drawing_statistics, generate, main
from robodk import robolink as api
from robodk.robomath import eye, Mat


class Item:
    def __init__(self, rdk, name, kind, parent=None, protected=False):
        self.rdk, self.name, self.kind, self.parent = rdk, name, kind, parent
        self.children, self.references, self.params, self.instructions = [], [], {}, []
        self.pose, self.protected, self.deleted = eye(4), protected, False
        if parent is not None:
            parent.children.append(self)

    def Valid(self):
        return not self.deleted

    def Name(self):
        return self.name

    def setName(self, name):
        self.name = name

    def Type(self):
        return self.kind

    def Parent(self):
        return self.parent

    def Childs(self):
        return list(self.children) + list(self.references)

    def getParam(self, key):
        return self.params.get(key, b"")

    def setParam(self, key, value):
        self.params[key] = value

    def Delete(self):
        if self.protected:
            raise AssertionError("Protected station item was deleted")
        for child in list(self.children):
            child.Delete()
        self.deleted = True
        self.parent.children.remove(self)
        self.rdk.deleted.append(self.name)

    def PoseAbs(self):
        return self.pose.copy()

    Pose = PoseAbs
    PoseTool = PoseAbs

    def Joints(self):
        return Mat([0.] * 6)

    def SolveIK(self, pose, seed, tool=None, reference=None):
        self.rdk.ik_requests.append(pose.copy())
        return Mat([0.] * 6)

    def SolveFK(self, joints):
        return eye(4)

    def setJoints(self, values):
        if self.protected:
            raise AssertionError("Robot state/calibration must not be written")
        self.joints = values

    def setVisible(self, visible, visible_frame=None):
        self.visible = visible

    def setPose(self, pose):
        if self.protected:
            raise AssertionError("Calibration was changed")
        self.pose = pose.copy()

    def setAsCartesianTarget(self):
        pass

    def setAsJointTarget(self):
        pass

    def setRunType(self, value):
        assert value == api.PROGRAM_RUN_ON_SIMULATOR

    def setPoseFrame(self, value):
        self.instructions.append(("frame", api.INS_TYPE_CHANGEFRAME, None))

    def setPoseTool(self, value):
        if self.protected:
            raise AssertionError("TCP calibration was changed")
        self.instructions.append(("tool", api.INS_TYPE_CHANGETOOL, None))

    def setSpeed(self, *values):
        self.instructions.append(("speed", api.INS_TYPE_CHANGESPEED, None))
        self.rdk.speed = values
        self.rdk.speed_history.append(values)

    def setRounding(self, value):
        self.instructions.append(("rounding", api.INS_TYPE_ROUNDING, None))
        self.rdk.rounding = value
        self.rdk.rounding_history = getattr(self.rdk, "rounding_history", []) + [value]

    def RunInstruction(self, name, kind):
        self.instructions.append((name, api.INS_TYPE_CODE, None))
        if kind == api.INSTRUCTION_CALL_PROGRAM:
            self.references += [p for p in self.rdk.ItemList(api.ITEM_TYPE_PROGRAM) if p.Name() == name]

    def move(self, target, kind):
        if target.Name() == self.rdk.fail_target:
            raise RuntimeError("Injected unreachable target for offline failure test")
        self.instructions.append((target.Name(), api.INS_TYPE_MOVE, kind))
        self.rdk.moves.append((target.Name(), target.pose.Pos(), kind))

    def MoveJ(self, target):
        self.move(target, api.MOVE_TYPE_JOINT)

    def MoveL(self, target):
        self.move(target, api.MOVE_TYPE_LINEAR)

    def InstructionCount(self):
        return len(self.instructions)

    def Instruction(self, index):
        return self.instructions[index]

    def Update(self, collision, timeout_sec, mm_step=-1, deg_step=-1):
        self.rdk.update_calls.append((collision, timeout_sec))
        self.rdk.check_steps = (mm_step, deg_step)
        if self.rdk.update_error:
            raise RuntimeError("Injected API timeout")
        if self.rdk.bad_counts:
            raise AssertionError("Update should not run after failed statistics")
        return self.rdk.update_result or (len(self.instructions), 10.0, 100.0, 1.0, "offline fake")


class FakeRoboDK:
    def __init__(self, settings):
        self.settings = settings
        self.deleted, self.moves, self.update_calls = [], [], []
        self.speed_history = []
        self.ik_requests = []
        self.fail_target = self.update_result = self.update_error = None
        self.bad_counts = False
        self.disconnected = False
        self.station = Item(self, "DrawBot", api.ITEM_TYPE_STATION)
        self.robot = Item(self, settings["robot_name"], api.ITEM_TYPE_ROBOT, self.station, True)
        self.board = Item(self, settings["board_frame"], api.ITEM_TYPE_FRAME, self.station, True)
        self.tool = Item(self, settings["tool_name"], api.ITEM_TYPE_TOOL, self.robot, True)
        self.reference = Item(self, settings["orientation_target"], api.ITEM_TYPE_TARGET, self.board, True)
        mount = Item(self, settings["hardware"]["mount_frame_name"], api.ITEM_TYPE_FRAME, self.robot, True)
        Item(self, settings["hardware"]["gripper_name"], api.ITEM_TYPE_ROBOT, mount, True)
        self.geometry = Item(self, "Board & image", api.ITEM_TYPE_OBJECT, self.board, True)
        self.manual = Item(self, "ManualProgram", api.ITEM_TYPE_PROGRAM, self.station, True)

    def ActiveStation(self):
        return self.station

    def Item(self, name, kind):
        return next((x for x in self.ItemList(kind) if x.Name() == name), InvalidItem())

    def Version(self):
        return "offline-fake"

    def Save(self, filename, station):
        Path(filename).write_text('offline fake station for test only')

    def Render(self, enabled):
        self.render_enabled = enabled

    def CollisionActivePairList(self):
        gripper = next(x for x in self.ItemList(api.ITEM_TYPE_ROBOT) if x is not self.robot)
        return [(self.robot, self.geometry, 6, 0), (gripper, self.geometry, 0, 0),
                (self.robot, gripper, 5, 0)]

    def ItemList(self, kind):
        def walk(node):
            for child in node.children:
                yield child
                yield from walk(child)
        return [node for node in walk(self.station) if node.kind == kind]

    def setRunMode(self, mode):
        assert mode == api.RUNMODE_SIMULATE

    def AddFrame(self, name, parent):
        return Item(self, name, api.ITEM_TYPE_FRAME, parent)

    def AddTarget(self, name, parent, robot):
        assert robot is self.robot
        return Item(self, name, api.ITEM_TYPE_TARGET, parent)

    def AddProgram(self, name, robot):
        assert robot is self.robot
        node = Item(self, name, api.ITEM_TYPE_PROGRAM, self.station)
        if self.bad_counts and name == self.settings["program_name"]:
            original = node.Instruction
            node.Instruction = lambda i: ("not a move", api.INS_TYPE_CODE, None) if i == 5 else original(i)
        return node

    def Disconnect(self):
        self.disconnected = True


class InvalidItem:
    def Valid(self):
        return False


class GenerationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.settings = json.loads((ROOT / "config/drawing.json").read_text())
        self.settings.pop("home", None)
        self.settings["_project_root"] = str(Path(self.temporary.name).resolve())
        self.palette = validate_palette(self.settings)
        self.data = json.loads((ROOT / "data/sample_strokes.json").read_text())
        self.strokes = validate_drawing(self.data, self.palette)
        self.rdk = FakeRoboDK(self.settings)
        self.network = patch("drawbot.robot.connection.socket.create_connection")
        self.network.start()
        self.addCleanup(self.network.stop)

    def run_generator(self, report=None):
        with patch.object(api, "Robolink", return_value=self.rdk), contextlib.redirect_stdout(io.StringIO()):
            return generate(self.settings, self.strokes, self.palette, report)

    def test_full_real_sample_counts_coordinates_names_and_requests(self):
        before = copy.deepcopy(self.data)
        report = self.run_generator()
        self.assertEqual(drawing_statistics(self.strokes, self.palette), {
            "stroke_count": 5, "path_point_count": 339, "motion_count": 349,
            "joint_motion_count": 5, "linear_motion_count": 344, "pen_change_count": 1,
        })
        self.assertEqual(report["status"], "success")
        self.assertEqual(report["motion_count"], 349)
        self.assertEqual(report["target_count"], 349)
        self.assertTrue(report["counts_match"])
        self.assertEqual([(p["stroke"], p["marker"], p["program"]) for p in report["pen_requests"]],
                         [(1, "black", "ChangePen_Black")])
        expected_names = []
        for i, stroke in enumerate(self.strokes, 1):
            prefix = f"S{i:02d}"
            expected_names += [f"{prefix}_Approach", f"{prefix}_PenDown"]
            expected_names += [f"{prefix}_Draw_P{j:03d}" for j in range(2, len(stroke["points"]) + 1)]
            expected_names += [f"{prefix}_Retract"]
            manifest = [m for m in report["motion_manifest"] if m["stroke"] == i]
            self.assertEqual([m["xyz_mm"][:2] for m in manifest[1:-1]], stroke["points"])
            self.assertEqual([m["motion"] for m in manifest], ["J"] + ["L"] * (len(stroke["points"]) + 1))
        self.assertEqual([name for name, _, _ in self.rdk.moves], expected_names)
        self.assertEqual(self.data, before)
        self.assertEqual(self.rdk.speed, (20, 30, 100, 60))
        self.assertEqual(self.rdk.rounding, -1)
        self.assertEqual(self.rdk.update_calls, [(api.COLLISION_ON, 120)])
        self.assertFalse(self.rdk.manual.deleted)
        self.assertTrue(self.rdk.disconnected)

    def test_regeneration_only_deletes_owned_outputs(self):
        self.run_generator()
        stubs = [p for p in self.rdk.ItemList(api.ITEM_TYPE_PROGRAM) if p.Name().startswith("ChangePen_")]
        self.rdk.moves.clear()
        report = self.run_generator()
        self.assertEqual(report["motion_count"], 349)
        self.assertNotIn("ManualProgram", self.rdk.deleted)
        self.assertFalse(any(p.deleted for p in stubs))
        self.assertIn("SVG_Draw_JSON", self.rdk.deleted)
        self.assertIn("SVG_JSON_Targets", self.rdk.deleted)

    def test_unowned_same_name_is_preserved_before_any_deletion(self):
        old = self.rdk.AddProgram(self.settings["program_name"], self.rdk.robot)
        report = {}
        self.run_generator(report)
        self.assertFalse(old.deleted)
        self.assertEqual(self.rdk.deleted, [])
        self.assertTrue(old.Name().startswith("SVG_Draw_JSON_preserved_"))
        self.assertEqual(len(report["preserved_legacy_outputs"]), 1)
        self.assertEqual(report["stage"], "complete")

    def test_manual_child_under_owned_frame_is_preserved(self):
        self.run_generator()
        frame = next(f for f in self.rdk.ItemList(api.ITEM_TYPE_FRAME) if f.Name() == "SVG_JSON_Targets")
        manual = Item(self.rdk, "ManualTarget", api.ITEM_TYPE_TARGET, frame, True)
        report = self.run_generator()
        self.assertFalse(manual.deleted)
        self.assertTrue(frame.Name().startswith("SVG_JSON_Targets_preserved_"))
        self.assertEqual(len(report["preserved_legacy_outputs"]), 1)

    def test_failed_target_retains_partial_program_and_context(self):
        self.rdk.fail_target = "S02_Draw_P003"
        report = {}
        with self.assertRaisesRegex(RuntimeError, "unreachable"):
            self.run_generator(report)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["context"]["stroke"], 2)
        self.assertEqual(report["context"]["target"], "S02_Draw_P003")
        self.assertEqual(report["context"]["source"], "3.0")
        self.assertGreater(report["target_count"], report["motion_count"])
        self.assertEqual(self.rdk.deleted, [])

    def test_failed_path_retains_raw_metrics_and_labels_candidate(self):
        self.rdk.update_result = (8, 2.5, 30.0, .2, "Injected collision")
        report = {}
        with self.assertRaisesRegex(RuntimeError, "collision"):
            self.run_generator(report)
        self.assertEqual(report["valid_instructions"], 8)
        self.assertEqual(report["valid_ratio"], .2)
        self.assertEqual(report["motion_count"], 349)
        self.assertIn("confirm in GUI", report["diagnostic_basis"])
        self.assertIsNotNone(report["diagnostic_candidate"]["target"])
        self.assertEqual(self.rdk.deleted, [])

    def test_invalid_update_metrics_cannot_pass(self):
        self.rdk.update_result = (354, 2.5, 30.0, float("nan"), "bad ratio")
        report = {}
        with self.assertRaisesRegex(RuntimeError, "Path check failed"):
            self.run_generator(report)
        self.assertIsNone(report["valid_ratio"])
        self.assertEqual(report["invalid_update_values"], {"valid_ratio": "nan"})
        json.dumps(report, allow_nan=False)

    def test_update_exception_preserves_manifest_and_unknown_validity(self):
        self.rdk.update_error = True
        report = {}
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            self.run_generator(report)
        self.assertEqual(report["stage"], "path_check")
        self.assertEqual(len(report["motion_manifest"]), 349)
        self.assertIsNone(report["valid_ratio"])

    def test_statistics_mismatch_is_a_failure(self):
        self.rdk.bad_counts = True
        with self.assertRaisesRegex(RuntimeError, "counts do not match"):
            self.run_generator()
        self.assertEqual(self.rdk.update_calls, [])

    def test_missing_hardware_does_not_import_or_modify_models(self):
        mount = self.rdk.tool.Parent().children[-1]
        mount.name = "Different mount"
        with self.assertRaisesRegex(RuntimeError, "Missing RoboDK item"):
            self.run_generator()
        self.assertEqual(self.rdk.deleted, [])

    def test_main_saves_failed_and_successful_reports_without_real_robodk(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            shutil.copytree(ROOT / "config", root / "config")
            shutil.copytree(ROOT / "data", root / "data")
            self.rdk.update_result = (8, 2.5, 30.0, .2, "Injected collision")
            with patch.object(api, "Robolink", return_value=self.rdk), patch.object(sys, "argv", ["macro"]):
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                    main(root)
            first = json.loads((root / "logs/last_json_generation.json").read_text())
            self.assertEqual(first["status"], "failed")
            self.assertEqual(first["data_path"], str(root / "data/sample_strokes.json"))
            self.assertEqual(first["motion_count"], 352)
            self.rdk.update_result = None
            with patch.object(api, "Robolink", return_value=self.rdk), patch.object(sys, "argv", ["macro"]):
                with contextlib.redirect_stdout(io.StringIO()):
                    main(root)
            second = json.loads((root / "logs/last_json_generation.json").read_text())
            self.assertEqual(second["status"], "success")
            self.assertEqual(len(list((root / "logs").glob("json_generation_*.json"))), 2)

    def test_invalid_input_report_does_not_connect(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            shutil.copytree(ROOT / "config", root / "config")
            (root / "data").mkdir()
            (root / "data/sample_strokes.json").write_text('{}')
            with patch.object(api, "Robolink") as connect, patch.object(sys, "argv", ["macro"]):
                with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
                    main(root)
            connect.assert_not_called()
            report = json.loads((root / "logs/last_json_generation.json").read_text())
            self.assertEqual(report["stage"], "input_validation")
            self.assertEqual(report["status"], "failed")

    def test_home_moves_wrap_the_drawing_and_leave_the_robot_alone(self):
        self.settings["home"] = {"board_xyz_mm": [150.0, 150.0, -150.0], "seed_joints_deg": [81.4, -61.0, -85.6, -33.4, 98.6, 0.0]}
        self.rdk = FakeRoboDK(self.settings)
        report = self.run_generator()
        self.assertEqual(report["motion_count"], 351)
        self.assertEqual(report["joint_motion_count"], 7)
        names = [m["target"] for m in report["motion_manifest"]]
        self.assertEqual((names[0], names[-1]), ("Home_Start", "Home_End"))

    def test_park_upright_adds_a_last_joint_move(self):
        self.settings["home"] = {"board_xyz_mm": [150.0, 150.0, -150.0], "seed_joints_deg": [81.4, -61.0, -85.6, -33.4, 98.6, 0.0],
                                 "park_upright": True}
        self.rdk = FakeRoboDK(self.settings)
        report = self.run_generator()
        self.assertEqual((report["motion_count"], report["joint_motion_count"]), (352, 8))
        self.assertEqual(report["motion_manifest"][-1]["target"], "Home_Park")
        self.assertEqual(report["park_joints_deg"][1:], [-90.0, 0.0, -90.0, 0.0, 0.0])

    def test_program_ends_at_the_home_pose_from_the_ui(self):
        home = [90.0, -140.0, 113.0, -66.0, -90.0, -99.0]
        self.settings["home"] = {"board_xyz_mm": [150.0, 150.0, -150.0], "seed_joints_deg": [81.4, -61.0, -85.6, -33.4, 98.6, 0.0],
                                 "park_upright": True, "park_joints_deg": home}
        self.rdk = FakeRoboDK(self.settings)
        report = self.run_generator()
        self.assertEqual(report["motion_manifest"][-1]["target"], "Home_Park")
        self.assertEqual(report["park_joints_deg"], home)

    def test_tmp_embedded_entry_with_unrelated_working_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            entry = Path(directory) / "robot_program_json.py"
            shutil.copyfile(ROOT / "scripts/robot_program_json.py", entry)
            for args, summary, motions in [
                ([], "5 strokes, 339 points", "352 motions (8 MoveJ, 344 MoveL)"),
                (["--data", "data/test_strokes.json"], "2 strokes, 4 points", "11 motions (5 MoveJ, 6 MoveL)"),
            ]:
                with self.subTest(args=args):
                    result = subprocess.run(
                        [sys.executable, "-I", str(entry), "--validate-only", *args],
                        cwd=directory, capture_output=True, text=True, timeout=10,
                        env={**os.environ, "DRAWBOT_HOME": str(ROOT)})
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn(summary, result.stdout)
                    self.assertIn(motions, result.stdout)
                    self.assertIn("1 pen request(s)", result.stdout)


if __name__ == "__main__":
    unittest.main()
