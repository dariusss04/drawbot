import socket
import struct
import sys
import threading
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drawbot import board_fit, robot_link  # noqa: E402
from drawbot.live_speed import SpeedLink  # noqa: E402

W, H = 500.0, 300.0


def depth_at(matrix, x, y):
    """Where the corrected plane is, along the measured board z, above board point (x, y)."""
    origin, normal = matrix[:3, 3], matrix[:3, 2]
    return origin[2] - (normal[0] * x + normal[1] * y - normal[0] * origin[0] - normal[1] * origin[1]) / normal[2]


class PlaneFit(unittest.TestCase):
    def test_same_value_everywhere_is_a_pure_shift(self):
        result = board_fit.fit({k: 1.0 for k in board_fit.POINTS}, W, H)
        self.assertAlmostEqual(result["a"], 1.0)
        self.assertAlmostEqual(result["b"], 0.0)
        self.assertAlmostEqual(result["c"], 0.0)
        expected = np.eye(4)
        expected[2, 3] = 1.0
        np.testing.assert_allclose(board_fit.correction_matrix(1.0, 0, 0), expected, atol=1e-12)

    def test_left_side_deeper_tilts_around_the_vertical_axis(self):
        values = {"tl": 1.0, "bl": 1.0, "c": 0.5, "tr": 0.0, "br": 0.0}
        result = board_fit.fit(values, W, H)
        for k in board_fit.POINTS:
            self.assertAlmostEqual(result["applied"][k], values[k], places=9)
        self.assertLess(result["b"], 0)
        self.assertAlmostEqual(result["c"], 0.0, places=12)
        m = board_fit.correction_matrix(result["a"], result["b"], result["c"])
        where = board_fit.positions(W, H)
        for k in ("tl", "br"):
            self.assertAlmostEqual(depth_at(m, *where[k]), values[k], places=6)
        np.testing.assert_allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-12)

    def test_a_bent_board_leaves_something_over(self):
        result = board_fit.fit({"c": 1.0}, W, H)
        self.assertGreater(abs(result["leftover"]["c"]), 0.5)

    def test_large_corrections_are_applied_too(self):
        pose, result = board_fit.corrected_pose(np.eye(4).tolist(), {k: 8.0 for k in board_fit.POINTS}, W, H)
        self.assertAlmostEqual(pose[2][3], 8.0)

    def test_touch_test_has_a_cross_at_every_point(self):
        strokes = board_fit.touch_test_strokes(W, H).strokes
        self.assertEqual(len(strokes), 2 * len(board_fit.POINTS))
        for s in strokes:
            for x, y in s.points:
                self.assertTrue(0 <= x <= W and 0 <= y <= H)

    def test_corrections_survive_the_setup_file(self):
        values = {"fit_tl": 1, "fit_br": -2}
        setup = robot_link.build_setup(values)
        back, _ = robot_link.read_setup(setup)
        self.assertEqual(robot_link.fit_values(back), {"tl": 1.0, "tr": 0.0, "c": 0.0, "bl": 0.0, "br": -2.0})


class SetupFile(unittest.TestCase):
    def test_every_setting_survives_save_and_load(self):
        values = {"robot_ip": "10.0.0.5", "robot_port": 30003, "laptop_port": 30010, "pen_mm": 280.0, "nopen_mm": 150.0,
                  "grip_force": 180, "manual_grip": True, "go_home": False, "home_joints": "0, -90, 0, -90, 0, 0",
                  "press_mm": 1.0, "fit_tl": 2, "fit_tr": 0, "fit_c": 1, "fit_bl": 2,
                  "fit_br": -1, "slot_1": "green", "slot_2": "orange", "slot_3": "purple", "slot_4": "black",
                  "pick_speed": 40.0, "pick_joint_speed": 20.0, "stand_held": "none", "v_draw": 120.0, "v_lift": 80.0,
                  "v_joint": 90.0, "approach": 60.0, "blend": 0.5, "park_upright": False, "live_speed": 35,
                  "speed_manual": True, "board_w": 500.0, "board_h": 300.0, "margin": 15.0, "spacing": 3.0,
                  "optimize": False, "marker_mode": "Multiple markers", "single_marker": "orange", "keep_colors": False,
                  "job_export_mode": "Pen stand: the robot changes pens itself",
                  "fill_on": True, "fill_pen": 1.0, "fill_spacing": 2.5, "fill_min_seg": 5.0, "fill_min_spacing": 2.0,
                  "fill_mode": "contour"}
        self.assertEqual(set(values), set(robot_link.SETUP_FIELDS))
        back, _ = robot_link.read_setup(robot_link.build_setup(values))
        self.assertEqual(back, values)

    def test_values_the_ui_does_not_offer_are_dropped(self):
        back, _ = robot_link.read_setup(robot_link.build_setup({"slot_1": "red", "marker_mode": "Lots", "v_draw": 50}))
        self.assertEqual(back, {"v_draw": 50.0})


class FakeRobot(threading.Thread):
    """Answers the RTDE handshake like a UR controller and records the data packages."""

    def __init__(self, types="UINT32,DOUBLE"):
        super().__init__(daemon=True)
        self.server = socket.socket()
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.types, self.packages, self.outputs = types, [], None
        self.received = threading.Event()

    def run(self):
        conn, _ = self.server.accept()
        with conn:
            while True:
                head = conn.recv(3)
                if len(head) < 3:
                    return
                size, kind = struct.unpack(">HB", head)
                body = conn.recv(size - 3) if size > 3 else b""
                if kind == ord("V"):
                    conn.sendall(struct.pack(">HBB", 4, kind, 1))
                elif kind == ord("I"):
                    assert body == b"speed_slider_mask,speed_slider_fraction"
                    reply = bytes([1]) + self.types.encode()
                    conn.sendall(struct.pack(">HB", 3 + len(reply), kind) + reply)
                elif kind == ord("O"):
                    self.outputs = body[8:]
                    reply = bytes([2]) + b"DOUBLE"
                    conn.sendall(struct.pack(">HB", 3 + len(reply), kind) + reply)
                elif kind == ord("S"):
                    if self.outputs is None:
                        text = b"No outputs configured"
                        msg = bytes([len(text)]) + text + bytes([4]) + b"RTDE" + bytes([1])
                        conn.sendall(struct.pack(">HB", 3 + len(msg), ord("M")) + msg)
                    conn.sendall(struct.pack(">HBB", 4, kind, 1 if self.outputs is not None else 0))
                    if self.outputs is not None:
                        clock = bytes([2]) + struct.pack(">d", 12.5)
                        conn.sendall(struct.pack(">HB", 3 + len(clock), ord("U")) + clock)
                elif kind == ord("U"):
                    self.packages.append(struct.unpack(">BId", body))
                    self.received.set()


class LiveSpeed(unittest.TestCase):
    def test_sets_the_speed_slider(self):
        robot = FakeRobot()
        robot.start()
        link = SpeedLink()
        link.connect("127.0.0.1", robot.port)
        self.assertEqual(link.set(0.35), 0.35)
        self.assertEqual(link.set(1.7), 1.0)
        robot.received.wait(2)
        link.close()
        robot.join(2)
        self.assertEqual(robot.packages, [(1, 1, 0.35), (1, 1, 1.0)])
        self.assertEqual(robot.outputs, b"timestamp")

    def test_start_refused_names_the_robots_reason(self):
        from drawbot import live_speed
        robot = FakeRobot()
        robot.start()
        link = SpeedLink()
        original = live_speed.SETUP_OUTPUTS
        try:
            live_speed.SETUP_OUTPUTS = ord("X")
            with self.assertRaises(Exception):
                link.connect("127.0.0.1", robot.port, timeout=1.0)
        finally:
            live_speed.SETUP_OUTPUTS = original
        self.assertFalse(link.connected)

    def test_refused_when_someone_else_owns_the_slider(self):
        robot = FakeRobot(types="IN_USE,IN_USE")
        robot.start()
        link = SpeedLink()
        with self.assertRaisesRegex(RuntimeError, "RoboDK's robot driver"):
            link.connect("127.0.0.1", robot.port)
        self.assertFalse(link.connected)
        with self.assertRaisesRegex(RuntimeError, "Not connected"):
            link.set(0.5)


if __name__ == "__main__":
    unittest.main()
