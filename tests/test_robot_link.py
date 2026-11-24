import socket
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
from drawbot import robot_link
from test_calibration import sample_lines


def free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    return port


class TeachScriptTests(unittest.TestCase):
    def test_session_values_go_into_the_script(self):
        text = robot_link.teach_script(262, False, "10.0.0.5", 31000)
        self.assertIn('HOST = "10.0.0.5"', text)
        self.assertIn("PORT = 31000", text)
        self.assertIn("TCP_PEN   = p[0, 0, 0.2620, 0, 0, 0]", text)
        self.assertIn("global USE_GRIPPER = False", text)
        self.assertIn("TCP_NOPEN = p[0, 0, 0.1550, 0, 0, 0]", text)
        self.assertIn("TCP_NOPEN = p[0, 0, 0.1420, 0, 0, 0]", robot_link.teach_script(262, True, "10.0.0.5", nopen_mm=142))
        self.assertIn('grip_cmd("SET FOR 150")', text)
        self.assertIn('grip_cmd("SET FOR 255")', robot_link.teach_script(262, True, "10.0.0.5", grip_force=300))
        self.assertIn("global GO_HOME = False", text)
        homed = robot_link.teach_script(262, True, "10.0.0.5", home_deg=[90, -90, 0, 0, 0, 180])
        self.assertIn("global GO_HOME = True", homed)
        self.assertIn("global HOME_Q = [1.570796, -1.570796, 0.000000, 0.000000, 0.000000, 3.141593]", homed)
        self.assertLess(homed.index("Pen gripped firmly"), homed.index("movej(HOME_Q"))
        self.assertLess(homed.index("movej(HOME_Q"), homed.index('record("origin"'))
        back_off = homed.index("p[0, 0, -0.05, 0, 0, 0]")
        self.assertLess(homed.index('record("yaxis"'), back_off)
        self.assertLess(back_off, homed.rindex("movej(HOME_Q"))
        self.assertNotIn("HOME_KEEP_BASE", homed)
        self.assertIn("global HOME_Q = [0.000000, -1.570796, 0.000000, -1.570796, 0.000000, 0.000000]",
                      robot_link.teach_script(262, True, "10.0.0.5", home_deg=robot_link.UPRIGHT_DEG))

    def test_home_joints_are_checked(self):
        self.assertEqual(robot_link.parse_joints("221.4, -65.9 102.1,-158.6, -78.5, -31.3"), [221.4, -65.9, 102.1, -158.6, -78.5, -31.3])
        for bad in ("1, 2, 3", "1, 2, 3, 4, 5, 999", "a, b, c, d, e, f"):
            with self.assertRaises(ValueError):
                robot_link.parse_joints(bad)

    def test_a_changed_script_is_refused(self):
        with tempfile.NamedTemporaryFile("w", suffix=".script", delete=False) as f:
            f.write("def x():\n  HOST = \"1.2.3.4\"\nend\n")
        with self.assertRaisesRegex(RuntimeError, "could not set"):
            robot_link.teach_script(275, True, "10.0.0.5", source=f.name)

    def test_laptop_address_towards_a_robot(self):
        self.assertEqual(robot_link.laptop_ip_towards("127.0.0.1"), "127.0.0.1")


class LinkTests(unittest.TestCase):
    def test_reachable_and_send(self):
        port = free_port()
        self.assertFalse(robot_link.reachable("127.0.0.1", port, 0.5))
        server = socket.socket(); server.bind(("127.0.0.1", port)); server.listen(1)
        got = {}
        def accept():
            conn, _ = server.accept(); got["text"] = conn.recv(65536).decode(); conn.close()
        t = threading.Thread(target=accept); t.start()
        robot_link.send_script("127.0.0.1", "def p():\nend\np()", port=port)
        t.join(5); server.close()
        self.assertEqual(got["text"], "def p():\nend\np()\n")

    def test_program_script_from_a_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / "DrawBot_Drawing.script"; script.write_text("x")
            self.assertEqual(robot_link.program_script({"exported_files": [str(Path(tmp) / "a.urp"), str(script)]}), script)
        self.assertIsNone(robot_link.program_script(None))
        self.assertIsNone(robot_link.program_script({"exported_files": ["/missing/x.script"]}))


class MeasurementTests(unittest.TestCase):
    def test_full_measurement_with_a_fake_robot(self):
        port, sent = free_port(), {}

        def fake_robot(script):
            sent["script"] = script
            def run():
                time.sleep(0.1)
                with socket.create_connection(("127.0.0.1", port), timeout=5) as s:
                    for line in sample_lines():
                        s.sendall((line + "\n").encode()); time.sleep(0.01)
            threading.Thread(target=run, daemon=True).start()

        job = robot_link.Measurement()
        with tempfile.TemporaryDirectory() as tmp:
            job.start("127.0.0.1", 275, True, -20, port=port, root=tmp, timeout_s=10, script_port=30003,
                      apply=lambda records: {"board_size_mm": [300.0, 250.0], "records": len(records)}, send=fake_robot)
            job.thread.join(15)
            self.assertEqual(job.state, "done", job.error)
            self.assertEqual(job.records, 8)
            self.assertEqual(job.report["board_size_mm"], [300.0, 250.0])
            self.assertTrue((Path(job.folder) / "ur5e_data.json").is_file())
            self.assertEqual(robot_link.latest_measurement(tmp), Path(job.folder) / "ur5e_data.json")
        self.assertIn('HOST = "127.0.0.1"', sent["script"])
        self.assertIn(f"PORT = {port}", sent["script"])

    def test_cancel_stops_a_waiting_measurement(self):
        job = robot_link.Measurement()
        with tempfile.TemporaryDirectory() as tmp:
            job.start("127.0.0.1", 275, True, -20, port=free_port(), root=tmp, timeout_s=60,
                      apply=lambda r: {}, send=lambda script: None)
            time.sleep(0.3)
            self.assertTrue(job.running)
            job.cancel()
        self.assertFalse(job.running)
        self.assertEqual(job.state, "error")
        self.assertIn("Cancelled", job.error)

    def test_send_failure_is_reported(self):
        def broken(script):
            raise ConnectionRefusedError("robot refused")
        job = robot_link.Measurement()
        with tempfile.TemporaryDirectory() as tmp:
            job.start("127.0.0.1", 275, True, -20, port=free_port(), root=tmp, timeout_s=5, apply=lambda r: {}, send=broken)
            job.thread.join(10)
        self.assertEqual(job.state, "error")
        self.assertIn("refused", job.error)
        self.assertFalse(job.running)


if __name__ == "__main__":
    unittest.main()


class PortCheckTests(unittest.TestCase):
    def test_a_port_held_by_another_listener_is_refused(self):
        holder = socket.socket()
        holder.bind(("127.0.0.1", 0))
        holder.listen(1)
        port = holder.getsockname()[1]
        try:
            self.assertTrue(robot_link.port_taken("127.0.0.1", port))
            with self.assertRaisesRegex(RuntimeError, "already used by another program"):
                robot_link.Measurement().start("127.0.0.1", 275, True, -20, port=port, send=lambda text: None)
        finally:
            holder.close()
        self.assertFalse(robot_link.port_taken("127.0.0.1", port))


class DashboardTests(unittest.TestCase):
    def fake_dashboard(self, answer):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)

        def serve():
            conn, _ = server.accept()
            with conn:
                conn.sendall(b"Connected: Universal Robots Dashboard Server\n")
                if conn.recv(100).startswith(b"running"):
                    conn.sendall(answer)
            server.close()
        threading.Thread(target=serve, daemon=True).start()
        return server.getsockname()[1]

    def test_running_program_is_seen(self):
        self.assertTrue(robot_link.program_running("127.0.0.1", self.fake_dashboard(b"Program running: true\n")))
        self.assertFalse(robot_link.program_running("127.0.0.1", self.fake_dashboard(b"Program running: false\n")))

    def test_no_dashboard_is_unknown(self):
        probe = socket.socket(); probe.bind(("127.0.0.1", 0)); port = probe.getsockname()[1]; probe.close()
        self.assertIsNone(robot_link.program_running("127.0.0.1", port, timeout=0.5))


class SendListenTests(unittest.TestCase):
    def test_robot_messages_on_the_sending_connection_are_returned(self):
        import struct
        server = socket.socket(); server.bind(("127.0.0.1", 0)); server.listen(1)
        port = server.getsockname()[1]

        def message(kind, text):
            body = struct.pack(">QbB", 0, -2, kind) + text.encode()
            return struct.pack(">iB", 5 + len(body), 20) + body

        def serve():
            conn, _ = server.accept()
            with conn:
                conn.recv(1000)
                conn.sendall(message(3, "URControl") + message(10, "syntax_error_on_line:3:end:"))
                time.sleep(0.3)
            server.close()
        threading.Thread(target=serve, daemon=True).start()
        messages = robot_link.send_script("127.0.0.1", "def p():\nend\n", port=port, listen_s=1.0)
        self.assertEqual(messages, [(10, "syntax_error_on_line:3:end:")])


class RunWatchTests(unittest.TestCase):
    def test_messages_arriving_later_are_collected(self):
        import struct
        server = socket.socket(); server.bind(("127.0.0.1", 0)); server.listen(1)
        port = server.getsockname()[1]

        def message(kind, text):
            body = struct.pack(">QbB", 0, -2, kind) + text.encode()
            return struct.pack(">iB", 5 + len(body), 20) + body

        def serve():
            conn, _ = server.accept()
            with conn:
                conn.recv(1000)
                time.sleep(1.5)
                conn.sendall(message(10, "runtime_error_on_line:200:halt"))
                time.sleep(0.5)
            server.close()
        threading.Thread(target=serve, daemon=True).start()
        watch = robot_link.RunWatch()
        watch.start("127.0.0.1", "def p():\nend\n", name="p.script", port=port, max_s=5)
        watch.thread.join(5)
        self.assertEqual(watch.errors(), ["runtime_error_on_line:200:halt"])


class SetupFileTests(unittest.TestCase):
    VALUES = {"robot_ip": "192.168.0.100", "robot_port": 30002, "laptop_port": 30000, "pen_mm": 262.0, "nopen_mm": 142.0,
              "grip_force": 180, "manual_grip": False, "go_home": True, "home_joints": "0, -90, 0, -90, 0, 0",
              "press_mm": -20.0}

    def test_save_and_load_round_trip(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ur5e_data.json"
            from test_calibration import sample_lines
            from drawbot import calibration
            path.write_text(json.dumps(calibration.parse_text("\n".join(sample_lines()))))
            setup = json.loads(json.dumps(robot_link.build_setup(self.VALUES, path)))
        self.assertEqual(setup["summary"]["board_size_mm"], [300.0, 250.0])
        self.assertEqual(setup["summary"]["pen_tcp_mm"], [0.0, 0.0, 275.0])
        self.assertEqual(setup["summary"]["no_pen_tcp_mm"], [0.0, 0.0, 155.0])
        values, records = robot_link.read_setup(setup)
        self.assertEqual(values, self.VALUES)
        self.assertEqual(len(records), 8)

    def test_settings_only_and_bad_files(self):
        values, records = robot_link.read_setup(robot_link.build_setup(self.VALUES))
        self.assertIsNone(records)
        self.assertEqual(values["robot_ip"], "192.168.0.100")
        for bad in ({}, {"format": "other"}, {"format": "drawbot-setup", "measurement": [{"label": "1"}]},
                    {"format": "drawbot-setup", "home": {"joints_deg": "1, 2"}}):
            with self.assertRaises(ValueError):
                robot_link.read_setup(bad)

    def test_loaded_measurement_becomes_the_latest(self):
        import json
        lab = json.loads((ROOT / "tests/fixtures/pen_stand_ur5e_data.json").read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path = robot_link.store_measurement(lab, root=tmp)
            self.assertEqual(robot_link.latest_measurement(tmp), path)


class SplitMeasurementTests(unittest.TestCase):
    def test_setup_file_keeps_board_and_stand(self):
        import json
        lab = json.loads((ROOT / "tests/fixtures/pen_stand_ur5e_data.json").read_text())
        with tempfile.TemporaryDirectory() as tmp:
            board = Path(tmp) / "board.json"; board.write_text(json.dumps([r for r in lab if r["label"] not in "1234"]))
            stand = Path(tmp) / "stand.json"; stand.write_text(json.dumps([r for r in lab if r["label"] in "1234"]))
            setup = json.loads(json.dumps(robot_link.build_setup({}, board, stand)))
        _, records = robot_link.read_setup(setup)
        self.assertEqual({r["label"] for r in records}, {"origin", "xaxis", "yaxis", "whiteboard"})
        self.assertEqual([r["label"] for r in robot_link.read_stand(setup)], ["1", "2", "3", "4"])
        self.assertIsNone(robot_link.read_stand(robot_link.build_setup({})))

    def test_teach_script_runs_only_the_chosen_part(self):
        board = robot_link.teach_script(280, True, "10.0.0.5", parts="board")
        stand = robot_link.teach_script(280, True, "10.0.0.5", parts="stand")
        both = robot_link.teach_script(280, True, "10.0.0.5")
        self.assertIn("global DO_STAND = False", board); self.assertIn("global DO_BOARD = True", board)
        self.assertIn("global DO_STAND = True", stand); self.assertIn("global DO_BOARD = False", stand)
        self.assertIn("global DO_STAND = True", both); self.assertIn("global DO_BOARD = True", both)
        self.assertEqual((robot_link.PARTS["board"], robot_link.PARTS["stand"], robot_link.PARTS["both"]), (4, 4, 8))

    def test_a_stand_measurement_does_not_replace_the_board(self):
        import json, os, time as _t
        lab = json.loads((ROOT / "tests/fixtures/pen_stand_ur5e_data.json").read_text())
        board_only = [r for r in lab if r["label"] not in ("1", "2", "3", "4")]
        stand_only = [r for r in lab if r["label"] in ("1", "2", "3", "4")]
        with tempfile.TemporaryDirectory() as tmp:
            for name, records in (("20261112T100000Z-board", board_only), ("20261112T110000Z-stand", stand_only)):
                folder = Path(tmp) / "logs/calibration" / name
                folder.mkdir(parents=True)
                (folder / "ur5e_data.json").write_text(json.dumps(records))
            self.assertEqual(robot_link.latest_measurement(tmp).parent.name, "20261112T100000Z-board")
            from drawbot import pen_stand
            self.assertEqual(pen_stand.stand_measurement(tmp).parent.name, "20261112T110000Z-stand")
