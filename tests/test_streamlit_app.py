from dataclasses import asdict, is_dataclass
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from streamlit.testing.v1 import AppTest
from drawbot import ui_bridge as bridge

UPLOAD_A = b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 20"><path d="M0 0 L40 20" fill="none" stroke="orange"/></svg>'
UPLOAD_B = b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 30"><path d="M0 0 L20 30 M0 30 L20 0" fill="none" stroke="purple"/></svg>'


class StreamlitTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        original_submit = bridge.submit_drawing
        self.submit_patch = patch.object(bridge, 'submit_drawing', side_effect=lambda *a, **k: original_submit(*a, project_root=Path(self.temp.name), **k))
        self.submit_patch.start()
        self.addCleanup(self.submit_patch.stop)
        self.status_patch = patch.object(bridge.robot_api, 'robodk_status', return_value='DrawBot')
        self.status_patch.start()
        self.addCleanup(self.status_patch.stop)
        self.size_patch = patch.object(bridge.robot_api, 'station_board_size', return_value=None)
        self.size_patch.start()
        self.addCleanup(self.size_patch.stop)
        self.api_patch = patch.object(bridge.robot_api, 'draw_in_robodk', side_effect=self.fake_api)
        self.api_patch.start()
        self.addCleanup(self.api_patch.stop)
        self.app = AppTest.from_file(ROOT / "app/streamlit_app.py", default_timeout=60).run()
        self.assert_no_exception()

    def svg_uploader(self):
        return next(u for u in self.app.file_uploader if u.key == "svg_upload")

    def fake_api(self, drawing, **kwargs):
        self.assertTrue(is_dataclass(drawing))
        data = bridge.drawing_dict(drawing)
        self.calls.append((data, kwargs, drawing))
        n = len(data['strokes']); p = sum(len(s['points']) for s in data['strokes'])
        previous = None; requests = []
        for i, s in enumerate(data['strokes'], 1):
            if s['marker'] != previous:
                requests.append({'stroke': i, 'marker': s['marker']})
                previous = s['marker']
        return {'ok': True, 'success': True, 'message': 'offline boundary mock: checked',
                'program_name': kwargs['program_name'], 'stroke_count': n, 'drawing_point_count': p,
                'target_count': p+2*n, 'motion_count': p+2*n, 'simulation_time_s': 42.123,
                'valid_ratio': 1.0, 'tcp_distance_mm': 500.123, 'errors': [], 'warnings': [],
                'exported_files': [], 'report_path': 'mock-boundary-report',
                'report': {'input_sha256': hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest(),
                           'pen_change_count': len(requests), 'pen_requests': requests}}

    def assert_no_exception(self):
        self.assertEqual(len(self.app.exception), 0, str(self.app.exception))

    def button(self, label):
        return next(b for b in self.app.button if b.label == label)

    def select(self, label):
        return next(b for b in self.app.selectbox if b.label == label)

    def simulate(self):
        self.button('Simulate in RoboDK').click().run()
        self.assert_no_exception()
        return self.calls[-1][0]

    def mode(self, name):
        self.app.session_state["mode"] = name
        self.app.run()
        self.assert_no_exception()

    def test_baseline_and_real_status_entry(self):
        self.assertTrue(any('DrawBot' in m.value for m in self.app.markdown))
        self.mode("Robot")
        self.assertEqual(self.app.number_input(key='approach').value, 80)
        self.assertEqual(self.app.number_input(key='v_draw').value, 150)
        self.assertEqual(self.app.number_input(key='v_lift').value, 150)
        self.assertEqual(self.app.number_input(key='v_joint').value, 120)
        self.assertEqual(self.app.slider(key='live_speed').value, 20)
        self.assertFalse(self.calls)

    def test_builtin_circle_uses_current_full_precision_dataclass(self):
        data = self.simulate()
        self.assertEqual(len(data['strokes']), 1)
        self.assertEqual(len(data['strokes'][0]['points']), 65)
        self.assertEqual(data['placement']['width'], 100)
        self.assertTrue(any(abs(x-round(x, 2)) > 1e-5 for x, y in data['strokes'][0]['points']))
        self.assertIn('RoboDK estimated runtime (s)', self.app.json[0].value)
        self.assertFalse(self.app.session_state['job_result']['animation_played'])

    def test_single_marker_draws_everything_with_it_and_can_export(self):
        self.select('or load a sample').select('Color test, 8 colors').run()
        self.app.button(key='single_orange').click().run()
        data = self.simulate()
        self.assertEqual({s['marker'] for s in data['strokes']}, {'orange'})
        self.assertEqual(self.calls[-1][1]['pen_change_mode'], 'manual_preloaded')
        self.assertEqual(self.calls[-1][1]['preloaded_marker'], 'orange')
        self.assertFalse(self.button('Generate robot program').disabled)

    def test_robot_panel_needs_a_program_and_a_safety_tick(self):
        from drawbot import robot_link
        self.app.session_state["mode"] = "Robot"
        self.app.run()
        self.assertTrue(any("Generate the robot program in Drawing mode first" in c.value for c in self.app.caption))
        self.assertFalse(any(b.label == "Send to robot" for b in self.app.button))
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / "DrawBot_Drawing.script"
            script.write_text("def p():\nend\np()")
            from drawbot import robot_panel
            link = robot_panel.LINK
            with patch.object(robot_link, "program_script", return_value=script), \
                 patch.object(robot_link.WATCH, "start") as send, patch.object(robot_link.WATCH, "errors", return_value=[]), \
                 patch("time.sleep"), \
                 patch.object(link, "connect") as connect, patch.object(link, "set") as set_speed, \
                 patch.object(robot_link, "program_running", side_effect=[False, True] * 10):
                self.app.run()
                self.assertTrue(self.button("Send to robot").disabled)
                self.assertFalse(self.app.checkbox(key="speed_manual").value)
                self.app.checkbox(key="speed_manual").check().run()
                self.app.checkbox(key="robot_safe").check().run()
                self.button("Send to robot").click().run()
                self.assert_no_exception()
                connect.assert_not_called()
                send.assert_called_once_with("192.168.0.100", "def p():\nend\np()", name="DrawBot_Drawing.script", port=30002)
                self.assertTrue(any("pendant slider" in m.value for m in self.app.success))
                self.app.checkbox(key="speed_manual").uncheck().run()
                self.app.checkbox(key="robot_safe").check().run()
                self.button("Send to robot").click().run()
                self.assertEqual(send.call_count, 2)
                connect.assert_called_once_with("192.168.0.100")
                set_speed.assert_called_with(20 / 150)
                self.app.number_input(key="robot_port").set_value(30003).run()
                self.app.text_input(key="robot_ip").set_value("10.0.0.7").run()
                self.app.checkbox(key="robot_safe").check().run()
                self.button("Send to robot").click().run()
                send.assert_called_with("10.0.0.7", "def p():\nend\np()", name="DrawBot_Drawing.script", port=30003)
                self.assertFalse(self.app.session_state["robot_safe"])
                self.assertTrue(any("is running now" in m.value for m in self.app.success))
                connect.side_effect = OSError("no route")
                self.app.checkbox(key="robot_safe").check().run()
                self.button("Send to robot").click().run()
                self.assertEqual(send.call_count, 3)
                self.assertTrue(any("Not sent" in m.value for m in self.app.error))

    def test_station_mode_sets_slots_speed_and_held_pen(self):
        self.mode("Station")
        self.assertEqual([self.app.selectbox(key=f"slot_{i}").value for i in range(1, 5)], ["green", "black", "purple", "orange"])
        self.app.selectbox(key="slot_1").set_value("orange").run()
        self.app.selectbox(key="slot_4").set_value("green").run()
        self.app.slider(key="pick_speed").set_value(40).run()
        self.app.selectbox(key="stand_held").set_value("none").run()
        self.assert_no_exception()
        from drawbot.robot_panel import station_settings
        state = self.app.session_state
        settings = station_settings({k: state[k] for k in ("slot_1", "slot_2", "slot_3", "slot_4", "pick_speed",
                                                           "pick_joint_speed", "stand_held", "grip_force")})
        self.assertEqual(settings["slots"], {"orange": 0, "black": 1, "purple": 2, "green": 3})
        self.assertEqual((settings["speed_mm_s"], settings["held_marker"]), (40.0, None))
        self.app.selectbox(key="slot_2").set_value("green").run()
        self.assertTrue(any("different colour" in w.value for w in self.app.warning))

    def test_nothing_is_sent_while_a_program_runs_on_the_robot(self):
        from drawbot import robot_link
        self.mode("Robot")
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / "DrawBot_Drawing.script"
            script.write_text("def p():\nend\np()")
            with patch.object(robot_link, "program_script", return_value=script), \
                 patch.object(robot_link.WATCH, "start") as send, \
                 patch.object(robot_link, "program_running", return_value=True):
                self.app.run()
                self.app.checkbox(key="robot_safe").check().run()
                self.button("Send to robot").click().run()
                send.assert_not_called()
                self.assertTrue(any("RoboDK's robot driver" in m.value for m in self.app.error))

    def test_robot_errors_after_sending_are_shown(self):
        from drawbot import robot_link
        self.mode("Robot")
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / "DrawBot_Drawing.script"
            script.write_text("def p():\nend\np()")
            with patch.object(robot_link, "program_script", return_value=script), \
                 patch.object(robot_link.WATCH, "start"), patch("time.sleep"), \
                 patch.object(robot_link.WATCH, "errors", return_value=["runtime_error_on_line:87:foo"]), \
                 patch.object(robot_link, "program_running", return_value=False):
                self.app.run()
                self.app.checkbox(key="speed_manual").check().run()
                self.app.checkbox(key="robot_safe").check().run()
                self.button("Send to robot").click().run()
                self.assertTrue(any("runtime_error_on_line:87" in m.value for m in self.app.error))

    def test_values_survive_a_mode_switch(self):
        self.app.number_input(key="spacing").set_value(3.0).run()
        self.app.session_state["mode"] = "Robot"
        self.app.run()
        self.app.text_input(key="robot_ip").set_value("10.0.0.8").run()
        self.app.session_state["mode"] = "Board"
        self.app.run()
        self.app.number_input(key="fit_tl").set_value(2).run()
        self.app.session_state["mode"] = "Drawing"
        self.app.run()
        self.assert_no_exception()
        self.assertEqual(self.app.number_input(key="spacing").value, 3.0)
        self.app.session_state["mode"] = "Robot"
        self.app.run()
        self.assertEqual(self.app.text_input(key="robot_ip").value, "10.0.0.8")
        self.app.session_state["mode"] = "Board"
        self.app.run()
        self.assertEqual(self.app.number_input(key="fit_tl").value, 2)

    def test_load_setup_fills_the_panel_and_updates_robodk(self):
        from drawbot import robot_link
        setup = {"format": "drawbot-setup", "version": 1, "robot": {"ip": "192.168.0.100", "robot_port": 30003},
                 "pen": {"length_mm": 262.0, "grip_force": 180}, "board": {"pen_offset_mm": 1.0},
                 "measurement": [{"label": "origin", "tcp": "tcp-pen", "joints": None, "pose": [0.3, 0.6, 0.1, 0, 0, 0], "tcp_offset": None},
                                 {"label": "whiteboard", "tcp": "tcp-pen", "joints": None, "pose": [0.3, 0.6, 0.1, 0, 0, 0], "tcp_offset": None}]}
        self.app.session_state["mode"] = "Robot"
        self.app.run()
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(robot_link, "store_measurement", return_value=Path(tmp) / "m.json") as store, \
             patch.object(robot_link, "apply_saved", return_value={"board_size_mm": [508.0, 307.0]}) as apply:
            uploader = next(u for u in self.app.file_uploader if u.key == "setup_upload")
            uploader.set_value(("my-setup.json", json.dumps(setup).encode(), "application/json")).run()
            self.assert_no_exception()
            self.assertEqual(self.app.session_state["robot_ip"], "192.168.0.100")
            self.assertEqual(self.app.session_state["robot_port"], 30003)
            self.assertEqual(self.app.session_state["pen_mm"], 262.0)
            self.assertEqual(self.app.session_state["press_mm"], 1.0)
            store.assert_called_once()
            apply.assert_called_once_with(Path(tmp) / "m.json", 1.0, fit={"tl": 0.0, "tr": 0.0, "c": 0.0, "bl": 0.0, "br": 0.0})
            self.assertTrue(any("Loaded my-setup.json" in m.value for m in self.app.success))

    def test_builtin_multicolor_uses_dynamic_markers(self):
        self.app.radio(key='marker_mode').set_value('Multiple markers').run()
        self.app.selectbox(key='job_export_mode').set_value('STUB test export — no physical pen changes').run()
        self.select('or load a sample').select('Color test, 8 colors').run()
        data = self.simulate()
        self.assertEqual(len(data['strokes']), 8)
        self.assertEqual({s['marker'] for s in data['strokes']}, {'black', 'orange', 'green', 'purple'})
        self.assertTrue(self.button('Generate robot program').disabled)

    def test_upload_widget_accepts_two_different_user_svgs(self):
        captures = []
        for name, content in [('my-first.svg', UPLOAD_A), ('my-second.svg', UPLOAD_B)]:
            self.svg_uploader().set_value((name, content, 'image/svg+xml')).run()
            data = self.simulate()
            captures.append(data)
            result = self.app.session_state['job_result']
            evidence = json.loads(Path(result['ui_request_path']).read_text())
            self.assertEqual(evidence['svg_sha256'], hashlib.sha256(content).hexdigest())
            self.assertEqual(evidence['source_name'], name)
            self.assertEqual(Path(result['ui_request_path']).with_name('input.svg').read_bytes(), content)
        self.assertNotEqual(captures[0]['strokes'], captures[1]['strokes'])
        self.assertNotEqual(sum(len(s['points']) for s in captures[1]['strokes']), 339)

    def test_width_xy_and_motion_changes_reach_job_without_extra_transform(self):
        self.svg_uploader().set_value(('line.svg', UPLOAD_A, 'image/svg+xml')).run()
        first = self.simulate()
        self.app.number_input(key='in_w').set_value(60.0).run()
        self.app.number_input(key='in_x').set_value(20.0).run()
        self.app.number_input(key='in_y').set_value(30.0).run()
        self.mode("Robot")
        self.app.number_input(key='v_draw').set_value(25.0).run()
        self.mode("Drawing")
        second = self.simulate()
        self.assertEqual(second['placement'], {'x':20.0, 'y':30.0, 'width':60.0, 'height':30.0})
        self.assertEqual(second['strokes'][0]['points'][0], [20.0,30.0])
        self.assertEqual(second['strokes'][0]['points'][-1], [80.0,60.0])
        self.assertNotEqual(first['strokes'], second['strokes'])
        self.assertEqual(self.calls[-1][1]['draw_speed'], 25)

    def test_manual_marker_changes_keep_points_and_original_svg_color(self):
        self.svg_uploader().set_value(('line.svg', UPLOAD_A, 'image/svg+xml')).run()
        self.app.radio(key='marker_mode').set_value('Multiple markers').run()
        self.app.selectbox(key='job_export_mode').set_value('STUB test export — no physical pen changes').run()
        before = self.simulate()
        self.app.toggle(key='keep_colors').set_value(False).run()
        self.app.button(key='all_green').click().run()
        after = self.simulate()
        self.assertEqual({s['marker'] for s in after['strokes']}, {'green'})
        self.assertEqual(before['strokes'][0]['points'], after['strokes'][0]['points'])
        self.assertEqual(before['strokes'][0]['svg_color'], after['strokes'][0]['svg_color'])

    def test_changed_inputs_clear_previous_result_and_same_input_repeat_is_new_request(self):
        self.simulate()
        previous = self.app.session_state['job_result']['ui_request_path']
        self.simulate()
        self.assertNotEqual(previous, self.app.session_state['job_result']['ui_request_path'])
        self.app.number_input(key='in_x').set_value(30.0).run()
        with self.assertRaises(KeyError):
            self.app.session_state['job_result']
        self.assertFalse(any(b.label.startswith('Download UI_') for b in self.app.get('download_button')))

    def test_stub_export_requires_visible_explicit_confirmation(self):
        self.app.radio(key='marker_mode').set_value('Multiple markers').run()
        self.app.selectbox(key='job_export_mode').set_value('STUB test export — no physical pen changes').run()
        self.select('or load a sample').select('Color test, 8 colors').run()
        self.select('Export pen mode').select('STUB test export — no physical pen changes').run()
        self.assertTrue(self.button('Generate robot program').disabled)
        self.app.checkbox(key='job_stub_ack').check().run()
        self.assertFalse(self.button('Generate robot program').disabled)
        self.button('Generate robot program').click().run()
        self.assert_no_exception()
        kwargs = self.calls[-1][1]
        self.assertEqual(kwargs['mode'], 'export')
        self.assertTrue(kwargs['allow_stub_export_for_testing'])
        self.assertTrue(any('STUB TEST EXPORT' in w.value for w in self.app.warning))

    def test_invalid_uploaded_svg_shows_failure_without_job_call(self):
        self.svg_uploader().set_value(('broken.svg', b'<svg broken', 'image/svg+xml')).run()
        self.assert_no_exception()
        self.assertGreater(len(self.app.error), 0)
        self.assertFalse(self.calls)

    def test_api_failure_displays_errors_and_diagnostic_path(self):
        failure = {'ok':False, 'success':False, 'message':'RoboDK offline API unavailable',
                   'errors':['RoboDK offline API unavailable'], 'warnings':[],
                   'report':{}, 'report_path':'diagnostic-path.json', 'exported_files':[]}
        with patch.object(bridge.robot_api, 'draw_in_robodk', return_value=failure):
            self.button('Simulate in RoboDK').click().run()
        self.assert_no_exception()
        self.assertTrue(any('unavailable' in error.value for error in self.app.error))
        self.assertTrue(any('diagnostic-path.json' in caption.value for caption in self.app.caption))
        self.assertFalse(self.app.session_state['job_result']['ok'])


    def test_fill_toggle_shows_settings_and_adds_hatches(self):
        self.app.file_uploader[0].set_value(('square.svg', b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10"><rect width="10" height="10" fill="blue"/></svg>', 'image/svg+xml')).run()
        self.assertFalse(any(n.key == 'fill_pen' for n in self.app.number_input))
        self.assertTrue(any('not hatched' in w.value for w in self.app.warning))
        outlines = self.simulate()['strokes']
        self.app.toggle(key='fill_on').set_value(True).run()
        self.assertEqual(self.app.number_input(key='fill_spacing').value, 3.2)
        self.app.number_input(key='fill_spacing').set_value(5.0).run()
        hatched = self.simulate()['strokes']
        self.assertGreater(len(hatched), len(outlines))
        self.assertFalse(any('not hatched' in w.value for w in self.app.warning))
        self.assertTrue(self.app.success)
        self.app.toggle(key='fill_on').set_value(False).run()
        self.app.toggle(key='fill_on').set_value(True).run()
        self.assertEqual(self.app.number_input(key='fill_spacing').value, 5.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
