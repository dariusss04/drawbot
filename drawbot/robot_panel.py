import json
import time

from . import board_fit, robot_link
from .live_speed import LINK

COLORS = ("black", "green", "purple", "orange")
DOT = {"black": "⚫", "green": "🟢", "purple": "🟣", "orange": "🟠"}
NO_PEN = "none"

DEFAULTS = (("robot_ip", robot_link.DEFAULT_ROBOT_IP), ("robot_port", robot_link.SCRIPT_PORT),
            ("laptop_port", robot_link.RECEIVE_PORT), ("pen_mm", 280.0), ("nopen_mm", 155.0), ("grip_force", 150),
            ("go_home", True), ("manual_grip", False), ("press_mm", -20.0),
            ("robot_safe", False), ("touch_safe", False), ("robot_msg", None), ("job_seen", "idle"),
            ("last_report", None), ("live_speed", 20), ("speed_manual", False), ("fit_applied", None),
            ("slot_1", "green"), ("slot_2", "black"), ("slot_3", "purple"), ("slot_4", "orange"),
            ("pick_speed", 50.0), ("pick_joint_speed", 30.0), ("stand_held", "black"),
            *((key, 0) for key in robot_link.FIT_KEYS.values()))
KEEP = ("robot_ip", "robot_port", "laptop_port", "pen_mm", "nopen_mm", "grip_force", "go_home",
        "manual_grip", "press_mm", "robot_safe", "touch_safe", "live_speed", "home_joints", "speed_manual",
        *robot_link.SLOT_KEYS, "pick_speed", "pick_joint_speed", "stand_held",
        *robot_link.FIT_KEYS.values())


def defaults(ss):
    for key, value in DEFAULTS:
        ss.setdefault(key, value)
    if "home_joints" not in ss:
        ss["home_joints"] = ", ".join(f"{v:g}" for v in robot_link.UPRIGHT_DEG)


def _message(st):
    ss = st.session_state
    if ss.robot_msg:
        kind, text = ss.robot_msg
        (st.success if kind == "success" else st.error)(text)


def _set_speed_on_robot(ss):
    """Put the robot's speed slider at the chosen speed. The program holds the top speed, so this is the real one."""
    if not LINK.connected or LINK.ip != ss.robot_ip:
        LINK.connect(ss.robot_ip)
    return LINK.set(_speed_fraction(ss))


def _send(ss, script_path, safe_key):
    ss[safe_key] = False
    if robot_link.program_running(ss.robot_ip):
        ss.robot_msg = ("error", "Not sent. " + robot_link.RUNNING_HINT)
        return
    speed = f"at the pendant slider's speed (set it to {_speed_fraction(ss):.0%} for {ss.live_speed} mm/s)"
    if ss.speed_manual:
        LINK.close()
    else:
        try:
            _set_speed_on_robot(ss)
            speed = f"at {ss.live_speed} mm/s"
        except Exception as exc:
            ss.robot_msg = ("error", f"Not sent: could not set the speed to {ss.live_speed} mm/s ({exc}). "
                                     "Check the connection, or tick 'I set the pendant slider myself' above the Send button.")
            return
    try:
        watch = robot_link.WATCH
        watch.start(ss.robot_ip, script_path.read_text(encoding="utf-8"), name=script_path.name, port=int(ss.robot_port))
        time.sleep(3.0)
        errors = watch.errors()
        if errors:
            ss.robot_msg = ("error", f"The robot stopped {script_path.name}: " + " | ".join(errors))
        elif robot_link.program_running(ss.robot_ip) is False:
            ss.robot_msg = ("error", f"Sent {script_path.name}, but no program is running on the robot after 3 s. "
                                     "See Robot messages below.")
        else:
            ss.robot_msg = ("success", f"Sent {script_path.name}. It is running now, {speed}.")
    except Exception as exc:
        ss.robot_msg = ("error", f"Could not send: {exc}")


def _check(ss):
    where = f"{ss.robot_ip}:{int(ss.robot_port)}"
    ok = robot_link.reachable(ss.robot_ip, int(ss.robot_port))
    ss.robot_msg = ("success", f"Robot at {where} answers.") if ok else (
        "error", f"No answer from {where}. Check the cable or Wi-Fi, the IP and port, and Remote Control on the pendant.")


def _measure(ss, parts="board"):
    if robot_link.program_running(ss.robot_ip):
        ss.robot_msg = ("error", "Not started. " + robot_link.RUNNING_HINT)
        return
    try:
        home = robot_link.parse_joints(ss.home_joints) if ss.go_home else None
        robot_link.JOB.start(ss.robot_ip, ss.pen_mm, not ss.manual_grip, ss.press_mm,
                             port=int(ss.laptop_port), script_port=int(ss.robot_port), grip_force=int(ss.grip_force),
                             home_deg=home, nopen_mm=ss.nopen_mm, parts=parts)
        ss.robot_msg = None
        ss.job_seen = "running"
    except Exception as exc:
        ss.robot_msg = ("error", f"Could not start: {exc}")


def _apply(ss):
    """Pen offset and board corrections into RoboDK, from the last measurement. The robot does not move."""
    path = robot_link.latest_measurement()
    fit = robot_link.fit_values(ss)
    try:
        report = robot_link.apply_saved(path, ss.press_mm, fit=fit)
        ss.last_report = report
        ss.fit_applied = {"press_mm": ss.press_mm, "fit": fit}
        ss.robot_msg = ("success", "Applied in RoboDK. Make a new touch test, or generate the drawing again.")
    except Exception as exc:
        ss.robot_msg = ("error", f"Could not apply: {exc}")


def _reset_fit(ss):
    for key in robot_link.FIT_KEYS.values():
        ss[key] = 0


def _show_stand(ss):
    from .pen_stand import show_stand, stand_measurement
    from .robot.connection import connect_existing
    try:
        rdk = connect_existing()
        try:
            x, y, z, rz = show_stand(rdk, stand_measurement(), slots=robot_link.slot_colors(ss))
        finally:
            rdk.Disconnect()
        ss.robot_msg = ("success", f"Pen stand placed in RoboDK at x {x:.0f}, y {y:.0f}, z {z:.0f} mm, "
                                   f"turned {rz:.0f} deg. Check it against the real stand.")
    except Exception as exc:
        ss.robot_msg = ("error", f"Could not place the pen stand: {exc}")


def _make_touch_test(ss, settings, size):
    from . import ui_bridge
    w, h = size
    result = ui_bridge.submit_drawing(board_fit.touch_test_strokes(w, h), "export", settings,
                                      svg_bytes=b"", source_name="touch test", program_name="DrawBot_TouchTest",
                                      export_mode="manual_preloaded", preloaded_marker="black")
    ss.touch_result = result
    ss.robot_msg = ("success", "Touch test ready.") if result["ok"] else (
        "error", f"Touch test failed: {result.get('message')}")


def _speed_fraction(ss):
    """Live speed in mm/s as a fraction of the program's drawing speed: what the pendant slider needs."""
    top = float(ss.get("v_draw") or 150)
    return min(1.0, ss.live_speed / top)


def _speed_changed(ss):
    """Moving the slider while connected changes the speed at once, also in the middle of a drawing."""
    if not LINK.connected:
        return
    try:
        LINK.set(_speed_fraction(ss))
    except Exception as exc:
        ss.robot_msg = ("error", f"Speed not changed, connection lost: {exc}")


def _load_setup(ss):
    upload = ss.get("setup_upload")
    if upload is None:
        return
    try:
        data = json.loads(upload.getvalue())
        values, records = robot_link.read_setup(data)
        stand = robot_link.read_stand(data)
    except (ValueError, json.JSONDecodeError) as exc:
        ss.robot_msg = ("error", f"Could not read {upload.name}: {exc}")
        return
    from .ui_bridge import EXPORT_MODES
    if values.get("job_export_mode") not in (None, *EXPORT_MODES):
        del values["job_export_mode"]
    for key, value in values.items():
        ss[key] = value
    if stand:
        robot_link.store_measurement(stand, tag="stand-loaded")
    if not records:
        ss.robot_msg = ("success", f"Loaded the settings from {upload.name}. It has no board measurement.")
        return
    try:
        path = robot_link.store_measurement(records)
        fit = robot_link.fit_values(ss)
        ss.last_report = robot_link.apply_saved(path, ss.press_mm, fit=fit)
        ss.fit_applied = {"press_mm": ss.press_mm, "fit": fit}
        ss.robot_msg = ("success", f"Loaded {upload.name}: settings filled in, board and pen TCP set in RoboDK "
                                   f"with a pen offset of {ss.press_mm:+.0f} mm. Simulate and generate again.")
    except Exception as exc:
        ss.robot_msg = ("error", f"Settings loaded, but RoboDK was not updated: {exc}. Open the station and load again.")


def _report_lines(report):
    out = []
    if not report:
        return out
    if "board_size_mm" in report:
        out.append("Board: {:.0f} × {:.0f} mm".format(*report["board_size_mm"]))
    if "pen_tcp_mm" in report:
        out.append(f"Pen TCP: {report['pen_tcp_mm'][2]:.0f} mm")
    if report.get("no_pen_tcp_mm"):
        out.append(f"No-pen TCP: {report['no_pen_tcp_mm'][2]:.0f} mm")
    if "stand_gaps_mm" in report:
        out.append("Pen stand saved. Slot spacing: " + ", ".join(f"{g:.0f}" for g in report["stand_gaps_mm"]) + " mm (50 expected)")
    if "arm_shape_coverage" in report:
        c = report["arm_shape_coverage"]
        out.append(f"Arm shape reaches {c['best']} of {c['points']} board test points")
    return out


def _board_settings(st):
    """Sidebar of Board mode: board size, then the settings Measure board uses (set once, rarely changed)."""
    ss = st.session_state
    st.subheader("Board")
    c1, c2, c3 = st.columns(3)
    c1.number_input("Width (mm)", 50.0, 2000.0, step=10.0, key="board_w")
    c2.number_input("Height (mm)", 50.0, 2000.0, step=10.0, key="board_h")
    c3.number_input("Margin (mm)", 0.0, 50.0, step=1.0, key="margin", help="Drawing closer to the edge warns.")
    if ss.get("measured_board"):
        st.caption("Measured: {:.0f} × {:.0f} mm".format(*ss.measured_board))
    st.subheader("Measuring")
    c1, c2 = st.columns(2)
    c1.number_input("Pen TCP (mm)", 100.0, 450.0, step=1.0, key="pen_mm",
                    help="Flange to marker tip, with the pen in the gripper.")
    c2.number_input("No-pen TCP (mm)", 50.0, 400.0, step=1.0, key="nopen_mm",
                    help="Flange to the point between the open fingers. Used for the pen stand points.")
    st.toggle("I open and close the gripper myself", key="manual_grip")
    st.toggle("Slowly to home before the corners", key="go_home")
    st.text_input("Home joints (deg)", key="home_joints", disabled=not ss.go_home,
                  help="Base, shoulder, elbow, wrist 1, 2, 3. Used before and after measuring, and at the end of "
                       "every drawing. The arm goes to exactly these, the base included. Default: straight up.")


def _measure_card(st):
    ss = st.session_state
    job = robot_link.JOB
    with st.container(border=True):
        st.markdown("**1 · Measure**")
        st.caption("Hold a pen ready. Follow the pendant. The pen stand is measured in Station mode.")
        if job.running:
            st.button("Cancel", on_click=job.cancel, use_container_width=True)
        st.button("Measure board", on_click=_measure, args=(ss, "board"), disabled=job.running, type="primary",
                  use_container_width=True,
                  help="Grip a pen, move home, then touch top-left, top-right, bottom-right with the pen tip.")
        _job_status(st, "board")


def _job_status(st, parts):
    """Live progress of a measurement, shown in the mode that started it."""
    ss = st.session_state
    job = robot_link.JOB
    if job.state != "idle" and job.parts != parts:
        return

    @st.fragment(run_every=1.0 if job.running else None)
    def live():
        if job.state not in ("idle", "done"):
            st.caption(f"{job.state.capitalize()}: {job.records} of {job.expected} points")
            if job.lines:
                st.caption(job.lines[-1])
        if job.state == "done":
            st.caption(" · ".join(_report_lines(job.report)))
            for warning in (job.report or {}).get("warnings", []):
                st.warning(warning)
        if job.state == "error":
            st.error(job.error)
        if ss.job_seen == "running" and not job.running:
            ss.job_seen = job.state
            if job.state == "done" and job.parts != "stand":
                ss.fit_applied = {"press_mm": ss.press_mm, "fit": None}
            st.rerun()
    live()


def _touch_card(st, settings, size):
    ss = st.session_state
    with st.container(border=True):
        st.markdown("**2 · Touch test**")
        st.caption("Draws a small cross at each corner and the centre.")
        st.button("Make touch test", on_click=_make_touch_test, args=(ss, settings, size), use_container_width=True,
                  disabled=size is None)
        script = robot_link.program_script(ss.get("touch_result"))
        if script is not None:
            st.checkbox("Area clear, hand on the emergency stop", key="touch_safe")
            st.checkbox("I set the pendant slider myself", key="speed_manual",
                        help="Do not set the speed over RTDE. The program then runs at its top speed "
                             "times the pendant slider.")
            st.button("Send touch test", on_click=_send, args=(ss, script, "touch_safe"), type="primary",
                      disabled=not ss.touch_safe, use_container_width=True)


def _correct_card(st, size):
    ss = st.session_state
    keys = robot_link.FIT_KEYS
    with st.container(border=True):
        st.markdown("**3 · Correct**")
        st.caption("Per cross, in mm: **+** deeper (faint or missing), **−** less deep (pressed too hard).")
        with st.container(border=True):
            for row in (("tl", None, "tr"), (None, "c", None), ("bl", None, "br")):
                for col, point in zip(st.columns(3), row):
                    if point:
                        col.number_input(board_fit.LABELS[point], -board_fit.INPUT_RANGE_MM, board_fit.INPUT_RANGE_MM,
                                         step=1, key=keys[point])
        fit = robot_link.fit_values(ss)
        if size and not board_fit.is_zero(fit):
            result = board_fit.fit(fit, *size)
            st.caption("→ " + board_fit.tilt_text(result, *size))
            worst = max(board_fit.POINTS, key=lambda k: abs(result["leftover"][k]))
            if abs(result["leftover"][worst]) >= 0.25:
                st.caption(f"Not flat: {board_fit.LABELS[worst].lower()} stays {result['leftover'][worst]:+.1f} mm off.")
        st.number_input("Pen offset (mm)", -50.0, 5.0, step=0.5, key="press_mm",
                        help="On top of the corrections. -20 = dry run in the air, 0 to 1 = drawing.")
        c1, c2 = st.columns(2)
        c1.button("Apply", on_click=_apply, args=(ss,), type="primary", use_container_width=True,
                  disabled=robot_link.latest_measurement() is None or robot_link.JOB.running)
        c2.button("Reset", on_click=_reset_fit, args=(ss,), use_container_width=True)
        applied = ss.fit_applied
        if applied:
            now = applied["fit"] or {}
            same = applied["press_mm"] == ss.press_mm and all(
                abs(now.get(k, 0.0) - fit[k]) < 1e-9 for k in board_fit.POINTS)
            shown = "no corrections" if board_fit.is_zero(now) else "corrections on"
            st.caption(f"In RoboDK: offset {applied['press_mm']:+g} mm, {shown}"
                       + ("" if same else " · **not applied yet**"))


def board_page(st, settings):
    ss = st.session_state
    defaults(ss)
    with st.sidebar:
        _board_settings(st)
    _message(st)
    size = ss.get("measured_board") or (ss.get("board_w"), ss.get("board_h"))
    size = tuple(size) if all(size) else None
    left, right = st.columns([1, 1.3], gap="medium")
    with left:
        _measure_card(st)
        _touch_card(st, settings, size)
    with right:
        _correct_card(st, size)


def station_settings(ss):
    """The pen stand settings for a program, from the session (see ui_bridge)."""
    defaults(ss)
    held = ss.get("stand_held", "black")
    return {"slots": robot_link.slot_colors(ss), "held_marker": None if held == NO_PEN else held,
            "grip_force": int(ss.get("grip_force", 150)), "speed_mm_s": float(ss.get("pick_speed", 50)),
            "joint_speed_deg_s": float(ss.get("pick_joint_speed", 30))}


def station_page(st):
    from .pen_stand import check_pickup_points, check_slots, stand_measurement
    ss = st.session_state
    defaults(ss)
    with st.sidebar:
        st.subheader("Gripper")
        st.slider("Grip force", 0, 255, step=5, key="grip_force", help="Hand-E force on the pen holder.")
        st.selectbox("Pen in the gripper now", [NO_PEN, *COLORS], key="stand_held",
                     format_func=lambda c: "none" if c == NO_PEN else f"{DOT[c]} {c}",
                     help="After Measure board the corner pen is still gripped. A drawing puts it back first if needed.")
    _message(st)
    with st.container(border=True):
        st.markdown("**Slots**")
        for i, (col, key) in enumerate(zip(st.columns(4), robot_link.SLOT_KEYS), 1):
            col.selectbox(f"Slot {i}", COLORS, key=key, format_func=lambda c: f"{DOT[c]} {c}")
        try:
            check_slots(robot_link.slot_colors(ss) or {})
            st.caption("Slot 1 to 4 = points 1 to 4 of Measure pen stand.")
        except ValueError as exc:
            st.warning(str(exc))
    left, right = st.columns(2, gap="medium")
    with left, st.container(border=True):
        st.markdown("**Pick and place**")
        st.slider("Straight moves (mm/s)", 10, 150, step=5, key="pick_speed")
        st.slider("Joint moves (deg/s)", 5, 90, step=5, key="pick_joint_speed")
        st.caption("Top speeds. The speed in Robot mode scales them like the drawing.")
    with right, st.container(border=True):
        st.markdown("**Pen stand**")
        job = robot_link.JOB
        if job.running and job.parts == "stand":
            st.button("Cancel", on_click=job.cancel, use_container_width=True, key="cancel_stand")
        st.button("Measure pen stand", on_click=_measure, args=(ss, "stand"), disabled=job.running, type="primary",
                  use_container_width=True,
                  help="All pens in their slots. Points 1 to 4: open gripper around the pen in slot 1 to 4, at the "
                       "height it should grip. The board stays as it is.")
        _job_status(st, "stand")
        measurement = stand_measurement()
        try:
            check_pickup_points(measurement)
            source = ("the fixed calibration (calibration/pen_stand)" if measurement.parent.name == "pen_stand"
                      else f"measurement {measurement.parent.name}")
            st.caption(f"Stand from {source}. Measure again only when the stand has moved.")
        except ValueError as exc:
            st.caption(str(exc))
        st.button("Show pen stand in RoboDK", on_click=_show_stand, args=(ss,), use_container_width=True,
                  disabled=measurement is None or robot_link.JOB.running)


def _connection_settings(st):
    """Sidebar of Robot mode."""
    ss = st.session_state
    st.subheader("Connection")
    st.text_input("Robot IP", key="robot_ip")
    c1, c2 = st.columns(2)
    c1.number_input("Robot port", 1, 65535, step=1, key="robot_port", help="Scripts go here. UR: 30002.")
    c2.number_input("Laptop port", 1024, 65535, step=1, key="laptop_port",
                    help="Measured points come back here. If RoboDK is connected to the robot, use 30010.")
    st.button("Check connection", on_click=_check, args=(ss,), use_container_width=True)
    st.subheader("Motion")
    st.caption("Top speeds in the program. The robot runs slower, at the speed on the right.")
    c1, c2 = st.columns(2)
    c1.number_input("Drawing (mm/s)", 5.0, 150.0, step=5.0, key="v_draw")
    c2.number_input("Pen up/down (mm/s)", 5.0, 150.0, step=5.0, key="v_lift")
    c1.number_input("Joints (deg/s)", 5.0, 180.0, step=5.0, key="v_joint", help="Moves between strokes.")
    c2.number_input("Approach (mm)", 5.0, 200.0, step=5.0, key="approach", help="Pen height between strokes.")
    c1.number_input("Blend (mm)", 0.0, 5.0, step=1.0, key="blend",
                    help="0 = stop at every point. 1 to 5 = the pen rounds gentle bends without stopping. "
                         "Whole millimetres: the robot script cannot hold finer blends.")
    st.toggle("Finish at the home pose", key="park_upright",
              help="After the drawing the arm moves to the home joints set in Board mode (Measuring).")
    st.subheader("Setup file")
    values = {k: ss[k] for k in robot_link.SETUP_FIELDS if k in ss}
    try:
        from .pen_stand import stand_measurement
        setup = robot_link.build_setup(values, robot_link.latest_measurement(), stand_measurement())
        st.download_button("Save setup", json.dumps(setup, indent=2), use_container_width=True,
                           file_name=time.strftime("drawbot-setup-%Y%m%d-%H%M.json"), mime="application/json",
                           help="Every setting of all four modes, with the board measurement, in one file.")
    except Exception as exc:
        st.caption(f"Cannot save the setup: {exc}")
    st.file_uploader("Load setup", type=["json"], key="setup_upload", on_change=_load_setup, args=(ss,),
                     help="Restores every setting and puts the saved board into RoboDK.")


def robot_page(st, result):
    ss = st.session_state
    defaults(ss)
    with st.sidebar:
        _connection_settings(st)
    _message(st)
    top = int(ss.get("v_draw") or 150)
    ss.live_speed = min(ss.live_speed, top)
    left, right = st.columns(2, gap="medium")
    with left, st.container(border=True):
        st.markdown("**Speed**")
        st.slider("Drawing speed (mm/s)", 5, top, step=5, key="live_speed", on_change=_speed_changed, args=(ss,),
                  help="Set on the robot when you send. While the robot runs, moving it changes the speed at once. "
                       "Pen up/down and travel scale with it.")
        if ss.speed_manual:
            st.caption(f"Set the **pendant speed slider to {_speed_fraction(ss):.0%}** for {ss.live_speed} mm/s. "
                       "(Live speed from the laptop is off.)")
        elif LINK.connected:
            st.caption(f"Live: connected to {LINK.ip}, robot at {round((LINK.fraction or 0) * top)} mm/s")
        else:
            st.caption("Default 20 mm/s. Connects when you send.")
    with right, st.container(border=True):
        st.markdown("**Draw**")
        script = robot_link.program_script(result)
        if script is None:
            st.caption("Generate the robot program in Drawing mode first.")
        else:
            st.caption(f"{script.parent.name}/{script.name}")
            st.checkbox("Area clear, hand on the emergency stop", key="robot_safe")
            st.checkbox("I set the pendant slider myself", key="speed_manual",
                        help="Do not set the speed over RTDE. The program then runs at its top speed "
                             "times the pendant slider.")
            st.button("Send to robot", type="primary", on_click=_send, args=(ss, script, "robot_safe"),
                      disabled=not ss.robot_safe, use_container_width=True, help="It starts at once.")

    _robot_messages(st)


def _robot_messages(st):
    """Everything the robot reported on the connection of the last program sent: errors, popups, stops."""
    watch = robot_link.WATCH
    if not watch.name:
        return

    @st.fragment(run_every=2.0 if watch.running else None)
    def live():
        with st.container(border=True):
            st.markdown(f"**Robot messages** · {watch.name}" + (" · listening" if watch.running else ""))
            with watch.lock:
                messages = list(watch.messages)
            if not messages:
                st.caption("None so far.")
            for stamp, kind, text in messages[-12:]:
                (st.error if kind in (6, 10) else st.caption)(f"{stamp}  {text}")
    live()
