import json
import math
import re
import socket
import struct
import threading
import time
from pathlib import Path

from . import calibration

ROOT = Path(__file__).resolve().parents[1]
TEACH_SCRIPT = ROOT / "calibration/teach_whiteboard.script"
DEFAULT_ROBOT_IP = "192.168.0.100"
UPRIGHT_DEG = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]
SCRIPT_PORT = 30002
RECEIVE_PORT = 30010


def reachable(ip, port=SCRIPT_PORT, timeout=2.0):
    try:
        socket.create_connection((ip, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def laptop_ip_towards(ip, port=SCRIPT_PORT):
    """The address of this computer on the network that leads to the robot."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((ip, port))
        return probe.getsockname()[0]
    finally:
        probe.close()


DASHBOARD_PORT = 29999


def program_running(ip, port=DASHBOARD_PORT, timeout=3.0):
    """Ask the robot's dashboard server whether a program runs. True, False, or None if it does not answer.

    A running program is usually RoboDK's robot driver (RoboDK connected to the robot). It restarts
    itself when another script replaces it, so a script sent from here would be stopped before it moves.
    """
    try:
        with socket.create_connection((ip, port), timeout=timeout) as s:
            f = s.makefile("rwb")
            f.readline()
            f.write(b"running\n")
            f.flush()
            answer = f.readline().decode(errors="replace").strip().lower()
    except OSError:
        return None
    return answer.endswith("true")


RUNNING_HINT = ("A program is already running on the robot, usually RoboDK's robot driver. In RoboDK open "
                "Connect to robot and click Disconnect, or stop the program on the pendant. Then try again.")


def port_taken(laptop_ip, port):
    """True if another program already listens on this laptop address and port.

    RoboDK's robot driver binds the robot-facing address directly. A receiver listening on all
    addresses would still start, but the robot's data would go to the driver and be lost.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind((laptop_ip, int(port)))
        return False
    except OSError:
        return True
    finally:
        probe.close()


def teach_script(pen_mm, auto_gripper, laptop_ip, port=RECEIVE_PORT, source=TEACH_SCRIPT, grip_force=150,
                 home_deg=None, nopen_mm=155.0, parts="both"):
    """The teach script with this session's pen length, gripper mode and laptop address.
    home_deg: the home pose before and after the corners, all six joints as given (base included).
    parts: "board" (3 corners), "stand" (pen stand points 1 to 4) or "both"."""
    if parts not in PARTS:
        raise ValueError(f"parts must be one of {', '.join(PARTS)}")
    text = Path(source).read_text(encoding="utf-8")
    replacements = [
        (r"global DO_STAND = (True|False)", f"global DO_STAND = {'True' if parts in ('stand', 'both') else 'False'}"),
        (r"global DO_BOARD = (True|False)", f"global DO_BOARD = {'True' if parts in ('board', 'both') else 'False'}"),
        (r'HOST = "[^"]*"', f'HOST = "{laptop_ip}"'),
        (r"PORT = \d+", f"PORT = {int(port)}"),
        (r"TCP_PEN   = p\[[^\]]*\]", f"TCP_PEN   = p[0, 0, {pen_mm / 1000:.4f}, 0, 0, 0]"),
        (r"TCP_NOPEN = p\[[^\]]*\]", f"TCP_NOPEN = p[0, 0, {nopen_mm / 1000:.4f}, 0, 0, 0]"),
        (r"global USE_GRIPPER = (True|False)", f"global USE_GRIPPER = {'True' if auto_gripper else 'False'}"),
        (r'grip_cmd\("SET FOR \d+"\)', f'grip_cmd("SET FOR {int(max(0, min(255, grip_force)))}")'),
        (r"global GO_HOME = (True|False)", f"global GO_HOME = {'True' if home_deg else 'False'}"),
        (r"global HOME_Q = \[[^\]]*\]",
         "global HOME_Q = [" + ", ".join(f"{math.radians(v):.6f}" for v in (home_deg or UPRIGHT_DEG)) + "]"),
    ]
    if home_deg is not None and len(home_deg) != 6:
        raise ValueError("Home pose needs 6 joint angles")
    for pattern, value in replacements:
        text, n = re.subn(pattern, value, text, count=1)
        if n != 1:
            raise RuntimeError(f"Teach script changed: could not set {value.split('=')[0].strip()}")
    return text


ROBOT_MESSAGE = 20
QUIET_MESSAGES = (3, 12, 17)


def _robot_messages(buf):
    """Split complete packages off buf. Returns ([(type, text)], rest of buf)."""
    out = []
    while len(buf) >= 5:
        size, kind = struct.unpack(">iB", buf[:5])
        if size < 5 or len(buf) < size:
            break
        package, buf = buf[:size], buf[size:]
        if kind == ROBOT_MESSAGE and len(package) > 15 and package[14] not in QUIET_MESSAGES:
            out.append((package[14], "".join(chr(c) for c in package[15:] if 32 <= c < 127).strip()))
    return out, buf


class RunWatch:
    """Keeps the connection a program was sent on and collects the robot's messages while it runs:
    errors, popups and stops arrive there, even minutes after sending."""

    def __init__(self):
        self.lock = threading.Lock()
        self.messages, self.thread, self.sock, self.name = [], None, None, None

    def start(self, ip, text, name="", port=SCRIPT_PORT, timeout=5.0, max_s=3600):
        self.stop()
        self.messages, self.name = [], name
        self.sock = socket.create_connection((ip, port), timeout=timeout)
        self.sock.sendall(text.encode("utf-8") + b"\n")
        self.thread = threading.Thread(target=self._read, args=(self.sock, max_s), daemon=True)
        self.thread.start()

    def _read(self, sock, max_s):
        end, buf = time.time() + max_s, b""
        sock.settimeout(1.0)
        while time.time() < end:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            found, buf = _robot_messages(buf + chunk)
            with self.lock:
                self.messages += [(time.strftime("%H:%M:%S"), kind, text) for kind, text in found]

    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def errors(self):
        with self.lock:
            return [text for _, kind, text in self.messages if kind in (6, 10)]

    def stop(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None


WATCH = RunWatch()


def send_script(ip, text, timeout=5.0, port=SCRIPT_PORT, listen_s=0.0):
    """Send a URScript program. With listen_s, keep the connection open that long and return the robot's
    messages on it, as [(type, text)]. The controller reports syntax and runtime errors there
    (type 10, for example "syntax_error_on_line:3"), and nowhere else it can be read."""
    messages = []
    with socket.create_connection((ip, port), timeout=timeout) as s:
        s.sendall(text.encode("utf-8") + b"\n")
        if listen_s <= 0:
            return messages
        s.settimeout(0.5)
        end, buf = time.time() + listen_s, b""
        while time.time() < end:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            while len(buf) >= 5:
                size, kind = struct.unpack(">iB", buf[:5])
                if size < 5 or len(buf) < size:
                    break
                package, buf = buf[:size], buf[size:]
                if kind == ROBOT_MESSAGE and len(package) > 15 and package[14] not in QUIET_MESSAGES:
                    messages.append((package[14], "".join(chr(c) for c in package[15:] if 32 <= c < 127).strip()))
    return messages


PARTS = {"board": len(calibration.BOARD_POINTS) + 1, "stand": len(calibration.PICKUP_POINTS),
         "both": calibration.EXPECTED}


def has_board(path):
    """True if a measurement file has the 3 board corners (or the robot's plane)."""
    try:
        records = json.loads(Path(path).read_text(encoding="utf-8"))
        return calibration.board_frame_from_records({r["label"]: r for r in records}) is not None
    except (OSError, ValueError, KeyError, TypeError):
        return False


def latest_measurement(root=ROOT):
    """Newest measurement with the board corners, from this computer, else saved in the repo.
    A pen stand measurement alone does not count: the board stays from the last board measurement."""
    for folder in ("logs/calibration", "calibration/runs"):
        for path in sorted((Path(root) / folder).glob("*/ur5e_data.json"), reverse=True):
            if has_board(path):
                return path
    return None


def _settings(root=ROOT):
    return json.loads((Path(root) / "config/drawing.json").read_text(encoding="utf-8"))


def apply_saved(path, press_mm, root=ROOT, fit=None):
    """Re-apply a saved measurement, for example to switch between dry run and drawing,
    or with board corrections from a touch test (fit, see board_fit)."""
    from .robot.connection import connect_existing
    records = json.loads(Path(path).read_text(encoding="utf-8"))
    rdk = connect_existing()
    try:
        return calibration.apply_to_station(rdk, records, _settings(root), press_mm, log=lambda m: None, fit=fit)
    finally:
        rdk.Disconnect()


def drawing_home_joints(root=ROOT):
    """Start pose of the drawing programs: the station's calibrated seed, else the config seed."""
    try:
        from .robot.connection import connect_existing
        rdk = connect_existing()
        try:
            seed = calibration.station_home_seed(rdk)
        finally:
            rdk.Disconnect()
        if seed:
            return [round(v, 1) for v in seed]
    except Exception:
        pass
    return [round(v, 1) for v in _settings(root)["home"]["seed_joints_deg"]]


def parse_joints(text):
    values = [float(v) for v in re.split(r"[,\s]+", text.strip()) if v]
    if len(values) != 6 or any(abs(v) > 360 for v in values):
        raise ValueError("Enter 6 joint angles in degrees, each between -360 and 360")
    return values


SETUP_FORMAT = "drawbot-setup"
SETUP_FIELDS = {
    "robot_ip": ("robot", "ip", str), "robot_port": ("robot", "robot_port", int),
    "laptop_port": ("robot", "laptop_port", int), "pen_mm": ("pen", "length_mm", float),
    "nopen_mm": ("pen", "no_pen_tcp_mm", float),
    "grip_force": ("pen", "grip_force", int), "manual_grip": ("pen", "manual_grip", bool),
    "go_home": ("home", "enabled", bool), "home_joints": ("home", "joints_deg", str),
    "press_mm": ("board", "pen_offset_mm", float),
    "fit_tl": ("board_fit", "top_left_mm", int), "fit_tr": ("board_fit", "top_right_mm", int),
    "fit_c": ("board_fit", "centre_mm", int), "fit_bl": ("board_fit", "bottom_left_mm", int),
    "fit_br": ("board_fit", "bottom_right_mm", int),
    "slot_1": ("station", "slot_1", str), "slot_2": ("station", "slot_2", str),
    "slot_3": ("station", "slot_3", str), "slot_4": ("station", "slot_4", str),
    "pick_speed": ("station", "pick_speed_mm_s", float), "pick_joint_speed": ("station", "pick_joint_speed_deg_s", float),
    "stand_held": ("station", "pen_in_gripper", str),
    "v_draw": ("motion", "draw_speed_mm_s", float), "v_lift": ("motion", "pen_up_down_speed_mm_s", float),
    "v_joint": ("motion", "joint_speed_deg_s", float), "approach": ("motion", "approach_mm", float),
    "blend": ("motion", "blend_mm", float), "park_upright": ("motion", "finish_upright", bool),
    "live_speed": ("motion", "robot_speed_mm_s", int), "speed_manual": ("motion", "pendant_slider_by_hand", bool),
    "board_w": ("drawing", "board_width_mm", float), "board_h": ("drawing", "board_height_mm", float),
    "margin": ("drawing", "margin_mm", float), "spacing": ("drawing", "curve_sampling_mm", float),
    "optimize": ("drawing", "optimize_order", bool), "marker_mode": ("drawing", "marker_mode", str),
    "single_marker": ("drawing", "single_marker", str), "keep_colors": ("drawing", "keep_svg_colors", bool),
    "job_export_mode": ("drawing", "pen_mode", str),
    "fill_on": ("fill", "enabled", bool), "fill_pen": ("fill", "pen_width_mm", float),
    "fill_spacing": ("fill", "hatch_spacing_mm", float), "fill_min_seg": ("fill", "min_segment_mm", float),
    "fill_min_spacing": ("fill", "min_spacing_mm", float), "fill_mode": ("fill", "mode", str),
}
SLOT_KEYS = ("slot_1", "slot_2", "slot_3", "slot_4")
_MARKERS = ("black", "green", "purple", "orange")
CHOICES = {"marker_mode": ("Single marker", "Multiple markers"), "single_marker": _MARKERS, "fill_mode": ("linear", "contour"),
           "stand_held": ("none", *_MARKERS), **{key: _MARKERS for key in SLOT_KEYS}}


def slot_colors(values):
    """{color: slot 0 to 3} from panel values slot_1 ... slot_4, or None if not set."""
    if not all(values.get(key) for key in SLOT_KEYS):
        return None
    return {values[key]: i for i, key in enumerate(SLOT_KEYS)}
FIT_KEYS = {"tl": "fit_tl", "tr": "fit_tr", "c": "fit_c", "bl": "fit_bl", "br": "fit_br"}


def fit_values(values):
    """Board corrections {point: mm} from panel values (session keys fit_tl ...)."""
    return {point: float(values.get(key) or 0.0) for point, key in FIT_KEYS.items()}


def build_setup(values, measurement_path=None, stand_path=None):
    """One JSON file with the board measurement, the pen stand measurement and the panel settings."""
    setup = {"format": SETUP_FORMAT, "version": 1, "saved_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    if stand_path:
        setup["stand_measurement"] = json.loads(Path(stand_path).read_text(encoding="utf-8"))
    for key, (section, name, kind) in SETUP_FIELDS.items():
        if key in values:
            setup.setdefault(section, {})[name] = kind(values[key])
    records = json.loads(Path(measurement_path).read_text(encoding="utf-8")) if measurement_path else None
    setup["measurement"] = records
    if records:
        by_label = {r["label"]: r for r in records}
        pose = calibration.board_frame_from_records(by_label)
        summary = {"board_pose_base_mm": pose}
        if all(k in by_label for k in calibration.BOARD_POINTS):
            w, h, _ = calibration.board_size(*(by_label[k] for k in calibration.BOARD_POINTS))
            summary["board_size_mm"] = [round(w, 2), round(h, 2)]
        for name, key in (("tcp-pen", "pen_tcp_mm"), ("tcp-nopen", "no_pen_tcp_mm")):
            offset = next((r["tcp_offset"] for r in records if r["tcp"] == name and r["tcp_offset"]), None)
            if offset:
                summary[key] = [round(v * 1000, 2) for v in offset[:3]]
        setup["summary"] = summary
    return setup


def read_setup(data):
    """Check an uploaded setup. Returns (panel values, measurement records or None)."""
    if not isinstance(data, dict) or data.get("format") != SETUP_FORMAT:
        raise ValueError("This is not a DrawBot setup file")
    values = {}
    for key, (section, name, kind) in SETUP_FIELDS.items():
        if name in data.get(section, {}):
            value = data[section][name]
            values[key] = value if kind is str else kind(value)
    if "home_joints" in values:
        parse_joints(values["home_joints"])
    for key, allowed in CHOICES.items():
        if key in values and values[key] not in allowed:
            del values[key]
    records = data.get("measurement")
    if records is not None:
        if not isinstance(records, list) or not all(isinstance(r, dict) and "label" in r for r in records):
            raise ValueError("The measurement in this setup is damaged")
        if calibration.board_frame_from_records({r["label"]: r for r in records}) is None:
            raise ValueError("The measurement in this setup has no board corners")
    return values, records


def read_stand(data):
    """The pen stand records of a setup file, or None. Older files keep the stand in "measurement"."""
    records = data.get("stand_measurement") if isinstance(data, dict) else None
    if records is None:
        return None
    if not isinstance(records, list) or not all(isinstance(r, dict) and "label" in r for r in records):
        raise ValueError("The pen stand measurement in this setup is damaged")
    return records


def store_measurement(records, root=ROOT, tag="loaded"):
    """Save uploaded records like a fresh measurement, so 'Apply offset' uses them."""
    folder = Path(root) / "logs" / "calibration" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + tag)
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / "ur5e_data.json"
    path.write_text(json.dumps(records, indent=2), encoding="utf-8")
    return path


def program_script(result):
    """The .script file of the last generated program, or None."""
    for f in (result or {}).get("exported_files", []):
        if str(f).endswith(".script") and Path(f).is_file():
            return Path(f)
    return None


class Measurement:
    """One board measurement, running in the background so the web page stays responsive."""

    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.reset()

    def reset(self):
        self.state, self.lines, self.records, self.report, self.error, self.folder = "idle", [], 0, None, None, None
        self.parts, self.expected = "both", calibration.EXPECTED
        self.cancelled = False
        self.started = None

    def cancel(self):
        """Stop waiting for the robot. Connecting to its own port wakes the listener, which then ends."""
        if not self.running:
            return
        self.cancelled = True
        try:
            socket.create_connection(("127.0.0.1", self.port), timeout=1).close()
        except OSError:
            pass
        self.thread.join(3)

    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def log(self, text):
        with self.lock:
            self.lines.append(time.strftime("%H:%M:%S ") + text)

    def start(self, robot_ip, pen_mm, auto_gripper, press_mm, port=RECEIVE_PORT, root=ROOT,
              timeout_s=900, apply=None, send=None, script_port=SCRIPT_PORT, grip_force=150, home_deg=None,
              nopen_mm=155.0, parts="both"):
        if self.running:
            raise RuntimeError("A measurement is already running")
        laptop_ip = laptop_ip_towards(robot_ip, script_port)
        if port_taken(laptop_ip, port):
            raise RuntimeError(f"Laptop port {port} is already used by another program on {laptop_ip}, usually "
                               "RoboDK's robot driver. The robot's points would go there and be lost. Disconnect "
                               "RoboDK from the robot, or choose another Laptop port in Robot mode.")
        self.reset()
        self.state, self.started = "starting", time.time()
        self.parts, self.expected = parts, PARTS[parts]
        if apply is None and parts == "stand":
            apply = lambda records: self._check_stand()
        apply = apply or (lambda records: self._apply(records, press_mm, root))
        send = send or (lambda text: send_script(robot_ip, text, port=script_port))
        script = teach_script(pen_mm, auto_gripper, laptop_ip, port, grip_force=grip_force,
                              home_deg=home_deg, nopen_mm=nopen_mm, parts=parts)
        self.port = port
        self.thread = threading.Thread(target=self._run, args=(script, port, timeout_s, apply, send, root), daemon=True)
        self.thread.start()

    def _check_stand(self):
        from .pen_stand import check_pickup_points
        gaps = check_pickup_points(self.folder / "ur5e_data.json")
        return {"stand_gaps_mm": [round(g, 1) for g in gaps], "warnings": []}

    def _apply(self, records, press_mm, root):
        from .robot.connection import connect_existing
        rdk = connect_existing()
        try:
            return calibration.apply_to_station(rdk, records, _settings(root), press_mm, log=lambda m: None)
        finally:
            rdk.Disconnect()

    def _run(self, script, port, timeout_s, apply, send, root):
        def on_listen():
            self.log(f"Listening on port {port}. Sending the teach script to the robot.")
            send(script)
            self.state = "waiting"
            self.log("Follow the popups on the pendant.")

        def on_record(text):
            self.records += 1
            self.state = "receiving"
            self.log("Received " + text.split("] ", 1)[-1])

        try:
            raw, records = calibration.receive(port, expected=self.expected, timeout_s=timeout_s,
                                               on_record=on_record, on_listen=on_listen)
            if self.cancelled:
                raise RuntimeError("Cancelled. RoboDK was not changed.")
            self.folder = calibration.save_records(root, raw, records, tag=self.parts if self.parts != "both" else "")
            if not records:
                raise RuntimeError("The robot sent no usable data. RoboDK was not changed.")
            self.state = "applying"
            self.log(f"Got {len(records)} of {self.expected} records."
                     + (" Checking the pen stand." if self.parts == "stand" else " Updating RoboDK."))
            self.report = apply(records)
            self.state = "done"
            self.log("Done. The pen stand is saved." if self.parts == "stand" else "Done. The board is in RoboDK.")
        except socket.timeout:
            self.state, self.error = "error", "Timed out waiting for the robot."
        except Exception as exc:
            self.state, self.error = "error", str(exc)
        if self.error:
            self.log("Error: " + self.error)


JOB = Measurement()
