from __future__ import annotations

import hashlib
import json
import math
import sys
import time
from pathlib import Path

import streamlit as st
import streamlit.components.v1 as components

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drawbot import backend
from drawbot.svg_pipeline import FILL_OUTLINE_WARNING
from drawbot.strokes import MARKERS, Placement, Stroke, StrokeList
from drawbot.ui_bridge import BASELINE, robot_actions
from drawbot import robot_panel
from drawbot.robot.api import robodk_status, station_board_size

SAMPLE_DIR = ROOT / "svg"
SAMPLES = {
    "Circle, 100 mm": ("circle-100mm.svg", 100.0),
    "Filled circle, 100 mm": ("circle-100mm-filled.svg", 100.0),
    "Robot face, 3 colors": ("robot-face.svg", 140.0),
    "Color test, 8 colors": ("color-test.svg", 200.0),
}
MARKER_IDS = list(MARKERS)
DOT = {"black": "⚫", "green": "🟢", "purple": "🟣", "orange": "🟠"}
STROKE_LIST_LIMIT = 300
FILL_DEFAULTS = backend.FillSettings()

board_component = components.declare_component("drawbot_board", path=str(Path(__file__).parent / "board"))

st.set_page_config(page_title="DrawBot Studio", layout="wide")
st.markdown(
    """<style>
    .block-container { padding-top: 4rem; }   /* clears Streamlit's fixed header bar */
    .db-title { font-size: 2rem; font-weight: 700; margin: 0 0 .2rem 0; letter-spacing: .01em; color: inherit; line-height: 1.2; }
    .db-title .bot { color: #d12f26; }
    .status { display: inline-block; font-size: .85rem; color: #6a6f76; border: 1px solid #e3e5e8;
              border-radius: 999px; padding: 2px 10px; white-space: nowrap; }
    section[data-testid="stSidebar"] { width: 380px !important; }
    .st-key-mode button { min-width: 6rem; padding: .45rem .8rem; }      /* Drawing, Board, Station, Robot */
    .st-key-mode button p { font-size: 1.05rem; font-weight: 600; }
    </style>""",
    unsafe_allow_html=True,
)

ss = st.session_state
for k, v in {
    "board_w": 300.0, "board_h": 300.0, "margin": 10.0, "spacing": 2.0, "approach": BASELINE["approach_mm"],
    "v_draw": BASELINE["draw_speed"], "v_lift": BASELINE["lift_speed"], "v_joint": BASELINE["joint_speed"],
    "blend": BASELINE["blend_mm"], "park_upright": BASELINE["park_upright"],
    "optimize": True, "marker_mode": "Single marker", "single_marker": "black", "keep_colors": True, "view": "Artwork",
    "manual": {}, "log": [], "svg_key": None, "place": None, "aspect": 1.0, "last_nonce": None,
    "fill_on": False, "fill_pen": FILL_DEFAULTS.pen_width_mm, "fill_spacing": FILL_DEFAULTS.hatch_spacing_mm,
    "fill_min_seg": FILL_DEFAULTS.min_segment_mm, "fill_min_spacing": FILL_DEFAULTS.min_spacing_mm,
    "fill_mode": FILL_DEFAULTS.mode,
}.items():
    ss.setdefault(k, v)
for k in ("fill_pen", "fill_spacing", "fill_min_seg", "fill_min_spacing", "fill_mode"):
    ss[k] = ss[k]

KEEP = ("sample", "board_w", "board_h", "margin", "spacing", "approach", "v_draw", "v_lift", "v_joint", "blend",
        "park_upright", "optimize", "marker_mode", "keep_colors", "view", "job_export_mode", "job_preloaded",
        "job_stub_ack", "job_external_ack", "fill_on", *robot_panel.KEEP)
for k in [k for k in ss.keys() if k in KEEP or str(k).startswith("mk_")]:
    ss[k] = ss[k]


def log(msg: str) -> None:
    ss.log.insert(0, f"{time.strftime('%H:%M:%S')}  {msg}")


def clamp(p: dict) -> dict:
    W, H, ar = ss.board_w, ss.board_h, ss.aspect
    p["w"] = min(max(p["w"], 5.0), W, H / ar)
    p["x"] = min(max(p["x"], 0.0), W - p["w"])
    p["y"] = min(max(p["y"], 0.0), H - p["w"] * ar)
    return p


def centered(w: float) -> dict:
    return clamp({"x": (ss.board_w - w) / 2, "y": (ss.board_h - w * ss.aspect) / 2, "w": w})


def fit_width() -> float:
    m = ss.margin
    return min(ss.board_w - 2 * m, (ss.board_h - 2 * m) / ss.aspect)


def default_place(pref: float | None) -> dict:
    fw = fit_width()
    return centered(pref if pref and pref <= fw else fw * 0.6)


def do_center():
    ss.place = centered(ss.place["w"])


def do_fit():
    ss.place = centered(fit_width())


def do_reset():
    ss.place = default_place(ss.get("pref_width"))


def on_number(field: str):
    p, ar, v = ss.place, ss.aspect, ss["in_" + field]
    if v is None:
        return
    if field in ("x", "y"):
        p[field] = v
    else:
        w = v if field == "w" else v / ar
        cx, cy = p["x"] + p["w"] / 2, p["y"] + p["w"] * ar / 2
        p.update(w=w, x=cx - w / 2, y=cy - w * ar / 2)
    clamp(p)


@st.cache_resource(max_entries=8, show_spinner=False)
def parse(key: str, data: bytes):
    return backend.load_svg(data)


@st.cache_data(max_entries=64, show_spinner=False)
def sample(_drawing, key: str, x: float, y: float, w: float, spacing: float, optimize: bool,
           fill: tuple | None) -> list[Stroke]:
    h = w * _drawing.aspect
    return backend.svg_to_strokes(_drawing, Placement(x, y, w, h), spacing_mm=spacing, optimize=optimize,
                                  fill=backend.FillSettings(*fill) if fill else None)


MODES = ["Drawing", "Board", "Station", "Robot"]
if ss.get("mode") not in MODES:
    ss.mode = ss.get("mode_last", "Drawing")
ss.mode_last = ss.mode
left, mid, right = st.columns([1.6, 3, 1], vertical_alignment="center")
mid.segmented_control("Mode", MODES, key="mode", label_visibility="collapsed")
left.markdown('<div class="db-title">Draw<span class="bot">Bot</span> Studio</div>', unsafe_allow_html=True)
station = robodk_status()
measured = station_board_size() if station else None
if measured and ss.get("measured_board") != measured:
    ss.measured_board = measured
    ss.board_w, ss.board_h = round(measured[0], 1), round(measured[1], 1)
right.markdown(
    f'<div style="text-align:right"><span class="status">RoboDK: {station or "not connected"}</span></div>',
    unsafe_allow_html=True,
)


def motion_settings() -> dict:
    return {"approach_mm": ss.approach, "draw_speed": ss.v_draw, "lift_speed": ss.v_lift, "joint_speed": ss.v_joint,
            "blend_mm": ss.blend, "park_upright": ss.park_upright, "park_joints": home_pose()}


def home_pose():
    """The home joints from Board mode: the program ends there. None if the field is not valid."""
    from drawbot import robot_link
    try:
        return robot_link.parse_joints(ss.get("home_joints") or ", ".join(f"{v:g}" for v in robot_link.UPRIGHT_DEG))
    except ValueError:
        return None


if ss.mode == "Board":
    robot_panel.board_page(st, motion_settings())
    st.stop()
if ss.mode == "Station":
    robot_panel.station_page(st)
    st.stop()
if ss.mode == "Robot":
    robot_panel.robot_page(st, ss.get("job_result"))
    st.stop()


def on_svg_upload():
    """Keep the uploaded file in session state: the uploader is empty again after a mode switch."""
    up = ss.get("svg_upload")
    ss.svg_file = (up.name, up.getvalue()) if up is not None else None


def drop_svg_upload():
    ss.svg_file = None

with st.sidebar:
    st.subheader("1 · Drawing")
    st.file_uploader("Choose an SVG", type=["svg"], key="svg_upload", on_change=on_svg_upload)
    upload = ss.get("svg_file")
    if upload and ss.get("svg_upload") is None:
        c1, c2 = st.columns([3, 1], vertical_alignment="center")
        c1.caption(f"Using {upload[0]}")
        c2.button("Remove", on_click=drop_svg_upload, use_container_width=True)
    sample_label = st.selectbox("or load a sample", list(SAMPLES), key="sample", disabled=bool(upload))

if upload:
    name, data = upload
    pref = None
else:
    fname, pref = SAMPLES[sample_label]
    path = (SAMPLE_DIR / fname).resolve()
    data, name = path.read_bytes(), path.name

key = hashlib.sha1(data).hexdigest()
try:
    drawing = parse(key, data)
except backend.SvgError as e:
    st.sidebar.error(str(e))
    st.error(f"{name}: {e}")
    st.stop()

if ss.svg_key != key:
    ss.svg_key, ss.aspect, ss.pref_width, ss.manual = key, drawing.aspect, pref, {}
    for k in [k for k in ss.keys() if str(k).startswith("mk_")]:
        del ss[k]
    ss.place = default_place(pref)
    log(f"Loaded {name}")

moved = ss.get("board")
if moved and moved.get("nonce") != ss.last_nonce:
    ss.last_nonce = moved["nonce"]
    ss.place = clamp({"x": moved["x"], "y": moved["y"], "w": moved["width"]})
    log(f"Placed at x {ss.place['x']:.1f}, y {ss.place['y']:.1f}, width {ss.place['w']:.1f} mm")
ss.place = clamp(ss.place)
p = ss.place

fill = (ss.fill_pen, ss.fill_spacing, ss.fill_min_seg, ss.fill_min_spacing, ss.fill_mode) if ss.fill_on else None
fill_error = None
try:
    try:
        strokes = sample(drawing, key, p["x"], p["y"], p["w"], ss.spacing, ss.optimize, fill)
    except backend.SvgError as e:
        if fill is None:
            raise
        fill, fill_error = None, str(e)
        strokes = sample(drawing, key, p["x"], p["y"], p["w"], ss.spacing, ss.optimize, None)
except backend.SvgError as e:
    st.error(f"{name}: {e}")
    st.stop()


def single_mode() -> bool:
    return ss.marker_mode == "Single marker"


def marker_of(s: Stroke) -> str:
    if single_mode():
        return ss.single_marker
    return s.marker if ss.keep_colors else ss.manual.get(s.source, s.marker)


def set_single(m: str):
    ss.single_marker = m
    log(f"Single marker: {m}")


def set_all(m: str):
    for s in strokes:
        ss.manual[s.source] = m
        ss["mk_" + s.source] = m
    log(f"All strokes → {m}")


def set_one(src: str):
    ss.manual[src] = ss["mk_" + src]
    log(f"Stroke {src} → {ss.manual[src]}")


def on_keep():
    if not ss.keep_colors:
        for s in strokes:
            ss.manual.setdefault(s.source, s.marker)
    log("Colors: matched from the SVG" if ss.keep_colors else "Colors: chosen by hand")


with st.sidebar:
    st.caption(f"{name} · {drawing.bbox[2]:.1f} × {drawing.bbox[3]:.1f} SVG units")
    st.subheader("2 · Marker")
    st.radio("Marker setup", ["Single marker", "Multiple markers"], key="marker_mode", horizontal=True,
             label_visibility="collapsed")
    if single_mode():
        cols = st.columns(2)
        for i, m in enumerate(MARKER_IDS):
            cols[i % 2].button(f"{DOT[m]} {m.capitalize()}", key=f"single_{m}", on_click=set_single, args=(m,),
                               type="primary" if ss.single_marker == m else "secondary", use_container_width=True)
        st.caption(f"Load the {ss.single_marker} marker in the gripper. The whole drawing uses it, whatever colors the SVG has.")
    else:
        st.toggle("Keep SVG colors", key="keep_colors", on_change=on_keep,
                  help="Match each color in the file to the closest of the four markers.")
        if ss.keep_colors:
            n_colors = len({s.svg_color for s in strokes})
            n_markers = len({marker_of(s) for s in strokes})
            st.caption(f"{n_colors} color{'s' * (n_colors != 1)} in the file, drawn with {n_markers} marker{'s' * (n_markers != 1)}.")
        else:
            st.markdown(f"**Whole drawing** · {len(strokes)} strokes")
            same = {marker_of(s) for s in strokes}
            cols = st.columns(2)
            for i, m in enumerate(MARKER_IDS):
                cols[i % 2].button(f"{DOT[m]} {m.capitalize()}", key=f"all_{m}", on_click=set_all, args=(m,),
                                   type="primary" if same == {m} else "secondary", use_container_width=True)
            st.markdown("**Individual strokes**")
            with st.container(height=280):
                for i, s in enumerate(strokes[:STROKE_LIST_LIMIT]):
                    ss.setdefault("mk_" + s.source, marker_of(s))
                    c1, c2 = st.columns([1, 1], vertical_alignment="center")
                    c1.markdown(f"Stroke {i + 1} · <span style='color:#6a6f76'>{s.length():.0f} mm</span>", unsafe_allow_html=True)
                    c2.selectbox(f"Marker for stroke {i + 1}", MARKER_IDS, key="mk_" + s.source, on_change=set_one,
                                 args=(s.source,), format_func=lambda m: f"{DOT[m]} {m.capitalize()}",
                                 label_visibility="collapsed")
                if len(strokes) > STROKE_LIST_LIMIT:
                    st.caption(f"Showing the first {STROKE_LIST_LIMIT} of {len(strokes)} strokes. Use Whole drawing for the rest.")

    st.subheader("3 · Fill")
    st.toggle("Hatch filled areas", key="fill_on",
              help="Replace each filled area with pen strokes. The hatches show in the Robot path view.")
    if fill_error:
        st.error(f"Hatching failed, outlines only: {fill_error}")
    if ss.fill_on:
        with st.expander("Fill settings (measure on the real pen)", expanded=True):
            c1, c2 = st.columns(2)
            c1.number_input("Pen width (mm)", 0.1, 10.0, step=0.1, key="fill_pen",
                            help="Line width the marker leaves. Hatches stay this far inside the outline.")
            c2.number_input("Hatch spacing (mm)", 0.1, 50.0, step=0.1, key="fill_spacing")
            c1.number_input("Min segment (mm)", 0.0, 200.0, step=1.0, key="fill_min_seg",
                            help="Hatch strokes shorter than this are not drawn.")
            c2.number_input("Min spacing (mm)", 0.1, 50.0, step=0.1, key="fill_min_spacing",
                            help="The spacing used is never below this or the pen width.")
            st.radio("Fill type", ["linear", "contour"], key="fill_mode", horizontal=True,
                     format_func=str.capitalize, help="Parallel lines, or rings that follow the outline.")

    with st.expander("Drawing settings"):
        st.number_input("Curve sampling (mm)", 0.5, 20.0, step=0.5, key="spacing",
                        help="Upper limit before simplification. Curves keep only the points they need, within 0.2 mm.")
        st.checkbox("Optimize stroke order (shorter pen-up travel)", key="optimize")

def metrics(strokes: list[Stroke]) -> dict:
    ink = sum(s.length() for s in strokes)
    travel, pos = 0.0, (0.0, 0.0)
    for s in strokes:
        travel += math.dist(pos, s.points[0])
        pos = s.points[-1]
    if strokes:
        travel += math.dist(pos, (0.0, 0.0))
    scale = min(1.0, ss.get("live_speed", 20) / ss.v_draw)
    accel, travel_speed = 100.0, ss.v_joint * scale * 5
    def segment_time(length: float, speed: float) -> float:
        return 2 * math.sqrt(length / accel) if length < speed * speed / accel else length / speed + speed / accel
    draw_time = sum(segment_time(math.dist(a, b), ss.v_draw * scale) for s in strokes for a, b in zip(s.points, s.points[1:]))
    time_s = draw_time + travel / travel_speed + len(strokes) * 2 * segment_time(ss.approach, ss.v_lift * scale)
    return {"ink": ink, "travel": travel, "points": sum(len(s.points) for s in strokes),
            "markers": len({marker_of(s) for s in strokes}), "time": time_s}


h = p["w"] * ss.aspect
m = ss.margin
in_margin = p["x"] < m or p["y"] < m or p["x"] + p["w"] > ss.board_w - m or p["y"] + h > ss.board_h - m

with st.sidebar:
    st.subheader("Checks")
    shown = [w for w in drawing.warnings if not (fill and w == FILL_OUTLINE_WARNING)]
    for w in shown:
        st.warning(w)
    if in_margin:
        st.warning(f"The drawing reaches into the {m:.0f} mm edge margin. Drawing near the edge is less reliable.")
    if not strokes:
        st.warning("Nothing to draw: no visible strokes found.")
    if not shown and not in_margin and strokes:
        st.success(f"Ready: {len(strokes)} stroke{'s' * (len(strokes) != 1)} inside the safe area of a "
                   f"{ss.board_w:.0f} × {ss.board_h:.0f} mm board.")

c1, c2 = st.columns([1, 1], vertical_alignment="center")
c1.segmented_control("View", ["Artwork", "Robot path"], key="view", label_visibility="collapsed")
with c2:
    b1, b2, b3 = st.columns(3)
    b1.button("Center", on_click=do_center, use_container_width=True)
    b2.button("Fit to board", on_click=do_fit, use_container_width=True)
    b3.button("Reset", on_click=do_reset, use_container_width=True)

recolor: dict[str, str] = {}
for s in strokes:
    recolor.setdefault(s.source.split(".")[0], MARKERS[marker_of(s)])

art = {"markup": drawing.markup, "bbox": list(drawing.bbox), "key": key}
hatches = [s for s in strokes if s.source.startswith("fill-")]
if hatches:
    bx, by, bw, _ = drawing.bbox
    k = bw / p["w"]
    svg_xy = lambda q: f"{bx + (q[0] - p['x']) * k:.3f},{by + (q[1] - p['y']) * k:.3f}"
    lines = "".join(
        f'<path data-sid="{s.source.split(".")[0]}" d="M{" L".join(map(svg_xy, s.points))}" stroke="{s.svg_color}" '
        f'stroke-width="{ss.fill_pen * k:.4g}" stroke-linecap="round" stroke-linejoin="round" fill="none"/>'
        for s in hatches)
    head, _, tail = drawing.markup.rpartition("</svg>")
    art = {**art, "markup": head + lines + "</svg>" + tail, "key": f"{key}|{fill}|{p['w']:.3f}"}

board_component(
    board={"w": ss.board_w, "h": ss.board_h, "margin": ss.margin},
    place={"x": p["x"], "y": p["y"], "width": p["w"]},
    aspect=ss.aspect,
    art=art,
    view="path" if ss.view == "Robot path" else "art",
    strokes=[{"p": [[round(x, 2), round(y, 2)] for x, y in s.points], "c": MARKERS[marker_of(s)]} for s in strokes],
    recolor=recolor,
    key="board",
    default=None,
)

for f, v in (("x", p["x"]), ("y", p["y"]), ("w", p["w"]), ("h", h)):
    ss["in_" + f] = round(v, 1)
c = st.columns(4)
c[0].number_input("x, left edge (mm)", step=0.5, key="in_x", format="%.1f", on_change=on_number, args=("x",))
c[1].number_input("y, top edge (mm)", step=0.5, key="in_y", format="%.1f", on_change=on_number, args=("y",))
c[2].number_input("Width (mm)", step=0.5, min_value=0.0, key="in_w", format="%.1f", on_change=on_number, args=("w",))
c[3].number_input("Height (mm)", step=0.5, min_value=0.0, key="in_h", format="%.1f", on_change=on_number, args=("h",))
st.caption("Board coordinates match SVG: origin at the top-left corner, x to the right, y down. "
           "Drag the drawing to move it, drag a corner to scale it. Aspect ratio stays locked.")

mt = metrics(strokes)
cols = st.columns(6)
cols[0].metric("Strokes", len(strokes))
cols[1].metric("Points", mt["points"])
cols[2].metric("Ink", f"{mt['ink'] / 1000:.2f} m" if mt["ink"] >= 1000 else f"{mt['ink']:.0f} mm")
cols[3].metric("Pen-up travel", f"{mt['travel'] / 1000:.2f} m" if mt["travel"] >= 1000 else f"{mt['travel']:.0f} mm")
cols[4].metric("Markers", mt["markers"])
cols[5].metric("Preview estimate only", f"{int(mt['time'] // 60)}:{int(round(mt['time'] % 60)):02d} min")

final = [Stroke(points=s.points, svg_color=s.svg_color, marker=marker_of(s), source=s.source) for s in strokes]
if not single_mode():
    final = backend.group_by_marker(final, ss.optimize)
stroke_list = StrokeList(ss.board_w, ss.board_h, Placement(p["x"], p["y"], p["w"], h), final)
settings = motion_settings()

robot_actions(st, stroke_list, settings, svg_bytes=data, source_name=name,
              single_marker=ss.single_marker if single_mode() else None)
if robot_panel.robot_link.program_script(ss.get("job_result")):
    st.info("Robot program ready. Send it in **Robot** mode.")
tab_data, tab_log = st.tabs(["Stroke data", "Log"])
with tab_data:
    preview = stroke_list.to_dict()
    for s in preview["strokes"]:
        n = len(s["points"])
        s["n_points"] = n
        if n > 4:
            s["points"] = s["points"][:3] + ["…"] + s["points"][-1:]
    st.code(json.dumps(preview, indent=1)[:20000], language="json")
with tab_log:
    st.code("\n".join(ss.log[:200]) or "Nothing yet.", language=None)

