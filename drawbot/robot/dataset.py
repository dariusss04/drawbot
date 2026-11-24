import math

MARKERS = frozenset(("black", "green", "purple", "orange"))
ORIGIN = "board top-left, x right, y down (same as SVG)"


def is_number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def validate_palette(settings):
    if not isinstance(settings, dict):
        raise ValueError("Settings must be an object")
    palette = settings.get("palette", [])
    if not isinstance(palette, list) or len(palette) != 4:
        raise ValueError("Configure exactly four available pen colors")
    colors, names, programs = set(), set(), set()
    for pen in palette:
        if not isinstance(pen, dict):
            raise ValueError("Each palette entry must be an object")
        color = pen.get("hex", "")
        if not isinstance(color, str):
            raise ValueError("Palette hex colors must be strings")
        color = color.lower()
        if (len(color) != 7 or not color.startswith("#")
                or any(c not in "0123456789abcdef" for c in color[1:])):
            raise ValueError(f"Invalid palette hex color: {color}")
        name, program = pen.get("name"), pen.get("program")
        if not isinstance(name, str) or not name or not isinstance(program, str) or not program:
            raise ValueError("Each pen requires a name and a change-program name")
        marker = name.lower()
        if marker not in MARKERS:
            raise ValueError(f"Pen name must identify a marker: {sorted(MARKERS)}")
        if color in colors or marker in names or program in programs:
            raise ValueError("Pen colors, names and change-program names must be unique")
        colors.add(color)
        names.add(marker)
        programs.add(program)
    if settings.get("pen_change_mode") not in ("stub", "external", "manual_preloaded", "stand"):
        raise ValueError("pen_change_mode must be 'stub', 'external', 'manual_preloaded' or 'stand'")
    if (settings.get("pen_change_mode") == "manual_preloaded"
            and settings.get("preloaded_marker") not in MARKERS):
        raise ValueError("manual_preloaded requires a valid preloaded_marker")
    for field in ("lift_z_mm", "draw_z_mm"):
        if not is_number(settings.get(field)):
            raise ValueError(f"{field} must be a finite number")
    for field in ("linear_speed_mm_s", "joint_speed_deg_s",
                  "linear_accel_mm_s2", "joint_accel_deg_s2"):
        if not is_number(settings.get(field)) or settings[field] <= 0:
            raise ValueError(f"{field} must be positive")
    hardware = settings.get("hardware", {})
    if not isinstance(hardware, dict):
        raise ValueError("hardware must be an object")
    for field in ("mount_xyz_mm", "pen_tcp_xyz_mm"):
        xyz = hardware.get(field)
        if not isinstance(xyz, list) or len(xyz) != 3 or not all(is_number(v) for v in xyz):
            raise ValueError(f"hardware.{field} must contain three finite numbers")
    opening = hardware.get("initial_jaw_opening_mm")
    if not is_number(opening) or not 0 <= opening <= 50:
        raise ValueError("Hand-E jaw opening must be between 0 and 50 mm")
    return {pen["name"].lower(): pen for pen in palette}


def validate_drawing(data, palette):
    if not isinstance(data, dict) or data.get("units") != "mm":
        raise ValueError("StrokeList must be an object with units='mm'")
    if data.get("origin") != ORIGIN:
        raise ValueError(f"origin must be {ORIGIN!r}")
    width, height = data.get("board_width"), data.get("board_height")
    if not all(is_number(v) and v > 0 for v in (width, height)):
        raise ValueError("board_width and board_height must be positive finite numbers")
    placement = data.get("placement")
    if not isinstance(placement, dict):
        raise ValueError("placement must be an object")
    x, y, w, h = (placement.get(k) for k in ("x", "y", "width", "height"))
    if (not all(is_number(v) for v in (x, y, w, h))
            or x < 0 or y < 0 or w <= 0 or h <= 0):
        raise ValueError("placement needs finite non-negative x/y and positive width/height")
    if x + w > width + 1e-6 or y + h > height + 1e-6:
        raise ValueError("placement is outside the board")
    strokes = data.get("strokes")
    if not isinstance(strokes, list) or not strokes:
        raise ValueError("Drawing must contain at least one stroke")
    for i, stroke in enumerate(strokes, 1):
        if not isinstance(stroke, dict):
            raise ValueError(f"Stroke {i} must be an object")
        marker = stroke.get("marker")
        if not isinstance(marker, str) or marker not in MARKERS or marker not in palette:
            raise ValueError(f"Stroke {i}: unsupported marker {marker!r}; use {sorted(MARKERS)}")
        for field in ("svg_color", "source"):
            if not isinstance(stroke.get(field), str):
                raise ValueError(f"Stroke {i}: {field} must be a string")
        points = stroke.get("points")
        if not isinstance(points, list) or len(points) < 2:
            raise ValueError(f"Stroke {i} needs at least 2 points")
        for j, point in enumerate(points, 1):
            if (not isinstance(point, list) or len(point) != 2
                    or not all(is_number(v) for v in point)):
                raise ValueError(f"Stroke {i}, point {j}: expected two finite numbers")
            x, y = point
            if not (-1e-6 <= x <= width + 1e-6 and -1e-6 <= y <= height + 1e-6):
                raise ValueError(f"Stroke {i}, point {j}: outside board: {point}")
    return strokes


def stroke_moves(stroke_id, stroke, lift_z, draw_z):
    """Point 1 is PenDown; draw point numbering retains the JSON indices."""
    prefix = f"S{stroke_id:02d}"
    first, last = stroke["points"][0], stroke["points"][-1]
    yield f"{prefix}_Approach", [*first, lift_z], "J"
    yield f"{prefix}_PenDown", [*first, draw_z], "L"
    for point_id, point in enumerate(stroke["points"][1:], 2):
        yield f"{prefix}_Draw_P{point_id:03d}", [*point, draw_z], "L"
    yield f"{prefix}_Retract", [*last, lift_z], "L"


BLEND_LEVELS_MM = (1.0, 2.0, 3.0, 5.0)
CORNER_DEG = 30.0


def blend_radii(points, max_mm, corner_deg=CORNER_DEG):
    """Blend radius in mm for each point of a stroke; 0 means stop exactly there.

    A stroke's waypoints only approximate a curve, so stopping at each one adds braking but
    no accuracy. Gentle bends get a blend: the robot rounds through the point without stopping.
    The first and last point (pen down, before pen up) and sharp corners keep exact stops.
    The radius stays under 40% of the shorter neighbouring segment, so neighbouring blends never
    overlap (a UR stops with an error if they do). Radii snap to a few whole-millimetre levels
    that the UR script can write exactly; where even 1 mm is too much, the point stops exactly.
    """
    radii = [0.0] * len(points)
    if not max_mm or max_mm <= 0 or len(points) < 3:
        return radii
    for i in range(1, len(points) - 1):
        a, b, c = points[i - 1], points[i], points[i + 1]
        u = [b[k] - a[k] for k in range(2)]
        v = [c[k] - b[k] for k in range(2)]
        l1, l2 = math.hypot(*u), math.hypot(*v)
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        cos_turn = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) / (l1 * l2)))
        if math.degrees(math.acos(cos_turn)) > corner_deg:
            continue
        limit = min(max_mm, 0.4 * min(l1, l2))
        levels = [r for r in BLEND_LEVELS_MM if r <= limit + 1e-9]
        radii[i] = levels[-1] if levels else 0.0
    return radii


def color_changes(strokes, palette):
    """Request pens by marker, preserving order; svg_color is metadata only."""
    previous = None
    for stroke_id, stroke in enumerate(strokes, 1):
        marker = stroke["marker"]
        if marker != previous:
            yield stroke_id, palette[marker]
            previous = marker
