import math
from .dataset import is_number


def check_circle(data, center=None, diameter_mm=100.0, tolerance_mm=0.1,
                 closure_tolerance_mm=0.001):
    for value in (diameter_mm, tolerance_mm, closure_tolerance_mm):
        if not is_number(value) or value <= 0:
            raise ValueError("Circle dimensions/tolerances must be positive finite numbers")
    strokes = data["strokes"]
    if len(strokes) != 1:
        raise ValueError("Circle acceptance requires one continuous stroke")
    points = strokes[0]["points"]
    if center is None:
        p = data["placement"]
        center = [p["x"] + p["width"] / 2, p["y"] + p["height"] / 2]
    if len(center) != 2 or not all(is_number(v) for v in center):
        raise ValueError("Circle center must contain two finite numbers")
    cx, cy = center
    radius = diameter_mm / 2
    vertex_error = max(abs(math.hypot(x-cx, y-cy) - radius) for x, y in points)
    segment_error = 0.0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm2 = dx*dx + dy*dy
        t = max(0.0, min(1.0, ((cx-a[0])*dx + (cy-a[1])*dy) / norm2)) if norm2 else 0
        near = math.hypot(a[0]+t*dx-cx, a[1]+t*dy-cy)
        far = max(math.dist(a, center), math.dist(b, center))
        segment_error = max(segment_error, abs(near-radius), abs(far-radius))
    width = max(p[0] for p in points) - min(p[0] for p in points)
    height = max(p[1] for p in points) - min(p[1] for p in points)
    closure = math.dist(points[0], points[-1])
    angles = [math.atan2(y-cy, x-cx) for x, y in points]
    sweep = sum(math.atan2(math.sin(b-a), math.cos(b-a)) for a, b in zip(angles, angles[1:]))
    passed = (max(vertex_error, segment_error, abs(width-diameter_mm), abs(height-diameter_mm))
              <= tolerance_mm and closure <= closure_tolerance_mm
              and abs(abs(sweep)-2*math.pi) <= 0.01)
    return {
        "passed": passed, "point_count": len(points), "center_mm": list(center),
        "ideal_radius_mm": radius, "diameter_mm": diameter_mm,
        "measured_width_mm": width, "measured_height_mm": height,
        "max_vertex_radial_error_mm": vertex_error,
        "max_segment_radial_error_mm": segment_error, "closure_error_mm": closure,
        "sweep_radians": sweep, "tolerance_mm": tolerance_mm,
        "closure_tolerance_mm": closure_tolerance_mm,
        "tolerance_source": "configurable engineering acceptance; not a teacher-specified threshold",
    }
