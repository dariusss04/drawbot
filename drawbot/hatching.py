from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import polygonize, unary_union

from .colors import match_color
from .strokes import Placement, Stroke
from .svg_pipeline import SvgDrawing, SvgError, _sample_segment


@dataclass(frozen=True)
class FillSettings:
    """Physical drawing settings in mm. Starting values, calibrate them for the real pen."""
    pen_width_mm: float = 0.8
    hatch_spacing_mm: float = 3.2
    min_segment_mm: float = 10.0
    min_spacing_mm: float = 3.2
    mode: str = "linear"


def _polygons(geom) -> list[Polygon]:
    return [g for g in getattr(geom, "geoms", [geom]) if isinstance(g, Polygon) and not g.is_empty]


def _winding(x: float, y: float, ring) -> int:
    """Winding number of a closed ring around (x, y). Exact for self-intersecting rings."""
    w = 0
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        side = (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1)
        if y1 <= y < y2 and side > 0:
            w += 1
        elif y2 <= y < y1 and side < 0:
            w -= 1
    return w


def _region(rings, rule: str):
    """The filled area of one shape's subpaths under its SVG fill-rule (nonzero or evenodd)."""
    rings = [r for r in rings if len(set(r)) >= 3]
    if len(rings) == 1 and (single := Polygon(rings[0])).is_valid:
        return single
    if not rings:
        return Polygon()
    keep = []
    for face in polygonize(unary_union([LineString(r + r[:1]) for r in rings])):
        x, y = face.representative_point().coords[0]
        w = sum(_winding(x, y, r) for r in rings)
        if (w % 2 if rule == "evenodd" else w):
            keep.append(face)
    return unary_union(keep)


def _lines_at(poly: Polygon, angle: float, pitch: float) -> list[LineString]:
    c, s = math.cos(angle), math.sin(angle)
    coords = poly.exterior.coords
    across = [y * c - x * s for x, y in coords]
    along = [x * c + y * s for x, y in coords]
    lo, hi = min(across), max(across)
    t0, t1 = min(along) - 1, max(along) + 1
    n = int((hi - lo) / pitch)
    first = lo + (hi - lo - n * pitch) / 2
    lines = []
    for k in range(n + 1):
        o = first + k * pitch
        lines.append([(-s * o + c * t0, c * o + s * t0), (-s * o + c * t1, c * o + s * t1)])
    cut = poly.intersection(MultiLineString(lines))
    return [g for g in getattr(cut, "geoms", [cut]) if isinstance(g, LineString) and not g.is_empty]


def _linear(poly: Polygon, pitch: float) -> list[LineString]:
    """Parallel lines along the long axis or across it, whichever needs fewer pen lifts."""
    with np.errstate(all="ignore"):
        box = list(poly.minimum_rotated_rectangle.exterior.coords)
    (ax, ay), (bx, by) = max(zip(box, box[1:]), key=lambda e: math.dist(*e))
    axis = math.atan2(by - ay, bx - ax)
    return min((_lines_at(poly, a, pitch) for a in (axis, axis + math.pi / 2)), key=len)


def _contours(poly: Polygon, pitch: float) -> list[LineString]:
    """Inset rings every pitch inside the outline. The outline itself is already drawn."""
    rings, k = [], 1
    while not (inner := poly.buffer(-k * pitch)).is_empty:
        for p in _polygons(inner):
            rings += [p.exterior, *p.interiors]
        k += 1
    return rings


def hatch_strokes(drawing: SvgDrawing, placement: Placement, spacing_mm: float,
                  settings: FillSettings) -> list[Stroke]:
    values = (settings.pen_width_mm, settings.hatch_spacing_mm, settings.min_segment_mm, settings.min_spacing_mm)
    if not all(math.isfinite(v) for v in values) or min(values) < 0 or settings.pen_width_mm <= 0 \
            or settings.hatch_spacing_mm <= 0 or settings.mode not in ("linear", "contour"):
        raise SvgError("Fill settings must be positive numbers and the fill type linear or contour.")
    pitch = max(settings.hatch_spacing_mm, settings.min_spacing_mm, settings.pen_width_mm)
    bx, by, bw, _ = drawing.bbox
    scale = placement.width / bw

    rings: dict[str, list] = {}
    for path in drawing.fill_paths:
        points = []
        for segment in path:
            sampled = _sample_segment(segment, scale, spacing_mm)
            points.extend(sampled if not points else sampled[1:])
        rings.setdefault(path.element.get("data-sid"), []).append(
            [(placement.x + (p.real - bx) * scale, placement.y + (p.imag - by) * scale) for p in points])

    covered, visible = Polygon(), {}
    for sid in sorted(rings, key=int, reverse=True):
        color, rule = drawing.fills[sid]
        region = _region(rings[sid], rule)
        if region.is_empty:
            continue
        if color:
            visible.setdefault(color, []).append(region.difference(covered))
        covered = covered.union(region)

    strokes = []
    for color, parts in visible.items():
        for area in _polygons(unary_union(parts)):
            lines = _contours(area, pitch) if settings.mode == "contour" else []
            if not lines:
                for inner in _polygons(area.buffer(-settings.pen_width_mm / 2)):
                    lines += _linear(inner, pitch)
            for line in lines:
                if line.length >= settings.min_segment_mm:
                    points = [(float(x), float(y)) for x, y in line.coords]
                    strokes.append(Stroke(points, color, match_color(color), f"fill-{color.lstrip('#')}.{len(strokes)}"))
    return strokes
