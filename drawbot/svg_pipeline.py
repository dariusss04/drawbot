from __future__ import annotations

import colorsys
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import webcolors
from defusedxml import ElementTree as SafeET
from svgpathtools import Document, Line, QuadraticBezier, CubicBezier, Arc, parse_path
from svgpathtools.document import CONVERSIONS

from .strokes import Placement, Stroke

NS = "http://www.w3.org/2000/svg"
GEOMETRY = {"path", "line", "polyline", "polygon", "circle", "ellipse", "rect"}
CONTAINERS = {"svg", "g", "a", "switch"}
IGNORED = {"title", "desc", "metadata", "defs", "style", "symbol", "clipPath", "mask", "marker", "pattern",
           "linearGradient", "radialGradient", "filter", "animate", "animateTransform", "set"}
CURVE_TOLERANCE_MM = 0.2
SIMPLIFY_TOLERANCE_MM = 0.2
MAX_POINTS = 100000
MAX_USE_DEPTH = 16
FILL_OUTLINE_WARNING = "Filled shapes are drawn as outlines; filled areas are not hatched."
UNITS = {"": 1.0, "px": 1.0, "pt": 96 / 72, "pc": 16.0, "mm": 96 / 25.4, "cm": 96 / 2.54, "in": 96.0,
         "q": 96 / 101.6, "em": 16.0, "ex": 8.0}
WIDTH_ATTRS = {"x", "x1", "x2", "cx", "rx", "width"}
HEIGHT_ATTRS = {"y", "y1", "y2", "cy", "ry", "height"}


class SvgError(ValueError):
    """A user-facing SVG or placement error."""


@dataclass
class SvgDrawing:
    markup: str
    bbox: tuple[float, float, float, float]
    paths: list
    colors: dict[str, str | None]
    warnings: list[str] = field(default_factory=list)
    fills: dict[str, tuple[str | None, str]] = field(default_factory=dict)
    fill_paths: list = field(default_factory=list)

    @property
    def aspect(self):
        return self.bbox[3] / self.bbox[2]


def _tag(element):
    return element.tag.rsplit("}", 1)[-1]


def _foreign(element):
    """True for editor data such as sodipodi:namedview. It is never artwork, so it is dropped silently."""
    return element.tag.startswith("{") and not element.tag.startswith("{" + NS + "}")


def _length(value, reference=None):
    """A plain number from '2', '2px', '10mm' or '50%'. Raises ValueError if it cannot be resolved."""
    match = re.fullmatch(r"\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*([a-zA-Z%]*)\s*", str(value))
    if not match:
        raise ValueError(f"Not a length: {value}")
    number, unit = float(match[1]), match[2].lower()
    if unit == "%":
        if reference is None:
            raise ValueError("Percentage without a viewport")
        return number * reference / 100
    if unit not in UNITS:
        raise ValueError(f"Unknown unit: {unit}")
    return number * UNITS[unit]


def _fraction(value, default=1.0):
    """Opacity as 0..1 from '0.5' or '50%'. Anything unreadable counts as the default."""
    try:
        text = str(value).strip()
        number = float(text[:-1]) / 100 if text.endswith("%") else float(text)
        return number if math.isfinite(number) else default
    except ValueError:
        return default


def _hex(value):
    """'#rrggbb' from a solid CSS color, or None if it is fully transparent. Raises ValueError if unknown."""
    value = value.strip().lower()
    if value == "transparent":
        return None
    if value == "rebeccapurple":
        return "#663399"
    if re.fullmatch(r"#[0-9a-f]{3,4}", value):
        if len(value) == 5 and value[4] == "0":
            return None
        return "#" + "".join(c * 2 for c in value[1:4])
    if re.fullmatch(r"#[0-9a-f]{6}", value):
        return value
    if re.fullmatch(r"#[0-9a-f]{8}", value):
        return None if value[7:] == "00" else value[:7]
    match = re.fullmatch(r"(rgb|hsl)a?\(([^)]+)\)", value)
    if match:
        parts = match[2].replace(",", " ").replace("/", " ").split()
        if len(parts) not in (3, 4):
            raise ValueError("Invalid color")
        if len(parts) == 4 and _fraction(parts[3]) <= 0:
            return None
        if match[1] == "hsl":
            hue = float(parts[0].rstrip("deg")) / 360 % 1
            rgb = colorsys.hls_to_rgb(hue, float(parts[2].rstrip("%")) / 100, float(parts[1].rstrip("%")) / 100)
            numbers = [c * 255 for c in rgb]
        else:
            numbers = [float(p[:-1]) * 255 / 100 if p.endswith("%") else float(p) for p in parts[:3]]
        if not all(math.isfinite(n) for n in numbers):
            raise ValueError("Invalid color")
        return "#" + "".join(f"{round(max(0, min(255, n))):02x}" for n in numbers)
    return webcolors.name_to_hex(value).lower()


def _near_white(color):
    return all(int(color[i:i + 2], 16) >= 240 for i in (1, 3, 5))


def _css_rules(root, warn):
    """Simple stylesheet rules: .class, #id and tag selectors. Later rules win, as in CSS."""
    rules = {}
    for element in root.iter():
        if _foreign(element) or _tag(element) != "style":
            continue
        text = re.sub(r"/\*.*?\*/", "", element.text or "", flags=re.S)
        for selectors, body in re.findall(r"([^{}]+)\{([^{}]*)\}", text):
            declarations = {}
            for part in body.split(";"):
                if ":" in part:
                    key, value = part.split(":", 1)
                    declarations[key.strip()] = value.replace("!important", "").strip()
            for selector in selectors.split(","):
                selector = selector.strip()
                if re.fullmatch(r"[.#]?[\w-]+", selector):
                    rules.setdefault(selector, {}).update(declarations)
                elif selector:
                    warn("Some CSS selectors are too complex and were ignored. Check the colors in the preview.")
    return rules


def _viewport(root):
    """Width and height of the root viewport in user units, for percentages. None if unknown."""
    box = re.split(r"[\s,]+", (root.get("viewBox") or "").strip())
    try:
        if len(box) == 4 and float(box[2]) > 0 and float(box[3]) > 0:
            return float(box[2]), float(box[3])
        width, height = _length(root.get("width", "")), _length(root.get("height", ""))
        return (width, height) if width > 0 and height > 0 else None
    except ValueError:
        return None


def load_svg(data: bytes) -> SvgDrawing:
    """Parse vector shapes and produce safe, flattened preview markup.

    One bad shape never rejects the file: it is skipped with a warning. Fills are drawn as
    outlines. White shapes are skipped because they would be invisible on a whiteboard.
    Text and images cannot be drawn.
    """
    if not isinstance(data, bytes) or len(data) > 5_000_000:
        raise SvgError("Upload an SVG file smaller than 5 MB.")
    try:
        root = SafeET.fromstring(data)
    except Exception as exc:
        raise SvgError("Invalid SVG XML. Export the file again as plain SVG.") from exc
    if _tag(root) != "svg":
        raise SvgError("This file is not an SVG document.")
    warnings = []
    clean = ET.Element(f"{{{NS}}}svg")
    colors = {}
    fills = {}
    sid_count = 0

    def warn(message):
        if message not in warnings:
            warnings.append(message)

    rules = _css_rules(root, warn)
    viewport = _viewport(root)
    by_id = {e.get("id"): e for e in root.iter() if e.get("id")}

    def properties_of(source):
        """Presentation attributes, then stylesheet rules (tag, class, id), then inline style."""
        local = dict(source.attrib)
        local.update(rules.get(_tag(source), {}))
        for name in source.get("class", "").split():
            local.update(rules.get("." + name, {}))
        local.update(rules.get("#" + source.get("id", ""), {}))
        for part in source.get("style", "").split(";"):
            if ":" in part:
                key, value = part.split(":", 1)
                local[key.strip()] = value.replace("!important", "").strip()
        return local

    def paint(value, inherited):
        """Hex color of a stroke or fill, or None if it paints nothing."""
        value = value.strip()
        if value.lower() == "none":
            return None
        if value.lower() == "currentcolor":
            value = inherited.get("color", "black")
        if value.lower().startswith("url("):
            fallback = re.sub(r"url\([^)]*\)", "", value).strip()
            if fallback:
                return paint(fallback, inherited)
            warn("Gradients and patterns cannot be drawn with a marker. Those shapes use a black outline.")
            return "#000000"
        try:
            return _hex(value)
        except (ValueError, IndexError):
            warn(f"Unknown color '{value}' was drawn as black.")
            return "#000000"

    def numeric_geometry(source, tag):
        """Shape attributes as plain user-unit numbers. Returns None if a value cannot be resolved."""
        attrs = {k: v for k, v in source.attrib.items() if k in {
            "d", "x", "y", "x1", "y1", "x2", "y2", "width", "height", "rx", "ry", "cx", "cy", "r", "points", "transform"
        }}
        try:
            for key in list(attrs):
                if key in {"d", "points", "transform"}:
                    continue
                if attrs[key].strip() == "auto":
                    del attrs[key]
                    continue
                reference = None
                if viewport:
                    reference = (viewport[0] if key in WIDTH_ATTRS else viewport[1] if key in HEIGHT_ATTRS
                                 else math.hypot(*viewport) / math.sqrt(2))
                attrs[key] = repr(_length(attrs[key], reference))
            if tag != "path":
                parse_path(CONVERSIONS[tag](ET.Element(tag, attrs)))
        except Exception:
            return None
        return attrs

    def visit(source, target, inherited, hidden=False, opacity=1.0, depth=0, via_use=False):
        nonlocal sid_count
        if _foreign(source):
            return
        tag = _tag(source)
        if tag == "use":
            reference = by_id.get((source.get("href") or source.get("{http://www.w3.org/1999/xlink}href") or "").lstrip("#"))
            if reference is None or depth >= MAX_USE_DEPTH:
                warn("A <use> reference could not be resolved and was skipped.")
                return
            transforms = [source.get("transform", "")]
            try:
                transforms.append(f"translate({_length(source.get('x', 0))} {_length(source.get('y', 0))})")
            except ValueError:
                pass
            local = properties_of(source)
            node = ET.SubElement(target, f"{{{NS}}}g", {"transform": " ".join(t for t in transforms if t)})
            merged = dict(inherited)
            for key in ("stroke", "fill", "color", "visibility", "stroke-opacity", "fill-opacity", "stroke-width", "fill-rule"):
                if key in local and local[key] != "inherit":
                    merged[key] = local[key].strip()
            visit(reference, node, merged, hidden or local.get("display") == "none",
                  opacity * _fraction(local.get("opacity", "1")), depth + 1, True)
            return
        if tag == "symbol" and via_use:
            tag = "g"
        if tag not in CONTAINERS | GEOMETRY:
            if tag == "text":
                warn("Text is not drawn. Convert text to paths in Inkscape before uploading.")
            elif tag == "image":
                warn("Embedded images are not drawn. Only vector shapes can be drawn.")
            elif tag not in IGNORED:
                warn(f"Unsupported {tag} elements were removed. Export plain vector paths instead.")
            return
        local = properties_of(source)
        properties = dict(inherited)
        for key in ("stroke", "fill", "color", "visibility", "stroke-opacity", "fill-opacity", "stroke-width", "fill-rule"):
            if key in local and local[key] != "inherit":
                properties[key] = local[key].strip()
        hidden = hidden or local.get("display") == "none"
        opacity *= _fraction(local.get("opacity", "1"))
        if any(local.get(key, "none") != "none" for key in ("clip-path", "mask", "filter")):
            warn("Clipping, masks and filters are ignored. The full shape is drawn.")
        if tag in CONTAINERS or tag == "g":
            node = target if source is root else ET.SubElement(target, f"{{{NS}}}g")
            transform = source.get("transform", "")
            if tag == "svg" and source is not root:
                if source.get("viewBox"):
                    warn("Nested SVG viewports are drawn without their own scaling. Flatten them first if the result looks wrong.")
                try:
                    transform = f"{transform} translate({_length(source.get('x', 0))} {_length(source.get('y', 0))})".strip()
                except ValueError:
                    pass
            if transform:
                node.set("transform", transform)
            for child in source:
                visit(child, node, properties, hidden, opacity, depth, False)
            return
        sid = str(sid_count)
        sid_count += 1
        colors[sid] = None
        if hidden or opacity <= 0 or properties.get("visibility", "visible") in {"hidden", "collapse"}:
            return
        stroke = properties.get("stroke", "none")
        fill = properties.get("fill", "black")
        try:
            width = _length(properties.get("stroke-width", 1), 100)
        except ValueError:
            width = 1.0
        stroke_color = paint(stroke, properties) if _fraction(properties.get("stroke-opacity", 1)) > 0 and width > 0 else None
        fill_color = paint(fill, properties) if _fraction(properties.get("fill-opacity", 1)) > 0 else None
        if stroke_color is None and fill_color is None:
            return
        drawable = [c for c in (stroke_color, fill_color) if c and not _near_white(c)]
        if not drawable:
            warn("White shapes are skipped because they would be invisible on a whiteboard.")
            if fill_color is None:
                return
        color = drawable[0] if drawable else None
        attrs = numeric_geometry(source, tag)
        if attrs is None:
            if color:
                warn(f"A {tag} with unreadable dimensions was skipped. Use plain numbers in the SVG.")
            return
        colors[sid] = color
        if fill_color:
            fills[sid] = (None if _near_white(fill_color) else fill_color,
                          properties.get("fill-rule", "nonzero").strip().lower())
        attrs.update({"data-sid": sid, "stroke": color or "none", "fill": "none"})
        if tag == "path":
            chunks = re.split(r"(?=[Mm])", attrs.get("d", ""))
            current = 0j
            part = 0
            for chunk in chunks:
                if not chunk.strip():
                    continue
                try:
                    parsed = parse_path(chunk, current_pos=current)
                except Exception:
                    warn("A path with invalid data was skipped. Export the file again as plain SVG.")
                    break
                if not len(parsed):
                    continue
                current = parsed[-1].end
                child_attrs = dict(attrs, d=parsed.d(), **{"data-part": str(part)})
                ET.SubElement(target, f"{{{NS}}}path", child_attrs)
                part += 1
        else:
            attrs["data-part"] = "0"
            ET.SubElement(target, f"{{{NS}}}{tag}", attrs)

    visit(root, clean, {})
    try:
        every = [p for p in Document.from_svg_string(ET.tostring(clean, encoding="unicode")).paths() if len(p) and p.length() > 0]
        paths = [p for p in every if colors[p.element.get("data-sid")]]
        boxes = [p.bbox() for p in paths]
    except Exception as exc:
        raise SvgError("Could not read this geometry or transform. Export plain SVG with numeric dimensions.") from exc
    if not paths:
        raise SvgError("No visible vector shapes to draw. Convert text to paths and remove images.")
    xmin, xmax = min(b[0] for b in boxes), max(b[1] for b in boxes)
    ymin, ymax = min(b[2] for b in boxes), max(b[3] for b in boxes)
    if not all(math.isfinite(v) for v in (xmin, xmax, ymin, ymax)):
        raise SvgError("The SVG contains non-finite geometry.")
    if xmax <= xmin and ymax <= ymin:
        raise SvgError("The artwork has no size. Add geometry with a real width or height.")
    bbox = (float(xmin), float(ymin), max(float(xmax - xmin), 1e-3), max(float(ymax - ymin), 1e-3))
    preview = ET.Element(f"{{{NS}}}svg", {"viewBox": " ".join(map(str, bbox))})
    grouped = {}
    for path in paths:
        sid = path.element.get("data-sid")
        grouped.setdefault(sid, []).append(path.d())
    for sid, definitions in grouped.items():
        ET.SubElement(preview, f"{{{NS}}}path", {"data-sid": sid, "d": " ".join(definitions), "stroke": colors[sid], "fill": "none"})
    ET.register_namespace("", NS)
    fill_paths = [p for p in every if p.element.get("data-sid") in fills]
    if any(fills[p.element.get("data-sid")][0] and _encloses_area(p) for p in fill_paths):
        warnings.append(FILL_OUTLINE_WARNING)
    return SvgDrawing(ET.tostring(preview, encoding="unicode"), bbox, paths, colors, warnings, fills, fill_paths)


def _encloses_area(path):
    """True if a fill would paint something: any curve, or straight edges with a nonzero shoelace area."""
    if any(not isinstance(s, Line) for s in path):
        return True
    pts = [s.start for s in path] + [path[-1].end]
    return abs(sum((a.conjugate() * b).imag for a, b in zip(pts, pts[1:] + pts[:1]))) > 1e-9


def _distance(p, a, b):
    direction = b-a
    if abs(direction) == 0:
        return abs(p-a)
    t = max(0.0, min(1.0, ((p-a).real*direction.real+(p-a).imag*direction.imag)/abs(direction)**2))
    return abs(p-(a+t*direction))


def _rdp(points, tolerance):
    keep = {0, len(points)-1}
    stack = [(0, len(points)-1)]
    while stack:
        a, b = stack.pop()
        if b-a < 2:
            continue
        index = max(range(a+1, b), key=lambda k: _distance(points[k], points[a], points[b]))
        if _distance(points[index], points[a], points[b]) > tolerance:
            keep.add(index)
            stack.extend([(a,index), (index,b)])
    result = [points[i] for i in sorted(keep)]
    if points[0] == points[-1] and len(set(result)) < 3:
        return points
    return result


def _sample_segment(segment, scale, spacing):
    points = [segment.start]
    pending = [(segment, 0)]
    while pending:
        seg, depth = pending.pop()
        if isinstance(seg, Line):
            bound = 0.0
            upper_length = abs(seg.end-seg.start)*scale
        elif isinstance(seg, (QuadraticBezier, CubicBezier)):
            controls = list(seg.bpoints())
            bound = max(_distance(p, seg.start, seg.end) for p in controls)*scale
            upper_length = sum(abs(b-a) for a,b in zip(controls,controls[1:]))*scale
        elif isinstance(seg, Arc):
            angle = abs(math.radians(seg.delta))
            radius = max(abs(seg.radius.real),abs(seg.radius.imag))*scale
            bound = radius*angle**2/8
            upper_length = radius*angle
        else:
            raise SvgError("Unsupported curve segment.")
        if bound <= CURVE_TOLERANCE_MM and upper_length <= spacing:
            points.append(seg.end)
        else:
            if depth >= 24 or len(points)+len(pending) > MAX_POINTS:
                raise SvgError("Drawing is too complex. Increase point spacing or simplify the SVG.")
            pending.extend([(seg.cropped(.5,1),depth+1),(seg.cropped(0,.5),depth+1)])
    points[-1] = segment.end
    return points


def _travel(strokes):
    position = (0.,0.)
    total = 0.
    for stroke in strokes:
        total += math.dist(position,stroke.points[0])
        position = stroke.points[-1]
    return total+math.dist(position,(0.,0.))


def order_strokes(strokes, start=(0.,0.)):
    """Nearest neighbour with endpoint reversal and an origin-round-trip fallback."""
    remaining, ordered, position = list(strokes), [], start
    while remaining:
        i, reverse = min(((i,r) for i in range(len(remaining)) for r in (False,True)), key=lambda pair: math.dist(position,remaining[pair[0]].points[-1 if pair[1] else 0]))
        source = remaining.pop(i)
        stroke = Stroke(list(reversed(source.points)) if reverse else list(source.points),source.svg_color,source.marker,source.source)
        ordered.append(stroke)
        position = stroke.points[-1]
    if start == (0.,0.) and _travel(ordered) > _travel(strokes):
        return list(strokes)
    return ordered


def group_by_marker(strokes, optimize=True):
    """All strokes of one marker together, so each pen is picked up only once.

    Markers keep the order in which they first appear. Inside a group the stroke order is kept,
    or with optimize, ordered again by nearest neighbour from where the last group ended.
    """
    order = list(dict.fromkeys(s.marker for s in strokes))
    result, position = [], (0., 0.)
    for marker in order:
        group = [s for s in strokes if s.marker == marker]
        if optimize:
            group = order_strokes(group, position)
        result.extend(group)
        position = result[-1].points[-1]
    return result


def svg_to_strokes(drawing: SvgDrawing, placement: Placement, spacing_mm: float = 2.0,
                   optimize: bool = True, fill=None) -> list[Stroke]:
    """Return board-mm strokes. spacing_mm limits the curve sampling before simplification.

    Curve approximation and RDP each use 0.2 mm directed tolerances, well under the marker width, so curves keep
    only the points they need and straight lines keep only their ends.
    This is not a robot accuracy or topology guarantee.
    fill: a hatching.FillSettings to also hatch filled areas, or None for outlines only.
    """
    from .colors import match_color
    values = (placement.x,placement.y,placement.width,placement.height,spacing_mm)
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
        raise SvgError("Placement and spacing must be finite numbers.")
    if placement.x < 0 or placement.y < 0 or min(placement.width,placement.height,spacing_mm) <= 0:
        raise SvgError("Use non-negative placement coordinates and positive dimensions and spacing.")
    if not math.isclose(placement.height,placement.width*drawing.aspect,rel_tol=1e-6,abs_tol=1e-6):
        raise SvgError("Placement must preserve the artwork aspect ratio.")
    bx,by,bw,bh = drawing.bbox
    scale = placement.width/bw
    result = []
    total_points = 0
    for path in drawing.paths:
        sid = path.element.get("data-sid")
        color = drawing.colors.get(sid)
        if color is None:
            continue
        part = path.element.get("data-part","0")
        points = []
        for segment in path:
            sampled = _sample_segment(segment,scale,spacing_mm)
            points.extend(sampled if not points else sampled[1:])
        board = [complex(placement.x+(p.real-bx)*scale,placement.y+(p.imag-by)*scale) for p in points]
        simplified = _rdp(board,SIMPLIFY_TOLERANCE_MM)
        x0,x1 = placement.x,placement.x+placement.width
        y0,y1 = placement.y,placement.y+placement.height
        output = [(min(max(float(p.real),x0),x1),min(max(float(p.imag),y0),y1)) for p in simplified]
        if len(output)<2:
            continue
        total_points += len(output)
        if total_points > MAX_POINTS:
            raise SvgError("Too many drawing points. Reduce the SVG complexity.")
        result.append(Stroke(output,color,match_color(color),f"{sid}.{part}"))
    if fill is not None:
        from .hatching import hatch_strokes
        for hatch in hatch_strokes(drawing,placement,spacing_mm,fill):
            simple = _rdp([complex(x,y) for x,y in hatch.points],SIMPLIFY_TOLERANCE_MM)
            hatch.points = [(float(q.real),float(q.imag)) for q in simple]
            result.append(hatch)
        if sum(len(s.points) for s in result) > MAX_POINTS:
            raise SvgError("Too many drawing points. Increase the hatch spacing or simplify the SVG.")
    return order_strokes(result) if optimize else result
