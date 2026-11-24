import hashlib
import json
from pathlib import Path

from ..backend import load_svg, svg_to_strokes
from ..strokes import Placement, StrokeList


def prepare_circle(root, svg=None):
    root = Path(root).resolve()
    svg = Path(svg) if svg else Path(__file__).resolve().parents[2] / "svg/circle-100mm.svg"
    if not svg.is_file():
        raise FileNotFoundError(f"Need the circle SVG at {svg}")
    drawing = load_svg(svg.read_bytes())
    placement = Placement(20.0, 30.0, 100.0, 100.0 * drawing.aspect)
    strokes = svg_to_strokes(drawing, placement, spacing_mm=2.0)
    destination = root / "data/circle_100mm_strokes.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(StrokeList(300.0, 300.0, placement, strokes).to_json(decimals=6), encoding="utf-8")
    provenance = {"source_svg": str(svg), "source_svg_sha256": hashlib.sha256(svg.read_bytes()).hexdigest(),
                  "output_json": str(destination), "output_sha256": hashlib.sha256(destination.read_bytes()).hexdigest()}
    return destination, provenance
