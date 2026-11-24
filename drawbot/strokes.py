from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field

MARKERS: dict[str, str] = {
    "black": "#1d1d1f",
    "purple": "#7b3fb5",
    "green": "#1f8a4c",
    "orange": "#f08a24",
}


@dataclass
class Placement:
    """Where the drawing sits on the board: its top-left corner and size, in mm."""
    x: float
    y: float
    width: float
    height: float


@dataclass
class Stroke:
    points: list[tuple[float, float]]
    svg_color: str = "#000000"
    marker: str = "black"
    source: str = ""

    def length(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))


@dataclass
class StrokeList:
    board_width: float
    board_height: float
    placement: Placement
    strokes: list[Stroke] = field(default_factory=list)
    units: str = "mm"
    origin: str = "board top-left, x right, y down (same as SVG)"

    def to_dict(self, decimals: int = 2) -> dict:
        d = asdict(self)
        for s in d["strokes"]:
            s["points"] = [[round(x, decimals), round(y, decimals)] for x, y in s["points"]]
        return d

    def to_json(self, decimals: int = 2) -> str:
        return json.dumps(self.to_dict(decimals))
