from __future__ import annotations

import math

import numpy as np

from .strokes import Placement, Stroke, StrokeList

POINTS = ("tl", "tr", "c", "bl", "br")
LABELS = {"tl": "Top left", "tr": "Top right", "c": "Centre", "bl": "Bottom left", "br": "Bottom right"}
INSET_MM = 40.0
CROSS_MM = 12.0
INPUT_RANGE_MM = 50


def positions(width, height, inset=INSET_MM):
    """Test points {name: (x, y)} in board mm (origin top-left, x right, y down)."""
    inset = min(inset, width / 4, height / 4)
    return {"tl": (inset, inset), "tr": (width - inset, inset), "c": (width / 2, height / 2),
            "bl": (inset, height - inset), "br": (width - inset, height - inset)}


def fit(values, width, height):
    """Best plane through the corrections. values: {point: mm, + = deeper}, missing points count as 0.

    Returns {"a", "b", "c"} (depth at the origin in mm, mm per mm along x and y),
    "applied" {point: mm the plane gives there} and "leftover" {point: wanted - applied}.
    """
    where = positions(width, height)
    rows = [[1.0, where[k][0], where[k][1]] for k in POINTS]
    wanted = [float(values.get(k) or 0.0) for k in POINTS]
    (a, b, c), *_ = np.linalg.lstsq(np.array(rows), np.array(wanted), rcond=None)
    applied = {k: float(a + b * where[k][0] + c * where[k][1]) for k in POINTS}
    return {"a": float(a), "b": float(b), "c": float(c), "applied": applied,
            "leftover": {k: w - applied[k] for k, w in zip(POINTS, wanted)}}


def is_zero(values):
    return all(abs(float(values.get(k) or 0.0)) < 1e-9 for k in POINTS)


def correction_matrix(a, b, c):
    """4x4 pose of the corrected board in the measured board frame (mm).

    The corrected plane is z = a + b*x + c*y, with z into the board as in BoardFrame.
    Its x axis follows the measured x axis, so drawings keep their place and direction.
    """
    x = np.array([1.0, 0.0, b])
    x /= np.linalg.norm(x)
    z = np.array([-b, -c, 1.0])
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = x, y, z, [0.0, 0.0, a]
    return m


def corrected_pose(pose, values, width, height):
    """Measured board pose (4x4, robot base, mm) with the corrections applied. Returns (pose, fit result)."""
    result = fit(values, width, height)
    corrected = np.array(pose, dtype=float) @ correction_matrix(result["a"], result["b"], result["c"])
    return corrected.tolist(), result


def tilt_text(result, width, height):
    """The fitted plane in words, for the UI."""
    parts = [f"{result['a'] + result['b'] * width / 2 + result['c'] * height / 2:+.1f} mm deeper in the middle"]
    lr, tb = result["b"] * width, result["c"] * height
    if abs(lr) >= 0.05:
        parts.append(f"right edge {lr:.1f} mm deeper than the left edge" if lr > 0
                     else f"left edge {-lr:.1f} mm deeper than the right edge")
    if abs(tb) >= 0.05:
        parts.append(f"bottom edge {tb:.1f} mm deeper than the top edge" if tb > 0
                     else f"top edge {-tb:.1f} mm deeper than the bottom edge")
    angle = math.degrees(math.atan(math.hypot(result["b"], result["c"])))
    return ", ".join(parts) + f" (tilt {angle:.2f}°)"


def touch_test_strokes(width, height, marker="black"):
    """A small cross at every test point: the program that shows where the pen touches."""
    strokes = []
    half = CROSS_MM / 2
    for name, (x, y) in positions(width, height).items():
        strokes.append(Stroke([(x - half, y), (x + half, y)], "#000000", marker, f"{name}.0"))
        strokes.append(Stroke([(x, y - half), (x, y + half)], "#000000", marker, f"{name}.1"))
    return StrokeList(width, height, Placement(0.0, 0.0, width, height), strokes)
