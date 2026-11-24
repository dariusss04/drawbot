from __future__ import annotations

import colorsys


def match_color(hex_color: str) -> str:
    """Greys, white and near-black go to black. Otherwise the hue angle decides.

    The markers are black, green, purple and orange: red and yellow go to orange (the closest hue),
    cyan and blue to purple.
    """
    r, g, b = (int(hex_color[i:i + 2], 16) / 255 for i in (1, 3, 5))
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    if s < 0.25 or v < 0.25:
        return "black"
    deg = h * 360
    if deg < 70 or deg >= 330:
        return "orange"
    if deg < 170:
        return "green"
    return "purple"
