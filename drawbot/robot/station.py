"""Check that the open RoboDK station has what DrawBot needs.

The station's name does not matter. What matters is that the robot, the board frame and the pen tool named in
config/drawing.json exist and belong to the open station, so any station set up with these items works.
"""


def check_station(rdk, settings, extra=()):
    """Return the active station, or raise RuntimeError naming the first missing item.

    extra: more (name, item type) pairs to require, on top of the robot, board frame and pen tool.
    """
    from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL

    station = rdk.ActiveStation()
    if not station.Valid():
        raise RuntimeError("No station is open in RoboDK. Open stations/drawbot.rdk or your own station.")
    required = [(settings["robot_name"], ITEM_TYPE_ROBOT, "Robot"),
                (settings["board_frame"], ITEM_TYPE_FRAME, "Board frame"),
                (settings["tool_name"], ITEM_TYPE_TOOL, "Pen tool")]
    required += [(name, kind, "Item") for name, kind in extra]
    for name, kind, label in required:
        item = rdk.Item(name, kind)
        if not item.Valid() or item.Name() != name:
            raise RuntimeError(f"{label} '{name}' not found in the open station. Check the names in config/drawing.json.")
        parent = item.Parent()
        while parent.Valid() and parent != station:
            parent = parent.Parent()
        if parent != station:
            raise RuntimeError(f"{label} '{name}' belongs to a different station. Close the other station in RoboDK.")
    return station
