import importlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PEN_CHANGER = ROOT / "pen_changer"
STAND_FILE = Path("calibration/pen_stand/ur5e_data.json")
PICKUP_LABELS = ["1", "2", "3", "4"]
SLOT_SPACING_MM = 50.0
SPACING_TOLERANCE_MM = 10.0
NO_PEN = "none"


def module():
    """The pen_changer module, imported from pen_changer/src."""
    src = str(PEN_CHANGER / "src")
    if src not in sys.path:
        sys.path.insert(0, src)
    return importlib.import_module("pen_changer")


PEN_CHECK_WINDOW = (60, 204)
PICK_SPEED_MM_S = 50.0
PICK_JOINT_SPEED_DEG_S = 30.0


DEFAULT_SLOTS = {"green": 0, "black": 1, "purple": 2, "orange": 3}
RGBA = {"black": [0.1, 0.1, 0.1, 1], "green": [0, 0.7, 0, 1], "purple": [0.5, 0.2, 0.7, 1], "orange": [1, 0.55, 0.1, 1]}


def pens(slots=None):
    """{color: slot (0 to 3)}: the Station mode's slots if given, else the default slots."""
    return {color: int(slot) for color, slot in (slots or DEFAULT_SLOTS).items()}


def check_slots(slots):
    """Each slot holds one pen and each colour is in one slot."""
    if sorted(slots.values()) != [0, 1, 2, 3] or len(slots) != 4:
        raise ValueError("Every slot needs a different colour.")


def check_pickup_points(path):
    """Raise ValueError unless points 1 to 4 of a measurement look like a taught pen stand.

    Clicking Continue without moving the arm records 4 times the same pose. Their fit would
    also refuse that, but this message says what to do.
    """
    if not path or not Path(path).is_file():
        raise ValueError("No board measurement yet. Measure the board first: points 1 to 4 teach the pen stand.")
    records = {r.get("label"): r for r in json.loads(Path(path).read_text(encoding="utf-8"))}
    missing = [label for label in PICKUP_LABELS if not (records.get(label) or {}).get("joints")]
    if missing:
        raise ValueError(f"The measurement has no pen stand point {', '.join(missing)}. Measure the board again.")
    poses = [[v * 1000 for v in records[label]["pose"][:3]] for label in PICKUP_LABELS]
    gaps = [math.dist(a, b) for a, b in zip(poses, poses[1:])]
    if any(abs(g - SLOT_SPACING_MM) > SPACING_TOLERANCE_MM for g in gaps):
        raise ValueError("Pen stand points 1 to 4 are not about 50 mm apart ("
                         + ", ".join(f"{g:.0f}" for g in gaps) + " mm). They were not taught at the pens: "
                         "measure again and move the open gripper around each pen before Continue.")
    return gaps


def stand_measurement(root=ROOT):
    """Measurement the pen stand comes from: the newest one with usable points 1 to 4, else STAND_FILE.

    The stand is screwed down, so it need not be taught with every board measurement: clicking
    through points 1 to 4 keeps the stand from the last measurement that taught it.
    """
    for folder in ("logs/calibration", "calibration/runs"):
        for path in sorted((Path(root) / folder).glob("*/ur5e_data.json"), reverse=True):
            try:
                check_pickup_points(path)
                return path
            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                continue
    fixed = Path(root) / STAND_FILE
    return fixed if fixed.is_file() else None


def configure(pickup_file, grip_force=None, slots=None, speed_mm_s=None, joint_speed_deg_s=None):
    """The pen changer module with the station names, the measurement, and the Station mode settings:
    grip force, which colour is in which slot, and the pick and place speeds."""
    pc = module()
    pc.PENS = {color: {"slot": slot, "color": RGBA[color]} for color, slot in pens(slots).items()}
    pc.SPEED_LINEAR = float(speed_mm_s or PICK_SPEED_MM_S)
    pc.SPEED_JOINTS = float(joint_speed_deg_s or PICK_JOINT_SPEED_DEG_S)
    pc.PEN_TIP_TOOL_NAME = "PenTCP"
    pc.PICKUP_DATA_FILE = str(pickup_file)
    pc.CALIBRATION_FILE = str(PEN_CHANGER / "config/stand_calibration.json")
    pc.PEN_CHECK_SHOW_POSITION = False
    pc.DEFAULT_HELD_SLOT = None
    if grip_force is not None:
        pc.GRIPPER_FORCE = int(max(0, min(255, grip_force)))
    pc.gripper_start_code = lambda: gripper_start_code(pc)
    pc.PEN_CHECK_POS_MIN = PEN_CHECK_WINDOW[0]
    pc.PEN_CHECK_POS_MAX = PEN_CHECK_WINDOW[1]
    if not hasattr(pc, "_their_check_pen"):
        pc._their_check_pen = pc.check_pen
    pc.check_pen = lambda color, RDK=None, program=None: check_pen_warning_only(pc, color, RDK, program)
    return pc


def check_pen_warning_only(pc, color, RDK=None, program=None):
    """Their pen check, but it only warns: the popup shows the finger position, Continue goes on drawing,
    Stop Program stops. Their version stopped after the popup in any case. The window is an estimate,
    and the orange pen, held firmly, read 199: a hard stop there cancelled good drawings."""
    captured = []
    add_code = pc.add_code
    pc.add_code = lambda prog, lines: captured.extend(lines)
    try:
        ok = pc._their_check_pen(color, RDK, program)
    finally:
        pc.add_code = add_code
    lines = []
    for i, line in enumerate(captured):
        if line.strip() == "halt" and i > 0 and "not between the fingers" in captured[i - 1]:
            continue
        if "not between the fingers" in line:
            line = line.replace("Check the gripper and the pen stand.", "Continue = draw anyway, Stop Program = stop.")
            line = line.replace("error=True", "warning=True")
        lines.append(line)
    pc.add_code(program, lines)
    return ok


def gripper_start_code(pc):
    """URScript at the start of a pen change: connect, activate the gripper only when it says it is not active.

    Their version read one answer to "GET ACT" after 0.5 s and activated whenever it lacked "ACT 1". That
    answer was often a late "ack" of an earlier command, so the gripper did a full open-close stroke above
    the board, with a pen in the fingers. Here the answer is read until it really contains "ACT", and only
    an explicit "ACT 0" activates, after a popup. Without a clear answer nothing is activated.
    """
    host, port = pc.GRIPPER_SOCKET
    lines = [f'if not socket_open("{host}", {port}, "pen_gripper"):',
             '  popup("Robotiq gripper not reachable (URCap socket). Is the URCap installed and the gripper connected?", "Pen change", error=True, blocking=True)',
             '  halt',
             'end',
             'socket_send_string("GET ACT", "pen_gripper")',
             'socket_send_byte(10, "pen_gripper")',
             'pen_act_reply = ""',
             'pen_act_tries = 0',
             'while str_find(pen_act_reply, "ACT") < 0 and pen_act_tries < 10:',
             '  pen_act_reply = socket_read_string("pen_gripper", timeout=0.5)',
             '  pen_act_tries = pen_act_tries + 1',
             'end',
             'if str_find(pen_act_reply, "ACT 0") >= 0:',
             '  popup("The gripper is not activated. Continue = it activates now: the fingers open and close fully. Hold any pen that is in the gripper.", "Pen change", warning=True, blocking=True)']
    lines += ['  ' + line for line in pc.gripper_command('SET ACT 1')]
    lines += ['  pen_sta_reply = ""',
              '  pen_sta_tries = 0',
              '  while str_find(pen_sta_reply, "STA 3") < 0 and pen_sta_tries < 20:',
              '    sleep(0.5)',
              '    socket_send_string("GET STA", "pen_gripper")',
              '    socket_send_byte(10, "pen_gripper")',
              '    pen_sta_reply = socket_read_string("pen_gripper", timeout=0.5)',
              '    pen_sta_tries = pen_sta_tries + 1',
              '  end',
              'end']
    return (lines + pc.gripper_command('SET GTO 1') + pc.gripper_command(f'SET SPE {pc.GRIPPER_SPEED}')
            + pc.gripper_command(f'SET FOR {pc.GRIPPER_FORCE}'))


def quiet_collisions(rdk):
    """Collision pairs that are contact by design, switched off.

    The V-groove pad sits on a finger, and the pens are held by the fingers or rest in the stand.
    Kept on: the pad and the gripper against the stand, the board and the robot.
    """
    from robodk.robolink import COLLISION_OFF, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT
    pc = module()
    gripper = rdk.Item(pc.GRIPPER_NAME, ITEM_TYPE_ROBOT)
    pad = rdk.Item(pc.GRIPPER_ATTACHMENT_NAME, ITEM_TYPE_OBJECT)
    pen_items = [rdk.Item(f"Pen {color}", ITEM_TYPE_OBJECT) for color in pens()]
    pen_items = [p for p in pen_items if p.Valid()]
    robots = [r for r in rdk.ItemList(ITEM_TYPE_ROBOT)]
    objects = [o for o in rdk.ItemList(ITEM_TYPE_OBJECT)]

    def off(a, b):
        ids_a = range(8) if a.Type() == ITEM_TYPE_ROBOT else [0]
        ids_b = range(8) if b.Type() == ITEM_TYPE_ROBOT else [0]
        for i in ids_a:
            for j in ids_b:
                rdk.setCollisionActivePair(COLLISION_OFF, a, b, i, j)

    if pad.Valid() and gripper.Valid():
        off(pad, gripper)
    for pen in pen_items:
        for other in robots + objects:
            if other != pen:
                off(pen, other)


def off_by_design(arm_name, object_name):
    """True for a robot / object pair that quiet_collisions() switches off on purpose: a pen (in the
    fingers or in the stand, any colour, also from an earlier colour set) or the pad on its finger.
    The collision map check does not require these. The stand model stays required."""
    pc = module()
    is_pen = object_name.startswith("Pen ") and not object_name.startswith(pc.STAND_FRAME_NAME)
    return is_pen or (arm_name == pc.GRIPPER_NAME and object_name == pc.GRIPPER_ATTACHMENT_NAME)


def show_stand(rdk, pickup_file, held=None, slots=None):
    """Place the stand and the pens in RoboDK from a measurement. The robot does not move."""
    check_pickup_points(pickup_file)
    pc = configure(pickup_file, slots=slots)
    position = pc.show_stand(None, held, rdk)
    quiet_collisions(rdk)
    return position
