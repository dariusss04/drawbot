from robodk import robolink, robomath
from types import SimpleNamespace
import argparse
import json
import math
import os
import time

REPO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
STAND_FILE = "3D_Models/pen_stand.stl"
PEN_FILE = "3D_Models/marker_casing.stl"
GRIPPER_ATTACHMENT_FILE = "3D_Models/gripper_finger.stl"
PENS_FILE = "config/pens.json"
PICKUP_DATA_FILE = "config/ur5e_data.json"
CALIBRATION_FILE = "config/stand_calibration.json"

STAND_FILE, PEN_FILE, GRIPPER_ATTACHMENT_FILE, PENS_FILE, PICKUP_DATA_FILE, CALIBRATION_FILE = (
    os.path.normpath(os.path.join(REPO_DIR, path))
    for path in (STAND_FILE, PEN_FILE, GRIPPER_ATTACHMENT_FILE, PENS_FILE, PICKUP_DATA_FILE, CALIBRATION_FILE))

ROBOT_NAME = 'UR5e'
GRIPPER_NAME = 'RobotiQ Hand-E Gripper'

STAND_FRAME_NAME = 'Pen Stand'
STAND_CAD_POSE = robomath.roty(robomath.pi) * robomath.transl(0, 0, 52)

SLOTS = [[0, 0], [0, 50], [0, 100], [0, 150]]
PEN_SEAT_DEPTH = 52.0

PICKUP_TARGETS = ['1', '2', '3', '4']
GRASP_ROTATION_WARN = 10.0
STRAIGHTEN_TAUGHT_PENS = True
PEN_ALIGN_HEIGHT = 22.0
CALIBRATION_DISTANCE_TOLERANCE = 10.0
CALIBRATION_LEVEL_TOLERANCE = 3.0

with open(PENS_FILE) as f:
    PENS = json.load(f)

APPROACH = 100.0
PLACE_DROP_HEIGHT = 5.0

PEN_LENGTH = 127.0
PEN_TIP_TOOL_NAME = 'Pen_TCP'
PEN_GRASP_HEIGHT = 126.0
PEN_GRASP_HEIGHT_OVERRIDE = None

GRIPPER_TOOL_NAME = 'Gripper'
GRIPPER_ATTACHMENT_NAME = 'Gripper Attachment'
GRIPPER_ATTACHMENT_POSE = robomath.rotz(robomath.pi / 2) * robomath.transl(-52.9, 49.3, -139)
GRIPPER_ATTACHMENT_OFFSET = [0.0, 3.6, -15.8]
GRIPPER_STEPS = 15
GRIPPER_ANIMATION_TIME = 0.3
GRIPPER_OPEN = 50.0
GRIPPER_CLOSED = 24.9
GRIPPER_ROTATION = 180.0
WAIT_TIME = 0.5

GRIPPER_SOCKET = ('127.0.0.1', 63352)
GRIPPER_SPEED = 128
GRIPPER_FORCE = 50
GRIPPER_MOVE_TIME = 1.5

PROGRAMS_DIR = os.path.normpath(os.path.join(REPO_DIR, "programs"))
PROGRAM_NAME = 'ChangePen'
ROBOT_IP = '192.168.0.100'

CHECK_PEN = True
PEN_CHECK_POS_MIN = 110
PEN_CHECK_POS_MAX = 160
PEN_CHECK_SHOW_POSITION = False
PEN_HOLDER_Z = (-20.8, 27.1)
PEN_CHECK_MIN_OVERLAP = 10.0
PEN_CHECK_MAX_OFF_AXIS = 3.0

SPEED_LINEAR = 100.0
SPEED_JOINTS = 30.0
ACCEL_LINEAR = 500.0
ACCEL_JOINTS = 60.0

DEFAULT_STAND_POSITION = [400, -75, 62, 0]

SIMULATION_SPEED = 1.0
NO_PEN = 'none'
HELD_PEN_PARAM = 'PenHeld'
DEFAULT_HELD_SLOT = 0

def find_item(RDK, name, item_type):
    """Item with exactly this name (RDK.Item() falls back to a similar name), None if there is none."""
    item = RDK.Item(name, item_type)
    return item if item.Valid() and item.Name() == name else None


def gripper_tcp_pose(RDK):
    """TCP of the gripper model in the robot flange frame (between the finger tips)."""
    gripper = RDK.Item(GRIPPER_NAME, robolink.ITEM_TYPE_ROBOT)
    if not gripper.Valid():
        raise RuntimeError(f'Gripper {GRIPPER_NAME} not found')
    return gripper.SolveFK(gripper.Joints())


def stand_position_from_pen_tips(pen_tips):
    """Stand position [x, y, z, rz] from the pen tip positions {slot: [x, y, z, ...]} (in "UR5e Base") of pens seated in 2 or more slots."""
    if len(pen_tips) < 2:
        raise ValueError(f'Expected pen tip positions for at least 2 slots, got {len(pen_tips)}')
    tips = sorted((int(slot), list(pos[:3])) for slot, pos in pen_tips.items())
    (slot_a, a), (slot_b, b) = tips[0], tips[-1]

    distance_cad = math.dist(SLOTS[slot_a], SLOTS[slot_b])
    distance = math.dist(a[:2], b[:2])
    if abs(distance - distance_cad) > CALIBRATION_DISTANCE_TOLERANCE:
        raise ValueError(f'The measured positions are {distance:.1f} mm apart, slot {slot_a} and slot {slot_b} '
                         f'are {distance_cad:.1f} mm apart. Were the right slots measured?')
    heights = [tip[2] for _, tip in tips]
    if max(heights) - min(heights) > CALIBRATION_LEVEL_TOLERANCE:
        raise ValueError(f'The measured positions differ by {max(heights) - min(heights):.1f} mm in height. '
                         f'Is the stand level, and were all pens fully seated?')

    dx, dy = b[0] - a[0], b[1] - a[1]
    cad_dx, cad_dy = SLOTS[slot_b][0] - SLOTS[slot_a][0], SLOTS[slot_b][1] - SLOTS[slot_a][1]
    rz = math.degrees(math.atan2(dy, dx) - math.atan2(cad_dy, cad_dx))
    rz = (rz + 180) % 360 - 180

    z = sum(heights) / len(heights) + PEN_SEAT_DEPTH

    def offset(slot):
        return (robomath.rotz(math.radians(rz)) * robomath.transl(SLOTS[slot][0], SLOTS[slot][1], 0)).Pos()
    x = sum(tip[0] - offset(slot)[0] for slot, tip in tips) / len(tips)
    y = sum(tip[1] - offset(slot)[1] for slot, tip in tips) / len(tips)

    for slot, tip in tips:
        error = math.dist(tip[:2], [x + offset(slot)[0], y + offset(slot)[1]])
        if error > CALIBRATION_DISTANCE_TOLERANCE:
            raise ValueError(f'The position of slot {slot} is {error:.1f} mm away from where the stand puts it. Was it measured at the right slot?')
    return [round(x, 2), round(y, 2), round(z, 2), round(rz, 3)]


def taught_joints(RDK):
    """Joints {slot: [j1 ... j6] in deg} of the pickup points taught during calibration and where they come from:
    PICKUP_DATA_FILE if it exists, else the joint targets PICKUP_TARGETS in the station."""
    if os.path.exists(PICKUP_DATA_FILE):
        with open(PICKUP_DATA_FILE) as f:
            records = {r['label']: r for r in json.load(f)}
        joints = {slot: [math.degrees(j) for j in records[label]['joints']]
                  for slot, label in enumerate(PICKUP_TARGETS) if label in records and records[label].get('joints')}
        return joints, os.path.basename(PICKUP_DATA_FILE)

    joints = {}
    for slot, name in enumerate(PICKUP_TARGETS):
        target = find_item(RDK, name, robolink.ITEM_TYPE_TARGET)
        if target is not None:
            joints[slot] = target.Joints().list()
    return joints, 'the station'


def taught_pen_tcp():
    """Pen tip TCP [x, y, z] (mm, in the flange frame) from the "tcp-pen" records in PICKUP_DATA_FILE, None if there are none."""
    if not os.path.exists(PICKUP_DATA_FILE):
        return None
    with open(PICKUP_DATA_FILE) as f:
        for record in json.load(f):
            if record.get('tcp') == 'tcp-pen' and record.get('tcp_offset'):
                return [v * 1000 for v in record['tcp_offset'][:3]]
    return None


def align_wrist(robot, joints, tcp, orientation):
    """Joints with only the last joint (wrist 3) turned, so the gripper TCP is rotated around its Z axis like orientation
    (pose in "UR5e Base", only its X axis = finger direction is used). The gripper TCP lies on the wrist axis, so its position
    does not change, joints 1 to 5 stay as taught. Returns (joints, rotation in deg)."""
    pose = robot.SolveFK(joints) * tcp
    x_wanted = (robomath.invH(pose) * orientation).VX()
    angle = math.degrees(math.atan2(x_wanted[1], x_wanted[0]))
    joints = list(joints)
    joints[5] += angle
    lower, upper, _ = robot.JointLimits()
    while joints[5] > upper.list()[5]:
        joints[5] -= 360
    while joints[5] < lower.list()[5]:
        joints[5] += 360
    return joints, angle


def straighten_grasp(robot, joints, tcp, orientation, tip_offset):
    """Joints of a vertical grasp for a pen taught slightly tilted (see STRAIGHTEN_TAUGHT_PENS): the point PEN_ALIGN_HEIGHT above
    the tip on the taught pen axis is kept, the gripper TCP goes vertically above it with the orientation (pose in "UR5e Base",
    gripper pointing down, fingers across the row). Joints closest to the taught ones (same arm configuration).
    Returns (joints, tilt of the taught pose in deg)."""
    taught = robot.SolveFK(joints) * tcp
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -taught.VZ()[2]))))

    to_point = tip_offset[2] - PEN_ALIGN_HEIGHT
    point = (taught * robomath.transl(tip_offset[0], tip_offset[1], to_point)).Pos()
    at_point = orientation.copy()
    at_point.setPos(point)
    grasp = at_point * robomath.transl(-tip_offset[0], -tip_offset[1], -to_point)

    vertical = robot.SolveIK(grasp * robomath.invH(tcp), joints)
    if len(vertical.list()) < 6:
        raise RuntimeError('A taught pickup point cannot be reached vertically. Set STRAIGHTEN_TAUGHT_PENS = False')
    return vertical.list(), tilt


def load_calibration(RDK, stand_position, tip_offset):
    """Stand position [x, y, z, rz] and taught grasp joints {slot: [j1 ... j6]} (may be empty): the pickup points,
    with the last joint turned so the fingers are across the row of pens.
    stand_position given: used as is, without taught poses. None: from the pickup points in the station (at least 2),
    else from CALIBRATION_FILE, else DEFAULT_STAND_POSITION."""
    if stand_position is not None:
        return list(stand_position), {}

    joints, source = taught_joints(RDK)
    if len(joints) >= 2:
        robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
        tcp = gripper_tcp_pose(RDK)
        grasps = {slot: robot.SolveFK(j) * tcp for slot, j in joints.items()}
        pen_tips = {slot: (pose * robomath.transl(*tip_offset)).Pos() for slot, pose in grasps.items()}
        print(f'Stand from the pickup points {[PICKUP_TARGETS[slot] for slot in joints]} in {source}')
        stand_position = stand_position_from_pen_tips(pen_tips)

        orientation = stand_pose(stand_position) * ORIENT_TOOL_DOWN
        grasp_joints = {}
        for slot, j in joints.items():
            if STRAIGHTEN_TAUGHT_PENS:
                grasp_joints[slot], tilt = straighten_grasp(robot, j, tcp, orientation, tip_offset)
                if tilt > 0.5:
                    print(f'Pickup point {PICKUP_TARGETS[slot]} was taught {tilt:.1f} deg tilted, gripped vertically')
            else:
                grasp_joints[slot], angle = align_wrist(robot, j, tcp, orientation)
                if abs(angle) > GRASP_ROTATION_WARN:
                    print(f'Pickup point {PICKUP_TARGETS[slot]} was taught with the gripper turned by {-angle:.0f} deg, corrected with the last joint')

        if STRAIGHTEN_TAUGHT_PENS:
            pen_tips = {slot: (robot.SolveFK(j) * tcp * robomath.transl(*tip_offset)).Pos() for slot, j in grasp_joints.items()}
            stand_position = stand_position_from_pen_tips(pen_tips)
        return stand_position, grasp_joints
    if joints:
        print(f'Only {len(joints)} pickup point in {source}, at least 2 are needed. Not used')

    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE) as f:
            calibration = json.load(f)
        print(f'Stand from {CALIBRATION_FILE}')
        return stand_position_from_pen_tips(calibration['pen_tips']), {}

    print(f'Stand not calibrated (no pickup points, no {CALIBRATION_FILE}), using {DEFAULT_STAND_POSITION}')
    return list(DEFAULT_STAND_POSITION), {}


def pen_tip_offset(RDK):
    """Position [x, y, z] of the pen tip in the gripper TCP frame: from the "tcp-pen" TCP in PICKUP_DATA_FILE (the current
    calibration), else the PEN_TIP_TOOL_NAME tool in the station, else [0, 0, PEN_GRASP_HEIGHT]. PEN_GRASP_HEIGHT_OVERRIDE
    replaces z. Z is the distance from where the gripper holds the pen to the tip."""
    pen_tcp = taught_pen_tcp()
    tip_tool = find_item(RDK, PEN_TIP_TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    if pen_tcp is not None:
        offset = (robomath.invH(gripper_tcp_pose(RDK)) * robomath.transl(*pen_tcp)).Pos()
    elif tip_tool is not None:
        offset = (robomath.invH(gripper_tcp_pose(RDK)) * tip_tool.PoseTool()).Pos()
    else:
        print(f'No tcp-pen in {PICKUP_DATA_FILE} and no "{PEN_TIP_TOOL_NAME}" tool in the station, using PEN_GRASP_HEIGHT = {PEN_GRASP_HEIGHT} mm')
        offset = [0, 0, PEN_GRASP_HEIGHT]
    if PEN_GRASP_HEIGHT_OVERRIDE is not None:
        print(f'PEN_GRASP_HEIGHT_OVERRIDE: gripping {PEN_GRASP_HEIGHT_OVERRIDE} mm above the pen tip instead of {offset[2]:.1f} mm')
        offset = [offset[0], offset[1], PEN_GRASP_HEIGHT_OVERRIDE]
    return offset


ORIENT_TOOL_DOWN = robomath.rotx(robomath.pi) * robomath.rotz(GRIPPER_ROTATION * robomath.pi / 180)


def stand_pose(stand_position):
    """Pose of the stand frame in "UR5e Base" from [x, y, z, rz]."""
    x, y, z, rz = stand_position
    return robomath.transl(x, y, z) * robomath.rotz(rz * robomath.pi / 180)


def slot_pose(slot):
    """Pose of the pen tip in a slot, in the stand frame (Z up)."""
    x, y = SLOTS[slot]
    return robomath.transl(x, y, -PEN_SEAT_DEPTH)


def pen_in_slot_pose(slot):
    """Pose of the pen object standing on its tip in a slot (the CAD Z axis points towards the tip)."""
    return slot_pose(slot) * robomath.transl(0, 0, PEN_LENGTH) * robomath.rotx(robomath.pi)


def seated_pen_pose(slot, tip_offset, grasps):
    """Pose of the pen object seated in a slot, in the stand frame. Taught slot: where the gripper held it when teaching
    (the pen was seated while gripped), so it hangs centered in the gripper. Otherwise: upright in the CAD slot."""
    if slot not in grasps:
        return pen_in_slot_pose(slot)
    target, _ = grasp_targets(slot, tip_offset, {})
    held = robomath.invH(target) * pen_in_slot_pose(slot)
    return grasps[slot] * held


def grasp_targets(slot, tip_offset, grasps):
    """Grasp pose (gripper TCP, pointing down) and the approach pose above it, in the stand frame.
    grasps: taught grasp poses {slot: pose in the stand frame}, used if the slot is in it. Otherwise the gripper TCP is placed
    so that the pen tip (tip_offset in the gripper TCP frame, see pen_tip_offset()) is on the seat."""
    if slot in grasps:
        target = grasps[slot]
    else:
        x, y, z = tip_offset
        target = slot_pose(slot) * ORIENT_TOOL_DOWN * robomath.transl(-x, -y, -z)
    target_app = target * robomath.transl(0, 0, -APPROACH)
    return target, target_app


def setup_station(RDK, stand_position, current_color, tip_offset, grasp_joints):
    """Get the robot and the gripper, create / update the stand and the pens in the station.
    The pen current_color is put into the gripper, all other pens into their slots."""
    robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f'Robot {ROBOT_NAME} not found')
    base = robot.Parent()

    gripper = RDK.Item(GRIPPER_NAME, robolink.ITEM_TYPE_ROBOT)
    if not gripper.Valid():
        raise RuntimeError(f'Gripper {GRIPPER_NAME} not found')

    show_frames = RDK.getParam('FramesHidden') != 'True'

    gripper_tcp = gripper_tcp_pose(RDK)
    tool = RDK.Item(GRIPPER_TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    if not tool.Valid():
        tool = robot.AddTool(gripper_tcp, GRIPPER_TOOL_NAME)
    tool.setPoseTool(gripper_tcp)
    tool.setVisible(show_frames)

    attachment = RDK.Item(GRIPPER_ATTACHMENT_NAME, robolink.ITEM_TYPE_OBJECT)
    if not attachment.Valid():
        attachment = RDK.AddFile(GRIPPER_ATTACHMENT_FILE, tool)
        if not attachment.Valid():
            raise RuntimeError(f'Could not load {GRIPPER_ATTACHMENT_FILE}')
        attachment.setName(GRIPPER_ATTACHMENT_NAME)
    attachment.setParent(tool)
    attachment.setVisible(True, show_frames)

    gripper.setJoints([GRIPPER_CLOSED if current_color in PENS else GRIPPER_OPEN])
    attachment.setPose(attachment_pose(gripper.Joints().list()[0]))

    frame = RDK.Item(STAND_FRAME_NAME, robolink.ITEM_TYPE_FRAME)
    if not frame.Valid():
        frame = RDK.AddFrame(STAND_FRAME_NAME, base)
    frame.setParent(base)
    frame.setPose(stand_pose(stand_position))
    frame.setVisible(show_frames)
    grasps = {slot: robomath.invH(stand_pose(stand_position)) * robot.SolveFK(j) * gripper_tcp
              for slot, j in grasp_joints.items()}

    stand_model_name = f'{STAND_FRAME_NAME} Model'
    stand = RDK.Item(stand_model_name, robolink.ITEM_TYPE_OBJECT)
    if not stand.Valid():
        stand = RDK.AddFile(STAND_FILE, frame)
        if not stand.Valid():
            raise RuntimeError(f'Could not load {STAND_FILE}')
        stand.setName(stand_model_name)
    stand.setParent(frame)
    stand.setPose(STAND_CAD_POSE)
    stand.setVisible(True, show_frames)

    pens = {}
    for color, pen_data in PENS.items():
        pen_name = f'Pen {color}'
        pen = RDK.Item(pen_name, robolink.ITEM_TYPE_OBJECT)
        if not pen.Valid():
            pen = RDK.AddFile(PEN_FILE, frame)
            if not pen.Valid():
                raise RuntimeError(f'Could not load {PEN_FILE}')
            pen.setName(pen_name)
        pen.setColor(pen_data['color'])
        pen.setVisible(True, show_frames)

        slot = pen_data['slot']
        if color == current_color:
            target, _ = grasp_targets(slot, tip_offset, grasps)
            pen.setParent(tool)
            pen.setPose(robomath.invH(target) * seated_pen_pose(slot, tip_offset, grasps))
        else:
            pen.setParent(frame)
            pen.setPose(seated_pen_pose(slot, tip_offset, grasps))

        pens[color] = pen

    return SimpleNamespace(RDK=RDK, robot=robot, gripper=gripper, tool=tool, attachment=attachment, frame=frame, pens=pens,
                           tip_offset=tip_offset, grasps=grasps, grasp_joints=grasp_joints)


def attachment_pose(opening):
    """Pose of the gripper attachment in the gripper TCP frame for a finger opening (mm). It sits on one finger,
    whose inner face is opening / 2 away from the gripper axis."""
    dx, dy, dz = GRIPPER_ATTACHMENT_OFFSET
    return GRIPPER_ATTACHMENT_POSE * robomath.transl(dx, dy + (opening - 26.0) / 2, dz)


def at_end(program):
    """Select the last instruction of the program: RoboDK adds new instructions after the selected one,
    which otherwise moves around (instructions end up in the wrong order)."""
    program.InstructionSelect(program.InstructionCount() - 1)
    return program


def add_code(program, lines):
    """Add URScript lines to the program for the real robot (nothing if program is None)."""
    if program is not None:
        for line in lines:
            at_end(program).RunInstruction(line, robolink.INSTRUCTION_INSERT_CODE)


def gripper_command(command, reply='pen_gripper_reply'):
    """URScript lines: send a command to the Robotiq URCap socket and read its answer into the variable reply.
    The URCap answers without a newline, so the reply is read with a timeout (as in the teach script)."""
    return [f'socket_send_string("{command}", "pen_gripper")',
            'socket_send_byte(10, "pen_gripper")',
            f'{reply} = socket_read_string("pen_gripper", timeout=0.5)']


def gripper_start_code():
    """URScript lines at the start of a pen change: open the URCap socket, activate the gripper only if needed
    (activating moves the fingers through a full stroke and drops a held pen), set speed and force."""
    host, port = GRIPPER_SOCKET
    return ([f'if not socket_open("{host}", {port}, "pen_gripper"):',
             '  popup("Robotiq gripper not reachable (URCap socket). Is the URCap installed and the gripper connected?", "Pen change", error=True, blocking=True)',
             '  halt',
             'end']
            + gripper_command('GET ACT')
            + ['if str_find(pen_gripper_reply, "ACT 1") < 0:',
               '  popup("The gripper is not activated. Activating moves the fingers through a full stroke and drops a held pen. Continue when safe.", "Pen change", warning=True, blocking=True)']
            + ['  ' + line for line in gripper_command('SET ACT 1')]
            + ['  sleep(3.0)',
               'end']
            + gripper_command('SET GTO 1') + gripper_command(f'SET SPE {GRIPPER_SPEED}') + gripper_command(f'SET FOR {GRIPPER_FORCE}'))


def set_gripper(st, opening, close):
    """Open / close the gripper: animate it in the simulation (in steps, the attachment follows its finger),
    and add the command for the real robot to the program."""
    lower, upper, _ = st.gripper.JointLimits()
    start = st.gripper.Joints().list()[0]
    steps = GRIPPER_STEPS if st.animate else 1
    for i in range(1, steps + 1):
        value = start + (opening - start) * i / steps
        value = min(max(value, lower.list()[0]), upper.list()[0])
        st.gripper.setJoints([value])
        st.attachment.setPose(attachment_pose(value))
        if st.animate:
            time.sleep(GRIPPER_ANIMATION_TIME / steps)
    add_code(st.program, gripper_command('SET POS 255' if close else 'SET POS 0') + [f'sleep({GRIPPER_MOVE_TIME})'])
    if st.animate:
        st.robot.Pause(WAIT_TIME * 1000)
    if st.program is not None:
        at_end(st.program).Pause(WAIT_TIME * 1000)


def move(st, kind, target):
    """MoveJ ('J') or MoveL ('L') in the simulation, and the same move in the program for the real robot.
    Without animation the simulated robot jumps to the target."""
    if st.animate:
        st.robot.MoveJ(target) if kind == 'J' else st.robot.MoveL(target)
    elif hasattr(target, 'size') and target.size(0) == 4 and target.size(1) == 4:
        flange = st.frame.Pose() * target * robomath.invH(st.tool.PoseTool())
        st.robot.setJoints(st.robot.SolveIK(flange, st.robot.Joints()))
    else:
        st.robot.setJoints(target)
    if st.program is None:
        return
    if kind == 'J':
        at_end(st.program).MoveJ(target)
        return
    if not (hasattr(target, 'size') and target.size(0) == 4 and target.size(1) == 4):
        target = robomath.invH(st.frame.Pose()) * st.robot.SolveFK(target) * st.tool.PoseTool()
    at_end(st.program).MoveL(target)


def slot_targets(st, slot, lift=0.0):
    """Grasp target (lift mm above the grasp position, along the tool axis) and approach target for a slot.
    Taught slot: joints, so the robot keeps the taught arm configuration (only the last joint corrected, see align_wrist()).
    Otherwise: poses in the stand frame."""
    target, target_app = grasp_targets(slot, st.tip_offset, st.grasps)
    target = target * robomath.transl(0, 0, -lift)
    if slot not in st.grasp_joints:
        return target, target_app

    taught = st.grasp_joints[slot]

    def joints_near_taught(pose):
        flange = st.frame.Pose() * pose * robomath.invH(st.tool.PoseTool())
        joints = st.robot.SolveIK(flange, taught)
        if len(joints.list()) < 6:
            raise RuntimeError(f'Slot {slot}: the robot cannot reach the position {APPROACH} mm above the pickup point')
        return joints

    return (taught if lift == 0 else joints_near_taught(target)), joints_near_taught(target_app)


def pick(st, color):
    slot = PENS[color]['slot']
    target, target_app = slot_targets(st, slot)
    set_gripper(st, GRIPPER_OPEN, close=False)
    move(st, 'J', target_app)
    move(st, 'L', target)
    set_gripper(st, GRIPPER_CLOSED, close=True)
    st.pens[color].setParentStatic(st.tool)
    move(st, 'L', target_app)


def check_pen(color, RDK=None, program=None):
    """Check that the pen color is still between the fingers, e.g. after a pen change or before drawing.
    Real robot (added to program): reads the finger position, a popup and halt if it is outside PEN_CHECK_POS_MIN ... MAX
    (the fingers closed further than on the pen, or are open). The URCap socket must be open (see gripper_start_code()).
    Simulation: checks that the pen object is held centered between the fingers.
    Returns False if the simulation finds a problem (the real check runs on the robot and stops it there)."""
    if RDK is None:
        RDK = robolink.Robolink()

    lines = ['socket_send_string("GET POS", "pen_gripper")',
             'socket_send_byte(10, "pen_gripper")',
             'pen_check_reply = ""',
             'pen_check_tries = 0',
             'while str_find(pen_check_reply, "POS ") < 0 and pen_check_tries < 10:',
             '  pen_check_reply = socket_read_string("pen_gripper", timeout=0.5)',
             '  pen_check_tries = pen_check_tries + 1',
             'end',
             'pen_check_pos = -1',
             'pen_check_index = str_find(pen_check_reply, "POS ")',
             'if pen_check_index >= 0:',
             '  pen_check_pos = 0',
             '  pen_check_index = pen_check_index + 4',
             '  while pen_check_index < str_len(pen_check_reply):',
             '    pen_check_digit = str_find("0123456789", str_at(pen_check_reply, pen_check_index))',
             '    if pen_check_digit < 0:',
             '      break',
             '    end',
             '    pen_check_pos = pen_check_pos * 10 + pen_check_digit',
             '    pen_check_index = pen_check_index + 1',
             '  end',
             'end',
             'if pen_check_pos < 0:',
             '  popup(str_cat("Pen check: could not read the finger position from the gripper. Answer: ", pen_check_reply), "Pen check", warning=True, blocking=True)',
             'end']
    if PEN_CHECK_SHOW_POSITION:
        lines.append('popup(str_cat("Gripper position: ", pen_check_pos), "Pen check", blocking=True)')
    lines += [f'if pen_check_pos >= 0 and (pen_check_pos < {PEN_CHECK_POS_MIN} or pen_check_pos > {PEN_CHECK_POS_MAX}):',
              f'  popup(str_cat("The {color} pen is not between the fingers (gripper position ", str_cat(pen_check_pos, '
              f'"). Check the gripper and the pen stand.")), "Pen check", error=True, blocking=True)',
              '  halt',
              'end']
    add_code(program, lines)

    problem = pen_in_fingers_problem(RDK, color)
    if problem:
        print(f'Warning: {problem}')
        RDK.ShowMessage(f"Pen check: {problem}", False)
        return False
    return True


def pen_in_fingers_problem(RDK, color):
    """Simulation part of check_pen(): description of what is wrong, None if the pen is held correctly."""
    pen = RDK.Item(f'Pen {color}', robolink.ITEM_TYPE_OBJECT)
    tool = RDK.Item(GRIPPER_TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    gripper = RDK.Item(GRIPPER_NAME, robolink.ITEM_TYPE_ROBOT)
    if not pen.Valid() or not tool.Valid() or pen.Parent() != tool:
        return f'the {color} pen is not in the gripper'

    opening = gripper.Joints().list()[0]
    position = (GRIPPER_OPEN - opening) / GRIPPER_OPEN * 255
    if not PEN_CHECK_POS_MIN <= position <= PEN_CHECK_POS_MAX:
        return f'finger opening {opening:.1f} mm (position ~{position:.0f}) is outside {PEN_CHECK_POS_MIN} ... {PEN_CHECK_POS_MAX}'

    pose = pen.Pose()
    top, bottom = (pose * [0, 0, PEN_HOLDER_Z[0]]), (pose * [0, 0, PEN_HOLDER_Z[1]])
    off_axis = max(math.hypot(top[0], top[1]), math.hypot(bottom[0], bottom[1]))
    if off_axis > PEN_CHECK_MAX_OFF_AXIS:
        return f'the {color} pen is {off_axis:.1f} mm off the gripper axis'

    groove_low, groove_high = attachment_z_range(opening)
    overlap = min(max(top[2], bottom[2]), groove_high) - max(min(top[2], bottom[2]), groove_low)
    if overlap < PEN_CHECK_MIN_OVERLAP:
        return f'the holder of the {color} pen overlaps the V-groove only {max(overlap, 0):.1f} mm (min. {PEN_CHECK_MIN_OVERLAP})'
    return None


def attachment_z_range(opening):
    """Lowest and highest point of the gripper attachment along the TCP Z axis (mm), from its CAD file."""
    import struct
    with open(GRIPPER_ATTACHMENT_FILE, 'rb') as f:
        data = f.read()
    count = struct.unpack('<I', data[80:84])[0]
    pose = attachment_pose(opening)
    z = [(pose * list(struct.unpack('<3f', data[84 + i * 50 + 12 + k * 12:84 + i * 50 + 24 + k * 12])))[2]
         for i in range(count) for k in range(3)]
    return min(z), max(z)


def place(st, color):
    slot = PENS[color]['slot']
    target_release, target_app = slot_targets(st, slot, PLACE_DROP_HEIGHT)
    move(st, 'J', target_app)
    move(st, 'L', target_release)
    set_gripper(st, GRIPPER_OPEN, close=False)
    st.pens[color].setParentStatic(st.frame)
    st.pens[color].setPose(seated_pen_pose(slot, st.tip_offset, st.grasps))
    move(st, 'L', target_app)


def change_pen(stand_position, current_color, new_color, RDK=None, program=None, animate=True):
    """Put the pen the robot is holding back into the stand, pick up the new one and return to the start position.

    stand_position: [x, y, z, rz] of the stand in the "UR5e Base" frame (mm, deg), see "Pen stand" in the settings.
                    None: from the calibration, i.e. the pickup points taught during calibration (the robot then grips exactly
                    there), else config/stand_calibration.json, see load_calibration()
    current_color:  color of the pen in the gripper (name from config/pens.json), None if the gripper is empty
    new_color:      color of the pen to pick up, None to end with an empty gripper
    RDK:            RoboDK connection of the calling script, None to connect here
    program:        RoboDK program item to also write the pen change into, for the real robot (see make_program()).
                    None: simulation only. Your own moves can go into the same program before and after
    animate:        False: the simulated robot jumps to each target instead of moving (fast, e.g. to only make a program)

    Returns the color the robot is holding afterwards (= new_color).
    The active reference frame and tool of the robot are restored afterwards, the speed is not: set your own before moving again.
    """
    for color in (current_color, new_color):
        if color is not None and color not in PENS:
            raise ValueError(f'Unknown pen {color}, available: {list(PENS)}')

    if RDK is None:
        RDK = robolink.Robolink()
    tip_offset = pen_tip_offset(RDK)
    stand_position, grasp_joints = load_calibration(RDK, stand_position, tip_offset)
    st = setup_station(RDK, stand_position, current_color, tip_offset, grasp_joints)
    st.program = program
    st.animate = animate
    if current_color == new_color:
        return new_color

    start_joints = st.robot.Joints()
    start_frame = st.robot.getLink(robolink.ITEM_TYPE_FRAME)
    start_tool = st.robot.getLink(robolink.ITEM_TYPE_TOOL)

    for item in (st.robot, program):
        if item is not None:
            if item == program:
                at_end(program)
            item.setPoseFrame(st.frame)
            item.setPoseTool(st.tool)
            item.setSpeed(SPEED_LINEAR, SPEED_JOINTS, ACCEL_LINEAR, ACCEL_JOINTS)
    if program is not None:
        at_end(program).MoveJ(start_joints)
        add_code(program, gripper_start_code())

    try:
        if current_color is not None:
            RDK.ShowMessage(f"Putting the {current_color} pen back", False)
            place(st, current_color)
            RDK.setParam(HELD_PEN_PARAM, NO_PEN)

        if new_color is not None:
            RDK.ShowMessage(f"Picking the {new_color} pen", False)
            pick(st, new_color)
            RDK.setParam(HELD_PEN_PARAM, new_color)

        move(st, 'J', start_joints)
        if new_color is not None and CHECK_PEN:
            check_pen(new_color, RDK, program)
        add_code(program, ['socket_close("pen_gripper")'])
    finally:
        for item in (st.robot, program):
            if item is not None:
                if item == program:
                    at_end(program)
                if start_frame.Valid():
                    item.setPoseFrame(start_frame)
                if start_tool.Valid():
                    item.setPoseTool(start_tool)

    return new_color


def show_stand(stand_position=None, current_color=None, RDK=None):
    """Update the simulation without moving the robot: stand and pens at the calibrated position
    (stand_position None: from the calibration, see load_calibration()), the pen current_color in the gripper. Returns the stand position."""
    if RDK is None:
        RDK = robolink.Robolink()
    tip_offset = pen_tip_offset(RDK)
    stand_position, grasp_joints = load_calibration(RDK, stand_position, tip_offset)
    setup_station(RDK, stand_position, current_color, tip_offset, grasp_joints)
    return stand_position


def new_program(RDK, name=PROGRAM_NAME):
    """Empty RoboDK program item for the UR5e (an older one with the same name is replaced). Pass it to change_pen()."""
    old = find_item(RDK, name, robolink.ITEM_TYPE_PROGRAM)
    if old is not None:
        old.Delete()
    return RDK.AddProgram(name, RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT))


def save_program(program, folder=PROGRAMS_DIR):
    """Write the program as URScript (<name>.script, plus .urp) into folder and remove the program item. Returns the .script path.
    Uses MakeProgram() on the program item: setRunMode(RUNMODE_MAKE_ROBOTPROG) + ProgramStart() crashes RoboDK 6.0.6 on Linux."""
    name = program.Name()
    os.makedirs(folder, exist_ok=True)
    ok, log, _ = program.MakeProgram(folder)
    program.Delete()
    path = os.path.join(folder, f'{name}.script')
    if not ok or not os.path.exists(path):
        raise RuntimeError(f'Could not generate the program: {log}')
    return path


def send_program(path, robot_ip=ROBOT_IP):
    """Send a URScript file to the robot's primary interface (port 30002): the robot runs it at once (as calibration/send_script.py).
    The robot must be in remote control or local mode with the program not running, see the pendant."""
    import socket
    with open(path) as f:
        script = f.read()
    with socket.create_connection((robot_ip, 30002), timeout=5) as connection:
        connection.sendall(script.encode('utf-8') + b'\n')


def held_pen(RDK):
    """Color of the pen in the gripper as remembered in the station, None if empty or unknown."""
    color = RDK.getParam(HELD_PEN_PARAM)
    return color if color in PENS else None


def choose_color_gui(current_color):
    """Small window to choose the next pen. Returns (True, color or None), or (False, None) if cancelled."""
    import tkinter

    window = tkinter.Tk()
    window.title('Change pen')
    tkinter.Label(window, text=f'Holding: {current_color or NO_PEN}\nPick up:', justify='left').pack(anchor='w', padx=12, pady=(12, 4))

    choice = tkinter.StringVar(value=current_color or NO_PEN)
    for color, pen_data in PENS.items():
        r, g, b, _ = pen_data['color']
        swatch = f'#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}'
        tkinter.Radiobutton(window, text=color, variable=choice, value=color, fg=swatch, selectcolor='white').pack(anchor='w', padx=12)
    tkinter.Radiobutton(window, text=f'{NO_PEN} (empty gripper)', variable=choice, value=NO_PEN).pack(anchor='w', padx=12)

    result = {'ok': False}

    def ok():
        result['ok'] = True
        window.destroy()

    buttons = tkinter.Frame(window)
    buttons.pack(pady=12)
    tkinter.Button(buttons, text='OK', width=8, command=ok).pack(side='left', padx=4)
    tkinter.Button(buttons, text='Cancel', width=8, command=window.destroy).pack(side='left', padx=4)
    window.mainloop()

    if not result['ok']:
        return False, None
    return True, None if choice.get() == NO_PEN else choice.get()


def main(argv=None):
    """Run change_pen() from the terminal, see --help."""
    colors = list(PENS) + [NO_PEN]
    parser = argparse.ArgumentParser(description='Change the pen in the gripper.',
                                     epilog='Examples:  pen_changer.py --new red  |  pen_changer.py --new none  |  pen_changer.py --gui  |  pen_changer.py --show  |  '
                                            'pen_changer.py --new red --make-program  |  pen_changer.py --new red --send')
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--new', choices=colors, help='pen to pick up (none: empty gripper)')
    action.add_argument('--gui', action='store_true', help='choose the pen to pick up in a window')
    action.add_argument('--demo', action='store_true', help='pick red, change to blue, put blue back')
    action.add_argument('--show', action='store_true', help='only show the stand and the pens at the calibrated position, the robot does not move')
    parser.add_argument('--current', choices=colors, help='pen in the gripper now (default: the pen of slot DEFAULT_HELD_SLOT, see the settings)')
    parser.add_argument('--stand', nargs=4, type=float, metavar=('X', 'Y', 'Z', 'RZ'),
                        help='stand position in mm / deg, see "Pen stand" in the settings (default: from the calibration, see load_calibration())')
    parser.add_argument('--make-program', action='store_true',
                        help=f'also write the pen change as a program for the real robot ({os.path.relpath(PROGRAMS_DIR, REPO_DIR)}/{PROGRAM_NAME}.script)')
    parser.add_argument('--send', nargs='?', const=ROBOT_IP, metavar='ROBOT_IP',
                        help=f'make the program and send it to the robot, which runs it at once (default IP {ROBOT_IP})')
    args = parser.parse_args(argv)
    if args.send:
        args.make_program = True
    if args.show and args.make_program:
        parser.error('--show does not move the robot, there is no program to make')

    RDK = robolink.Robolink()
    RDK.setSimulationSpeed(SIMULATION_SPEED)
    if args.current is not None:
        current = None if args.current == NO_PEN else args.current
    elif DEFAULT_HELD_SLOT is not None:
        current = next(color for color, pen in PENS.items() if pen['slot'] == DEFAULT_HELD_SLOT)
    else:
        current = held_pen(RDK)

    if args.show:
        stand_position = show_stand(args.stand, current, RDK)
        print(f'Stand shown at {stand_position}, holding: {current or NO_PEN}')
        RDK.ShowMessage(f"Stand shown at {stand_position}", False)
        return

    if args.demo:
        changes = ['red', 'blue', None]
    elif args.gui:
        ok, new = choose_color_gui(current)
        if not ok:
            print('Cancelled')
            return
        changes = [new]
    else:
        changes = [None if args.new == NO_PEN else args.new]

    if args.make_program and changes == [current]:
        print(f'Already holding {current or NO_PEN}, nothing to do: no program made. Use --current to tell which pen the real robot holds')
        return
    program = new_program(RDK) if args.make_program else None
    for color in changes:
        current = change_pen(args.stand, current, color, RDK, program, animate=program is None)
    print(f'Holding: {current or NO_PEN}')
    RDK.ShowMessage(f"Holding: {current or NO_PEN}", False)

    if program is not None:
        path = save_program(program)
        print(f'Program for the real robot: {path}')
        if args.send:
            send_program(path, args.send)
            print(f'Sent to the robot at {args.send}, it runs now')


if __name__ == '__main__':
    main()
