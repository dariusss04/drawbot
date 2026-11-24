from robodk import robolink
import os
import sys


sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from pen_changer import change_pen, new_program, save_program, PENS  # noqa: E402

STAND_POSITION = None
MAKE_PROGRAM = True

RDK = robolink.Robolink()
robot = RDK.Item('UR5e', robolink.ITEM_TYPE_ROBOT)

program = new_program(RDK, 'DrawWithPens') if MAKE_PROGRAM else None

print('Available pens:', list(PENS))

held = None
for color in ['red', 'blue', 'green']:
    held = change_pen(STAND_POSITION, held, color, RDK, program)
    print('Now holding:', held)


held = change_pen(STAND_POSITION, held, None, RDK, program)
print('Now holding:', held)

if program is not None:
    path = save_program(program)
    print('Program for the real robot:', path)
