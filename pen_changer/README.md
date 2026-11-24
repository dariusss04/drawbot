# Pen changer

Automatic marker changes with a pen stand and a Robotiq Hand-E gripper. DrawBot calls it through `drawbot/pen_stand.py` when the export mode is "Pen stand";
you do not need to run it yourself.

`change_pen()` puts the pen the robot is holding back into the stand, picks up the new one and returns to where
the robot was. Given a RoboDK program item, it writes these moves and the gripper commands into that program, so
the exported URScript contains the pen changes.

```python
from pen_changer import change_pen

held = change_pen(None, None, "red", RDK, program)    # pick red
held = change_pen(None, held, "blue", RDK, program)   # put red back, pick blue
held = change_pen(None, held, None, RDK, program)     # put the last pen back
```

| Argument | Meaning |
|---|---|
| `stand_position` | `None`: from the calibration. Or `[x, y, z, rz]` in the robot base frame (mm, deg) |
| `current_color` | the pen in the gripper now, `None` = empty |
| `new_color` | the pen to pick up, `None` = end with an empty gripper |
| `program` | the RoboDK program item. Without it only the simulation moves |
| `animate` | `False` makes building long programs much faster |

## Notes

- Feed the returned colour back in as `current_color` on the next call.
- Set the program speed again after every `change_pen()`; frame and tool are restored, the speed is not.
- The program talks to the Hand-E over the Robotiq URCap socket (port 63352), so the gripper must be set up in
  the URCap on the pendant.
- After every pick the finger position is checked and the program stops with a popup if the pen is missing.
  The window is set by `PEN_CHECK_POS_MIN/MAX` in `src/pen_changer.py`.

## Calibration

The stand pose comes from the pen-stand points taught with `calibration/teach_whiteboard.script` (the pen seated
in slots 1 to 4). The stand position is fitted to these points and the robot grips at them. The distance from the
gripper TCP to the pen tip comes from the taught `tcp-pen` and `tcp-nopen` TCPs, so it must match the real pen.

## 3D models

`3D_Models/` has the pen stand, the pen holder and the gripper finger attachment (V-groove) as STL files for 3D
printing. With these parts the pen TCP is about 280 mm from the flange.

## Try it alone

With RoboDK open, from this folder:

```bash
python src/pen_changer.py --show
```

```bash
python src/pen_changer.py --demo --current none
```
