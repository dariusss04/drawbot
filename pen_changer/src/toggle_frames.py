from robodk import robolink
import json


RDK = robolink.Robolink()

frames_hidden = RDK.getParam('FramesHidden') == 'True'
show = frames_hidden

kept_hidden = set(json.loads(RDK.getParam('FramesKeptHidden') or '[]')) if show else set()
already_hidden = []
for item_type in (robolink.ITEM_TYPE_FRAME, robolink.ITEM_TYPE_TOOL):
    for item in RDK.ItemList(item_type):
        if show:
            if item.Name() not in kept_hidden:
                item.setVisible(True)
        elif item.Visible():
            item.setVisible(False)
        else:
            already_hidden.append(item.Name())
if not show:
    RDK.setParam('FramesKeptHidden', json.dumps(already_hidden))

for item_type in (robolink.ITEM_TYPE_OBJECT, robolink.ITEM_TYPE_TARGET):
    for item in RDK.ItemList(item_type):
        item.setVisible(item.Visible(), show)

robot_flags = robolink.VISIBLE_ROBOT_DEFAULT if show else robolink.VISIBLE_ROBOT_DEFAULT & ~robolink.VISIBLE_ROBOT_ALL_REFS & ~robolink.VISIBLE_ROBOT_FLANGE
for robot in RDK.ItemList(robolink.ITEM_TYPE_ROBOT):
    robot.setVisible(robot.Visible(), robot_flags)

RDK.setParam('FramesHidden', str(not show))
RDK.ShowMessage("Reference frames shown" if show else "Reference frames hidden", False)
