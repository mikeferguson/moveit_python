## moveit_python

This is a set of pure-python bindings to the ROS interface of MoveIt! based on the
earlier moveit_utils package that was developed as part of the chess_player package.

## Overview

There are three interfaces currently:

 * MoveGroupInterface -- used to move the arm using the move_group action.
 * PlanningSceneInterface -- used to add/remove collision and attached objects.
   Can also set the colors of objects.
 * PickPlaceInterface -- used to interface to the pick and place actions.

## MoveGroupInterface

The MoveGroupInterface is quite easy to use:

```python
import rospy
from moveit_python import *

rospy.init_node("moveit_py")
g = MoveGroupInterface("planning_group_name", "base_link")
```

Obviously, you might need different values for base_link, and your planning group
is probably not called "planning_group_name".

## PlanningSceneInterface

The PlanningSceneInterface allows you to easily insert and remove objects from
the MoveIt! planning scene. Unlike the moveit_commander, this module tracks when
objects are added removed, and will re-attempt add/remove if a message is loss.

```python
import rospy
from moveit_python import *

rospy.init_node("moveit_py")
# create a planning scene interface, provide name of root link
p = PlanningSceneInterface("base_link")

# add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
p.addCube("my_cube", 0.1, 1, 0, 0.5)

# do something

# remove the cube
p.removeCollisionObject("my_cube")
```

Each time an object is added or removed, the planning scene interface will
wait until it recieves confirmation that the request has been processed by MoveIt!
If sending a large number of objects, it would be more efficient to only wait
for synchronization at the end, this can be achieved by doing like so:

```python
p.addCube("my_cube", 0.1, 1, 0, 0.5, wait=False)
p.addCube("my_other_cube", 0.1, 2, 0, 0.5, wait=False)
p.waitForSync()
```

## PickPlaceInterface

```python
import rospy
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation

rospy.init_node("moveit_py")
# provide arm group and gripper group names
# also takes a third parameter "plan_only" which defaults to False
p = PickPlaceInterface("arm", "gripper")

g = Grasp()
# fill in g
# setup object named object_name using PlanningSceneInterface
p.pickup("object_name", [g, ], support_name = "supporting_surface")

l = PlaceLocation()
# fill in l
p.place("object_name" [l, ], goal_is_eef = True, support_name = "supporting_surface")
```

## Migration from moveit_utils

 * GraspingInterface renamed to PickPlaceInterface
   * The pick/place functions now return the entire action result, in moveit_utils they returned only the error_code. To access the error code as it used to be returned, you would use result.error_code.val
 * ObjectManager renamed to PlanningSceneInterface
   * remove function is now removeCollisionObject and removeAttachedObject
 * ArmInterface renamed to MoveGroupInterface
