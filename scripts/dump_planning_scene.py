#!/usr/bin/env python

# Copyright 2014-2021, Michael Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys, pickle
import rospy
from moveit_python.planning_scene_interface import (GetPlanningScene,
                                                    PlanningSceneComponents)

if __name__ == "__main__":
    rospy.init_node("dump_planning_scene")

    rospy.loginfo("Waiting for get_planning_scene")
    rospy.wait_for_service("get_planning_scene")
    service = rospy.ServiceProxy("get_planning_scene", GetPlanningScene)
    try:
        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.WORLD_OBJECT_NAMES + \
                         PlanningSceneComponents.WORLD_OBJECT_GEOMETRY  + \
                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene = service(req)
    except rospy.ServiceException as e:
        print("Failed to get planning scene: %s" % e)
        exit(-1)

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "scene.saved"

    print("dumping")
    print(scene)
    print("to %s" % filename)

    pickle.dump(scene, open(filename, "wb"))

