#!/usr/bin/env python

# Copyright 2011-2018, Michael Ferguson
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

import argparse
import rospy
from moveit_python import PlanningSceneInterface

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Remove objects from the MoveIt planning scene.")
    parser.add_argument("name",
                        nargs="?",
                        help="Name of object to remove")
    parser.add_argument("--all",
                        help="Remove all objects.",
                        action="store_true")
    parser.add_argument("--attached",
                        help="Remove an attached object.",
                        action="store_true")
    args = parser.parse_args()

    if args.all:
        rospy.init_node("remove_objects")
        scene = PlanningSceneInterface("base_link")
        for name in scene.getKnownCollisionObjects():
            print("Removing %s" % name)
            scene.removeCollisionObject(name, use_service=False)
        scene.waitForSync()
    elif args.name:
        rospy.init_node("remove_objects")
        scene = PlanningSceneInterface("base_link")
        print("Removing %s" % args.name)
        if args.attached:
            scene.removeAttachedObject(args.name)
        else:
            scene.removeCollisionObject(args.name)
    else:
        parser.print_help()

