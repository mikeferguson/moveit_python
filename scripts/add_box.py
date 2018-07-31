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
        description="Add a box to the MoveIt planning scene.")
    parser.add_argument("name", help="Name of the box to add")
    parser.add_argument("x", type=float, help="X coordinate of center of box")
    parser.add_argument("y", type=float, help="Y coordinate of center of box")
    parser.add_argument("z", type=float, help="Z coordinate of center of box")
    parser.add_argument("size_x", type=float, help="Size of box in x dimension")
    parser.add_argument("size_y", type=float, help="Size of box in y dimension")
    parser.add_argument("size_z", type=float, help="Size of box in z dimension")
    args = parser.parse_args()

    if args.name:
        rospy.init_node("add_box")
        scene = PlanningSceneInterface("/base_link")
        print("Adding Box with name: %s" % args.name)
        scene.addBox(args.name, args.size_x, args.size_y, args.size_z,
                     args.x, args.y, args.z, use_service=True)
    else:
        parser.print_help()

