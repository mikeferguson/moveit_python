# Copyright 2011-2014, Michael Ferguson
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

import rospy
import actionlib
from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal

## @brief Simple interface to pick and place actions
class PickPlaceInterface:

    ## @brief Create a grasp manager, connect actions
    ## @param group Name of arm planning group
    ## @param ee_group Name of end effector planning group
    ## @param plan_only Should we only plan, but not execute?
    def __init__(self, group = 'arm', ee_group = 'gripper', plan_only = False):
        self._group = group
        self._effector = ee_group
        self._pick_action = actionlib.SimpleActionClient('pickup', PickupAction)
        self._pick_action.wait_for_server()
        self._place_action = actionlib.SimpleActionClient('place', PlaceAction)
        self._place_action.wait_for_server()
        self._plan_only = plan_only

    ## @brief Plan and grasp something
    ## @param name Name of the object to grasp
    ## @param grasps Grasps to try (moveit_msgs/Grasp)
    ## @param support_name Name of the support surface
    ## @returns Error code from MoveIt! (type: MoveItErrorCodes)
    def pickup(self, name, grasps, support_name = 'table',
               allow_gripper_support_collision = True,
               allowed_touch_objects = list()):
        """ This will try to pick up an object. """
        g = PickupGoal()
        g.target_name = name
        g.group_name = self._group
        g.end_effector = self._effector
        g.possible_grasps = grasps
        g.support_surface_name = support_name
        g.allow_gripper_support_collision = allow_gripper_support_collision
        g.attached_object_touch_links = list() # empty list = use all links of end-effector
        #g.path_constraints = ??
        #g.planner_id = ??
        g.allowed_touch_objects = allowed_touch_objects
        g.allowed_planning_time = 30.0
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.plan_only = self._plan_only
        self._pick_action.send_goal(g)
        self._pick_action.wait_for_result()
        return self._pick_action.get_result().error_code.val

    ## @brief Plan and grasp something
    ## @param name Name of the object to grasp
    ## @param grasps Grasps to try (moveit_msgs/Grasp)
    ## @param support_name Name of the support surface
    ## @param goal_is_eef Set to true if the place goal is for the
    ##        end effector frame, default is object frame.
    ## @returns Error code from MoveIt! (type: MoveItErrorCodes)
    def place(self, name, locations, support_name = 'table',
              allow_gripper_support_collision = True,
              allowed_touch_objects = list(),
              goal_is_eef = False):
        g = PlaceGoal()
        g.group_name = self._group
        g.attached_object_name = name
        g.place_locations = locations
        g.place_eef = goal_is_eef
        g.support_surface_name = support_name
        g.allow_gripper_support_collision = allow_gripper_support_collision
        #g.path_constraints = ??
        #g.planner_id = ??
        g.allowed_touch_objects = allowed_touch_objects
        g.allowed_planning_time = 30.0
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.plan_only = self._plan_only
        self._place_action.send_goal(g)
        self._place_action.wait_for_result()
        return self._place_action.get_result().error_code.val

