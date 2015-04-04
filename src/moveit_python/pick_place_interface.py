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
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal

## @brief Simple interface to pick and place actions
class PickPlaceInterface:

    ## @brief Create a grasp manager, connect actions
    ## @param group Name of arm planning group
    ## @param ee_group Name of end effector planning group
    ## @param plan_only Should we only plan, but not execute?
    def __init__(self, group = "arm", ee_group = "gripper", plan_only = False, verbose = False):
        self._verbose = verbose
        self._group = group
        self._effector = ee_group
        if self._verbose:
            rospy.loginfo("Connecting to pickup action...")
        self._pick_action = actionlib.SimpleActionClient("pickup", PickupAction)
        self._pick_action.wait_for_server()
        if self._verbose:
            rospy.loginfo("...connected")
            rospy.loginfo("Connecting to place action...")
        self._place_action = actionlib.SimpleActionClient("place", PlaceAction)
        self._place_action.wait_for_server()
        if self._verbose:
            rospy.loginfo("...connected")
        self._plan_only = plan_only
        self.allowed_planning_time = 30.0

    ## @brief Plan and grasp something
    ## @param name Name of the object to grasp
    ## @param grasps Grasps to try (moveit_msgs/Grasp)
    ## @param support_name Name of the support surface
    ## @returns moveit_msgs/PickupResult
    def pickup(self, name, grasps, support_name = "",
               allow_gripper_support_collision = True,
               allowed_touch_objects = list()):
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
        g.allowed_planning_time = self.allowed_planning_time
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.plan_only = self._plan_only
        self._pick_action.send_goal(g)
        self._pick_action.wait_for_result()
        return self._pick_action.get_result()

    ## @brief Plan and grasp something
    ## @param name Name of the object to grasp
    ## @param grasps Grasps to try (moveit_msgs/Grasp)
    ## @param support_name Name of the support surface
    ## @param goal_is_eef Set to true if the place goal is for the
    ##        end effector frame, default is object frame.
    ## @returns moveit_msgs/PlaceResult
    def place(self, name, locations, support_name = "",
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
        g.allowed_planning_time = self.allowed_planning_time
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.plan_only = self._plan_only
        self._place_action.send_goal(g)
        self._place_action.wait_for_result()
        return self._place_action.get_result()

    ## Common usage pattern
    ## TODO document
    def pick_with_retry(self, name, grasps, retries=5, scene = None,
                        support_name = "", allow_gripper_support_collision = True,
                        allowed_touch_objects = list()):
        if self._verbose:
            rospy.loginfo("Beginning to pick.")
        while retries > 0:
            retries += -1
            pick_result = self.pickup(name, grasps, support_name=support_name)
            if pick_result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Pick succeeded")
                return [True, pick_result]
            elif pick_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                rospy.logerr("Pick failed in the planning stage, try again...")
                rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                continue
            elif scene and \
                pick_result.error_code.val == MoveItErrorCodes.CONTROL_FAILED or \
                pick_result.error_code.val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE or \
                pick_result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                rospy.logerr("Pick failed during execution, attempting to cleanup.")
                if name in scene.getKnownAttachedObjects():
                    rospy.loginfo("Pick managed to grab object, retreat must have failed, continuing anyways")
                    return [True, pick_result]
                else:
                    rospy.loginfo("Pick did not grab object, try again...")
                    continue
            else:
                rospy.logerr("Pick failed with error code: %d. Will retry..." % pick_result.error_code.val)
                continue
        rospy.logerr("Pick failed, and all retries are used")
        return [False, pick_result]

    ## Common usage pattern
    ## TODO document
    def place_with_retry(self, name, locations, retries=5, scene = None,
                         support_name = "", allow_gripper_support_collision = True,
                         allowed_touch_objects = list(),
                         goal_is_eef = False):
        if self._verbose:
            rospy.loginfo("Beginning to place.")
        while retries > 0:
            retries += -1
            place_result = self.place(name, locations, support_name, allow_gripper_support_collision, allowed_touch_objects, goal_is_eef)
            if place_result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place succeeded")
                return [True, place_result]
            elif place_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                rospy.logerr("Place failed in the planning stage, try again...")
                rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                continue
            elif scene and \
                 place_result.error_code.val == MoveItErrorCodes.CONTROL_FAILED or \
                 place_result.error_code.val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE or \
                 place_result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                rospy.logerr("Place failed during execution, attempting to cleanup.")
                if name in scene.getKnownAttachedObjects():
                    rospy.loginfo("Place did not place object, approach must have failed, will retry...")
                    continue
                else:
                    rospy.loginfo("Object no longer in gripper, must be placed, continuing...")
                    return [True, place_result]
            else:
                rospy.logerr("Place failed with error code: %d. Will retry..." % place_result.error_code.val)
                continue
        rospy.logerr("Place failed, and all retries are used")
        return [False, place_result]

