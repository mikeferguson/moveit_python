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
#  * Neither the name of Unbounded Robotics, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
from tf.listener import TransformListener
from geometry_msgs.msg import *
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

## @brief Pure python interface to move_group action
class MoveGroupInterface:

    ## @brief Constructor for this utility
    ## @param group Name of the MoveIt! group to command
    ## @param frame Name of the fixed frame in which planning happens
    ## @param listener A TF listener instance (optional, will create a new one if None)
    ## @param plan_only Should we only plan, but not execute?
    def __init__(self, group, frame, listener = None, plan_only = False):
        self._group = group
        self._fixed_frame = frame
        self._action = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        self._action.wait_for_server()
        if listener == None:
            self._listener = TransformListener()
        else:
            self._listener = listener
        self.plan_only = plan_only

    def moveToJointPosition(self, joints, positions, tolerance = 0.01, start_state = None):
        g = MoveGroupGoal()
        # 1. fill in workspace_parameters
        # 2. fill in start_state
        if start_state:
            g.request.start_state = start_state
        # 3. fill in goal_constraints
        c1 = Constraints()
        for i in range(len(joints)):
            c1.joint_constraints.append(JointConstraint())
            c1.joint_constraints[i].joint_name = joints[i]
            c1.joint_constraints[i].position = positions[i]
            c1.joint_constraints[i].tolerance_above = tolerance
            c1.joint_constraints[i].tolerance_below = tolerance
            c1.joint_constraints[i].weight = 1.0
        g.request.goal_constraints.append(c1)
        # 4. fill in path constraints
        # 5. fill in trajectory constraints
        # 6. fill in planner id
        # 7. fill in group name
        g.request.group_name = self._group
        # 8. fill in number of planning attempts
        g.request.num_planning_attempts = 1
        # 9. fill in allowed planning time
        g.request.allowed_planning_time = 15.0
        # TODO: fill in
        # g.planning_options.planning_scene_diff.allowed_collision_matrix
        g.planning_options.plan_only = self.plan_only
        g.planning_options.look_around = False
        g.planning_options.replan = False
        self._action.send_goal(g)
        self._action.wait_for_result()
        return self._action.get_result()

    def moveToPose(self, pose_stamped, gripper_frame, tolerance = 0.01):
        """ Move the arm, based on a goal pose_stamped for the end effector. """
        g = MoveGroupGoal()
        pose_transformed = self._listener.transformPose(self._fixed_frame, pose_stamped)

        # 1. fill in workspace_parameters
        # 2. fill in start_state
        # 3. fill in goal_constraints
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self._fixed_frame
        c1.position_constraints[0].link_name = gripper_frame
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [tolerance * tolerance]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(pose_transformed.pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self._fixed_frame
        c1.orientation_constraints[0].orientation = pose_transformed.pose.orientation
        c1.orientation_constraints[0].link_name = gripper_frame
        c1.orientation_constraints[0].absolute_x_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_y_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_z_axis_tolerance = tolerance
        c1.orientation_constraints[0].weight = 1.0

        g.request.goal_constraints.append(c1)

        # 4. fill in path constraints
        # 5. fill in trajectory constraints
        # 6. fill in planner id
        # 7. fill in group name
        g.request.group_name = self._group
        # 8. fill in number of planning attempts
        g.request.num_planning_attempts = 1
        # 9. fill in allowed planning time
        g.request.allowed_planning_time = 15.0
        # TODO: fill in
        # g.planning_options.planning_scene_diff.allowed_collision_matrix
        g.planning_options.plan_only = self.plan_only
        g.planning_options.look_around = False
        g.planning_options.replan = False

        self._action.send_goal(g)
        self._action.wait_for_result()
        return self._action.get_result()
