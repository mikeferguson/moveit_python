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

## @package moveit_python.geometry Helpers for performing geometric
##          transformations on geometry_msgs messages

from geometry_msgs.msg import *
from tf.transformations import *

## @brief Get a translation matrix from a geometry_msgs/Point
## @param point geometry_msgs/Point to turn into matrix
def matrix_from_point_msg(point):
    return translation_matrix((point.x, point.y, point.z))

## @brief Get a rotation matrix from a geometry_msgs/Quaternion
## @param quaternion geometry_msgs/Quaternion to turn into matrix
def matrix_from_quaternion_msg(quaternion):
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return quaternion_matrix(q)

## @brief Get a transformation matrix from a geometry_msgs/Pose
## @param pose geometry_msgs/Pose to turn into matrix
def matrix_from_pose_msg(pose):
    t = matrix_from_point_msg(pose.position)
    r = matrix_from_quaternion_msg(pose.orientation)
    return concatenate_matrices(t, r)

## @brief Get a geometry_msgs/Point from a transformation matrix
## @param transformation The matrix to convert to a point
def point_msg_from_matrix(transformation):
    msg = Point()
    msg.x = transformation[0][3]
    msg.y = transformation[1][3]
    msg.z = transformation[2][3]
    return msg

## @brief Get a geometry_msgs/Quaternion from a transformation matrix
## @param transformation The matrix to convert to a quaternion
def quaternion_msg_from_matrix(transformation):
    q = quaternion_from_matrix(transformation)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg

## @brief Get a geometry_msgs/Pose from a transformation matrix
## @param transformation The matrix to convert to a pose
def pose_msg_from_matrix(transformation):
    msg = Pose()
    msg.position = point_msg_from_matrix(transformation)
    msg.orientation = quaternion_msg_from_matrix(transformation)
    return msg

## @brief Translate a geometry_msgs/Pose
## @param pose The pose to translate
## @param x The displacement in X coordinate axis
## @param y The displacement in Y coordinate axis
## @param z The displacement in Z coordinate axis
def translate_pose_msg(pose, x, y, z):
    initial = matrix_from_pose_msg(pose)
    transform = translation_matrix((x,y,z))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))

## @brief Rotate a geometry_msgs/Pose
## @param pose The pose to rotate
## @param r The roll
## @param p The pitch
## @param y The yaw
def rotate_pose_msg_by_euler_angles(pose, r, p, y):
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))

## @brief Rotate a geometry_msgs/Pose
## @param pose The pose to rotate
## @param r The roll
## @param p The pitch
## @param y The yaw
def rotate_pose_msg_about_origin(pose, r, p, y):
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(transform, initial))
