# Copyright 2015, Fetch Robotics Inc.
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
from sensor_msgs.msg import JointState

## @brief This provides an interface to the "fake_controller" aspects of
##        the MoveIt demo.launch files.
class FakeGroupInterface(object):
    
    ## @brief Signature intended to match real interface, params not used
    def __init__(self, group, frame, listener=None, plan_only=False):
        self.pub = rospy.Publisher("/move_group/fake_controller_joint_states",
                                   JointState,
                                   queue_size=10)

    ## @brief This is modeled such that we can use the fake interface as a
    ##        drop in replacement for the real interface.
    def moveToJointPosition(self, joints, positions, **kwargs):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = joints
        msg.position = positions
        self.pub.publish(msg)
