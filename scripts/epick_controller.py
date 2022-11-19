#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
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
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
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
#
# Author: Toshio Ueshiba
#
import rospy, os
import numpy as np
from aist_robotiq.msg import (CModelStatus, CModelCommand,
                              EPickCommandAction, EPickCommandGoal,
                              EPickCommandResult, EPickCommandFeedback)
from actionlib        import SimpleActionServer

#########################################################################
#  class EPickController                                                #
#########################################################################
class EPickController(object):
    def __init__(self):
        super(EPickController, self).__init__()

        self._name = rospy.get_name()

        # Status recevied from driver, command sent to driver
        self._status_sub  = rospy.Subscriber('~status', CModelStatus,
                                             self._status_cb, queue_size=1)
        self._command_pub = rospy.Publisher('~command', CModelCommand,
                                            queue_size=1)

        # Configure and start the action server
        self._server = SimpleActionServer('~gripper_cmd', EPickCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()

        rospy.logdebug('(%s) Started' % self._name)

    def _status_cb(self, status):
        # Return if no active goals
        if not self._server.is_active():
            return

        # Handle the active goal
        if not self._is_active(status):
            rospy.logwarn('(%s) abort goal because the gripper is not yet active' % self._name)
            self._server.set_aborted()
        elif self._error(status) != 0:
            rospy.logwarn('(%s) faulted with code: %x'
                          % (self._name, self._error(status)))
            self._server.set_aborted()
        elif self._stalled(status):
            rospy.loginfo('(%s) stalled' % self._name)
            self._server.set_succeeded(
                EPickCommandResult(*self._status_values(status)))
        else:
            self._server.publish_feedback(
                EPickCommandFeedback(*self._status_values(status)))

    def _goal_cb(self):
        goal = self._server.accept_new_goal()  # requested goal

        # Check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        self._send_move_command(goal.command.advanced_mode,
                                goal.command.max_pressure,
                                goal.command.min_pressure,
                                goal.command.timeout)

    def _preempt_cb(self):
        self._stop()
        rospy.loginfo('(%s) preempted' % self._name)
        self._server.set_preempted()

    def _send_move_command(self, advanced_mode,
                           max_pressure, min_pressure, timeout):
        max_prs = np.clip(int(max_pressure) + 100, 0, 255)
        min_prs = np.clip(int(min_pressure) + 100, 0, 100)
        tout    = np.clip(int(10.0*timeout.to_sec()), 0, 255)
        self._send_raw_move_command(advanced_mode, max_prs, min_prs, tout)

    def _send_raw_move_command(self, advanced_mode, max_prs, min_prs, tout):
        command = CModelCommand()
        command.rACT = 1
        command.rMOD = 1 if advanced_mode else 0
        command.rGTO = 1
        command.rATR = 0
        command.rPR  = max_prs
        command.rSP  = tout
        command.rFR  = min_prs  # threshold for object detection(gOBJ)
        self._command_pub.publish(command)

    def _stop(self):
        command = CModelCommand()
        command.rACT = 1
        command.rGTO = 0
        self._command_pub.publish(command)
        rospy.logdebug('(%s) stopping' % (self._name))

    def _pressure(self, status):
        return status.gPO - 100

    def _stalled(self, status):
        return status.gOBJ == 1 or status.gOBJ == 2

    def _status_values(self, status):
        return self._pressure(status), self._stalled(status)

    def _error(self, status):
        return status.gFLT

    def _is_active(self, status):
        return status.gSTA == 3 and status.gACT == 1


if __name__ == '__main__':
    rospy.init_node('epick_controller')
    controller = EPickController()
    rospy.spin()
