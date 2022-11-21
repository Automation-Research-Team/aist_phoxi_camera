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
import rospy
from aist_robotiq import EPickGripper

if __name__ == '__main__':

    def is_float(s):
        try:
            float(s)
        except ValueError:
            return False
        else:
            return True

    rospy.init_node('test_epick_client')

    prefix             = rospy.get_param('~prefix', 'a_bot_gripper_')
    advanced_mode      = rospy.get_param('~advanced_mode',      False)
    grasp_pressure     = rospy.get_param('~grasp_pressure',     -78.0)
    detection_pressure = rospy.get_param('~detection_pressure', -10.0)
    release_pressure   = rospy.get_param('~release_pressure',     0.0)
    timeout            = rospy.Duration(rospy.get_param('~timeout',  1.0))

    gripper = EPickGripper(prefix, advanced_mode, grasp_pressure,
                           detection_pressure, release_pressure)

    while not rospy.is_shutdown():
        print('==== Available commands ====')
        print('  g:         Grasp')
        print('  r:         Release')
        print('  <numeric>: Set gripper a specified pressure value')
        print('  q:         Quit\n')

        key = raw_input('>> ')
        if key == 'g':
            result = gripper.grasp(timeout)
        elif key == 'r':
            result = gripper.release()
        elif is_float(key):
            result = gripper.move(float(key), detection_pressure, timeout)
        elif key=='q':
            break
        else:
            print('unknown command: %s' % key)
            continue

        print('---- Result ----')
        print(result)
