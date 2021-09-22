#!/usr/bin/env python
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

import sys, rospy
from aist_robotiq.cmodel_base import RobotiqCModel, ComModbusRtu
from aist_robotiq             import msg as amsg

def mainLoop(device):
    # Gripper is a C-Model with a TCP connection
    gripper = RobotiqCModel()
    gripper.client = ComModbusRtu()
    # We connect to the device name received as an argument
    rospy.loginfo("Connecting to device " + str(device))
    gripper.client.connectToDevice(device)
    # The Gripper status
    pub = rospy.Publisher('/status', amsg.CModelStatus, queue_size=3)
    # The Gripper command
    rospy.Subscriber('/command', amsg.CModelCommand, gripper.refreshCommand)

    while not rospy.is_shutdown():
        # Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)
        # Wait a little
        rospy.sleep(0.05)
        # Send the most recent command
        gripper.sendCommand()
        # Wait a little
        rospy.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node('cmodel_rtu_driver')
    # Run the main loop
    try:
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException: pass
