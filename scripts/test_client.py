#!/usr/bin/env python

import rospy
from aist_robotiq import RobotiqGripper

if __name__ == '__main__':

    def is_float(s):
        try:
            float(s)
        except ValueError:
            return False
        else:
            return True

    rospy.init_node('test_client')

    prefix  = rospy.get_param('~prefix', 'a_bot_gripper_')
    gripper = RobotiqGripper(prefix)

    while not rospy.is_shutdown():
        print('==== Available commands ====')
        print('  g:         Grasp')
        print('  r:         Release')
        print('  <numeric>: Open gripper with a specified gap value')
        print('  q:         Quit')

        key = raw_input('>> ')
        if key == 'g':
            result = gripper.grasp()
        elif key == 'r':
            result = gripper.release()
        elif is_float(key):
            result = gripper.move(float(key))
        elif key=='q':
            break
        else:
            print('unknown command: %s' % key)
            continue

        print('---- Result ----')
        print(result)
