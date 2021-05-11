#!/usr/bin/env python
import rospy, os
import numpy as np
from sensor_msgs  import msg as smsg
from control_msgs import msg as cmsg
from aist_robotiq import msg as amsg
from actionlib    import SimpleActionServer

class CModelController(object):
    def __init__(self):
        self._name = rospy.get_name()

        # Read configuration parameters
        self._min_position   = rospy.get_param('~min_position', 0.0)
        self._max_position   = rospy.get_param('~max_position', 0.085)
        self._min_velocity   = rospy.get_param('~min_velocity', 0.013)
        self._max_velocity   = rospy.get_param('~max_velocity', 0.1)
        self._min_effort     = rospy.get_param('~min_effort', 40.0)
        self._max_effort     = rospy.get_param('~max_effort', 100.0)
        self._joint_name     = rospy.get_param('~joint_name', 'finger_joint')

        # Status recevied from driver, command sent to driver
        self._status_sub      = rospy.Subscriber('~status', amsg.CModelStatus,
                                                 self._status_cb, queue_size=1)
        self._command_pub     = rospy.Publisher('~command', amsg.CModelCommand,
                                                queue_size=1)
        self._joint_state_pub = rospy.Publisher('/joint_states',
                                                smsg.JointState, queue_size=1)
        self._goal_rPR        = 0

        # Position parameters to be calibrated
        self._min_gap_counts   = 255  # gap counts at full-close position
        self._max_gap_counts   = 0    # gap counts at full-open position
        self._calibration_step = 0    # ready for calibration

        # Configure and start the action server
        self._server = SimpleActionServer('~gripper_cmd',
                                          cmsg.GripperCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()

        # Calibrate gripper
        rospy.sleep(2.0)              # wait for server comes up
        self._calibrate()

        rospy.logdebug('(%s) Started' % self._name)

    def _status_cb(self, status):
        # Publish the joint_states for the gripper
        joint_state = smsg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name         = [self._joint_name]
        joint_state.position     = [self._position(status)]
        joint_state.effort       = [self._effort(status)]
        self._joint_state_pub.publish(joint_state)

        # Handle calibration process if not moving
        if not self._is_moving(status):
            if self._calibration_step == 1:
                self._calibration_step = 2
                self._send_raw_move_command(0, 64, 1)    # full-open
                rospy.sleep(0.5)
            elif self._calibration_step == 2:
                self._max_gap_counts = status.gPO        # record at full-open
                self._calibration_step = 3
                self._send_raw_move_command(255, 64, 1)  # full-close
                rospy.sleep(0.5)
            elif self._calibration_step == 3:
                self._min_gap_counts = status.gPO        # record at full-close
                self._calibration_step = 0
                self._send_raw_move_command(0, 64, 1)    # full-open
                rospy.loginfo('(%s) calibrated to [%d, %d]'
                              % (self._name,
                                 self._min_gap_counts, self._max_gap_counts))

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
        elif self._reached_goal(status):
            rospy.loginfo('(%s) reached goal' % self._name)
            self._server.set_succeeded(
                cmsg.GripperCommandResult(*self._status_values(status)))
        elif self._stalled(status):
            rospy.loginfo('(%s) stalled' % self._name)
            self._server.set_succeeded(
                cmsg.GripperCommandResult(*self._status_values(status)))
        else:
            self._server.publish_feedback(
                cmsg.GripperCommandFeedback(*self._status_values(status)))

    def _goal_cb(self):
        goal = self._server.accept_new_goal()  # requested goal

        # Check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        self._goal_rPR = self._send_move_command(goal.command.position,
                                                 (self._min_velocity +
                                                  self._max_velocity) * 0.5,
                                                 goal.command.max_effort)

    def _preempt_cb(self):
        self._stop()
        rospy.loginfo('(%s) preempted' % self._name)
        self._server.set_preempted()

    def _calibrate(self):
        self._calibration_step = 1

    def _send_move_command(self, position, velocity, effort):
        pos = np.clip(int((position - self._min_position)
                          / self.position_per_tick + self._min_gap_counts),
                      self._max_gap_counts, self._min_gap_counts)
        vel = np.clip(int((velocity - self._min_velocity)
                          / self.velocity_per_tick),
                      0, 255)
        eff = np.clip(int((effort - self._min_effort) / self.effort_per_tick),
                      0, 255)
        self._send_raw_move_command(pos, vel, eff)
        return pos

    def _send_raw_move_command(self, pos, vel, eff):
        command = amsg.CModelCommand()
        command.rACT = 1
        command.rGTO = 1
        command.rPR  = pos
        command.rSP  = vel
        command.rFR  = eff
        self._command_pub.publish(command)

    def _stop(self):
        command = amsg.CModelCommand()
        command.rACT = 1
        command.rGTO = 0
        self._command_pub.publish(command)
        rospy.logdebug('(%s) stopping' % (self._name))

    def _position(self, status):
        return (status.gPO - self._min_gap_counts) * self.position_per_tick \
             + self._min_position

    def _effort(self, status):
        return status.gCU * self.effort_per_tick + self._min_effort

    def _stalled(self, status):
        return status.gOBJ == 1 or status.gOBJ == 2

    def _reached_goal(self, status):
        return abs(status.gPO - self._goal_rPR) <= 1

    def _status_values(self, status):
        return self._position(status), self._effort(status), \
               self._stalled(status),  self._reached_goal(status)

    def _error(self, status):
        return status.gFLT

    def _is_active(self, status):
        return status.gSTA == 3 and status.gACT == 1

    def _is_moving(self, status):
        return status.gGTO == 1 and status.gOBJ == 0

    @property
    def position_per_tick(self):
        return (self._max_position - self._min_position) \
             / (self._max_gap_counts - self._min_gap_counts)

    @property
    def velocity_per_tick(self):
        return (self._max_velocity - self._min_velocity) / 255

    @property
    def effort_per_tick(self):
        return (self._max_effort - self._min_effort) / 255


if __name__ == '__main__':
    rospy.init_node('cmodel_controller')
    controller = CModelController()
    rospy.spin()
