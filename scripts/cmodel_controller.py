#!/usr/bin/env python
import rospy, os
import numpy as np
from sensor_msgs  import msg as smsg
from control_msgs import msg as cmsg
from aist_robotiq import msg as amsg
from actionlib    import SimpleActionServer

class CModelController(object):
    def __init__(self, activate=False):
        self._ns = rospy.get_namespace()

        # Read configuration parameters
        self._min_gap_counts = rospy.get_param('~min_gap_counts', 230)
        self._max_gap_counts = 0
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

        # Configure and start the action server
        self._server = SimpleActionServer('~gripper_cmd',
                                          cmsg.GripperCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()

        # Activate gripper
        rospy.sleep(1.0)   # Wait before checking status with self._ready()
        if activate and not self._ready():
            rospy.sleep(2.0)
            if not self._activate():
                return
        rospy.logdebug('(%s) Started' % self._ns)

    def _status_cb(self, status):
        # Publish the joint_states for the gripper
        joint_state = smsg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name         = [self._joint_name]
        joint_state.position     = [self._get_position(status)]
        self._joint_state_pub.publish(joint_state)

        if not self._server.is_active():
            return

        if not self._is_active(status):
            if not self._activate():
                rospy.logwarn('(%s) could not accept goal because the gripper is not yet active' % self._ns)
            return
        elif self._error(status) != 0:
            rospy.logwarn('(%s) faulted with code: %x'
                          % (self._ns, self._error(status)))
            self._server.set_aborted()
        elif self._reached_goal(status):
            rospy.loginfo('(%s) succeeded' % self.ns)
            self._server.set_succeeded(
                cmsg.GripperCommandResult(*self._status_values(status)))
        else:
            self._server.publish_feedback(
                cmsg.GripperCommandFeedback(*self._status_values(status)))

    def _goal_cb(self):
        # Get requested goal
        goal = self._server.accept_new_goal()

        # check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        self._goal_rPR = self._send_move_command(goal.command.position,
                                                 (self._min_velocity +
                                                  self._max_velocity) * 0.5,
                                                 goal.command.max_effort)

    def _preempt_cb(self):
        self._stop()
        rospy.loginfo('(%s) Preempted' % self._ns)
        self._server.set_preempted()

    def _activate(self, timeout=5.0):
        command = amsg.CModelCommand()
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150
        start_time = rospy.get_time()
        while not self._ready():
            if rospy.is_shutdown():
                self._preempt()
                return False
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn('(%s) failed to activate' % (self._ns))
                return False
            self._command_pub.publish(command)
            rospy.sleep(0.1)

        rospy.loginfo('(%s) successfully activated' % (self._ns))
        return True

    def _send_move_command(self, position, velocity, effort):
        command = amsg.CModelCommand()
        command.rACT = 1
        command.rGTO = 1
        command.rPR  = int(np.clip((position - self._min_position) \
                                   / self.position_per_tick
                                   + self._min_gap_counts,
                                   self._max_gap_counts, self._min_gap_counts))
        command.rSP  = int(np.clip((velocity - self._min_velocity) \
                                   / self.velocity_per_tick,
                                   0, 255))
        command.rFR  = int(np.clip((effort - self._min_effort) \
                                   / self.effort_per_tick,
                                   0, 255))
        self._command_pub.publish(command)
        return command.rPR

    def _stop(self):
        command = amsg.CModelCommand()
        command.rACT = 1
        command.rGTO = 0
        self._command_pub.publish(command)
        rospy.logdebug('(%s) stopping' % (self._ns))

    def _position(self, status):
        return (status.gPO - self._min_gap_counts) * self.position_per_tick \
             + self._min_position

    def _effort(self, status):
        return status.gCU * self.effort_per_tick + self._min_effort

    def _stalled(self, status):
        return status.gOBJ == 1 or status.gOBJ == 2

    def _reached_goal(self, status):
        return status.gPO == self._goal_rPR

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
    controller = CModelController(False)
    rospy.spin()
