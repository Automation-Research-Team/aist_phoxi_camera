"""
Clients of gripper action controller of control_msg/GripperCommandAction type.
@file   __init__.py
@author t.ueshiba@aist.go.jp
"""
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs       import msg as cmsg

######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(object):
    """
    Gripper client of control_msg/GripperCommandAction type.
    """
    def __init__(self, action_ns, min_position=0.0, max_position=0.1, max_effort=5.0):
        """
        Constructor
        @param action_ns    namespace of action server to be connected
        @param min_position position when fully closed
        @param max_position position when fully opened
        @param max_effort   maximum effort applied when gripping objects
        """
        super(GenericGripper, self).__init__()

        self._feedback = cmsg.GripperCommandFeedback()
        self._client   = actionlib.SimpleActionClient(action_ns,
                                                      cmsg.GripperCommandAction)
        self._client.wait_for_server()

        self._parameters = {'grasp_position':   min_position,
                            'release_position': max_position,
                            'max_effort':       max_effort}

        rospy.loginfo('%s initialized.', action_ns)

    @property
    def parameters(self):
        """
        Return a dictionary of grippaer parameters
        @return a dictionary of grippaer parameters with string keys
        """
        return self._parameters

    @parameters.setter
    def parameters(self, parameters):
        """
        Set a dictionary of grippaer parameters
        @param parameters a dictionary of grippaer parameters with string keys
        """
        for key, value in parameters.items():
            self._parameters[key] = value

    def grasp(self, timeout=0):
        """
        Grasp an object with the gripper.
        Desired finger position and applied effort are specified by parameters
        with 'grasp_position' and 'max_effort' keys, respectively,
        @param timeout If positive, wait timeout seconds until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        return self.move(self.parameters['grasp_position'],
                         self.parameters['max_effort'], timeout)

    def release(self, timeout=0):
        """
        Release an object grasped by the gripper.
        Desired finger position is specified by a parameter
        with 'release_position' key. No effort is applied.
        @param timeout If positive, wait timeout seconds until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        return self.move(self.parameters['release_position'], 0, timeout)

    def move(self, position, max_effort=0, timeout=0):
        """
        Move fingers to the specified position with specified effort
        @param position   finger position
        @param max_effort maximum effort to be applied
        @param timeout    If positive, wait timeout seconds until
                          the gripper completing the movement.
                          If zero, wait forever until the completion.
                          If negative, return immediately without waiting
                          for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        self._client.send_goal(cmsg.GripperCommandGoal(
                                   cmsg.GripperCommand(position, max_effort)),
                               feedback_cb=self._feedback_cb)
        return self.wait(timeout)

    def wait(self, timeout=0):
        """
        Wait the gripper for completing the movement.
        @param timeout If positive, wait timeout seconds until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        if timeout < 0:
            return cmsg.GripperCommandResult(0, 0, False, False)
        elif not self._client.wait_for_result(rospy.Duration(timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         timeout)
            return cmsg.GripperCommandResult(self._feedback.position,
                                             self._feedback.effort,
                                             self._feedback.stalled,
                                             self._feedback.reached_goal)
        return self._client.get_result()

    def cancel(self):
        """
        Cancel the latest motion command sent to the gripper.
        """
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _feedback_cb(self, feedback):
        self._feedback = feedback

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GenericGripper):
    def __init__(self, prefix='a_bot_gripper_', max_effort=0.0):
        """
        Constructor
        @param prefix     string prefix for identifying a specific gripper
                          from multiple devices
        @param max_effort maximum effort applied when gripping objects
        """
        ns = prefix + 'controller'
        self._min_gap      = rospy.get_param(ns + '/min_gap',      0.000)
        self._max_gap      = rospy.get_param(ns + '/max_gap',      0.085)
        self._min_position = rospy.get_param(ns + '/min_position', 0.81)
        self._max_position = rospy.get_param(ns + '/max_position', 0.00)

        assert self._min_gap < self._max_gap
        assert self._min_position != self._max_position

        super(RobotiqGripper, self).__init__(ns + '/gripper_cmd',
                                             self._min_gap, self._max_gap,
                                             max_effort)

    def move(self, gap, max_effort=0, timeout=0):
        return super(RobotiqGripper, self).move(self._position(gap),
                                                max_effort, timeout)

    def wait(self, timeout=0):
        result = super(RobotiqGripper, self).wait(timeout)
        result.position = self._gap(result.position)
        return result

    def _position(self, gap):
        return (gap - self._min_gap) * self._position_per_gap \
             + self._min_position

    def _gap(self, position):
        return (position - self._min_position) / self._position_per_gap \
             + self._min_gap

    @property
    def _position_per_gap(self):
        return (self._max_position - self._min_position) \
             / (self._max_gap - self._min_gap)
