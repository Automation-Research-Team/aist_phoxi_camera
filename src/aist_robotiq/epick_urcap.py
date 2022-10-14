"""Module to control Robotiq's suction gripper EPick"""
# BASED ON: https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85#latest
# Ported to ROS by felixvd

import socket
import threading
import time
from enum import Enum

# Added for ROS
from aist_robotiq.msg import EPickStatus, EPickCommand
import rospy
import std_msgs.msg
import os, rospkg

class RobotiqEPickURCap:
    """
    Communicates with the gripper directly via socket with string commands, leveraging string names for variables.
    Uses port 63352 which is opened by the Robotiq Gripper URCap and receives ASCII commands.
    """
    # READ/WRITE VARIABLES
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    MOD = 'MOD'  # mod : (0: automatic, 1: advanced, 2,3: reserved)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
    # WRITE VARIABLES
    ATR = 'ATR'  # atr : auto-release (0: normal, 1: emergency slow move)
    FOR = 'PR'   # pr  : maximum vacuum/pressure request in manual/auto mode
    SPE = 'SP'   # spe : action timeout
    POS = 'FR'   # fr  : minimum vacuum/pressure request
    # READ VARIABLES
    STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
    OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    VAS = 'VAS'  # vacuum actuator status (0: standby, 1: gripping, 2: passive releasing, 3: active releasing)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)
    PR =  'PR'   # pressure request (echo of last commanded pressure)
    PO =  'PO'   # actual pressure


    ENCODING = 'UTF-8'  # ASCII and UTF-8 both seem to work

    class GripperStatus(Enum):
        """Gripper status reported by the gripper. The integer values have to match what the gripper sends."""
        RESET = 0
        # UNUSED = 1
        # UNUSED = 2
        ACTIVE = 3

    class ObjectStatus(Enum):
        """Object status reported by the gripper. The integer values have to match what the gripper sends."""
        UNKNOWN_OBJECT_DETECTED = 0
        OBJECT_DETECTED_WITH_MIN_PRESSURE = 1
        OBJECT_DETECTED_WITH_MAX_PRESSURE = 2
        NO_OBJECT_DETECTED = 3

    def __init__(self, address):
        """Constructor."""
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_pressure = 0
        self._max_pressure = 255

        self.connect(address)

    def connect(self, hostname, port = 63352, socket_timeout = 2.0):
        """Connects to a gripper at the given address.

        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        # print("Connecting to: " + str(hostname) + ", port: " + str(port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(socket_timeout)

    def disconnect(self):
        """Closes the connection with the gripper."""
        self.socket.close()

    def sendCommand(self, command):
        self.move(command.rPR, command.rSP, command.rFR)

    def getStatus(self):
        message = EPickStatus()
        message.gACT = self._get_var(self.ACT)
        message.gMOD = self._get_var(self.MOD)
        message.gGTO = self._get_var(self.GTO)
        message.gSTA = self._get_var(self.STA)
        message.gOBJ = self._get_var(self.OBJ)
        message.gFLT = self._get_var(self.FLT)
        message.gPR  = self._get_var(self.PR)
        message.gPO  = self._get_var(self.PO)
        return message

    def _set_vars(self, var_dict):
        """Sends the appropriate command via socket to set the value
        of n variables, and waits for its 'ack' response.

        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack
        was received, indicating the set may not have been effective.
        """
        # construct unique command
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += " " + variable + " " + str(value)
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)

        return self._is_ack(data)

    def _set_var(self, variable, value):
        """Sends the appropriate command via socket to set the value
        of a variable, and waits for its 'ack' response.

        :param variable: Variable to set.
        :param value: Value to set for the variable.
        :return: True on successful reception of ack, false if no ack
        was received, indicating the set may not have been effective.
        """
        return self._set_vars({variable:value})

    def _get_var(self, variable):
        """Sends the appropriate command to retrieve the value
        of a variable from the gripper, blocking until the
        response is received or the socket times out.

        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        with self.command_lock:
            cmd = "GET " + variable + "\n"
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)

        # Expect data of the form 'VAR x', where VAR is an echo
        # of the variable name, and X the value.
        # Note some special variables (like FLT) may send 2 bytes,
        # instead of an integer. We assume integer here
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError("Unexpected response " + str(data)
                             + " does not match '" + variable + "'")
        return int(value_str)

    @staticmethod
    def _is_ack(data):
        return data == b'ack'

    def activate(self, auto_calibrate):
        """Resets the activation flag in the gripper, and sets it back to one,
        clearing previous fault flags.

        :param auto_calibrate: Whether to calibrate the minimum
        and maximum pressures based on actual motion.
        """
        # clear and then reset ACT
        self._set_var(self.STA, 0)
        self._set_var(self.STA, 1)

        print("Waiting for activation")
        # wait for activation to go through
        while not self.is_active() and not rospy.is_shutdown():
            time.sleep(0.01)
        print("Activated.")
        # auto-calibrate pressure range if desired
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self):
        """Returns whether the gripper is active."""
        status = self._get_var(self.STA)
        return self.GripperStatus(status) == self.GripperStatus.ACTIVE

    def get_current_pressure(self):
        """Returns the current pressure as returned by the physical hardware."""
        return self._get_var(self.POS)

    def move(self, min_pressure, max_pressure, timeout):
        if min_pressure < self._min_pressure:
            min_pressure = self._min_pressure
        if max_pressure > self._max_pressure:
            max_pressure = self._max_pressure

        var_dict = dict([(self.POS, max_pressure),
                         (self.SPE, timeout),
                         (self.FOR, min_pressure),
                         (self.GTO, 1)])
        return self._set_vars(var_dict), min_pressure, max_pressure

    def move_and_wait_for_pos(self, min_pressure, max_pressure, timeout):
        set_ok, cmd_pos = self.move(min_pressure, max_pressure, timeout)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested pressure
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)

        # wait until not moving
        cur_obj = self._get_var(self.OBJ)
        while self.ObjectStatus(cur_obj) == self.ObjectStatus.MOVING:
            cur_obj = self._get_var(self.OBJ)

        # report the actual pressure and the object status
        final_pos = self._get_var(self.POS)
        final_obj = cur_obj
        return final_pos, self.ObjectStatus(final_obj)


if __name__ == '__main__':
    gripper = RobotiqEPickURCap('10.66.171.53')
    # gripper.connect('192.168.1.41', 63352)
    gripper.activate(True)
    gripper.move_and_wait_for_pos(100, 255, 255)
    gripper.move_and_wait_for_pos(150, 255, 255)
    gripper.move_and_wait_for_pos(100, 255, 255)
