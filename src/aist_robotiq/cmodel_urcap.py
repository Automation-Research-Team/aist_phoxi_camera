"""Module for controlling Robotiq's grippers"""
# BASED ON: https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85#latest
# Ported to ROS by felixvd
# Modified by T.Ueshiba

import rospy, socket, threading
from aist_robotiq.cmodel_base import CModelBase
from aist_robotiq.msg         import CModelStatus

#########################################################################
#  class CModelURCap                                                    #
#########################################################################
class CModelURCap(CModelBase):
    """
    Communicates with the gripper directly via socket with string commands,
    leveraging string names for variables.
    Uses port 63352 which is opened by the Robotiq Gripper URCap
    and receives ASCII commands.
    """
    # WRITE VARIABLES (CAN ALSO READ)
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    MOD = 'MOD'  # mod : mode for EPick suction gripper (0: auto, 1: advanced)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
    ATR = 'ATR'  # atr : auto-release (emergency slow move)
    ARD = 'ARD'  # ard : auto-release direction (open(1) or close(0) during auto-release)
    POS = 'POS'  # pos : position (0-255), 0 = open
    SPE = 'SPE'  # spe : speed (0-255)
    FOR = 'FOR'  # for : force (0-255)
    # READ VARIABLES
    STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
    OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    PRE = 'PRE'  # position request (echo of last commanded position)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)
    CUR = 'CUR'  # cur : current (0-255)

    ENCODING = 'UTF-8'  # ASCII and UTF-8 both seem to work

    def __init__(self, address):
        """
        Constructor
        """
        super(CModelURCap, self).__init__()
        self._lock   = threading.Lock()
        self._socket = self.connect(address)
        #self.activate()

    def connect(self, hostname, port=63352, socket_timeout=2.0):
        """
        Connects to a gripper at the given address.
        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        # print("Connecting to: " + str(hostname) + ", port: " + str(port))
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((hostname, port))
        s.settimeout(socket_timeout)
        return s

    def disconnect(self):
        """
        Closes the connection with the gripper
        """
        self._socket.close()

    def activate(self):
        # Clear and then reset ACT
        self._set_var(self.ACT, 0)
        self._set_var(self.ACT, 1)

        # Wait until STA == 3.
        while self._get_var(self.STA) != 3:
            rospy.sleep(0.01)

    def put_command(self, command):
        command  = self._clip_command(command)
        # Do not set variable 'ACT' because setting zero value will cause
        # the device reset.
        var_dict = dict([(self.MOD, command.rMOD),
                         (self.GTO, command.rGTO),
                         (self.ATR, command.rATR),
                         (self.ARD, command.rARD),
                         (self.POS, command.rPR),
                         (self.SPE, command.rSP),
                         (self.FOR, command.rFR)])
        self._set_vars(var_dict)

    def get_status(self):
        status = CModelStatus()
        # Assign values to their respective variables
        status.gACT = self._get_var(self.ACT)
        status.gMOD = self._get_var(self.MOD)
        status.gGTO = self._get_var(self.GTO)
        status.gSTA = self._get_var(self.STA)
        status.gOBJ = self._get_var(self.OBJ)
        status.gFLT = self._get_var(self.FLT)
        status.gPR  = self._get_var(self.PRE)
        status.gPO  = self._get_var(self.POS)
        #status.gCU  = self._get_var(self.CUR)
        return status

    def _set_vars(self, var_dict):
        """
        Sends the appropriate command via socket to set the value
        of n variables, and waits for its 'ack' response.

        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return:         True on successful reception of ack,
                         false if no ack was received,
                         indicating the set may not have been effective.
        """
        # construct unique command
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += " " + variable + " " + str(value)
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self._lock:
            self._socket.sendall(cmd.encode(self.ENCODING))
            data = self._socket.recv(1024)

        return self._is_ack(data)

    def _set_var(self, variable, value):
        """
        Sends the appropriate command via socket to set the value
        of a variable, and waits for its 'ack' response.

        :param variable: Variable to set.
        :param value:    Value to set for the variable.
        :return:         True on successful reception of ack,
                         false if no ack was received,
                         indicating the set may not have been effective.
        """
        return self._set_vars({variable:value})

    def _get_var(self, variable):
        """
        Sends the appropriate command to retrieve the value
        of a variable from the gripper, blocking until the response
        is received or the socket times out.

        :param variable: Name of the variable to retrieve.
        :return:         Value of the variable as integer.
        """
        # atomic commands send/rcv
        with self._lock:
            cmd = "GET " + variable + "\n"
            self._socket.sendall(cmd.encode(self.ENCODING))
            data = self._socket.recv(1024)

        # expect data of the form 'VAR x', where VAR is an echo
        # of the variable name, and X the value
        # note some special variables (like FLT) may send 2 bytes,
        # instead of an integer. We assume integer here
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError("Unexpected response " + str(data)
                             + " does not match '" + variable + "'")
        return int(value_str)

    @staticmethod
    def _is_ack(data):
        return data == b'ack'
