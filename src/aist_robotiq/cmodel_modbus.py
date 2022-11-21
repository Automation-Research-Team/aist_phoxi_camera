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
import threading
from aist_robotiq.cmodel_base import CModelBase
from aist_robotiq.msg         import CModelStatus
from pymodbus.exceptions      import ModbusIOException
from pymodbus.client.sync     import ModbusTcpClient, ModbusSerialClient

#########################################################################
#  class CModelModbusBase                                               #
#########################################################################
class CModelModbusBase(CModelBase):
    def __init__(self, slave_id):
        super(CModelModbusBase, self).__init__()
        self._slave_id = slave_id

    def disconnect(self):
        if self._client:          # (self._client is defined in derived class)
            self._client.close()

    def put_command(self, command):
        # Clip each field of command within a valid range.
        command = self._clip_command(command)

        # Convert the command to a byte array of 6-length.
        self.data = []
        self.data.append(command.rACT +
                         (command.rMOD << 1) + (command.rGTO << 3) +
                         (command.rATR << 4) + (command.rARD << 5))  # Byte0
        self.data.append(0)                                          # Byte1
        self.data.append(0)                                          # Byte2
        self.data.append(command.rPR)                                # Byte3
        self.data.append(command.rSP)                                # Byte4
        self.data.append(command.rFR)                                # Byte5
        self._put_command(self.data)

    def get_status(self):
        # Acquire status from the Gripper
        data = self._get_status(6)

        # Assign the values to their respective variables
        status = CModelStatus()
        status.gACT =  data[0]       & 0x01
        status.gMOD = (data[0] >> 1) & 0x03
        status.gGTO = (data[0] >> 3) & 0x01
        status.gSTA = (data[0] >> 4) & 0x03
        status.gOBJ = (data[0] >> 6) & 0x03
        status.gVAS =  data[1]       & 0x03
        status.gFLT =  data[2]       & 0x0f
        status.gPR  =  data[3]
        status.gPO  =  data[4]
        status.gCU  =  data[5]
        return status

    def _put_command(self, data):
        # Make sure data has an even number of elements
        if len(data) % 2 == 1:
            data.append(0)

        # Compose every two bytes into one register word in big-endian order.
        message = []
        for i in range(0, len(data), 2):
            message.append((data[i] << 8) + data[i+1])
        self._write_registers(message)            # (defined in derived class)

    def _get_status(self, nbytes):
        nregs    = 2*((nbytes - 1)/2)
        response = self._read_registers(nregs)    # (defined in derived class)

        if isinstance(response, ModbusIOException):
            raise RuntimeError(response)

        # Decompose each register word to two bytes in little-endian order.
        data = []
        for val in response.registers:
            data.append((val & 0xFF00) >> 8)
            data.append( val & 0x00FF)
        return data

#########################################################################
#  class CModelModbusTCP                                                #
#########################################################################
class CModelModbusTCP(CModelModbusBase):
    def __init__(self, ip_address, slave_id=0x0009):
        super(CModelModbusTCP, self).__init__(slave_id)
        self._lock   = threading.Lock()
        self._client = ModbusTcpClient(ip_address)
        self._client.connect()

    def _write_registers(self, message):
        with self._lock:
            self._client.write_registers(0, message)

    def _read_registers(self, nregs):
        with self._lock:
            return self._client.read_input_registers(0, nregs)

#########################################################################
#  class CModelModbusRTU                                                #
#########################################################################
class CModelModbusRTU(CModelModbusBase):
    def __init__(self, port, slave_id=0x0009):
        super(CModelModbusRTU, self).__init__(slave_id)
        self._lock   = threading.Lock()
        self._client = ModbusSerialClient(method='rtu', port=port,
                                          stopbits=1, bytesize=8, parity='N',
                                          baudrate=115200, timeout=0.2)
        self._client.connect()

    def _write_registers(self, message):
        with self._lock:
            self._client.write_registers(0x03E8, message, unit=self._slave_id)

    def _read_registers(self, nregs):
        with self._lock:
            return self._client.read_input_registers(0x07D0, nregs,
                                                     unit=self._slave_id)
