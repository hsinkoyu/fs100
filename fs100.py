#!/usr/bin/env python
#
# YASKAWA FS100 High Speed Ethernet Server Functions
#
# Copyright (c) 2019-2022 FIH Mobile Limited
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
#
# Authors:
#    Hsinko Yu <hsinkoyu@fih-foxconn.com>
#

import ntpath
import os
import socket
import struct
import threading
from enum import IntEnum


# common header excluding sub-header
class FS100PacketHeader:
    HEADER_IDENTIFIER = 'YERC'
    HEADER_SIZE = 0x20
    HEADER_RESERVED_1 = 3
    HEADER_DIVISION_ROBOT_CONTROL = 1
    HEADER_DIVISION_FILE_CONTROL = 2
    HEADER_ACK_REQUEST = 0
    HEADER_ACK_NOT_REQUEST = 1
    HEADER_BLOCK_NUMBER_REQ = 0
    HEADER_RESERVED_2 = '99999999'
    HEADER_PADDING = 0

    def __init__(self, packet=None, data_size=None, division=None, ack=None, req_id=None, block_no=None):
        if packet is not None:
            self.data_size = struct.unpack('<H', packet[6:8])[0]
            self.division = struct.unpack('B', packet[9:10])[0]
            self.ack = struct.unpack('B', packet[10:11])[0]
            self.req_id = struct.unpack('B', packet[11:12])[0]
            self.block_no = struct.unpack('<I', packet[12:16])[0]
        else:
            self.data_size = data_size
            self.division = division
            self.ack = ack
            self.req_id = req_id
            self.block_no = block_no

    def to_bytes(self):
        h = FS100PacketHeader.HEADER_IDENTIFIER.encode(encoding='ascii')
        h += struct.pack('<H', FS100PacketHeader.HEADER_SIZE)
        h += struct.pack('<H', self.data_size)
        h += struct.pack('B', FS100PacketHeader.HEADER_RESERVED_1)
        h += struct.pack('B', self.division)
        h += struct.pack('B', self.ack)
        h += struct.pack('B', self.req_id)
        h += struct.pack('<I', self.block_no)
        h += FS100PacketHeader.HEADER_RESERVED_2.encode(encoding='ascii')
        return h


class FS100ReqPacket(FS100PacketHeader):

    def __init__(self, division, req_id, cmd_no, inst, attr, service, data, data_size):
        FS100PacketHeader.__init__(self, None, data_size, division, FS100PacketHeader.HEADER_ACK_REQUEST, req_id,
                                   FS100PacketHeader.HEADER_BLOCK_NUMBER_REQ)
        self.cmd_no = cmd_no
        self.inst = inst
        self.attr = attr
        self.service = service
        self.data = data

    def to_bytes(self):
        h = FS100PacketHeader.to_bytes(self)
        h += struct.pack('<H', self.cmd_no)
        h += struct.pack('<H', self.inst)
        h += struct.pack('B', self.attr)
        h += struct.pack('B', self.service)
        h += struct.pack('<H', FS100PacketHeader.HEADER_PADDING)
        h += self.data
        return h


class FS100AnsPacket(FS100PacketHeader):

    def __init__(self, packet):
        FS100PacketHeader.__init__(self, packet)
        self.service = struct.unpack('B', packet[24:25])[0]
        self.status = struct.unpack('B', packet[25:26])[0]
        self.added_status_size = struct.unpack('B', packet[26:27])[0]
        self.added_status = struct.unpack('<H', packet[28:30])[0]
        self.data = packet[FS100PacketHeader.HEADER_SIZE:FS100PacketHeader.HEADER_SIZE + self.data_size]


class FS100:
    """YASKAWA FS100 High Speed Ethernet Server Functions

    This class implements most of YASKAWA FS100 High Speed Ethernet Server Functions.
    https://www.motoman.com/getmedia/16B5CD92-BD0B-4DE0-9DC9-B71D0B6FE264/160766-1CD.pdf.aspx?ext=.pdf

    Attributes:
        ip (str): IP address of the controller
        timeout (int, optional): Communication timeout value in second between PC and Controller.
            Defaults to 0.8.
        errno (int): Number of last error

    Methods:
        switch_power(): Turn on/off the power supply
        mov(): Make robot move to a specified position
        pmov(): Make robot move to a specified pulse position
        select_cycle(): Select the way a job in pendant plays
        select_job(): Select a job in pendant for later playing
        play_job(): Start playing a job in pendant
        read_executing_job_info(): Read the info of executing job
        read_axis_name(): Read the name of each axis
        read_position(): Read the robot position
        read_position_error(): Read the robot position error data
        read_torque(): Read the robot torque data of each axis
        read_variable(): Read a robot variable
        read_variables(): Read multiple robot variable with plural commands
        write_variable(): Write a robot variable
        get_status(): Retrieve various status of the robot
        read_alarm_info(): Retrieve info of the specified alarm
        get_last_alarm(): Retrieve info of the latest alarm
        reset_alarm(): To reset alarms or cancel errors
        acquire_system_info(): Acquire system information
        acquire_management_time(): Acquire usage time of an action
        show_text_on_pendant(): Show text on pendant
        get_file_list(): Retrieve list of files ended with extension in pendant
        send_file(): Send a local file to pendant
        recv_file(): Receive a file from pendant
        delete_file(): Delete a file in pendant
    """
    DEBUG = False

    UDP_PORT_ROBOT_CONTROL = 10040
    UDP_PORT_FILE_CONTROL = 10041

    TRANSMISSION_SEND = 1
    TRANSMISSION_SEND_AND_RECV = 2

    ERROR_SUCCESS = 0
    ERROR_CONNECTION = 1
    ERROR_NO_SUCH_FILE_OR_DIRECTORY = 2

    # power supply command
    POWER_TYPE_HOLD = 1
    POWER_TYPE_SERVO = 2
    POWER_TYPE_HLOCK = 3
    POWER_SWITCH_ON = 1
    POWER_SWITCH_OFF = 2

    # cycle selection command
    CYCLE_TYPE_STEP = 1
    CYCLE_TYPE_ONE_CYCLE = 2
    CYCLE_TYPE_CONTINUOUS = 3

    # move command
    MOVE_TYPE_JOINT_ABSOLUTE_POS = 1
    MOVE_TYPE_LINEAR_ABSOLUTE_POS = 2
    MOVE_TYPE_LINEAR_INCREMENTAL_POS = 3
    MOVE_SPEED_CLASS_PERCENT = 0  # for joint operation
    MOVE_SPEED_CLASS_MILLIMETER = 1
    MOVE_SPEED_CLASS_DEGREE = 2
    MOVE_COORDINATE_SYSTEM_BASE = 16
    MOVE_COORDINATE_SYSTEM_ROBOT = 17
    MOVE_COORDINATE_SYSTEM_USER = 18
    MOVE_COORDINATE_SYSTEM_TOOL = 19

    # reset alarm command
    RESET_ALARM_TYPE_ALARM = 1
    RESET_ALARM_TYPE_ERROR = 2

    # threading safe lock
    transmission_lock = threading.RLock()

    def __init__(self, ip, timeout=0.8):
        self.ip = ip
        self.timeout = timeout
        self._sock = None

        # number of last error
        self.errno = 0

    def _connect(self, port=UDP_PORT_ROBOT_CONTROL):
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(self.timeout)
            self._sock.connect((self.ip, port))

    def _disconnect(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _connected(self):
        return self._sock is not None

    def _generate_error_ans_packet(self, result, errno):
        # when error, result and error number are what callers care about
        p = bytearray(25)
        p += struct.pack('B', result)
        p += bytearray(2)
        p += struct.pack('<H', errno)
        p += bytearray(2)
        return p

    def _transmit(self, packet, direction=TRANSMISSION_SEND_AND_RECV):
        # transmission_lock is a reentrant lock. Owning thread may acquire it again without blocking.
        # This prevents sending robot control command during file control transmission.
        FS100.transmission_lock.acquire()

        if self._connected():
            to_disc = False
        else:
            self._connect()
            to_disc = True

        try:
            self._sock.sendall(packet)
            if FS100.DEBUG:
                print("PC -> FS100: {}, Len={}".format(packet, len(packet)))
            if direction == FS100.TRANSMISSION_SEND_AND_RECV:
                ans_packet, addr = self._sock.recvfrom(512)
                if FS100.DEBUG:
                    print("PC <- FS100: {}, Len={}".format(ans_packet, len(ans_packet)))
        except socket.error as error:
            print("ConnectionError: {}".format(error))
            errno = error.errno
            if errno is None:
                errno = FS100.ERROR_CONNECTION
            ans_packet = self._generate_error_ans_packet(FS100.ERROR_CONNECTION, errno)
        finally:
            pass

        if direction == FS100.TRANSMISSION_SEND_AND_RECV:
            ans = FS100AnsPacket(ans_packet)
        else:
            ans = None

        if to_disc:
            self._disconnect()

        FS100.transmission_lock.release()
        return ans

    def switch_power(self, power_type, switch):
        """Turn on/off the power supply

        Args:
            power_type (int): Type of power supply. One of following:
                FS100.POWER_TYPE_HOLD,
                FS100.POWER_TYPE_SERVO,
                FS100.POWER_TYPE_HLOCK
            switch (int): Type of switch. One of following:
                FS100.POWER_SWITCH_ON,
                FS100.POWER_SWITCH_OFF

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x83, power_type, 0x01, 0x10,
                             struct.pack('<I', switch), 4)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed switching power supply, err={}".format(hex(ans.added_status)))
        return ans.status

    def select_cycle(self, cycle_type):
        """Select the way a job in pendant plays

        Args:
            cycle_type (int): Type of playing. One of following:
                FS100.CYCLE_TYPE_STEP,
                FS100.CYCLE_TYPE_ONE_CYCLE,
                FS100.CYCLE_TYPE_CONTINUOUS

        Note:
            If the robot is in hold, FS100.CYCLE_TYPE_CONTINUOUS can be selected
            to resume playing.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x84, 2, 0x01, 0x10,
                             struct.pack('<I', cycle_type), 4)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed to select cycle, err={}".format(hex(ans.added_status)))
        return ans.status

    def mov(self, move_type, coordinate, speed_class, speed, pos, form=0, extended_form=0, robot_no=1,
                 station_no=0, tool_no=0, user_coor_no=0):
        """Make robot move to a specified position

        Args:
            move_type (int): Type of move path. One of following:
                FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS,
                FS100.MOVE_TYPE_LINEAR_ABSOLUTE_POS,
                FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS
            coordinate (int): Coordinate system. One of following:
                FS100.MOVE_COORDINATE_SYSTEM_BASE,
                FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                FS100.MOVE_COORDINATE_SYSTEM_USER,
                FS100.MOVE_COORDINATE_SYSTEM_TOOL
            speed_class (int): Type of move speed. One of following:
                FS100.MOVE_SPEED_CLASS_PERCENT,
                FS100.MOVE_SPEED_CLASS_MILLIMETER,
                FS100.MOVE_SPEED_CLASS_DEGREE
            speed (int): Move speed.
                in 0.01 % for speed type FS100.MOVE_SPEED_CLASS_PERCENT,
                in 0.1 mm/s for speed type FS100.MOVE_SPEED_CLASS_MILLIMETER,
                in 0.1 degree/s for speed type FS100.MOVE_SPEED_CLASS_DEGREE
            pos (tuple): Target position in tuple (x, y, z, Rx, Ry, Rz, Re). x, y, z are in 0.000001 m,
                whereas Rx, Ry, Rz, Re are in 0.0001 degree
            form (int, optional): Robot pose. Defaults to 0.
            extended_form (int, optional): Robot extended pose. Defaults to 0.
            robot_no (int, optional): Robot number (1 to 2). Defaults to 1.
            station_no (int, optional): Station number. Defaults to 0.
            tool_no (int, optional): Tool number (0 to 63). Defaults to 0.
            user_coor_no (int, optional): User coordinate number (0 to 63). Defaults to 0.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        data = struct.pack('<I', robot_no)
        data += struct.pack('<I', station_no)
        data += struct.pack('<I', speed_class)
        data += struct.pack('<I', speed)
        data += struct.pack('<I', coordinate)
        data += struct.pack('<iiiiiii', pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6])
        data += struct.pack('<I', 0)  # reserved
        data += struct.pack('<I', form)
        data += struct.pack('<I', extended_form)
        data += struct.pack('<I', tool_no)
        data += struct.pack('<I', user_coor_no)
        data += bytearray(36)

        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x8a, move_type, 0x01, 0x02, data,
                             len(data))
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed moving to the target position, err={}".format(hex(ans.added_status)))
        return ans.status

    def pmov(self, move_type, speed_class, speed, pulse, robot_no=1, station_no=0, tool_no=0):
        """Make robot move to a specified pulse position

        Args:
            move_type (int): Type of move path. One of following:
                FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS,
                FS100.MOVE_TYPE_LINEAR_ABSOLUTE_POS
            speed_class (int): Type of move speed. One of following:
                FS100.MOVE_SPEED_CLASS_PERCENT,
                FS100.MOVE_SPEED_CLASS_MILLIMETER,
                FS100.MOVE_SPEED_CLASS_DEGREE
            speed (int): Move speed.
                in 0.01 % for speed type FS100.MOVE_SPEED_CLASS_PERCENT,
                in 0.1 mm/s for speed type FS100.MOVE_SPEED_CLASS_MILLIMETER,
                in 0.1 degree/s for speed type FS100.MOVE_SPEED_CLASS_DEGREE
            pulse (tuple): Target position in tuple (S, L, U, R, B, T, E).
            robot_no (int, optional): Robot number (1 to 2). Defaults to 1.
            station_no (int, optional): Station number. Defaults to 0.
            tool_no (int, optional): Tool number (0 to 63). Defaults to 0.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        data = struct.pack('<I', robot_no)
        data += struct.pack('<I', station_no)
        data += struct.pack('<I', speed_class)
        data += struct.pack('<I', speed)
        data += struct.pack('<iiiiiii', pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6])
        data += struct.pack('<I', 0)  # reserved
        data += struct.pack('<I', tool_no)
        data += bytearray(36)

        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x8b, move_type, 0x01, 0x02, data,
                             len(data))
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed moving to the target position, err={}".format(hex(ans.added_status)))
        return ans.status

    def get_last_alarm(self, alarm):
        """Retrieve info of the latest alarm

        Args:
            alarm (dict): Where the retrieved info is written to

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x70, 1, 0, 0x01, bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed getting the last alarm, err={}".format(hex(ans.added_status)))
        else:
            alarm['code'] = struct.unpack('<I', ans.data[0:4])[0]
            alarm['data'] = struct.unpack('<I', ans.data[4:8])[0]
            alarm['type'] = struct.unpack('<I', ans.data[8:12])[0]
            alarm['time'] = ans.data[12:28].decode('ascii').rstrip('\x00')
            alarm['name'] = ans.data[28:60].decode('utf-8').rstrip('\x00')
        return ans.status

    def read_alarm_info(self, alarm_num, alarm_info):
        """Retrieve info of the specified alarm

        Args:
            alarm_num (int): 1 to 100    : Major failure
                             1001 to 1100: Monitor alarm
                             2001 to 2100: User alarm (system)
                             3001 to 3100: User alarm (user)
                             4001 to 4100: Off line alarm
            alarm_info (dict): Where the retrieved info is written to

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x71, alarm_num, 0, 0x01, bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed to read the alarm info, err={}".format(hex(ans.added_status)))
        else:
            alarm_info['code'] = struct.unpack('<I', ans.data[0:4])[0]
            alarm_info['data'] = struct.unpack('<I', ans.data[4:8])[0]
            alarm_info['type'] = struct.unpack('<I', ans.data[8:12])[0]
            alarm_info['time'] = ans.data[12:28].decode('ascii').rstrip('\x00')
            alarm_info['name'] = ans.data[28:60].decode('utf-8').rstrip('\x00')
        return ans.status

    def reset_alarm(self, alarm_type):
        """To reset alarms or cancel errors

        Args:
            alarm_type (int): Type of alarm. One of following:
                FS100.RESET_ALARM_TYPE_ALARM,
                FS100.RESET_ALARM_TYPE_ERROR

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x82, alarm_type, 1, 0x10,
                             struct.pack('<I', 1), 4)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed resetting alarms or cancelling errors, err={}".format(hex(ans.added_status)))
        return ans.status

    def get_status(self, status):
        """Retrieve various status of the robot

        Args:
            status (dict): Where the retrieved status is written to

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x72, 1, 0, 0x01, bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed getting the status, err={}".format(hex(ans.added_status)))
        else:
            data_1 = struct.unpack('<I', ans.data[0:4])[0]
            data_2 = struct.unpack('<I', ans.data[4:8])[0]
            status['step'] = bool(data_1 & 0x01)
            status['one_cycle'] = bool(data_1 & 0x02)
            status['auto_and_cont'] = bool(data_1 & 0x04)
            status['running'] = bool(data_1 & 0x08)
            status['guard_safe'] = bool(data_1 & 0x10)
            status['teach'] = bool(data_1 & 0x20)
            status['play'] = bool(data_1 & 0x40)
            status['cmd_remote'] = bool(data_1 & 0x80)
            status['hold_by_pendant'] = bool(data_2 & 0x02)
            status['hold_externally'] = bool(data_2 & 0x04)
            status['hold_by_cmd'] = bool(data_2 & 0x08)
            status['alarming'] = bool(data_2 & 0x10)
            status['error_occurring'] = bool(data_2 & 0x20)
            status['servo_on'] = bool(data_2 & 0x40)
        return ans.status

    def read_executing_job_info(self, info):
        """Read the info of executing job

        Args:
            info (dict): Where the job info is stored

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x73, 1, 0, 0x01, bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed to read the info of executing job, err={}".format(hex(ans.added_status)))
        else:
            info['job_name'] = ans.data[0:32].decode('utf-8').rstrip('\x00')
            info['line_num'] = struct.unpack('<I', ans.data[32:36])[0]
            info['step_num'] = struct.unpack('<I', ans.data[36:40])[0]
            info['speed_override_value'] = struct.unpack('<I', ans.data[40:44])[0]
        return ans.status

    def play_job(self):
        """Start playing a job in pendant

        Note:
            select_job() should be performed before this method.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x86, 1, 1, 0x10,
                             struct.pack('<I', 1), 4)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed playing job, err={}".format(hex(ans.added_status)))
        return ans.status

    def select_job(self, job_name, line_num=0):
        """Select a job in pendant for later playing

        Args:
            job_name (str): Name of the job file
            line_num (int, optional): The beginning line number when playing. Defaults to 0.

        Raises:
            ValueError: Length of the job name exceeds the maximum 32 characters.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if job_name.upper().endswith('.JBI'):
            job_name = job_name[:-4]
        data = job_name.encode(encoding='utf-8')
        if len(data) > 32:
            raise ValueError('Job name is too long')
        data += bytearray(32 - len(data))
        data += struct.pack('<I', line_num)
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x87, 1, 0, 0x02, data, len(data))
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed selecting the job, err={}".format(hex(ans.added_status)))
        return ans.status

    def delete_file(self, file_name):
        """Delete a file in pendant

        Args:
            file_name (str): Name of the file to be deleted

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        self.transmission_lock.acquire()
        self._connect(FS100.UDP_PORT_FILE_CONTROL)
        data = file_name.encode(encoding='utf-8')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x09, data, len(data))
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed deleting the file, err={}".format(hex(ans.added_status)))
        self._disconnect()
        self.transmission_lock.release()
        return ans.status

    def get_file_list(self, extension, list):
        """Retrieve list of files ended with `extension` in pendant

        Args:
            extension (str): Should be '*.JBI', '*.DAT', '*.CND', '*.PRM', '*.SYS' or '*.LST'
            list (list): Where to store the retrieved list

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        self.transmission_lock.acquire()
        self._connect(FS100.UDP_PORT_FILE_CONTROL)
        raw = ''
        data = extension.encode(encoding='ascii')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x32, data, len(data))
        ans = self._transmit(req.to_bytes())
        while ans.status == FS100.ERROR_SUCCESS:
            raw += ans.data.decode('utf-8')
            req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x32, bytearray(0), 0)
            # not a request but an ack
            req.ack = FS100PacketHeader.HEADER_ACK_NOT_REQUEST
            req.block_no = ans.block_no
            if ans.block_no & 0x80000000 != 0:
                # The last file list data arrived. Send the final ack.
                self._transmit(req.to_bytes(), FS100.TRANSMISSION_SEND)
                break
            else:
                ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed getting the file list, err={}".format(hex(ans.added_status)))
        else:
            list.extend(raw.splitlines())
        self._disconnect()
        self.transmission_lock.release()
        return ans.status

    def send_file(self, filename):
        """Send a local file to pendant

        Args:
            filename (str): Path of the local file

        Raises:
            ValueError: Empty content of the local file

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        try:
            with open(filename, 'rb') as f:
                context = f.read()
        except FileNotFoundError:
            print("file not found: '{}'".format(filename))
            self.errno = 0xe2b3
            return FS100.ERROR_NO_SUCH_FILE_OR_DIRECTORY

        if len(context) == 0:
            raise ValueError('An empty file')

        self.transmission_lock.acquire()
        self._connect(FS100.UDP_PORT_FILE_CONTROL)

        CHUNK_SIZE = 400
        block_no = 0
        data = ntpath.basename(filename).encode(encoding='utf-8')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x15, data, len(data))
        ans = self._transmit(req.to_bytes())
        while ans.status == FS100.ERROR_SUCCESS:
            if ans.block_no & 0x80000000 != 0:
                # we have sent the last piece of data and this is the final ack from controller
                break
            block_no += 1
            if CHUNK_SIZE * block_no >= len(context):
                data = context[(block_no - 1) * CHUNK_SIZE:]
                block_no |= 0x80000000
            else:
                data = context[(block_no - 1) * CHUNK_SIZE:block_no * CHUNK_SIZE]
            req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x15, data, len(data))
            req.ack = FS100PacketHeader.HEADER_ACK_NOT_REQUEST
            req.block_no = block_no
            ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed sending the file, err={}".format(hex(ans.added_status)))

        self._disconnect()
        self.transmission_lock.release()
        return ans.status

    def recv_file(self, filename, local_dir):
        """Receive a file from pendant

        Args:
            filename (str): Name of the file in pendant
            local_dir (str): Where in local to save the file

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if not os.path.isdir(local_dir):
            print("directory not found: '{}'".format(local_dir))
            self.errno = 0xe2b3
            return FS100.ERROR_NO_SUCH_FILE_OR_DIRECTORY

        self.transmission_lock.acquire()
        self._connect(FS100.UDP_PORT_FILE_CONTROL)

        context = bytearray(0)
        data = filename.encode(encoding='utf-8')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x16, data, len(data))
        ans = self._transmit(req.to_bytes())
        while ans.status == FS100.ERROR_SUCCESS:
            context += ans.data
            req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x16, bytearray(0), 0)
            # not a request but an ack
            req.ack = FS100PacketHeader.HEADER_ACK_NOT_REQUEST
            req.block_no = ans.block_no
            if ans.block_no & 0x80000000 != 0:
                # Got the last piece of file. Send the final ack.
                self._transmit(req.to_bytes(), FS100.TRANSMISSION_SEND)
                break
            else:
                ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed receiving the file, err={}".format(hex(ans.added_status)))
        else:
            with open("{}/{}".format(local_dir, filename), 'wb') as f:
                f.write(context)

        self._disconnect()
        self.transmission_lock.release()
        return ans.status

    def read_axis_name(self, axis_name, robot_no=101):
        """Read the name of each axis

        Args:
            axis_name (dict): Where the name of each axis is stored
            robot_no (int, optional): Robot number. Defaults to 101 (R1 in cartesian coordinate).

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x74, robot_no, 0, 0x01,
                             bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed to read the name of each axis, err={}".format(hex(ans.added_status)))
        else:
            axis_name['1st_axis'] = ans.data[0:4].decode('utf-8').rstrip('\x00')
            axis_name['2nd_axis'] = ans.data[4:8].decode('utf-8').rstrip('\x00')
            axis_name['3rd_axis'] = ans.data[8:12].decode('utf-8').rstrip('\x00')
            axis_name['4th_axis'] = ans.data[12:16].decode('utf-8').rstrip('\x00')
            axis_name['5th_axis'] = ans.data[16:20].decode('utf-8').rstrip('\x00')
            axis_name['6th_axis'] = ans.data[20:24].decode('utf-8').rstrip('\x00')
            axis_name['7th_axis'] = ans.data[24:28].decode('utf-8').rstrip('\x00')
        return ans.status

    def read_position(self, pos_info, robot_no=101):
        """Read the robot position

        Args:
            pos_info (dict): Where the robot position data is stored
            robot_no (int, optional): Robot number. Defaults to 101 (R1 in cartesian coordinate).

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x75, robot_no, 0, 0x01,
                             bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed reading the position info, err={}".format(hex(ans.added_status)))
        else:
            pos_info['data_type'] = struct.unpack('<I', ans.data[0:4])[0]
            pos_info['form'] = struct.unpack('<I', ans.data[4:8])[0]
            pos_info['tool_no'] = struct.unpack('<I', ans.data[8:12])[0]
            pos_info['user_coor_no'] = struct.unpack('<I', ans.data[12:16])[0]
            pos_info['extended_form'] = struct.unpack('<I', ans.data[16:20])[0]
            p = list()
            for n in range(20, len(ans.data), 4):
                p.append(struct.unpack('<i', ans.data[n:n+4])[0])
            pos_info['pos'] = tuple(p)
        return ans.status

    def read_position_error(self, axis_data, robot_no=1):
        """Read the robot position error data

        Args:
            axis_data (dict): Where the robot position error data is stored
            robot_no (int, optional): Robot number. Defaults to 1.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x76, robot_no, 0, 0x01,
                             bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed reading the position error info, err={}".format(hex(ans.added_status)))
        else:
            axis_data['1st_axis'] = struct.unpack('<I', ans.data[0:4])[0]
            axis_data['2nd_axis'] = struct.unpack('<I', ans.data[4:8])[0]
            axis_data['3rd_axis'] = struct.unpack('<I', ans.data[8:12])[0]
            axis_data['4th_axis'] = struct.unpack('<I', ans.data[12:16])[0]
            axis_data['5th_axis'] = struct.unpack('<I', ans.data[16:20])[0]
            axis_data['6th_axis'] = struct.unpack('<i', ans.data[20:24])[0]
            axis_data['7th_axis'] = struct.unpack('<i', ans.data[24:28])[0]
        return ans.status

    def read_torque(self, torque_data, robot_no=1):
        """Read the robot torque data of each axis

        Args:
            torque_data (dict): Where the robot torque data is stored
            robot_no (int, optional): Robot number. Defaults to 1.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x77, robot_no, 0, 0x01,
                             bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed to read the torque data, err={}".format(hex(ans.added_status)))
        else:
            torque_data['1st_axis'] = struct.unpack('<i', ans.data[0:4])[0]
            torque_data['2nd_axis'] = struct.unpack('<i', ans.data[4:8])[0]
            torque_data['3rd_axis'] = struct.unpack('<i', ans.data[8:12])[0]
            torque_data['4th_axis'] = struct.unpack('<i', ans.data[12:16])[0]
            torque_data['5th_axis'] = struct.unpack('<i', ans.data[16:20])[0]
            torque_data['6th_axis'] = struct.unpack('<i', ans.data[20:24])[0]
            torque_data['7th_axis'] = struct.unpack('<i', ans.data[24:28])[0]
        return ans.status

    class VarType(IntEnum):
        IO = 0x78  # 1 byte
        REGISTER = 0x79  # 2 bytes
        BYTE = 0x7a  # 1 byte
        INTEGER = 0x7b  # 2 bytes
        DOUBLE = 0x7c  # 4 bytes
        REAL = 0x7d  # 4 bytes
        STRING = 0x7e  # max. 16 bytes
        ROBOT_POSITION = 0x7f
        BASE_POSITION = 0x80
        EXTERNAL_AXIS = 0x81

    class Variable:
        """FS100 Variable

        Attributes:
            type (FS100.VarType): Variable type
            num (int): Variable number
            val (*, optional): Variable value
        """
        def __init__(self, type, num, val=None):
            self.type = type
            self.num = num
            self.val = val

        def set_val(self, raw_bytes):
            if self.type == FS100.VarType.IO:
                self.val = struct.unpack('B', raw_bytes[0:1])[0]
            elif self.type == FS100.VarType.REGISTER:
                self.val = struct.unpack('<H', raw_bytes[0:2])[0]
            elif self.type == FS100.VarType.BYTE:
                self.val = struct.unpack('B', raw_bytes[0:1])[0]
            elif self.type == FS100.VarType.INTEGER:
                self.val = struct.unpack('<h', raw_bytes[0:2])[0]
            elif self.type == FS100.VarType.DOUBLE:
                self.val = struct.unpack('<i', raw_bytes[0:4])[0]
            elif self.type == FS100.VarType.REAL:
                self.val = struct.unpack('<f', raw_bytes[0:4])[0]
            elif self.type == FS100.VarType.STRING:
                self.val = raw_bytes.decode('utf-8')
            elif self.type == FS100.VarType.ROBOT_POSITION:
                if self.val is None: self.val = {}
                self.val['data_type'] = struct.unpack('<I', raw_bytes[0:4])[0]
                self.val['form'] = struct.unpack('<I', raw_bytes[4:8])[0]
                self.val['tool_no'] = struct.unpack('<I', raw_bytes[8:12])[0]
                self.val['user_coor_no'] = struct.unpack('<I', raw_bytes[12:16])[0]
                self.val['extended_form'] = struct.unpack('<I', raw_bytes[16:20])[0]
                self.val['pos'] = (struct.unpack('<i', raw_bytes[20:24])[0],
                                   struct.unpack('<i', raw_bytes[24:28])[0],
                                   struct.unpack('<i', raw_bytes[28:32])[0],
                                   struct.unpack('<i', raw_bytes[32:36])[0],
                                   struct.unpack('<i', raw_bytes[36:40])[0],
                                   struct.unpack('<i', raw_bytes[40:44])[0],
                                   struct.unpack('<i', raw_bytes[44:48])[0])
            elif self.type in (FS100.VarType.BASE_POSITION, FS100.VarType.EXTERNAL_AXIS):
                if self.val is None: self.val = {}
                self.val['data_type'] = struct.unpack('<I', raw_bytes[0:4])[0]
                self.val['1st_axis'] = struct.unpack('<i', raw_bytes[4:8])[0]
                self.val['2nd_axis'] = struct.unpack('<i', raw_bytes[8:12])[0]
                self.val['3rd_axis'] = struct.unpack('<i', raw_bytes[12:16])[0]
                self.val['4th_axis'] = struct.unpack('<i', raw_bytes[16:20])[0]
                self.val['5th_axis'] = struct.unpack('<i', raw_bytes[20:24])[0]
                self.val['6th_axis'] = struct.unpack('<i', raw_bytes[24:28])[0]
                self.val['7th_axis'] = struct.unpack('<i', raw_bytes[28:32])[0]

        def val_to_bytes(self):
            ret = None
            if self.type == FS100.VarType.IO:
                ret = struct.pack('B', self.val)
            elif self.type == FS100.VarType.REGISTER:
                ret = struct.pack('<H', self.val)
            elif self.type == FS100.VarType.BYTE:
                ret = struct.pack('B', self.val)
            elif self.type == FS100.VarType.INTEGER:
                ret = struct.pack('<h', self.val)
            elif self.type == FS100.VarType.DOUBLE:
                ret = struct.pack('<i', self.val)
            elif self.type == FS100.VarType.REAL:
                ret = struct.pack('<f', self.val)
            elif self.type == FS100.VarType.STRING:
                ret = self.val.encode(encoding='utf-8')
            elif self.type == FS100.VarType.ROBOT_POSITION:
                pos = self.val['pos']
                ret = struct.pack('<I', self.val['data_type'])
                ret += struct.pack('<I', self.val['form'])
                ret += struct.pack('<I', self.val['tool_no'])
                ret += struct.pack('<I', self.val['user_coor_no'])
                ret += struct.pack('<I', self.val['extended_form'])
                ret += struct.pack('<iiiiiii', pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6])
            elif self.type in (FS100.VarType.BASE_POSITION, FS100.VarType.EXTERNAL_AXIS):
                ret = struct.pack('<I', self.val['data_type'])
                ret += struct.pack('<i', self.val['1st_axis'])
                ret += struct.pack('<i', self.val['2nd_axis'])
                ret += struct.pack('<i', self.val['3rd_axis'])
                ret += struct.pack('<i', self.val['4th_axis'])
                ret += struct.pack('<i', self.val['5th_axis'])
                ret += struct.pack('<i', self.val['6th_axis'])
                ret += struct.pack('<i', self.val['7th_axis'])
            return ret

    def read_variable(self, var):
        """Read a robot variable

        Args:
            var (FS100.Variable): The variable being read

        Note:
            Value of the variable is stored in `val` attribute of var.

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> var_b0 = FS100.Variable(FS100.VarType.BYTE, 0)
            >>> robot.read_variable(var_b0)
            >>> print("var_b0={}".format(var_b0.val))

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if FS100.DEBUG:
            print("FS100.read_variable(VarType={}, {})".format(var.type, var.num))

        attr = 1
        service = 0x0e
        if var.type in (FS100.VarType.ROBOT_POSITION, FS100.VarType.BASE_POSITION, FS100.VarType.EXTERNAL_AXIS):
            # get all attributes
            attr = 0
            service = 0x01
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, var.type, var.num, attr, service,
                             bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed reading the variable, err={}".format(hex(ans.added_status)))
        else:
            var.set_val(ans.data)
        return ans.status

    def _group_nums(self, list):
        list2 = sorted(list)
        sublist = []

        while list2:
            v = list2.pop(0)

            if not sublist or sublist[-1] in [v, v-1]:
                sublist.append(v)
            else:
                yield sublist
                sublist = [v]

        if sublist:
            yield sublist

    def _read_consecutive_variables(self, vars):
        """Read multiple consecutive robot variables (of the same type)

        Args:
            vars (list[FS100.Variable]): The variables being read

        Raises:
            ValueError: Empty list given or doesn't contain var objects with the same type or
            var objects numbers not consecutive

        Note:
            Value of each variable is stored in `val` attribute of each var object.
            All var objects in vars must have the same type (i.e., INTEGER) and must have
            consecutive numbers, otherwise failure is immediately returned

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> vars = [FS100.Variable(FS100.VarType.INTEGER, 0), FS100.Variable(FS100.VarType.INTEGER, 1)]
            >>> robot.read_variables(vars)
            >>> print([(v.num, v.val) for v in vars])

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if FS100.DEBUG:
            print("FS100._read_consecutive_variables(VarType={}, list={})".format(vars[0].type if len(vars) > 0 else "", [v.num for v in vars]))

        attr = 0
        service = 0x33
        
        if len(vars) == 0:
            raise ValueError('Input list cannot be empty')
        
        # first variable gives the starting number and type and single values size
        var = vars[0]
        var_type_size = len(FS100.Variable(var.type, 0, 0).val_to_bytes())
        
        if any([v.type != var.type for v in vars]):
            raise ValueError('Input list must contain var objects of the same type')

        nums = [v.num for v in vars]
        if sorted(nums) != list(range(min(nums), max(nums) + 1)):
            raise ValueError('Input list must contain var objects with consecutive numbers')

        # data contains number of variables to be read from the starting number
        var_count = len(vars)
        if var_type_size == 1 and var_count % 2 == 1:
            # the protocol requires multiple of 2 for 1-byte variable types
            var_count += 1
        data = struct.pack('<I', var_count)

        # plural commands start with 0x300, which is var.type + 0x288
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, var.type + 0x288, var.num, attr, service,
                             data, len(data))
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed reading variables, err={}".format(hex(ans.added_status)))
        else:
            for ix, v in enumerate(vars):
                # returned data start with 4 bytes with number of variables, then individual consecutive values follow
                v.set_val(ans.data[4 + var_type_size * ix:4 + var_type_size * (ix + 1)])
        return ans.status

    def read_variables(self, vars):
        """Read multiple robot variables (of the same type)

        Args:
            vars (list[FS100.Variable]): The variables being read

        Raises:
            ValueError: Empty list given or doesn't contain var objects with the same type

        Note:
            Internally it will divide vars list into sublists containing var objects
            with consecutive numbers. Then it will execute `_read_consecutive_variables`
            method to read them from the robot in one call (using HSE plural command).
            
            Value of each variable is stored in `val` attribute of each var object.
            All var objects in vars must have the same type (i.e., INTEGER), otherwise
            failure is immediately returned

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> vars = [FS100.Variable(FS100.VarType.INTEGER, 3), FS100.Variable(FS100.VarType.INTEGER, 5), FS100.Variable(FS100.VarType.INTEGER, 6)]
            >>> robot.read_variables(vars)
            >>> print([(v.num, v.val) for v in vars])

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        
        # groups list of vars to list of lists of consecutive vars
        vars_dict = {v.num: v for v in vars}
        keys = [k for k, v in vars_dict.items()]
        keys_lists = [sublist for sublist in self._group_nums(keys)]

        ret = FS100.ERROR_SUCCESS
        for keys_consecutive in keys_lists:
            if len(keys_consecutive) == 1: ret |= self.read_variable(vars_dict[keys_consecutive[0]])
            elif len(keys_consecutive) > 1 and vars_dict[keys_consecutive[0]].type == FS100.VarType.STRING:
                for k in keys_consecutive: ret |= self.read_variable(vars_dict[k])
            elif len(keys_consecutive) > 1: ret |= self._read_consecutive_variables([vars_dict[k] for k in keys_consecutive])

        return ret

    def write_variable(self, var):
        """Write a robot variable

        Args:
            var (FS100.Variable): The variable being written

        Note:
            Value for the variable is specified in `val` attribute.

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> var_s1 = FS100.Variable(FS100.VarType.STRING, 1, 'Hello, World!')
            >>> robot.write_variable(var_s1)

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        attr = 1
        service = 0x10
        if var.type in (FS100.VarType.ROBOT_POSITION, FS100.VarType.BASE_POSITION, FS100.VarType.EXTERNAL_AXIS):
            # set all attributes
            attr = 0
            service = 0x02
        data = var.val_to_bytes()
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, var.type, var.num, attr, service,
                             data, len(data))
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed writing the variable, err={}".format(hex(ans.added_status)))
        return ans.status

    class SystemInfoType(IntEnum):
        R1 = 11
        R2 = 12
        S1 = 21
        S2 = 22
        S3 = 23
        APPLICATION = 101

    def acquire_system_info(self, type, info):
        """Acquire system information

        Args:
            type (FS100.SystemInfoType): Type of system
            info (dict): Where acquired information is stored

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x89, type, 0, 0x01, bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed acquiring system info, err={}".format(hex(ans.added_status)))
        else:
            info['software_version'] = ans.data[0:24].decode('utf-8')
            info['model'] = ans.data[24:40].decode('utf-8')
            info['parameter_version'] = ans.data[40:48].decode('utf-8')
        return ans.status

    class ManagementTimeType(IntEnum):
        CONTROL_POWER_ON = 1
        SERVO_POWER_ON_TOTAL = 10
        SERVO_POWER_ON_R1 = 11
        SERVO_POWER_ON_R2 = 12
        SERVO_POWER_ON_S1 = 21
        SERVO_POWER_ON_S2 = 22
        SERVO_POWER_ON_S3 = 23
        PLAYBACK_TOTAL = 110
        PLAYBACK_R1 = 111
        PLAYBACK_R2 = 112
        PLAYBACK_S1 = 121
        PLAYBACK_S2 = 122
        PLAYBACK_S3 = 123
        MOTION_TOTAL = 210
        MOTION_R1 = 211
        MOTION_R2 = 212
        MOTION_S1 = 221
        MOTION_S2 = 222
        MOTION_S3 = 223
        OPERATION = 301

    def acquire_management_time(self, type, time):
        """Acquire usage time of an action

        Args:
            type (FS100.ManagementTimeType): Type of action
            time (dict): Where acquired usage time is stored

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x88, type, 0, 0x01, bytearray(0), 0)
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed acquiring management time, err={}".format(hex(ans.added_status)))
        else:
            time['start'] = ans.data[0:16].decode('utf-8')
            time['elapse'] = ans.data[16:28].decode('utf-8')
        return ans.status

    def show_text_on_pendant(self, text):
        """Show text on pendant

        Args:
            text (str): Text to be shown on pendant

        Raises:
            ValueError: Length of the text exceeds the maximum 30 characters.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        data = text.encode(encoding='utf-8')
        if len(data) > 30:
            raise ValueError('Text is too long')
        data += bytearray(32 - len(data))
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x85, 1, 1, 0x10, data, len(data))
        ans = self._transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed to show text on pendant, err={}".format(hex(ans.added_status)))
        return ans.status
