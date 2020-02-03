#!/usr/bin/python2
#
# YASKAWA FS100 High Speed Ethernet Server Functions
#
# Copyright (C) 2019 FIH Mobile Limited
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# Authors:
#    Hsinko Yu <hsinkoyu@fih-foxconn.com>
#

import ntpath
import os
import socket
import struct
import threading
import time
from datetime import datetime
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


# request packet
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

    def clone(self, data=None):
        if data is None:
            data = self.data
        data_size = len(data)
        return FS100ReqPacket(self.division, self.req_id, self.cmd_no, self.inst, self.attr, self.service, data,
                              data_size)


# answer packet
class FS100AnsPacket(FS100PacketHeader):

    def __init__(self, packet):
        FS100PacketHeader.__init__(self, packet)
        self.service = struct.unpack('B', packet[24:25])[0]
        self.status = struct.unpack('B', packet[25:26])[0]
        self.added_status_size = struct.unpack('B', packet[26:27])[0]
        self.added_status = struct.unpack('<H', packet[28:30])[0]
        self.data = packet[FS100PacketHeader.HEADER_SIZE:FS100PacketHeader.HEADER_SIZE + self.data_size]

    # for debug purpose
    def to_bytes(self):
        h = FS100PacketHeader.to_bytes(self)
        h += struct.pack('B', self.service)
        h += struct.pack('B', self.status)
        h += struct.pack('B', self.added_status_size)
        h += struct.pack('B', FS100PacketHeader.HEADER_PADDING)
        h += struct.pack('<H', self.added_status)
        h += struct.pack('<H', FS100PacketHeader.HEADER_PADDING)
        h += self.data
        return h


class FS100:
    DEBUG = False

    UDP_PORT_ROBOT_CONTROL = 10040
    UDP_PORT_FILE_CONTROL = 10041

    TRANSMISSION_SEND = 1
    TRANSMISSION_SEND_AND_RECV = 2

    ERROR_SUCCESS = 0
    ERROR_CONNECTION = 1
    ERROR_NO_SUCH_FILE_OR_DIRECTORY = 2

    TRAVEL_STATUS_POLLING_DURATION = 0.1  # sec.
    TRAVEL_STATUS_START = 0
    TRAVEL_STATUS_END = 0xffffffff
    TRAVEL_STATUS_ERROR = -1  # errno for details

    # power supply command
    POWER_TYPE_HOLD = 1
    POWER_TYPE_SERVO = 2
    POWER_TYPE_HLOCK = 3
    POWER_SWITCH_ON = 1
    POWER_SWITCH_OFF = 2

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
    transmission_lock = threading.Lock()
    # TODO: no robot control command during file control transmissions
    file_control_lock = threading.Lock()

    def __init__(self, ip, timeout=0.8):
        self.ip = ip
        self.timeout = timeout
        self.sock = None

        # number of last error
        self.errno = 0

        # traveller thread
        self.traveller_thread = None
        self.stop_travelling = False

    def connect(self, port=UDP_PORT_ROBOT_CONTROL):
        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.ip, port))

    def disconnect(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def connected(self):
        return self.sock is not None

    def generate_error_ans_packet(self, result, errno):
        # when error, result and error number are what callers care about
        p = bytearray(25)
        p += struct.pack('B', result)
        p += bytearray(2)
        p += struct.pack('<H', errno)
        p += bytearray(2)
        return p

    def transmit(self, packet, direction=TRANSMISSION_SEND_AND_RECV):
        FS100.transmission_lock.acquire()
        if self.connected():
            to_disc = False
        else:
            self.connect()
            to_disc = True

        try:
            self.sock.sendall(packet)
            if FS100.DEBUG:
                print("PC -> FS100: {}, Len={}".format(packet, len(packet)))
            if direction == FS100.TRANSMISSION_SEND_AND_RECV:
                ans_packet, addr = self.sock.recvfrom(512)
                if FS100.DEBUG:
                    print("PC <- FS100: {}, Len={}".format(ans_packet, len(ans_packet)))
        except socket.error as error:
            print("ConnectionError: {}".format(error))
            errno = error.errno
            if errno is None:
                errno = FS100.ERROR_CONNECTION
            ans_packet = self.generate_error_ans_packet(FS100.ERROR_CONNECTION, errno)
        finally:
            pass

        if direction == FS100.TRANSMISSION_SEND_AND_RECV:
            ans = FS100AnsPacket(ans_packet)
        else:
            ans = None

        if to_disc:
            self.disconnect()

        FS100.transmission_lock.release()
        return ans

    def switch_power(self, power_type, switch):
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x83, power_type, 0x01, 0x10,
                             struct.pack('<I', switch), 4)
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed switching power supply, err={}".format(hex(ans.added_status)))
        return ans.status

    def traveller(self, bag, stops, cb_status):
        status = FS100.TRAVEL_STATUS_ERROR
        fs100_status = {}

        for idx, pos in enumerate(stops):
            new_data = bag.data[0:20] +\
                struct.pack('<iiiiiii', pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]) + bag.data[48:104]
            req = bag.clone(new_data)
            ans = self.transmit(req.to_bytes())
            self.errno = ans.added_status
            if ans.status != FS100.ERROR_SUCCESS:
                break
            else:
                cb_status(self, idx)

            while True:
                # user termination
                if self.stop_travelling:
                    self.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_ON)
                    self.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)
                    # exit thread
                    polling_err = True
                    break
                # poll for going to next stop
                if FS100.ERROR_SUCCESS == self.get_status(fs100_status):
                    if not fs100_status['running']:
                        polling_err = False
                        break
                    time.sleep(FS100.TRAVEL_STATUS_POLLING_DURATION)
                else:
                    polling_err = True
                    break
            if polling_err:
                break
        else:
            status = FS100.TRAVEL_STATUS_END

        cb_status(self, status)

    # travel in cartesian coordinate
    def move(self, cb_status, move_type, coordinate, speed_class, speed, pos, form=0, extended_form=0, robot_no=1,
             station_no=0, tool_no=0, user_coor_no=0, wait=False):
        """
        :param cb_status: travelling status callback
        :param move_type:
        :param coordinate:
        :param speed_class:
        :param speed: in 0.01 % for class MOVE_SPEED_CLASS_PERCENT
                      in 0.1 mm/s for class MOVE_SPEED_CLASS_MILLIMETER
                      in 0.1 degree/s for MOVE_SPEED_CLASS_DEGREE
        :param pos: list of tuples (x, y, z, Rx, Ry, Rz, Re). x, y, z are in 0.000001 m, whereas Rx, Ry, Rz, Re are in 0.0001 degree
        :param form: see documentation
        :param extended_form: see documentation
        :param robot_no:
        :param station_no:
        :param tool_no:
        :param user_coor_no:
        :param wait:
        :return:
        """
        data = struct.pack('<I', robot_no)
        data += struct.pack('<I', station_no)
        data += struct.pack('<I', speed_class)
        data += struct.pack('<I', speed)
        data += struct.pack('<I', coordinate)
        data += bytearray(28)
        data += struct.pack('<I', 0)  # reserved
        data += struct.pack('<I', form)
        data += struct.pack('<I', extended_form)
        data += struct.pack('<I', tool_no)
        data += struct.pack('<I', user_coor_no)
        data += bytearray(36)

        bag = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x8a, move_type, 0x01, 0x02, data,
                             len(data))
        self.traveller_thread = threading.Thread(target=self.traveller, args=(bag, pos, cb_status))
        self.traveller_thread.start()
        if wait:
            # wait for all stops visited
            self.traveller_thread.join()
            return self.errno
        else:
            return

    def stop(self):
        if self.traveller_thread is not None and self.traveller_thread.is_alive():
            self.stop_travelling = True
            self.traveller_thread.join()
            self.stop_travelling = False

    # move to one place, no status reported
    def one_move(self, move_type, coordinate, speed_class, speed, pos, form=0, extended_form=0, robot_no=1,
                 station_no=0, tool_no=0, user_coor_no=0):
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
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed moving to one place, err={}".format(hex(ans.added_status)))
        return ans.status

    def get_last_alarm(self, alarm):
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x70, 1, 0, 0x01, bytearray(0), 0)
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed getting the last alarm, err={}".format(hex(ans.added_status)))
        else:
            alarm['code'] = struct.unpack('<I', ans.data[0:4])[0]
            alarm['data'] = struct.unpack('<I', ans.data[4:8])[0]
            alarm['type'] = struct.unpack('<I', ans.data[8:12])[0]
            alarm['time'] = ans.data[12:28].decode('ascii')
            alarm['name'] = ans.data[28:60].decode('utf-8')
        return ans.status

    # reset alarms or cancel errors
    def reset_alarm(self, alarm_type):
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x82, alarm_type, 1, 0x10,
                             struct.pack('<I', 1), 4)
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed resetting alarms or cancelling errors, err={}".format(hex(ans.added_status)))
        return ans.status

    # get status
    def get_status(self, status):
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x72, 1, 0, 0x01, bytearray(0), 0)
        ans = self.transmit(req.to_bytes())
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

    # play job
    def play_job(self):
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x86, 1, 1, 0x10,
                             struct.pack('<I', 1), 4)
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed playing job, err={}".format(hex(ans.added_status)))
        return ans.status

    # select job
    def select_job(self, job_name, line_num):
        data = job_name.encode(encoding='utf-8')
        if len(data) > 32:
            raise ValueError('Job name is too long')
        data += bytearray([32 - len(data)])
        data += struct.pack('<I', line_num)
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x87, 1, 0, 0x02, data, len(data))
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed selecting the job, err={}".format(hex(ans.added_status)))
        return ans.status

    # delete the file
    def delete_file(self, file_name):
        self.file_control_lock.acquire()
        self.connect(FS100.UDP_PORT_FILE_CONTROL)
        data = file_name.encode(encoding='utf-8')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x09, data, len(data))
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed deleting the file, err={}".format(hex(ans.added_status)))
        self.disconnect()
        self.file_control_lock.releas()
        return ans.status

    # get file list
    def get_file_list(self, extension, list):
        self.file_control_lock.acquire()
        self.connect(FS100.UDP_PORT_FILE_CONTROL)
        raw = ''
        data = extension.encode(encoding='ascii')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x32, data, len(data))
        ans = self.transmit(req.to_bytes())
        while ans.status == FS100.ERROR_SUCCESS:
            raw += ans.data.decode('utf-8')
            req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x32, bytearray(0), 0)
            # not a request but an ack
            req.ack = FS100PacketHeader.HEADER_ACK_NOT_REQUEST
            req.block_no = ans.block_no
            if ans.block_no & 0x80000000 != 0:
                # The last file list data arrived. Send the final ack.
                self.transmit(req.to_bytes(), FS100.TRANSMISSION_SEND)
                break
            else:
                ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed getting the file list, err={}".format(hex(ans.added_status)))
        else:
            list.extend(raw.splitlines())
        self.disconnect()
        self.file_control_lock.release()
        return ans.status

    def send_file(self, filename):
        try:
            with open(filename, 'rb') as f:
                context = f.read()
        except FileNotFoundError:
            print("file not found: '{}'".format(filename))
            self.errno = 0xe2b3
            return FS100.ERROR_NO_SUCH_FILE_OR_DIRECTORY

        if len(context) == 0:
            raise ValueError('An empty file')

        self.file_control_lock.acquire()
        self.connect(FS100.UDP_PORT_FILE_CONTROL)

        CHUNK_SIZE = 400
        block_no = 0
        data = ntpath.basename(filename).encode(encoding='utf-8')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x15, data, len(data))
        ans = self.transmit(req.to_bytes())
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
            ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed sending the file, err={}".format(hex(ans.added_status)))

        self.disconnect()
        self.file_control_lock.release()
        return ans.status

    def recv_file(self, filename, local_dir):
        if not os.path.isdir(local_dir):
            print("directory not found: '{}'".format(local_dir))
            self.errno = 0xe2b3
            return FS100.ERROR_NO_SUCH_FILE_OR_DIRECTORY

        self.file_control_lock.acquire()
        self.connect(FS100.UDP_PORT_FILE_CONTROL)

        context = bytearray(0)
        data = filename.encode(encoding='utf-8')
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x16, data, len(data))
        ans = self.transmit(req.to_bytes())
        while ans.status == FS100.ERROR_SUCCESS:
            context += ans.data
            req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, 0, 0, 0, 0, 0x16, bytearray(0), 0)
            # not a request but an ack
            req.ack = FS100PacketHeader.HEADER_ACK_NOT_REQUEST
            req.block_no = ans.block_no
            if ans.block_no & 0x80000000 != 0:
                # Got the last piece of file. Send the final ack.
                self.transmit(req.to_bytes(), FS100.TRANSMISSION_SEND)
                break
            else:
                ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed receiving the file, err={}".format(hex(ans.added_status)))
        else:
            with open("{}/{}".format(local_dir, filename), 'wb') as f:
                f.write(context)

        self.disconnect()
        self.file_control_lock.release()
        return ans.status

    def read_position(self, pos_info, robot_no=1):
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, 0x75, 100 + robot_no, 0, 0x01,
                             bytearray(0), 0)
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed reading the position info, err={}".format(hex(ans.added_status)))
        else:
            pos_info['data_type'] = struct.unpack('<I', ans.data[0:4])[0]
            pos_info['form'] = struct.unpack('<I', ans.data[4:8])[0]
            pos_info['tool_no'] = struct.unpack('<I', ans.data[8:12])[0]
            pos_info['user_coor_no'] = struct.unpack('<I', ans.data[12:16])[0]
            pos_info['extended_form'] = struct.unpack('<I', ans.data[16:20])[0]
            pos_info['pos'] = (struct.unpack('<i', ans.data[20:24])[0],
                               struct.unpack('<i', ans.data[24:28])[0],
                               struct.unpack('<i', ans.data[28:32])[0],
                               struct.unpack('<i', ans.data[32:36])[0],
                               struct.unpack('<i', ans.data[36:40])[0],
                               struct.unpack('<i', ans.data[40:44])[0],
                               struct.unpack('<i', ans.data[44:48])[0])
        return ans.status

    class VarType(IntEnum):
        BYTE = 0x7a  # 1 byte
        INTEGER = 0x7b  # 2 bytes
        DOUBLE = 0x7c  # 4 bytes
        REAL = 0x7d  # 4 bytes
        STRING = 0x7e  # max. 16 bytes
        #ROBOT_POSITION = 0x7f
        #BASE_POSITION = 0x80

    class Variable:
        def __init__(self, type, num, val=None):
            self.type = type
            self.num = num
            self.val = val

        def set_val(self, raw_bytes):
            if self.type == FS100.VarType.BYTE:
                self.val = struct.unpack('B', raw_bytes[0:1])[0]
            elif self.type == FS100.VarType.INTEGER:
                self.val = struct.unpack('<h', raw_bytes[0:2])[0]
            elif self.type == FS100.VarType.DOUBLE:
                self.val = struct.unpack('<i', raw_bytes[0:4])[0]
            elif self.type == FS100.VarType.REAL:
                self.val = struct.unpack('<f', raw_bytes[0:4])[0]
            elif self.type == FS100.VarType.STRING:
                self.val = raw_bytes.decode('utf-8')

        def val_to_bytes(self):
            ret = None
            if self.type == FS100.VarType.BYTE:
                ret = struct.pack('B', self.val)
            elif self.type == FS100.VarType.INTEGER:
                ret = struct.pack('<h', self.val)
            elif self.type == FS100.VarType.DOUBLE:
                ret = struct.pack('<i', self.val)
            elif self.type == FS100.VarType.REAL:
                ret = struct.pack('<f', self.val)
            elif self.type == FS100.VarType.STRING:
                ret = self.val.encode(encoding='utf-8')
            return ret

    def read_variable(self, var):
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, var.type, var.num, 1, 0x0e,
                             bytearray(0), 0)
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed reading the variable, err={}".format(hex(ans.added_status)))
        else:
            var.set_val(ans.data)
        return ans.status

    def write_variable(self, var):
        data = var.val_to_bytes()
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, 0, var.type, var.num, 1, 0x10,
                             data, len(data))
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        if ans.status != FS100.ERROR_SUCCESS:
            print("failed writing the variable, err={}".format(hex(ans.added_status)))
        return ans.status


def travel_status(vehicle, status):
    now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    if status == FS100.TRAVEL_STATUS_START:
        print("[{}] start travelling".format(now))
    elif status == FS100.TRAVEL_STATUS_END:
        print("[{}] end travelling".format(now))
    elif status == FS100.TRAVEL_STATUS_ERROR:
        print("[{}] failed travelling, err={}".format(now, hex(vehicle.errno)))
    else:
        print("[{}] travelling at stop #{}".format(now, status))


if __name__ == '__main__':
    # servo on/off
    '''
    robot = FS100('10.0.0.2')
    if FS100.ERROR_SUCCESS != robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON):
        print("failed turning on servo power supply, err={}".format(hex(robot.errno)))
    time.sleep(2)
    if FS100.ERROR_SUCCESS != robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_OFF):
        print("failed turning off servo power supply, err={}".format(hex(robot.errno)))
    '''
    # travel
    '''
    robot = FS100('10.0.0.2')
    status = {}
    if FS100.ERROR_SUCCESS == robot.get_status(status):
        if not status['servo_on']:
            robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)
    stops = [(600, -254, 22, 180, 0, 0, 0),
             (320, -40, 165, 180, 0, 0, 0),
             (600, -254, 22, 180, 0, 0, 0),
             (320, -40, 165, 180, 0, 0, 0)]
    robot.move(travel_status, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
               FS100.MOVE_SPEED_CLASS_PERCENT, 250, stops)
    '''
    # get last alarm
    '''
    robot = FS100('10.0.0.2')
    alarm = {}
    if FS100.ERROR_SUCCESS == robot.get_last_alarm(alarm):
        print("the last alarm: code={}, data={}, type={}, time={}, name={}"
              .format(hex(alarm['code']), alarm['data'], alarm['type'], alarm['time'], alarm['name']))
    '''
    # reset alarm
    '''
    robot = FS100('10.0.0.2')
    if FS100.ERROR_SUCCESS != robot.reset_alarm(FS100.RESET_ALARM_TYPE_ALARM):
        print("failed resetting alarms, err={}".format(hex(robot.errno)))
    '''
    # get status
    '''
    robot = FS100('10.0.0.2')
    status = {}
    if FS100.ERROR_SUCCESS == robot.get_status(status):
        print(status)
    '''
    # file control of a job
    '''
    robot = FS100('10.0.0.2')
    jobs = []
    if FS100.ERROR_SUCCESS != robot.get_file_list('*.JBI', jobs):
        print("failed getting the list of jobs in controller, err={}".format(hex(robot.errno)))
        exit()
    if len(jobs) == 0:
        print("no job in controller")
        exit()
    my_job = jobs[-1]
    dir = 'D:\\Downloads\\'
    if FS100.ERROR_SUCCESS != robot.recv_file(my_job, dir):
        print("failed receiving the job file from controller, err={}".format(hex(robot.errno)))
        exit()
    if FS100.ERROR_SUCCESS != robot.delete_file(my_job):
        print("could not delete the job file in controller, err={}".format(hex(robot.errno)))
        exit()
    if FS100.ERROR_SUCCESS != robot.send_file(dir + my_job):
        print("failed sending the job file to controller, err={}".format(hex(robot.errno)))
        exit()
    print("The list of jobs in controller:")
    print(jobs)
    print("Successfully received/deleted/sent the job '{}'".format(my_job))
    '''
    # read position info
    '''
    robot = FS100('10.0.0.2')
    pos_info = {}
    if FS100.ERROR_SUCCESS == robot.read_position(pos_info):
        print(pos_info)
    '''
    # variable writing and reading
    '''
    robot = FS100('10.0.0.2')
    var_b = FS100.Variable(FS100.VarType.BYTE, 0, 0xaa)
    var_i = FS100.Variable(FS100.VarType.INTEGER, 1, 12345)
    var_d = FS100.Variable(FS100.VarType.DOUBLE, 2, -7654321)
    var_r = FS100.Variable(FS100.VarType.REAL, 3, -123.4567)
    var_s = FS100.Variable(FS100.VarType.STRING, 4, 'Hello, World!')
    
    ret = robot.write_variable(var_b)
    ret |= robot.write_variable(var_i)
    ret |= robot.write_variable(var_d)
    ret |= robot.write_variable(var_r)
    ret |= robot.write_variable(var_s)
    if ret != FS100.ERROR_SUCCESS:
        print("failed writing variables!")
        
    ret = robot.read_variable(var_b)
    ret |= robot.read_variable(var_i)
    ret |= robot.read_variable(var_d)
    ret |= robot.read_variable(var_r)
    ret |= robot.read_variable(var_s)
    if ret != FS100.ERROR_SUCCESS:
        print("failed reading variables!")
    else:
        print(var_b.val, var_i.val, var_d.val, var_r.val, var_s.val)
    '''
    pass
