#!/usr/bin/env python3

import os
import sys
import unittest

# include test module directory
test_dir = os.path.dirname(os.path.realpath(__file__))
module_dir = os.path.dirname(test_dir)
sys.path.append(module_dir)

from fs100 import FS100


class TestFS100Functions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._robot = FS100('192.168.10.2')

    @classmethod
    def tearDownClass(cls):
        pass

    def print_dict_result(self, result):
        print("")
        print("{")
        for key in result:
            print("  '{}': {}".format(key, result[key]))
        print("}")

    def print_list_result(self, result):
        print("")
        print("[")
        for item in result:
            print("  '{}'".format(item))
        print("]")

    def test_0x00_file_control(self):
        TEST_JOB_PATHNAME = '{}/{}'.format(test_dir, 'TEST_FS100.JBI')
        TEST_JOB_CONTEXT = "/JOB\r\n"\
                            "//NAME TEST_FS100\r\n"\
                            "//POS\r\n"\
                            "///NPOS 0,0,0,0,0,0\r\n"\
                            "//INST\r\n"\
                            "///DATE 2022/12/23 15:58\r\n"\
                            "///ATTR SC,RW\r\n"\
                            "///GROUP1 RB1\r\n"\
                            "NOP\r\n"\
                            "END\r\n"

        result = list()
        error = self._robot.get_file_list('*.JBI', result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_list_result(result)

        if os.path.basename(TEST_JOB_PATHNAME) in result:
            error = self._robot.recv_file(os.path.basename(TEST_JOB_PATHNAME), os.path.dirname(TEST_JOB_PATHNAME))
            self.assertEqual(error, FS100.ERROR_SUCCESS)
            os.rename(TEST_JOB_PATHNAME, TEST_JOB_PATHNAME + '.orig')
            error = self._robot.delete_file(os.path.basename(TEST_JOB_PATHNAME))
            self.assertEqual(error, FS100.ERROR_SUCCESS)

        with open(TEST_JOB_PATHNAME, 'wb') as f:
            f.write(TEST_JOB_CONTEXT.encode(encoding='ascii'))
        error = self._robot.send_file(TEST_JOB_PATHNAME)
        self.assertEqual(error, FS100.ERROR_SUCCESS)

        # Also test commands 0x87, 0x84, 0x83 and 0x86
        error = self._robot.select_job(os.path.basename(TEST_JOB_PATHNAME))
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        error = self._robot.select_cycle(FS100.CYCLE_TYPE_ONE_CYCLE)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        error = self._robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        error = self._robot.play_job()
        self.assertEqual(error, FS100.ERROR_SUCCESS)

        error = self._robot.recv_file(os.path.basename(TEST_JOB_PATHNAME), os.path.dirname(TEST_JOB_PATHNAME))
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        error = self._robot.delete_file(os.path.basename(TEST_JOB_PATHNAME))
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        os.remove(TEST_JOB_PATHNAME)

        if os.path.exists(TEST_JOB_PATHNAME + '.orig'):
            os.rename(TEST_JOB_PATHNAME + '.orig', TEST_JOB_PATHNAME)
            error = self._robot.send_file(TEST_JOB_PATHNAME)
            self.assertEqual(error, FS100.ERROR_SUCCESS)
            os.remove(TEST_JOB_PATHNAME)

    def test_0x70_get_last_alarm(self):
        result = dict()
        error = self._robot.get_last_alarm(result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x71_read_alarm_info(self):
        TEST_ALARM_NUMBER = 1
        result = dict()
        error = self._robot.read_alarm_info(TEST_ALARM_NUMBER, result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Alarm #{}:".format(TEST_ALARM_NUMBER), end='')
        self.print_dict_result(result)

    def test_0x72_get_status(self):
        result = dict()
        error = self._robot.get_status(result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x73_read_executing_job_info(self):
        result = dict()
        error = self._robot.read_executing_job_info(result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x74_read_axis_name(self):
        result = dict()
        error = self._robot.read_axis_name(result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x75_read_position(self):
        result = dict()
        error = self._robot.read_position(result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x76_read_position_error(self):
        result = dict()
        error = self._robot.read_position_error(result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x77_read_torque(self):
        result = dict()
        error = self._robot.read_torque(result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x78_io_data_rw(self):
        TEST_IO_NUMBER = 2501
        var = FS100.Variable(FS100.VarType.IO, TEST_IO_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read IO #{} -> {}".format(TEST_IO_NUMBER, var.val))
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write IO #{} <- {}".format(TEST_IO_NUMBER, var.val))

    def test_0x79_register_data_rw(self):
        TEST_REGISTER_NUMBER = 0
        var = FS100.Variable(FS100.VarType.REGISTER, TEST_REGISTER_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read Register #{} -> {}".format(TEST_REGISTER_NUMBER, var.val))
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write Register #{} <- {}".format(TEST_REGISTER_NUMBER, var.val))

    def test_0x7a_byte_variable_rw(self):
        TEST_BYTE_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.BYTE, TEST_BYTE_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read Byte Variable #{} -> {}".format(TEST_BYTE_VARIABLE_NUMBER, var.val))
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write Byte Variable #{} <- {}".format(TEST_BYTE_VARIABLE_NUMBER, var.val))

    def test_0x7b_integer_variable_rw(self):
        TEST_INTEGER_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.INTEGER, TEST_INTEGER_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read Integer Variable #{} -> {}".format(TEST_INTEGER_VARIABLE_NUMBER, var.val))
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write Integer Variable #{} <- {}".format(TEST_INTEGER_VARIABLE_NUMBER, var.val))

    def test_0x7c_double_variable_rw(self):
        TEST_DOUBLE_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.DOUBLE, TEST_DOUBLE_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read Double Variable #{} -> {}".format(TEST_DOUBLE_VARIABLE_NUMBER, var.val))
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write Double Variable #{} <- {}".format(TEST_DOUBLE_VARIABLE_NUMBER, var.val))

    def test_0x7d_real_variable_rw(self):
        TEST_REAL_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.REAL, TEST_REAL_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read Real Variable #{} -> {}".format(TEST_REAL_VARIABLE_NUMBER, var.val))
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write Real Variable #{} <- {}".format(TEST_REAL_VARIABLE_NUMBER, var.val))

    def test_0x7e_string_variable_rw(self):
        TEST_STRING_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.STRING, TEST_STRING_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read String Variable #{} -> '{}'".format(TEST_STRING_VARIABLE_NUMBER, var.val))
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write String Variable #{} <- '{}'".format(TEST_STRING_VARIABLE_NUMBER, var.val))

    def test_0x7f_robot_position_variable_rw(self):
        TEST_ROBOT_POSITION_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.ROBOT_POSITION, TEST_ROBOT_POSITION_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read Robot Position Variable #{} ->".format(TEST_ROBOT_POSITION_VARIABLE_NUMBER), end='')
        self.print_dict_result(var.val)
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write Robot Position Variable #{} <-".format(TEST_ROBOT_POSITION_VARIABLE_NUMBER), end='')
        self.print_dict_result(var.val)

    def test_0x80_base_position_variable_rw(self):
        TEST_BASE_POSITION_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.BASE_POSITION, TEST_BASE_POSITION_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read Base Position Variable #{} ->".format(TEST_BASE_POSITION_VARIABLE_NUMBER), end='')
        self.print_dict_result(var.val)
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write Base Position Variable #{} <-".format(TEST_BASE_POSITION_VARIABLE_NUMBER), end='')
        self.print_dict_result(var.val)

    def test_0x81_external_axis_variable_rw(self):
        TEST_EXTERNAL_AXIS_VARIABLE_NUMBER = 0
        var = FS100.Variable(FS100.VarType.EXTERNAL_AXIS, TEST_EXTERNAL_AXIS_VARIABLE_NUMBER)
        error = self._robot.read_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("")
        print("Read External Axis Variable #{} ->".format(TEST_EXTERNAL_AXIS_VARIABLE_NUMBER), end='')
        self.print_dict_result(var.val)
        error = self._robot.write_variable(var)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        print("Write External Axis Variable #{} <-".format(TEST_EXTERNAL_AXIS_VARIABLE_NUMBER), end='')
        self.print_dict_result(var.val)

    def test_0x82_reset_alarm(self):
        error = self._robot.reset_alarm(FS100.RESET_ALARM_TYPE_ALARM)
        self.assertEqual(error, FS100.ERROR_SUCCESS)

    def test_0x85_show_text_on_pendant(self):
        error = self._robot.show_text_on_pendant("Hello, FS100!")
        self.assertEqual(error, FS100.ERROR_SUCCESS)

    def test_0x88_acquire_management_time(self):
        result = dict()
        error = self._robot.acquire_management_time(FS100.ManagementTimeType.MOTION_R1, result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x89_acquire_system_info(self):
        result = dict()
        error = self._robot.acquire_system_info(FS100.SystemInfoType.R1, result)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        self.print_dict_result(result)

    def test_0x8a_mov(self):
        current_pos = dict()
        error = self._robot.read_position(current_pos, robot_no=101)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        p = list()
        for n in range(len(current_pos['pos'])):
            p.append(current_pos['pos'][n] + 10000)
        error = self._robot.mov(FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT, FS100.MOVE_SPEED_CLASS_PERCENT, 250, tuple(p))
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        while True:
            status = dict()
            error = self._robot.get_status(status)
            self.assertEqual(error, FS100.ERROR_SUCCESS)
            if not status['running']:
                break
        print("")

    def test_0x8b_pmov(self):
        current_pulse = dict()
        error = self._robot.read_position(current_pulse, robot_no=1)
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        p = list()
        for n in range(len(current_pulse['pos'])):
            p.append(current_pulse['pos'][n] + 1000)
        error = self._robot.pmov(FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_SPEED_CLASS_PERCENT, 250, tuple(p))
        self.assertEqual(error, FS100.ERROR_SUCCESS)
        while True:
            status = dict()
            error = self._robot.get_status(status)
            self.assertEqual(error, FS100.ERROR_SUCCESS)
            if not status['running']:
                break
        print("")


if __name__ == '__main__':
    unittest.main(verbosity=2)
