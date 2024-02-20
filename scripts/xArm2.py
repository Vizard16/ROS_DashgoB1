#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
# 
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import rospy
from std_msgs.msg import Int32
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading

from xarm import version
from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.31.221', baud_checkset=False)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

data = 0

def data_callback(msg):
    global data 
    data = msg.data


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {
            "Place": self.function_2,
            "Pick": self.function_1,
        }
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def function_1(self):
        """
        Describe this function...
        """
        code = self._arm.set_position(*[207.0, 0.0, 635.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[465.8, 0.0, 635.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[465.8, -107.9, 635.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[465.8, -107.9, 560.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_tgpio_digital(0, 1, delay_sec=0)
        if not self._check_code(code, 'set_tgpio_digital'):
            return
        code = self._arm.set_pause_time(3)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_position(*[465.8, -107.9, 655.5, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_servo_angle(angle=[-90.0, 17.7, -136.0, 118.3, -13.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*[0.0, -478.3, 267.4, 180.0, 0.0, -77.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[0.0, -297.5, 267.4, 180.0, 0.0, -77.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        self._arm.move_gohome()

    def function_2(self):
        """
        Describe this function...
        """
        code = self._arm.set_position(*[207.0, 0.0, 635.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[465.8, 0.0, 635.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[465.8, -107.9, 635.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[465.8, -107.9, 560.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_pause_time(2)
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
        if not self._check_code(code, 'set_tgpio_digital'):
            return
        code = self._arm.set_position(*[465.8, -107.9, 655.5, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[217.6, -107.9, 655.5, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[217.6, -107.9, 297.5, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        self._arm.move_gohome()

def main_routine():
    robot_main.function_1()
    data = 0
    flag.publish(1)

if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version: {}'.format(version.__version__))
    robot_main = RobotMain(arm)
    rospy.init_node("xarm")
    rospy.Subscriber("xarm_valor", Int32, data_callback)
    flag = rospy.Publisher("Flag", Int32, queue_size=1)

    while not rospy.is_shutdown():
        try:
            if data == 2:
                main_thread = threading.Thread(target=main_routine)
                main_thread.start()
                main_thread.join()  # Wait for the main routine to complete
                data = 0
        except rospy.ROSInterruptException:
            pass