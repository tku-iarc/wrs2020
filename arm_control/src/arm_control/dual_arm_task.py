#!/usr/bin/env python3
#-*- coding: utf-8 -*-

"""Use to generate dual_arm task and control both arms."""

import rospy
import tf
import queue
import threading
# import object_distribtion

from math import radians, degrees, sin, cos, pi
from numpy import multiply

import numpy as np

from arm_control import ArmTask, SuctionTask, Command, Status
from std_msgs.msg import String, Float64, Bool

class DualArmTask:
    """"""
    def __init__(self, _name, en_sim):
        self.name = _name
        self.en_sim = en_sim
        self.right_arm = ArmTask('right_arm', self.en_sim)
        self.left_arm = ArmTask('left_arm', self.en_sim)
        self.right_value = 0
        self.left_value = 0
        self.right_limit = False
        self.left_limit = False
                
        self.right_event = threading.Event()
        self.left_event = threading.Event()
        self.right_event.clear()
        self.left_event.clear()
        self.right_thread = threading.Thread(target=self.__right_arm_process_thread)
        self.left_thread = threading.Thread(target=self.__left_arm_process_thread)
        self.right_thread.start()
        self.left_thread.start()
        

    def __right_arm_process_thread(self):
        rate = rospy.Rate(20)
        while True:
            self.right_event.wait()
            if not self.right_arm.is_busy:
                self.right_arm.process()
            if self.right_arm.cmd_queue_empty:
                if self.right_arm.cmd_queue_2nd_empty:
                    self.right_arm.clear()
                elif self.right_arm.status == Status.idle:
                    self.right_arm.cmd_2to1()
            rate.sleep()

    def __left_arm_process_thread(self):
        rate = rospy.Rate(20)
        while True:
            self.left_event.wait()
            if not self.left_arm.is_busy:
                self.left_arm.process()
            if self.left_arm.cmd_queue_empty:
                if self.left_arm.cmd_queue_2nd_empty:
                    self.left_event.clear()
                elif self.left_arm.status == Status.idle:
                    self.left_arm.cmd_2to1()
            rate.sleep()

    def __choose_and_check_side(self, side, command_queue):
        self.right_value = 0
        self.left_value = 0
        self.right_limit = False
        self.left_limit = False

        if 'right' in side:
            while not command_queue.empty():
                cmd = command_queue.get()
                if cmd['cmd'] == 'ikMove':
                    self.right_value = self.right_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                    if self.right_value < 0:
                        return 'fail'
            return 'right'
        
        elif 'left' in side:
            while not command_queue.empty():
                cmd = command_queue.get()
                if cmd['cmd'] == 'ikMove':
                    self.left_value = self.left_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                    if self.left_value < 0:
                        return 'fail'
            return 'left'

        else:
            right_sum = 0
            left_sum = 0
            right_close_limit = False
            left_close_limit = False
            while not command_queue.empty():
                cmd = command_queue.get()
                if cmd['cmd'] == 'ikMove':
                    if not self.left_limit:
                        self.left_value = self.left_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                        if self.left_value > 0.85:
                            left_close_limit = True
                        if self.left_value < 0:
                            self.left_limit = True
                        else:
                            left_sum += self.left_value

                    if not self.right_limit:
                        self.right_value = self.right_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                        if self.right_value > 0.85:
                            right_close_limit = True
                        if self.right_value < 0:
                            self.right_limit = True
                        else:
                            right_sum += self.right_value
                        
            if not (self.left_limit or self.right_limit):
                if right_sum <= left_sum and self.right_arm.status == Status.idle:
                    return 'right'
                elif left_sum <= right_sum and self.left_arm.status == Status.idle:
                    return 'left'
                elif self.right_arm.status == Status.idle and not right_close_limit:
                    return 'right'
                elif self.left_arm.status == Status.idle and not left_close_limit:
                    return 'left'
                elif right_sum <= left_sum:
                    return 'right'
                elif left_sum <= right_sum:
                    return 'left'
            elif self.right_limit and self.left_limit:
                return 'fail'
            elif self.right_limit:
                return 'left'
            elif self.left_limit:
                return 'right'
                
            

    def send_cmd(self, side, priority, command_queue):
        side = self.__choose_and_check_side(side, command_queue)
        print('Choosed side : '+side+' =================')
        if side == 'right':
            if priority:
                self.right_arm.cmd_queue_put(command_queue)
            else:
                self.right_arm.cmd_queue_2nd_put(command_queue)
            if not self.right_event.is_set():
                self.right_event.set()
        elif side == 'left':
            if priority:
                self.left_arm.cmd_queue_put(command_queue)
            else:
                self.left_arm.cmd_queue_2nd_put(command_queue)
            if not self.left_event.is_set():
                self.left_event.set()
        return side