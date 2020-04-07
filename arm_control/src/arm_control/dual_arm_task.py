#!/usr/bin/env python3
#-*- coding: utf-8 -*-

"""Use to generate dual_arm task and control both arms."""

import rospy
import tf
import queue
import threading
import copy
# import object_distribtion

from math import radians, degrees, sin, cos, pi
from numpy import multiply 
# from Queue import Queue
from math import acos, cos, asin, sin, degrees
import numpy as np

from arm_control import ArmTask, SuctionTask, Command, Status
from std_msgs.msg import String, Float64, Bool

class DualArmTask:
    """"""
    def __init__(self, _name, en_sim):
        self.name = _name
        self.en_sim = en_sim
        self.right_arm = ArmTask('right', self.en_sim)
        self.left_arm = ArmTask('left', self.en_sim)
        self.right_value = 0
        self.left_value = 0
        self.right_limit = False
        self.left_limit = False
        self.right_event = threading.Event()
        self.left_event = threading.Event()
        self.stop_event = threading.Event()
        self.right_event.clear()
        self.left_event.clear()
        self.stop_event.clear()
        self.right_thread = threading.Thread(target=self.__right_arm_process_thread)
        self.left_thread = threading.Thread(target=self.__left_arm_process_thread)
        self.right_thread.start()
        self.left_thread.start()

    def shutdown(self):
        self.right_arm.clear_cmd()
        self.left_arm.clear_cmd()
        self.stop_event.set()
        self.right_event.set()
        self.left_event.set()

    def __right_arm_process_thread(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.right_event.wait()
            if self.stop_event.is_set():
                break
            if not self.right_arm.is_busy or self.right_arm.status == Status.grasping:
                self.right_arm.process()
            if self.right_arm.cmd_queue_empty:
                if self.right_arm.cmd_queue_2nd_empty:
                    self.right_event.clear()
                elif not self.right_arm.is_busy and not self.right_arm.status == Status.occupied:
                    self.right_arm.cmd_2to1()
            rate.sleep()

    def __left_arm_process_thread(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.left_event.wait()
            if self.stop_event.is_set():
                break
            if not self.left_arm.is_busy or self.left_arm.status == Status.grasping:
                self.left_arm.process()
            if self.left_arm.cmd_queue_empty:
                if self.left_arm.cmd_queue_2nd_empty:
                    self.left_event.clear()
                elif not self.left_arm.is_busy and not self.left_arm.status == Status.occupied:
                    self.left_arm.cmd_2to1()
            rate.sleep()

    def __choose_and_check_side(self, side, command_queue):
        self.right_value = 0
        self.left_value = 0
        self.right_limit = False
        self.left_limit = False
        cmd_q = queue.Queue()
        if 'right' in side:
            while not command_queue.empty():
                cmd = command_queue.get()
                if cmd['cmd'] == 'ikMove':
                    self.right_value, self.right_limit = self.right_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                    if self.right_limit:
                        return 'fail', cmd_q
                cmd_q.put(cmd)
            return 'right', cmd_q
        
        elif 'left' in side:
            while not command_queue.empty():
                cmd = command_queue.get()
                if cmd['cmd'] == 'ikMove':
                    self.left_value, self.left_limit = self.left_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                    if self.left_limit:
                        return 'fail', cmd_q
                cmd_q.put(cmd)
            return 'left', cmd_q

        else:
            right_sum = 0
            left_sum = 0
            right_close_limit = False
            left_close_limit = False
            while not command_queue.empty():
                cmd = command_queue.get()
                cmd_q.put(cmd)
                if cmd['cmd'] == 'ikMove':
                    if not self.left_limit:
                        self.left_value, self.left_limit = self.left_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                        if self.left_value > 0.2:
                            left_close_limit = True
                        else:
                            left_sum += self.left_value

                    if not self.right_limit:
                        self.right_value, self.right_limit = self.right_arm.check_range_limit(cmd['pos'], cmd['euler'], cmd['phi'])
                        if self.right_value > 0.2:
                            right_close_limit = True
                        else:
                            right_sum += self.right_value
                        
            if not (self.left_limit or self.right_limit):
                if right_sum <= left_sum and self.right_arm.status == Status.idle:
                    side = 'right'
                elif left_sum <= right_sum and self.left_arm.status == Status.idle:
                    side = 'left'
                elif self.right_arm.status == Status.idle and not right_close_limit:
                    side = 'right'
                elif self.left_arm.status == Status.idle and not left_close_limit:
                    side = 'left'
                elif right_sum <= left_sum:
                    side = 'right'
                elif left_sum <= right_sum:
                    side = 'left'
            elif self.right_limit and self.left_limit:
                side = 'fail'
            elif self.right_limit:
                side = 'left'
            elif self.left_limit:
                side = 'right'
            return side, cmd_q
            

    def send_cmd(self, side, priority, command_queue):
        # cq = copy.copy(command_queue)
        side, command_queue = self.__choose_and_check_side(side, command_queue)
        # print('Choosed side : '+side+' =================')
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

    def get_feedback(self, side):
        if side == 'right':
            return self.right_arm.get_fb()
        elif side == 'left':
            return self.left_arm.get_fb()

    def suc2vector(self, vec, euler):
        rotation = self.right_arm.euler2rotation(euler)
        vec = np.array(vec)
        rotation = np.array(rotation)
        a = rotation[0:3, 2].reshape(-1)
        sucangle = -acos(np.dot(a, -1*vec)/(np.linalg.norm(a) * np.linalg.norm(vec)))
        b = np.dot(a, vec)
        c = vec - b*a
        n = rotation[0:3, 0]
        roll = acos(np.dot(c, n)/(np.linalg.norm(c) * np.linalg.norm(n)))
        o = rotation[0:3, 1]
        cos_oc = np.dot(o, c)/(np.linalg.norm(o) * np.linalg.norm(c))
        if cos_oc < 0:
            roll = -1 * roll
        return degrees(sucangle), degrees(roll)



