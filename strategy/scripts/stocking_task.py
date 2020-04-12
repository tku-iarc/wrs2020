#!/usr/bin/env python

from enum import IntEnum
# from Queue import Queue

import rospy
import queue
import copy
import numpy as np
from std_msgs.msg import Bool, Int32
from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status
from get_image_info import GetObjInfo
from math import radians, degrees, sin, cos, pi

#pick_pose_left = [[[-0.2, 0.09, -0.705], [0, 0, 0]],
#                  #[[-0.25, 0.15, -0.67], [0, 0, 0]],
#                  [[-0.2, 0.23, -0.72], [0, 0, 0]],
#                  #[[-0.25, 0.15, -0.57], [0, 0, 0]],
#                  [[-0.2, 0.09, -0.745], [-135, 0, 0]],
#                  [[-0.25, 0.15, -0.79], [-135, 0, 0]]]
#
#pick_pose_right = [[[-0.2, -0.095, -0.71], [0, 0, 0]],
#                  # [[-0.25, -0.15, -0.67], [0, 0, 0]],
#                   [[-0.2, -0.225, -0.72], [0, 0, 0]],
#                  # [[-0.25, -0.15, -0.57], [0, 0, 0]],
#                   [[-0.2, -0.095, -0.745], [135, 0, 0]],
#                   [[-0.2, -0.095, -0.79], [135, 0, 0]]]
#
#place_pose_left = [[[0.57,  0.15, -0.178], [0, 90, 0]],
#                   #[[0.58,  0.15, -0.18], [0, 90, 0]],
#                   [[0.65,  0.15, -0.45], [0, 90, 0]],
#                   #[[0.58,  0.15, -0.48], [0, 90, 0]],
#                   [[0.67,  0.15, -0.91],  [0, 90, -45]],
#                   [[0.67,  0.15, -0.865],  [0, 90, -45]]]
#
#place_pose_right = [[[0.57, -0.15, -0.178], [0, 90, 0]],
#                    #[[0.58, -0.15, -0.18], [0, 90, 0]],
#                    [[0.65, -0.15, -0.45], [0, 90, 0]],
#                    #[[0.58, -0.15, -0.48], [0, 90, 0]],
#                    [[0.67, -0.15, -0.91],  [0, 90, 45]],
#                    [[0.67, -0.15, -0.865],  [0, 90, 45]]]
pick_pose_left = [[[-0.2, 0.09, -0.705], [0, 0, 0]],
                  [[-0.2, 0.23, -0.72], [0, 0, 0]],
                  [[-0.2, 0.09, -0.745], [-135, 0, 0]],
                  [[-0.25, 0.15, -0.79], [-135, 0, 0]]]

pick_pose_right = [[[-0.2, -0.095, -0.71], [0, 0, 0]],
                   [[-0.2, -0.225, -0.72], [0, 0, 0]],
                   [[-0.2, -0.095, -0.745], [135, 0, 0]],
                   [[-0.2, -0.095, -0.79], [135, 0, 0]]]

place_pose_left = [[[0.57,  0.15, -0.178], [0, 90, 0]],
                   [[0.65,  0.15, -0.45], [0, 90, 0]],
                   [[0.67,  0.15, -0.91],  [0, 90, -45]],
                   [[0.67,  0.15, -0.865],  [0, 90, -45]]]

place_pose_right = [[[0.57, -0.15, -0.178], [0, 90, 0]],
                    [[0.65, -0.15, -0.45], [0, 90, 0]],
                    [[0.67, -0.15, -0.91],  [0, 90, 45]],
                    [[0.67, -0.15, -0.865],  [0, 90, 45]]]


place_sucang_left = [0,-90,-90,-90,0,0]
place_sucang_right = [0,-90,-90,-90,0,0]

class State(IntEnum):
    init            = 0
    pick_and_place  = 1
    finish          = 2

class StockingTask:
    def __init__(self, _name, en_sim):
        self.name = _name
        self.en_sim = en_sim
        self.state = State.init
        self.dual_arm = DualArmTask(self.name, self.en_sim)
        self.pick_left_queue = queue.Queue()
        self.pick_right_queue = queue.Queue()
        self.place_left_queue = queue.Queue()
        self.place_right_queue = queue.Queue()
        self.sucang_left_queue = queue.Queue()
        self.sucang_right_queue = queue.Queue()
        self.init()
        self.pick_pose = {'left': self.pick_left_queue, 'right': self.pick_right_queue}
        self.place_pose = {'left': self.place_left_queue, 'right': self.place_right_queue}
        self.place_sucang = {'left': self.sucang_left_queue, 'right': self.sucang_right_queue}
    
    def init(self):
        for pose in pick_pose_left:
            self.pick_left_queue.put(pose)
        for pose in pick_pose_right:
            self.pick_right_queue.put(pose)
        for pose in place_pose_left:
            self.place_left_queue.put(pose)
        for pose in place_pose_right:
            self.place_right_queue.put(pose)
        for angle in place_sucang_left:
            self.sucang_left_queue.put(angle)
        for angle in place_sucang_right:
            self.sucang_right_queue.put(angle)

    def state_control(self, state, side):
        if state is None:
            state = State.init
        elif state == State.init:
            state = State.pick_and_place
        elif state == State.pick_and_place:
            if self.pick_pose[side].empty():
                state = State.finish
            else:
                state = State.pick_and_place
        elif state == State.finish:
            state = None
        return state

    def strategy(self, state, side):
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.8, 0, 2.57, 0, -0.87, 0]
            cmd['state'] = State.init
            cmd['speed'] = 20
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)

        elif state == State.pick_and_place:
            pick_pose = self.pick_pose[side].get()
            place_pose = self.place_pose[side].get()
            suc_angle = self.place_sucang[side].get()
            cmd['state'] = State.pick_and_place
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = pick_pose[0], pick_pose[1], 0
            cmd['suc_cmd'], cmd['noa'] = 0, [0, 0, -0.25]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = pick_pose[0], pick_pose[1], 0
            cmd['suc_cmd'], cmd['noa'] = 0, [0, 0, 0.045]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'], cmd['noa'] = 'grasping', 'line', [0, 0, 0.03]
            cmd['suc_cmd'], cmd['speed'] = 'On', 5
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'],  = 'relativePos', 'line'
            cmd['speed'], cmd['pos'] = 20, [0, 0, 0.33]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['suc_cmd'] = 'jointMove', 'calibration'
            cmd['jpos'] = [0, 0, -1.8, 0, 2.57, 0, -0.87, 0]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'] = [0.45, place_pose[0][1], place_pose[0][2]+0.05]
            cmd['euler'], cmd['phi'] = place_pose[1], 0
            if place_pose[0][2] > -0.25:
                cmd['mode'] = 'p2p'
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = place_pose[0], place_pose[1], 0
            cmd['suc_cmd'] = suc_angle
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'],  = 'noaMove', 'line'
            cmd['noa'] = [0, 0, -0.02]
            cmd['suc_cmd'] = 'Off'
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'] = [0.45, place_pose[0][1], place_pose[0][2]+0.03]
            cmd['euler'], cmd['phi'] = place_pose[1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['suc_cmd'] = 'jointMove', 'calibration'
            cmd['jpos'] = [0, 0, -1.8, 0, 2.57, 0, -0.87, 0]
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)

        elif state == State.finish:
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.finish
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)

    def process(self):
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.dual_arm.shutdown)
        while True:
            l_status = self.dual_arm.left_arm.status
            if l_status == Status.idle or l_status == Status.occupied:
                l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
                self.strategy(l_state, 'left')
            rate.sleep()
            r_status = self.dual_arm.right_arm.status
            if r_status == Status.idle or r_status == Status.occupied:
                r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
                self.strategy(r_state, 'right')
            rate.sleep()
            if l_state == State.finish and r_state == State.finish:
                return

if __name__ == '__main__':
    rospy.init_node('expired')
    strategy = StockingTask('dual_arm', False)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
        
