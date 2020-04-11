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


c_pose = {'left' :[[[0.38,  0.2, 0.1],  [0.0, 80, 0.0]],
                    [[0.38,  0.2, -0.3],  [0.0, 80, 0.0]],
                    [[0.38,  0.2, -0.7],    [0.0, 80, 0.0]]],
          'right':[[[0.38, -0.2, 0.1],  [0.0, 80, 0.0]],
                    [[0.38, -0.2, -0.3],  [0.0, 80, 0.0]],
                    [[0.38, -0.2, -0.7],    [0.0, 80, 0.0]]],
          'left_indx' : 0, 'right_indx' : 0}

place_pose = [[[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.45,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],                             
              [[-0.45,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0.02, -0.75],[0.0, 0.0, 0.0]],
              [[-0.45,  -0.02, -0.75],[0.0, 0.0, 0.0]],
              [[-0.38,  -0.02, -0.75],[0.0, 0.0, 0.0]],                             
              [[-0.45,  0.02, -0.75],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],
              [[-0.45,  0, -0.7],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],                             
              [[-0.45,  0, -0.7],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],
              [[-0.45,  0, -0.7],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],                             
              [[-0.42,  0, -0.7],[0.0, 0.0, 0.0]]]

obj_pose = [[[[0.465, -0.1, -0.18], [0, 90, 0]],
            [[0.465,  0.1, -0.18], [0, 90, 0]]],
            [[[0.545, -0.1, -0.43], [0, 90,  0]],
            [[0.545,  0.1, -0.43], [0, 90,  0]]],
            [[[0.6, -0.2, -0.883], [0, 90,  0]],
            [[0.6,  0.2, -0.883], [0, 90, 0]]]]



class ObjInfo(dict):
    def __init__(self):
        self['id']      = 0
        self['side_id'] = 'front'               # 'front', 'back', 'side'
        self['name']    = 'plum_riceball' # 'plum_riceball', 'salmon_riceball', 'sandwich', 'burger', 'drink', 'lunch_box'
        self['state']   = 'new'           # 'new', 'old', 'expired'
        self['pos']     = None
        self['euler']   = None
        self['sucang']  = 0

class State(IntEnum):
    init            = 0
    get_obj_inf     = 1
    select_obj      = 2
    move2obj        = 3
    check_pose      = 4
    pick            = 5
    place           = 6
    finish          = 7

class ExpiredTask:
    def __init__(self, _name, en_sim):
        self.name = _name
        self.en_sim = en_sim
        self.state = State.init
        self.dual_arm = DualArmTask(self.name, self.en_sim)
        self.camara = GetObjInfo()
        self.left_cpose_queue = queue.Queue()
        self.right_cpose_queue = queue.Queue()
        self.place_pose_queue = queue.Queue()
        self.object_queue = queue.Queue()
        self.object_list = []
        self.left_tar_obj = queue.Queue()
        self.right_tar_obj = queue.Queue()
        self.retry_obj_queue_left = queue.Queue()
        self.retry_obj_queue_right = queue.Queue()
        self.target_obj_queue = {'left' : self.left_tar_obj, 'right' : self.right_tar_obj}
        self.target_obj = {'left': None, 'right': None}
        self.retry_obj_queue = {'left': self.retry_obj_queue_left, 'right': self.retry_obj_queue_right}
        self.obj_done = np.zeros((100), dtype=bool)
        self.obj_retry = np.zeros((100), dtype=bool)
        self.next_level = {'left': False, 'right': False}
        self.init()
    
    def init(self):
        for pose in place_pose:
            self.place_pose_queue.put(pose)

    def get_obj_inf(self, side):
        fb = self.dual_arm.get_feedback(side)
        ids, mats, names, exps, side_ids = self.camara.get_obj_info(side, fb.orientation)
        
        if ids is None:
            return
        for _id, mat, name, exp, side_id in zip(ids, mats, names, exps, side_ids):
            obj = ObjInfo()
            obj['id'] = _id
            obj['name'] = name
            obj['expired'] = exp
            obj['side_id'] = side_id
            obj['pos'] = mat[0:3, 3]
            obj['vecter'] = mat[0:3, 2]
            obj['sucang'], roll = self.dual_arm.suc2vector(mat[0:3, 2], [0, 1.57, 0])
            obj['euler']   = [roll, 90, 0]
            if obj['vecter'][2] > -0.1:
                self.object_queue.put(obj)

    def arrange_obj(self, side):
        pass

    def check_pose(self, side):
        self.target_obj[side] = self.target_obj_queue[side].get()
        fb = self.dual_arm.get_feedback(side)
        ids, mats, _, _, _ = self.camara.get_obj_info(side, fb.orientation)
        if ids is None:
            return
        for _id, mat in zip(ids, mats):
            if _id == self.target_obj[side]['id']:
                self.target_obj[side]['pos'] = mat[0:3, 3]
                if mat[2, 2] > -0.1:
                    self.target_obj[side]['sucang'], roll = self.dual_arm.suc2vector(mat[0:3, 2], [0, 1.57, 0])
                    self.target_obj[side]['euler']   = [roll, 90, 0]
        pass

    def state_control(self, state, side):
        if state is None:
            state = State.init
        elif state == State.init:
            state = State.get_obj_inf
        elif state == State.get_obj_inf:
            state = State.select_obj
        elif state == State.select_obj:
            if self.object_queue.empty():
                if c_pose[side+'_indx'] >= 3:
                    state = State.finish
                else:
                    state = State.get_obj_inf
            else:
                state = State.move2obj
        elif state == State.move2obj:
            state = State.check_pose
        elif state == State.check_pose:
            state = State.pick
        elif state == State.pick:
            if side == 'left':
                is_grip = self.dual_arm.left_arm.suction.is_grip
            else:
                is_grip = self.dual_arm.right_arm.suction.is_grip
            if is_grip:
                state = State.place
            elif self.next_level[side] == True:
                self.next_level[side] = False
                if c_pose[side+'_indx'] >= 3:
                    state = State.finish
                else:
                    state = State.get_obj_inf
            else:
                self.retry_obj_queue[side].put(self.target_obj[side])
                state = State.move2obj
        elif state == State.place:
            if self.next_level[side] == True:
                self.next_level[side] = False
                if c_pose[side+'_indx'] >= 3:
                    state = State.finish
                else:
                    state = State.get_obj_inf
            else:
                state = State.move2obj
        return state
            
    def strategy(self, state, side):
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.init
            cmd['speed'] = 20
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            
        elif state == State.get_obj_inf:
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'] = 'occupied'
            cmd['state'] = State.get_obj_inf
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] += 1
            else:
                print('fuckfailfuckfailfuckfail')
                        
        elif state == State.select_obj:
            self.get_obj_inf(side)
            self.arrange_obj(side)
            cmd['cmd'], cmd['state'] = None, State.select_obj
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)
            
        elif state == State.move2obj:
            obj = None
            if self.retry_obj_queue[side].empty() and self.target_obj_queue[side].empty():
                if self.object_queue.empty():
                    self.next_level[side] = True
                    return
                for _ in range(self.object_queue.qsize()):
                    obj = self.object_queue.get()
                    if self.obj_done[obj['id']] == False:
                        if side == 'left' and obj['pos'][1] < -0.02:
                            self.object_queue.put(obj)
                            self.obj_done[obj['id']] = False
                            return
                        if side == 'right' and obj['pos'][1] > 0.02:
                            self.object_queue.put(obj)
                            self.obj_done[obj['id']] = False
                            return
                        self.obj_done[obj['id']] = True
                        break
            elif self.target_obj_queue[side].empty():
                obj = self.retry_obj_queue[side].get()
                if self.obj_retry[obj['id']] == False:
                    self.obj_retry[obj['id']] = True
                else:
                    return
            else:
                obj = self.target_obj_queue[side].get()
                if self.obj_done[obj['id']] == False:
                    self.obj_retry[obj['id']] = True
                else:
                    return

            pos = copy.deepcopy(obj['pos'])
            pos[1] += 0.032
            pos[2] += 0.065
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'], cmd['mode'], cmd['state'] = 'ikMove', 'p2p', State.move2obj
            cmd['pos'], cmd['euler'], cmd['phi'] = [0.4, pos[1], pos[2]], [0, 90, 0], 0
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'] = 'occupied'
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side == 'fail':
                self.object_queue.put(obj)
                self.obj_done[obj['id']] = False
                print('fffffffffffuuuuuuuuuuccccccccccckkkkkkkkkkk')
            else:
                self.target_obj_queue[side].put(obj)
                print('side = ', side, 'id = ',obj['id'])

        elif state == State.check_pose:
            self.check_pose(side)
            cmd['cmd'], cmd['state'] = 'occupied', State.check_pose
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)
            
        elif state == State.pick:
            obj = self.target_obj[side]
            cmd['state'] = State.pick
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = obj['pos'], obj['euler'], 0
            cmd['suc_cmd'], cmd['noa'] = obj['sucang'], [0, 0, -0.05]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'], cmd['noa'] = 'grasping', 'line', [0, 0, 0.08]
            cmd['suc_cmd'], cmd['speed'] = 'On', 10
            if obj['vecter'][2] < 0.2:
                cmd['speed'] = 20
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'],  = 'relativePos', 'line'
            cmd['speed'], cmd['suc_cmd'] = 20, 'calibration'
            cmd['pos'] = [0, 0, 0.03]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = [0.45, obj['pos'][1], obj['pos'][2]+0.065], obj['euler'], 0
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)

        elif state == State.place:
            cmd['state'] = State.place
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.5, 0, 2.07, 0, -0.57, 0]
            cmd_queue.put(copy.deepcopy(cmd))
            pose = self.place_pose_queue.get()
            pos, euler = pose[0], pose[1]
            if side == 'left':
                pos[1] += 0.12
            else:
                pos[1] -= 0.12
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = pos, euler, 0
            cmd['suc_cmd'], cmd['noa'] = 0, [0, 0, -0.2]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'], cmd['noa'] = 'noaMove', 'line', [0, 0, 0.2]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'], cmd['noa'] = 'noaMove', 'line', [0, 0, -0.2]
            cmd['suc_cmd'] = 'Off'
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.5, 0, 2.07, 0, -0.57, 0]
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)

        elif state == State.finish:
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.finish
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
        return side

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

    strategy = ExpiredTask('dual_arm', False)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
