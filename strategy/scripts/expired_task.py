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
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],                             
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],                             
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],                             
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],                             
              [[-0.42,  0, -0.796],[0.0, 0.0, 0.0]]]

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
    pick_and_place  = 5
    finish          = 6

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
        self.target_obj = {'left' : ObjInfo(), 'right' : ObjInfo()}
        self.obj_done = np.zeros((100), dtype=bool)
        self.init()
    
    def init(self):
        for pose in place_pose:
            self.place_pose_queue.put(pose)

    def get_obj_inf(self, side):
        fb = self.dual_arm.get_feedback(side)
        ids, mats, names, exps, side_ids = self.camara.get_obj_info(side, fb.orientation)
        obj = ObjInfo()
        if ids is None:
            return
        for _id, mat, name, exp, side_id in zip(ids, mats, names, exps, side_ids):
            obj['id'] = _id
            obj['name'] = name
            obj['expired'] = exp
            obj['side_id'] = side_id
            obj['pos'] = mat[0:3, 3]
#            obj['pos'][2] += 0.03
            obj['sucang'], roll = self.dual_arm.suc2vector(mat[0:3, 2], [0, 1.57, 0])
            obj['euler']   = [roll, 90, 0]
            self.object_queue.put(obj)
            # self.object_list.append(obj)

    def arrange_obj(self, side):
        # obj = ObjInfo()
        # for pose in obj_pose[c_pose[side+'_indx']-1]:
        #     obj['pos'], obj['euler'], obj['sucang'] = pose[0], pose[1], 0
        #     self.object_queue.put(copy.deepcopy(obj))
        pass

    def check_pose(self, side):
        fb = self.dual_arm.get_feedback(side)
        ids, mats, _, _, _ = self.camara.get_obj_info(side, fb.orientation)
        if ids is None:
            return
        for _id, mat in zip(ids, mats):
            if _id == self.target_obj[side]['id']:
                self.target_obj[side]['pos'] = mat[0:3, 3]
                self.target_obj[side]['sucang'], roll = self.dual_arm.suc2vector(mat[0:3, 2], [0, 1.57, 0])
                self.target_obj[side]['euler']   = [roll, 90, 0]
        pass

    def state_control(self, state, side):
        # if type(state) is tuple:
        #     state = state[0]
        if state is None:
            state = State.init
        elif state == State.init:
            state = State.get_obj_inf
        elif state == State.get_obj_inf:
            state = State.select_obj
        elif state == State.select_obj:
            state = State.move2obj
        elif state == State.move2obj:
            state = State.check_pose
        elif state == State.check_pose:
            state = State.pick_and_place
        elif state == State.pick_and_place:
            if self.object_queue.empty():
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
            while True:
                if self.object_queue.empty():
                    cmd['cmd'], cmd['state'] = None, State.pick_and_place # in order to change to next level
                    cmd_queue.put(copy.deepcopy(cmd))
                    self.dual_arm.send_cmd(side, True, cmd_queue)
                    return
                obj = self.object_queue.get()
                if self.obj_done[obj['id']] == False:
                    break
            pos, euler = copy.deepcopy(obj['pos']), obj['euler']
            pos[2] += 0.065
            cmd['cmd'], cmd['mode'], cmd['state'] = 'ikMove', 'p2p', State.move2obj
            cmd['pos'], cmd['euler'], cmd['phi'] = [0.4, pos[1], pos[2]], euler, 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'], cmd['mode'], cmd['state'] = 'ikMove', 'line', State.move2obj
            # cmd['pos'], cmd['euler'], cmd['phi'] = pos, euler, 0
            # cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'] = 'occupied'
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd('either', False, cmd_queue)
            if side != 'fail':
                self.target_obj[side] = obj
                print('side = ', side, 'id = ',obj['id'])
            else:
                self.object_queue.put(obj)
                print('fffffffffffuuuuuuuuuuccccccccccckkkkkkkkkkk')
            
        elif state == State.check_pose:
            # self.get_obj_inf(side)
            self.check_pose(side)
            cmd['cmd'], cmd['state'] = 'occupied', State.check_pose
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)
            
        elif state == State.pick_and_place:
            obj = self.target_obj[side]
            self.obj_done[obj['id']] = True
            cmd['state'] = State.pick_and_place
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = obj['pos'], obj['euler'], 0
            cmd['suc_cmd'], cmd['noa'] = obj['sucang'], [0, 0, -0.05]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'], cmd['noa'] = 'grasping', 'line', [0, 0, 0.08]
            cmd['suc_cmd'], cmd['speed'] = 'On', 5
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'],  = 'fromtNoaTarget', 'line'
            cmd['speed'] = 20
            cmd['pos'], cmd['euler'], cmd['phi'] = obj['pos'], obj['euler'], 0
            cmd['suc_cmd'], cmd['noa'] = obj['sucang'], [0, 0, -0.05]
            cmd_queue.put(copy.deepcopy(cmd))
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = [0.5, obj['pos'][1], obj['pos'][2]+0.065], obj['euler'], 0
            cmd_queue.put(copy.deepcopy(cmd))
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
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.finish, 
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
        return side

    def process(self):
        rate = rospy.Rate(2)
        rospy.on_shutdown(self.dual_arm.shutdown)
        while True:
            l_status = self.dual_arm.left_arm.status
            if l_status == Status.idle or l_status == Status.occupied:
                state = self.state_control(self.dual_arm.left_arm.state, 'left')
                self.strategy(state, 'left')
            r_status = self.dual_arm.right_arm.status
            if r_status == Status.idle or r_status == Status.occupied:
                state = self.state_control(self.dual_arm.right_arm.state, 'right')
                self.strategy(state, 'right')
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('expired')

    strategy = ExpiredTask('dual_arm', False)
    
    strategy.process()
    del strategy.dual_arm
