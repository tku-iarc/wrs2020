#!/usr/bin/env python3

from math import degrees
from enum import IntEnum

import rospy
import queue
from std_msgs.msg import Bool, Int32
from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status


c_pose = {'left' :[[[0.465,  0.1, -0.13],  [0.0, 90, 0.0]],
                    [[0.545,  0.1, -0.38],  [0.0, 90, 0.0]],
                    [[0.6,  0.2, -0.83],    [0.0, 90, 0.0]]],
          'right':[[[0.465, -0.1, -0.13],  [0.0, 90, 0.0]],
                    [[0.545, -0.1, -0.38],  [0.0, 90, 0.0]],
                    [[0.6, -0.2, -0.83],    [0.0, 90, 0.0]]],
          'left_indx' : 0, 'right_indx' : 0}

place_pose = [[[-0.38,  -0.2, -0.796],[0.0, 0.0, 0.0]],
              [[-0.46,  -0.2, -0.796],[0.0, 0.0, 0.0]],
              [[-0.38,  -0.08, -0.796],[0.0, 0.0, 0.0]],                             
              [[-0.46,  -0.08, -0.796],[0.0, 0.0, 0.0]]]

class ObjInfo(dict):
    def __init__(self):
        self['id']     : 0
        self['name']   : 'plum_riceball' # 'plum_riceball', 'salmon_riceball', 'sandwiche', 'burger', 'drink', 'lunch_box'
        self['state']  : 'new'           # 'new', 'old', 'expired'
        self['pos']    : None
        self['euler']  : None
        self['sucang'] : 0

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
        self.left_cpose_queue = queue.Queue()
        self.right_cpose_queue = queue.Queue()
        self.place_pose_queue = queue.Queue()
        self.object_queue = queue.Queue()
        self.target_obj = {'left' : ObjInfo(), 'right' : ObjInfo()}
        self.init()
    
    def init(self):
        for pose in place_pose:
            self.place_pose_queue.put(pose)

    def get_camara_inf(self, side):
        pass

    def arrange_obj(self, side):
        pass

    def check_pose(self, side):
        pass

    def process(self, state, side, arm):
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.init, 
            cmd_queue.put(cmd)
            self.dual_arm.send_cmd(side, False, cmd_queue)
            
        elif state == State.get_obj_inf:
            # if arm.status == Status.idle and not cpose_q.empty():
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(cmd)
            cmd['cmd'] = 'occupied'
            cmd['state'] = State.get_obj_inf
            cmd_queue.put(cmd)
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] += 1
                        
        elif state == State.select_obj:
            # if arm.Status == Status.occupied:
            self.get_camara_inf(side)
            self.arrange_obj(side)
            cmd['state'] = State.select_obj
            cmd_queue.put(cmd)
            self.dual_arm.send_cmd(side, True, cmd_queue)
            
        elif state == State.move2obj:
            obj = self.object_queue.get()
            pos, euler = obj['pos'], obj['euler']
            pos[2] += 0.03
            cmd['cmd'], cmd['mode'], cmd['state'] = 'ikMove', 'p2p', State.move2obj
            cmd['pos'], cmd['euler'], cmd['phi'] = [0.3, pos[1], pos[2]], euler, 0
            cmd_queue.put(cmd)
            cmd['cmd'], cmd['mode'], cmd['state'] = 'ikMove', 'line', State.move2obj
            cmd['pos'], cmd['euler'], cmd['phi'] = pos, euler, 0
            cmd_queue.put(cmd)
            cmd['cmd'] = 'occupied'
            cmd_queue.put(cmd)
            side = self.dual_arm.send_cmd('either', False, cmd_queue)
            if side != 'fail':
                self.target_obj[side] = obj
            else:
                self.object_queue.put(obj)
            
        elif state == State.check_pose:
            self.get_camara_inf(side)
            self.check_pose(side)
            cmd['cmd'], cmd['state'] = 'occupied', State.check_pose
            cmd_queue.put(cmd)
            self.dual_arm.send_cmd(side, True, cmd_queue)
            
        elif state == State.pick_and_place:
            obj = self.target_obj[side]
            cmd['cmd'], cmd['mode'], cmd['state'] = 'fromtNoaTarget', 'line', State.pick_and_place
            cmd['pos'], cmd['euler'], cmd['phi'] = obj['pos'], obj['euler'], 0
            cmd['suc_cmd'], cmd['a'] = obj['sucang'], -0.05
            cmd_queue.put(cmd)
            cmd['cmd'], cmd['mode'], cmd['a'] = 'grasping', 'line', 0.08
            cmd_queue.put(cmd)
            cmd['cmd'], cmd['mode'], cmd['state'] = 'fromtNoaTarget', 'line', State.pick_and_place
            cmd['pos'], cmd['euler'], cmd['phi'] = obj['pos'], obj['euler'], 0
            cmd['suc_cmd'], cmd['a'] = obj['sucang'], -0.05
            cmd_queue.put(cmd)
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = [0.3, obj['pos'][1], obj['pos'][2]], obj['euler'], 0
            cmd_queue.put(cmd)
            pose = self.place_pose_queue.get()
            pos, euler = pose[0], pose[1]
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = pos, euler, 0
            cmd['suc_cmd'], cmd['a'] = 0, -0.2
            cmd_queue.put(cmd)
            cmd['cmd'], cmd['mode'], cmd['a'] = 'noaMove', 'line', 0.2
            cmd_queue.put(cmd)
            cmd['cmd'], cmd['mode'], cmd['a'] = 'noaMove', 'line', -0.2
            cmd['suc_cmd'] = 'Off'
            cmd_queue.put(cmd)
            self.dual_arm.send_cmd(side, True, cmd_queue)
        elif state == State.finish:

            pass
        return state, side