#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from math import sin, cos, radians
from strategy.srv import ArUcoInfo, ArUcoInfoRequest

class GetObjInfo():
    def __init__(self):
        pass

    def get_ar_marker(self, side):
        rospy.wait_for_service('get_ar_marker')
        try:
            req = rospy.ServiceProxy(
                'get_ar_marker',
                ArUcoInfo
            )
            res = req(side)
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_obj_info(self, side, arm_ori):
        res = self.get_ar_marker(side)
        ids = np.array(res.ids)
        rvecs = np.array(res.rvecs)
        tvecs = np.array(res.tvecs)
        rvecs = rvecs.reshape(int(len(rvecs)/3), 3)
        tvecs = tvecs.reshape(int(len(tvecs)/3), 3)
        rotations = []
        for rvec in rvecs:
            rotations = np.append(rotations, cv2.Rodrigues(rvec)[0])
	if rotations == []:
	    return None, None, None, None, None
        rotations = rotations.reshape(int(len(rotations)/9), 3, 3)
        obj_mat, names, exps, side_ids = [], [], [], []
        for rot, tvec, id in zip(rotations, tvecs, ids):
            obj_mat = np.append(obj_mat, self.visiontoArm(rot, tvec, arm_ori))
            name, exp, side_id = self.get_obj_name(id)
            names = np.append(names, name)
            exps = np.append(exps, exp)
            side_ids = np.append(side_ids, side_id)
        obj_mat = obj_mat.reshape(int(len(obj_mat)/16), 4, 4)
        self.filter_obj(ids, obj_mat, names, exps, side_ids)
        return ids, obj_mat, names, exps, side_ids

    def filter_obj(self, ids, obj_mat, names, exps, side_ids):
        for i in range(len(ids)-1):
            if names[i] == names[i+1] and exps[i] == exps[i+1]:
                if np.linalg.norm(np.subtract(obj_mat[i, 0:3, 3], obj_mat[i+1, 0:3, 3])) < 0.05:
                    if side_ids[i] == 0:
                        ids[i+1] = -1
                    elif side_ids[i+1] == 0:
                        ids[i] = -1
                    elif side_ids[i] == 1:
                        ids[i] = -1
                    elif side_ids[i+1] == 1:
                        ids[i+1] = -1
                    else:
                        ids[i] = -1
                    if ids[i+1] == -1:
                        ids[i], ids[i+1] = ids[i+1], ids[i]
                        names[i], names[i+1] = names[i+1], names[i]
                        exps[i], exps[i+1] = exps[i+1], exps[i]
                        side_ids[i], side_ids[i+1] = side_ids[i+1], side_ids[i]
                        obj_mat[i], obj_mat[i+1] = obj_mat[i+1], obj_mat[i]


                    

    def get_obj_name(self, id):
        exp = 'expired'
        if id >= 60:
            id -= 53
            exp = 'old'
        if id >= 30:
            id -= 23
            exp = 'new'
        if id <= 7:
            return 'lunch_box', exp, id-7 
        elif id <= 12:
            return 'plum_riceball', exp, id-8
        elif id <= 17:
            return 'salmon_riceball', exp, id-13
        elif id <= 22:
            return 'sandwich', exp, id-18
        elif id <= 24:
            return 'burger', exp, id-23
        elif id <= 28:
            return 'drink', exp, id-25
        else:
            return 'fuck', exp, id

    def visiontoArm(self, rot, tvec, arm_ori):
        dx = -0.04   # unit:meter
        dy = 0.032      # unit:meter
        dz = -0.14  # unit:meter
        rad = radians(10)   

        TransMat_EndToImg = np.mat([[0, cos(rad), -sin(rad), dx],
                                    [-1,       0,         0, dy],
                                    [0, sin(rad),  cos(rad), dz],
                                    [0,        0,         0, 1]])

        T0_7 = np.identity(4)
        for i in range(0,4):
            for j in range(0,4):
                T0_7[i][j] = arm_ori[i*4+j]
        Mat_nVec_Pos = np.mat([ [rot[0, 0], rot[0, 1], rot[0, 2],tvec[0]],
                                [rot[1, 0], rot[1, 1], rot[1, 2],tvec[1]],
                                [rot[2, 0], rot[2, 1], rot[2, 2],tvec[2]],
                                [   0,         0,         0,        1   ] ])
    
        Mat_VecPos_ImgToBase = T0_7 * TransMat_EndToImg * Mat_nVec_Pos
        return Mat_VecPos_ImgToBase
