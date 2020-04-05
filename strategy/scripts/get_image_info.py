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
            # print(cv2.Rodrigues(rvec))
            # print(type(cv2.Rodrigues(rvec)[0]))
            # print(cv2.Rodrigues(rvec).shape)
            rotations = np.append(rotations, cv2.Rodrigues(rvec)[0])
	if rotations is None:
		return None
        rotations = rotations.reshape(int(len(rotations)/9), 3, 3)
        obj_mat = []
        for rot, tvec in zip(rotations, tvecs):
            obj_mat = np.append(obj_mat, self.visiontoArm(rot, tvec, arm_ori))
        obj_mat = obj_mat.reshape(int(len(obj_mat)/16), 4, 4)
        return ids, obj_mat

    def visiontoArm(self, rot, tvec, arm_ori):
        dx = -0.06   # unit:meter
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
        print(tvec[2])
        print(rot)
        Mat_nVec_Pos = np.mat([ [rot[0, 0], rot[0, 1], rot[0, 2],tvec[0]],
                                [rot[1, 0], rot[1, 1], rot[1, 2],tvec[1]],
                                [rot[2, 0], rot[2, 1], rot[2, 2],tvec[2]],
                                [   0,         0,         0,        1   ] ])
    
        Mat_VecPos_ImgToBase = T0_7 * TransMat_EndToImg * Mat_nVec_Pos
        print(Mat_VecPos_ImgToBase)
        return Mat_VecPos_ImgToBase
