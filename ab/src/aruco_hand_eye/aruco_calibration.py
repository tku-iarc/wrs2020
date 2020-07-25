#!/usr/bin/env python

import rospy
import sys
import numpy as np
from std_msgs.msg import Bool, Int32
from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status
from aruco_hand_eye.srv import hand_eye_calibration, hand_eye_calibrationRequest
from manipulator_h_base_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse
from geometry_msgs.msg import Transform

def get_fb(name):
    rospy.wait_for_service(name + '_arm/get_kinematics_pose')
    try:
        get_endpos = rospy.ServiceProxy(
            name + '_arm/get_kinematics_pose',
            GetKinematicsPose
        )
        res = get_endpos('arm')
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def hand_eye_client(req):
    rospy.wait_for_service('/camera/hand_eye_calibration')
    try:
        hand_eye = rospy.ServiceProxy('/camera/hand_eye_calibration', hand_eye_calibration)
        res = hand_eye(req)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    name = sys.argv[1]
    print(name)
    rospy.init_node('aruco_calibration')
    req = hand_eye_calibrationRequest()
    pose = get_fb(name)
    req.cmd = name
    req.end_trans.translation.x = pose.group_pose.position.x
    req.end_trans.translation.y = pose.group_pose.position.y
    req.end_trans.translation.z = pose.group_pose.position.z
    req.end_trans.rotation.w = pose.group_pose.orientation.w
    req.end_trans.rotation.x = pose.group_pose.orientation.x
    req.end_trans.rotation.y = pose.group_pose.orientation.y
    req.end_trans.rotation.z = pose.group_pose.orientation.z
    hand_eye_client(req)