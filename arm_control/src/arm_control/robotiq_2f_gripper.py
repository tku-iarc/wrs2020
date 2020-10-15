#!/usr/bin/env python

import rospy
from math import radians, degrees, pi
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty

# NOW
cam2tool_z = 0.18 #0.27 #0.26
gripper_length = 0.04
cam2center_y = 0.035
cam2center_y_4_tote = 0.035#0.06 #0.05


class RobotiqGripper:

    def __init__(self, _name='/robotiq'):
        """Inital object."""
        self.name    = _name
        self.gripped = False
        self.speed = 255
        self.force = 150
        self.curr_pos = 0
        print('name = ', self.name)
        if 'gazebo' in self.name:
            if 'right' in self.name:
                self.gripper_cmd_pub = rospy.Publisher(
                    'mobile_dual_arm/right_arm/Robotiq2FGripperRobotOutput',
                    outputMsg,
                    queue_size=1
                )
                self.grip_status_sub = rospy.Subscriber(
                    'mobile_dual_arm/right_arm/Robotiq2FGripperRobotInput',
                    inputMsg,
                    self.grip_status_callback,
                    queue_size=1
                )
            elif 'left' in self.name :
                self.gripper_cmd_pub = rospy.Publisher(
                    'mobile_dual_arm/left_arm/Robotiq2FGripperRobotOutput',
                    outputMsg,
                    queue_size=1
                )
                self.grip_status_sub = rospy.Subscriber(
                    'mobile_dual_arm/left_arm/Robotiq2FGripperRobotInput',
                    inputMsg,
                    self.grip_status_callback,
                    queue_size=1
                )
        else:
            if 'right' in self.name:
                self.gripper_cmd_pub = rospy.Publisher(
                    'mobile_dual_arm/right_arm/Robotiq2FGripperRobotOutput',
                    outputMsg,
                    queue_size=1
                )
                self.grip_status_sub = rospy.Subscriber(
                    'mobile_dual_arm/right_arm/Robotiq2FGripperRobotInput',
                    inputMsg,
                    self.grip_status_callback,
                    queue_size=1
                )
            elif 'left' in self.name :
                self.gripper_cmd_pub = rospy.Publisher(
                    'mobile_dual_arm/left_arm/Robotiq2FGripperRobotOutput',
                    outputMsg,
                    queue_size=1
                )
                self.grip_status_sub = rospy.Subscriber(
                    'mobile_dual_arm/left_arm/Robotiq2FGripperRobotInput',
                    inputMsg,
                    self.grip_status_callback,
                    queue_size=1
                )

    def grip_status_callback(self, msg):
        self.is_grip = msg.gOBJ
        self.curr_pos = msg.gPO

    def gripper_open(self):
        cmd = outputMsg()
        cmd.rACT = 0
        cmd.rGTO = 0
        cmd.rATR = 0 
        cmd.rPR  = 0
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        self.gripper_cmd_pub.publish(cmd)
        rospy.sleep(0.5)

    def gripper_close(self):
        cmd = outputMsg()
        cmd.rACT = 0
        cmd.rGTO = 0
        cmd.rATR = 0 
        cmd.rPR  = 255
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        self.gripper_cmd_pub.publish(cmd)
        rospy.sleep(0.3)

    def gripper_setting(self, speed = -1, force = -1):
        self.speed = self.speed if speed == -1 else speed
        self.force = self.force if force == -1 else force
        cmd = outputMsg()
        cmd.rACT = 0
        cmd.rGTO = 0
        cmd.rATR = 0 
        cmd.rPR  = self.curr_pos
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        self.gripper_cmd_pub.publish(cmd)
        

    def gripper_pos(self, pos):
        cmd = outputMsg()
        cmd.rACT = 0
        cmd.rGTO = 0
        cmd.rATR = 0 
        cmd.rPR  = int(pos)
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        self.gripper_cmd_pub.publish(cmd)


    @property
    def is_grip(self):
        return self.gripped


if __name__ == '__main__':
    rospy.init_node('test_gripper')
    print('test_gripper')

    right_gripper = RobotiqGripper(_name='right')
    left_gripper = RobotiqGripper(_name='left')

    while not rospy.is_shutdown():
        for gripper in [right_gripper, left_gripper]:
            gripper.gripper_open()
            rospy.sleep(2)

            gripper.gripper_close()
            rospy.sleep(2)

            gripper.gripper_setting(255, 150)
            rospy.sleep(2)

            gripper.gripper_pos(150)
            rospy.sleep(2)

            gripper.gripper_close()
            rospy.sleep(2)