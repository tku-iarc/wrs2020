#!/usr/bin/env python

"""Use to generate arm task and run."""

import os
import sys
import copy
from math import degrees
from enum import IntEnum


import rospy
from std_msgs.msg import Bool, Int32
from arm_control import ArmTask, SuctionTask





PICKORDER = 0
SPEED_R     = 20
SPEED_L     = 20
LUNCHBOX_H = 0.045
# The lesser one
lunchQuan = 1              
drinkQuan = 1
riceQuan  = 1

class Status(IntEnum):
    idle            = 0
    busy            = 1
    initPose        = 2
    frontSafetyPos  = 3
    rearSafetyPos   = 4
    move2Bin        = 5
    move2Shelf      = 6
    moveIn2Shelf    = 7
    leaveBin        = 8
    leaveShelf      = 9
    move2Object     = 10
    move2PlacedPos  = 11
    pickObject      = 12
    placeObject     = 13
    leavePlacePos   = 14
    grasping        = 15
    missObj         = 16
    finish          = 17
    backhome        = 18

objectName = ['lunchboxXX', 'lunchbox', 
              'drinkXX',    'drink',  
              'riceballXX', 'riceball']

lunchboxPos = [[-0.3,  0.1, -0.65],
               [-0.3,  0.2, -0.65]]

drinkPos =    [[-0.4, 0.1, -0.65],                              
               [-0.45, 0.22, -0.65]]

riceballPos = [[-0.39,  -0.22, -0.65],                             
               [-0.45,  -0.1, -0.65]]

lunchboxEu = [135, 0, 0]

drinkEu =    [0, 0, 0]
            
riceballXXEu = [45, 0, 0]
riceballEu   = [30, 0, 0]

               
objectPos = [lunchboxPos, drinkPos, riceballPos]
objectEu  = [lunchboxEu,  drinkEu,  riceballEu]

topRight    = [0.59, -0.25, -0.145]
topLeft     = [0.54,  0.19, -0.145]
middleRight = [0.57, -0.22, -0.46]
middleLeft  = [0.55,  0.1963, -0.46]
bottomRight = [0.57, -0.25, -0.9]
bottomLeft  = [0.54,  0.246, -0.85]

topRightEu    = [0, 90, 25]
topLeftEu     = [0, 90, 35]
middleRightEu = [0, 90,  -45]
middleLeftEu  = [0, 90,  -30]
bottomRightEu = [0, 90,  45]
bottomLeftEu  = [0, 90, -45]

topRightPhi    = -30
topLeftPhi     = 0
middleRightPhi = -25
middleLeftPhi  = -25
bottomRightPhi = -25
bottomLeftPhi  = -25

topRightSuc   = -68 
topLeftSuc    = -60


class DisposingTask:
    def __init__(self, _name = '/robotis'):
        """Initial object."""
        self.en_sim = False
        if len(sys.argv) >= 2:
            print(type(sys.argv[1]))
            if sys.argv[1] == 'True':
                rospy.set_param('self.en_sim', sys.argv[1])
                self.en_sim = rospy.get_param('self.en_sim')
        self.name = _name
        self.status = Status.initPose
        self.nowStatus = Status.initPose 
        self.nextStatus = Status.idle
        self.reGripCnt = 0
        self.arm = ArmTask(self.name + '_arm')
        self.pickListAll = len(lunchboxPos) + len(riceballPos) + len(drinkPos)
        self.pickList = PICKORDER
        self.pos   = [0, 0, 0]
        self.euler = [0, 0, 0]
        self.phi   = 0
        self.sucAngle = 0
        self.all_finish = False
        if self.name == 'right':
            self.is_right = 1
            self.speed = SPEED_R
        if self.name == 'left':
            self.is_right = -1
            self.speed = SPEED_L
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)
            rospy.on_shutdown(self.suction.gripper_vaccum_off)
        
    @property
    def finish(self):
        return self.pickList == self.pickListAll

    @property
    def all_finish(self):
        return self.all_finish
    
    @property
    def status(self):
        return self.status.name

    def setQuantity(self):
        for index in range(lunchQuan):
            objectName[index] = 'lunchboxXX'
            lunchboxPos[index][1] *= -1
            lunchboxPos[lunchQuan - index -1][2] += LUNCHBOX_H * index
        for index in range(2 - lunchQuan):
            lunchboxPos[2 - index -1][2] += LUNCHBOX_H * index
        for index in range(drinkQuan):
            objectName[index+2] = 'drinkXX'
        for index in range(riceQuan):
            objectName[index+4] = 'riceballXX'

    def getRearSafetyPos(self):
        self.pos   = [0, -0.5*self.is_right, -0.5]
        self.euler = [-90*self.is_right, -20, 30*self.is_right]
        self.phi   = -60*self.is_right

    def getFrontSafetyPos(self):
        self.pos   = (0.1, -0.4*self.is_right, -0.45)
        self.euler = (0, 0, 0*self.is_right)
        self.phi   = 45*self.is_right

    def getObjectPos(self):
        if self.finish:
            return
        while objectPos[self.pickList/2][self.pickList%2][1]*self.is_right > 0:
            self.pickList += 1 
            if self.finish:
                return
        self.pos   = objectPos[self.pickList/2][self.pickList%2][:]
        self.euler = objectEu[self.pickList/2][:]
        if objectName[self.pickList] == 'riceballXX':
            self.euler = riceballXXEu[:]
        self.euler[0] *= self.is_right
        self.euler[2] *= self.is_right
        self.phi   = -45*self.is_right
        if self.reGripCnt != 0:
            if self.reGripCnt == 1:
                if self.pickList == 4 or self.pickList == 6 or self.pickList == 8 or self.pickList == 10:
                    self.pos[0] += 0.005
                else:
                    self.pos[0] += 0.02
                self.pos[1] += 0.01
            if self.reGripCnt == 2:
                if self.pickList == 4 or self.pickList == 6 or self.pickList == 8 or self.pickList == 10:
                    self.pos[0] += 0.005
                else:
                    self.pos[0] += 0.02
                self.pos[1] -= 0.01
            if self.reGripCnt == 3:
                self.pos[0] -= 0.01
                self.pos[1] -= 0.01

    def getPlacePos(self):
        if objectName[self.pickList] == 'lunchboxXX':
            self.pos   = bottomRight[:]
            self.euler = bottomRightEu[:]
            self.phi   = bottomRightPhi*self.is_right
            self.sucAngle = -90
            # self.pos[2] += ((self.pickList%4))*0.05

        elif objectName[self.pickList] == 'lunchbox':
            self.pos   = bottomLeft[:]
            self.euler = bottomLeftEu[:]
            self.phi   = bottomLeftPhi*self.is_right
            self.sucAngle = -90
            # self.pos[2] += ((self.pickList%4) - lunchQuan)*0.05

        elif objectName[self.pickList] == 'drinkXX':
            self.pos   = middleRight[:]
            self.euler = middleRightEu[:]
            self.phi   = middleRightPhi*self.is_right
            self.sucAngle = -90
            # self.pos[0] += (drinkQuan - (self.pickList%4) - 1)*0.1

        elif objectName[self.pickList] == 'drink':
            self.pos   = middleLeft[:]
            self.euler = middleLeftEu[:]
            self.phi   = middleLeftPhi*self.is_right
            self.sucAngle = -90
            # self.pos[0] += (4 - (self.pickList%4) - 1)*0.1

        elif objectName[self.pickList] == 'riceballXX':
            self.pos   = topLeft[:]
            self.euler = topLeftEu[:]
            self.phi   = topLeftPhi*self.is_right
            self.sucAngle = topLeftSuc
            # self.pos[0] += (riceQuan - (self.pickList%4) - 1)*0.045

        elif objectName[self.pickList] == 'riceball':
            self.pos   = topRight[:]
            self.euler = topRightEu[:]
            self.phi   = topRightPhi*self.is_right
            self.sucAngle = topRightSuc
            # self.pos[0] += (4 - (self.pickList%4) - 1)*0.045      

    def process(self):
        if self.arm.is_stop:                                       # must be include in your strategy
            self.finish = True                                     # must be include in your strategy
            print "!!! Robot is stop !!!"                          # must be include in your strategy
            self.suction.gripper_vaccum_off()                      # must be include in your strategy
            return                                                 # must be include in your strategy

        if self.status == Status.idle:
            print "state 10"
            self.getObjectPos()
            if self.finish:
                self.status = Status.backhome
                return
            else:
                self.status = Status.frontSafetyPos
                
        elif self.status == Status.busy:
            if self.arm.is_busy:
                if (self.nowStatus == Status.leaveBin or self.nowStatus == Status.frontSafetyPos or self.nowStatus == Status.move2Shelf) and not self.suction.is_grip and not self.en_sim:
                    # self.status = Status.missObj
                    pass
                return
            else:
                self.status = self.nextStatus
                self.nowStatus = self.nextStatus
                return

        elif self.status == Status.initPose:
            self.status = Status.busy
            self.nextStatus = Status.idle
            self.arm.set_speed(self.speed)
            self.arm.jointMove(0, (0, -1, 0, 1.57, 0, -0.57, 0))
            self.suction.gripper_suction_deg(0)

        elif self.status == Status.leavePlacePos:
            self.status = Status.busy
            self.nextStatus = Status.leaveShelf
            self.arm.noa_move_suction('line', suction_angle=self.sucAngle, n=0, o=0, a=-0.02)

        elif self.status == Status.frontSafetyPos:
            self.status = Status.busy
            self.nextStatus = Status.move2Shelf
            self.getRearSafetyPos()
            self.euler[0] = -90*self.is_right
            if 'drink' in objectName[self.pickList]:
                self.pos[2] = -0.4
                self.euler[1] = -25
            elif 'lunchbox' in objectName[self.pickList]:
                self.pos[2] = -0.45
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.status == Status.rearSafetyPos:
            self.status = Status.busy
            self.nextStatus = Status.move2Bin
            self.getRearSafetyPos()
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
            self.suction.gripper_calibration()
            print "state 2"

        elif self.status == Status.move2Bin:
            print "state Status.move2Bin BBBBBBIIIIIIINNNNNN===================================================================="
            self.status = Status.busy
            self.nextStatus = Status.move2Object
            self.getObjectPos()
            self.pos[2] = -0.5
            self.euler[1] = -10
            if 'lunchbox' in objectName[self.pickList]:
                self.euler[0] *= 1.3
            self.arm.set_speed(self.speed)
            print(objectName[self.pickList])
            print(self.pos)
            print(self.euler)
            print(self.phi)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
            self.suction.gripper_suction_deg(0)
            rospy.sleep(1.0)
 
        elif self.status == Status.move2Shelf:
            self.status = Status.busy
            self.getPlacePos()
            self.nextStatus = Status.moveIn2Shelf
            self.pos[0] = 0.52
            self.pos[2] += 0.02
            self.arm.set_speed(self.speed)
            self.arm.noa_relative_pos('line', self.pos, self.euler, self.phi, suction_angle=0, n=0, o=0, a=-0.15)
        
        elif self.status == Status.moveIn2Shelf:
            self.status = Status.busy
            self.nextStatus = Status.move2PlacedPos
            self.getPlacePos()
            self.pos[2] += 0.02
            self.arm.set_speed(self.speed-10)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
            self.suction.gripper_suction_deg(-90)

        elif self.status == Status.leaveBin:
            print "state 8"
            self.status = Status.busy
            self.nextStatus = Status.idle
            self.arm.set_speed(self.speed)
            self.getObjectPos()
            self.pos[2] = -0.48
            if 'drink' in objectName[self.pickList]:
                self.pos[0] -= 0.02
                self.pos[2] = -0.42
                self.euler[1] = -30
                self.euler[2] = 40*self.is_right
            if self.pickList == 10:
                self.euler[1] = -6
                self.euler[2] = 10*self.is_right
            if 'lunchbox' in objectName[self.pickList]:
                self.euler[0] *= 1.1
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
            self.pickList += 1
            print "state 9"

        elif self.status == Status.leaveShelf:
            self.status = Status.busy
            self.nextStatus = Status.rearSafetyPos
            self.arm.set_speed(self.speed)
            self.getPlacePos()
            if 'riceball' in objectName[self.pickList]:
                self.euler[0] = 0
                self.nextStatus = Status.rearSafetyPos
                # self.pickList -= 1
            else:
                self.euler[0] = 0
            self.pos[0] = 0.45
            self.pos[2] += 0.01
            self.arm.set_speed(self.speed)
            self.arm.noa_relative_pos('line', self.pos, self.euler, self.phi, suction_angle=0, n=0, o=0, a=-0.08)

        elif self.status == Status.move2Object:
            print "state Status.move2Object===================================================================="
            self.status = Status.busy
            self.nextStatus = Status.placeObject
            self.getObjectPos()
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
            print(objectName[self.pickList])
            print(self.pos)

        elif self.status == Status.move2PlacedPos:
            self.status = Status.busy
            self.nextStatus = Status.pickObject
            self.getPlacePos()
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.status == Status.pickObject:
            self.status = Status.grasping
            self.suction.gripper_vaccum_on()
            self.arm.set_speed(5)
            self.arm.noa_move_suction('line', suction_angle=0, n=0, o=0, a=0.03)
            rospy.sleep(.1)
                        
        elif self.status == Status.placeObject:
            print "state place"
            self.status = Status.busy
            self.nextStatus = Status.leaveBin
            if 'lunchbox' in objectName[self.pickList]:
                self.nextStatus = Status.leaveBin
            rospy.sleep(.3)
            self.suction.gripper_vaccum_off()

        elif self.status == Status.grasping:
            if self.suction.is_grip or self.en_sim:
                self.arm.clear_cmd()
                self.status = Status.busy
                self.nextStatus = Status.leavePlacePos
                self.reGripCnt = 0
            elif not self.arm.is_busy:
                self.status = Status.missObj
        
        elif self.status == Status.missObj:
            print "state 7"
            if self.nowStatus == Status.pickObject or self.nowStatus == Status.leaveBin:
                self.status = Status.busy
                self.nextStatus = Status.move2Bin
                self.nowStatus = Status.idle
                self.arm.clear_cmd()
                self.reGripCnt += 1
                if self.reGripCnt > 3:
                    self.reGripCnt = 0
                    self.pickList += 1
                    if self.finish:
                        self.nextStatus = Status.idle
            elif self.nowStatus == Status.frontSafetyPos or self.nowStatus == Status.move2Shelf:
                self.status = Status.busy
                self.nextStatus = Status.idle
                self.nowStatus = Status.idle
                self.arm.clear_cmd()
                self.pickList += 1 
                self.arm.set_speed(self.speed)
        
        elif self.status == Status.backhome:
            rospy.loginfo('back home')
            self.status = Status.busy
            self.nextStatus = Status.finish
            self.arm.jointMove(0, (0, -1, 0, 2, 0, -0.7, 0))

        elif self.status == Status.finish:
            if self.all_finish:
                return
            self.status = Status.busy
            self.nextStatus = Status.finish
            self.arm.jointMove(0, (0, 0, 0, 0, 0, 0, 0))
            self.all_finish = True

def start_callback(msg):
    global is_start
    if msg.data == 1 and not is_start:
        is_start = True


if __name__ == '__main__':
    rospy.init_node('example')        # enable this node

    is_start = False
    rospy.Subscriber(
        'scan_black/dualarm_start_1',
        Int32,
        start_callback,
        queue_size=1
    )
    pub = rospy.Publisher(
        'scan_black/strategy_behavior',
        Int32,
        queue_size=1
    )

    right = DisposingTask('right')      # Set up right arm controller
    left  = DisposingTask('left')       # Set up left arm controller
    rospy.sleep(.3)
    right.setQuantity()

    while not rospy.is_shutdown() and not is_start:
        rospy.loginfo('waiting for start signal')
        rospy.sleep(.5)
    
    SuctionTask.switch_mode(True)

    rate = rospy.Rate(30)  # 30hz
    while not rospy.is_shutdown() and (not right.all_finish or not left.all_finish):
        left.process()
        right.process()
        rate.sleep()

    SuctionTask.switch_mode(False)
    # publish finish signal to wheels
    pub.publish(4)
