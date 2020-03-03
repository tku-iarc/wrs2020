#!/usr/bin/env python
import rospy
import sys
import math
from std_msgs.msg import String
from my_state_machine import MyStateMachine
import dynamic_reconfigure.client


HOST = "http://192.168.50.220:8080/v2.0.0"

class Strategy(object):
    def __init__(self, sim=False):
        rospy.init_node('core', anonymous=False)
        self.rate = rospy.Rate(200)
        self.robot = MyStateMachine(sim)
        self.mir = MIR(HOST)
        self.dclient = dynamic_reconfigure.client.Client(
            "core", timeout=30, config_callback=None)
        self.main()

    def main(self):
        while not rospy.is_shutdown():
            # s = self.robot.get_mir_status()
            # print(s['mir_state'])
            # print(self.robot.current_state)

            if self.robot.go_home and not self.robot.is_home:
                self.robot.toHome()

            if self.robot.is_home and self.robot.arrived_position("HOME"):
                self.robot.toIdle()

            if not self.robot.is_idle and not self.robot.start \
                                      and not self.robot.is_home:
                self.robot.toIdle()

            if self.robot.is_idle:
                if self.robot.start:
                    # self.robot.toMove("TKU_ToROOMA")
                    self.robot.toMove("TKU_ToSHELF")

            if self.robot.is_move:
                # if s['mir_state'] == "Ready" and self.robot.arrived_position("SHELF"):
                if self.mir.status['mir_state'] == "Ready" \
                   and self.mir.arrived_position("SHELF"):
                    print("Arrived")
                    self.robot.toIdle()

            if rospy.is_shutdown():
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        s = Strategy(True)  # True for simulated mode
    except rospy.ROSInterruptException:
        pass
