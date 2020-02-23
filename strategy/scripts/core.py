#!/usr/bin/env python
import rospy
import sys
import math
from std_msgs.msg import String
from my_state_machine import MyStateMachine
import dynamic_reconfigure.client


class Strategy(object):
    def __init__(self, sim=False):
        rospy.init_node('core', anonymous=False)
        self.rate = rospy.Rate(200)
        self.robot = MyStateMachine(sim)
        self.dclient = dynamic_reconfigure.client.Client(
            "core", timeout=30, config_callback=None)
        self.main()

    def main(self):
        while not rospy.is_shutdown():
            s = self.robot.get_mir_status()
            # print(s['mir_state'])

            if self.robot.go_home:
                self.robot.toMove("TKU_ToHOME")
                self.dclient.update_configuration({"go_home": False})

            if not self.robot.is_idle and not self.robot.start:
                self.robot.toIdle()

            if self.robot.is_idle:
                if self.robot.start:
                    self.robot.toMove("TKU_ToROOMA")

            if self.robot.is_move:
                if s['mir_state'] == "Ready":
                    self.robot.toMove("TKU_ToSHELF")
                    #self.dclient.update_configuration({"start": False})
                    ## TODO: How to detect MiR arrived/completed position/mission
                else:
                    print(type(s['mir_state']))
                    print(s['mir_state'])
                    print(s['mir_state'] is "Ready")
                    print(s['mir_state'] == "Ready")

            if rospy.is_shutdown():
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        s = Strategy(True)  # True is simulated mode
    except rospy.ROSInterruptException:
        pass
