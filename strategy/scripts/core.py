#!/usr/bin/env python
import rospy
import sys
import math
from std_msgs.msg import String
from my_state_machine import MyStateMachine
import dynamic_reconfigure.client
from mir_bridge.mir_bridge import MIR

## SSID: ASUS_TKU_5G_2
#HOST = "http://192.168.50.220:8080/v2.0.0"
## Wired Connected
# HOST = "http://192.168.12.20:8080/v2.0.0"

class Strategy(object):
    def __init__(self, sim=False):
        print("Initialized strategy core.py")
        rospy.init_node('core', anonymous=False)
        self.rate = rospy.Rate(200)
        self.sm = MyStateMachine(sim)
        # self.sm.mir = MIR(HOST)
        self.dclient = dynamic_reconfigure.client.Client(
            "core", timeout=30, config_callback=None)
        print("Initialized OK")
        self.main()

    def main(self):
        while not rospy.is_shutdown():
            # s = self.sm.get_mir_status()
            # print(s['mir_state'])
            # print(self.sm.current_state)

            if self.sm.go_home and not self.sm.is_home:
                self.sm.toHome()

            if self.sm.is_home and self.sm.mir.arrived_position("HOME"):
                self.sm.toIdle()

            if not self.sm.is_idle and not self.sm.start \
                                   and not self.sm.is_home:
                self.sm.toIdle()

            if self.sm.is_idle:
                if self.sm.start:
                    # self.sm.toMove("TKU_ToROOMA")
                    # self.sm.toMove("TKU_ToSHELF")
                    self.sm.toArm()

            if self.sm.is_move:
                # if s['mir_state'] == "Ready" and self.sm.arrived_position("SHELF"):
                if self.sm.mir.status['mir_state'] == "Ready" \
                   and self.sm.mir.arrived_position("SHELF"):
                    print("Arrived")
                    self.sm.toArm()

            if self.sm.is_arm:
                if self.sm.arm_result is not None:
                    if self.sm.arm_result.finish:
                        print("Arm task finished, Back to home!!!")
                        self.sm.toHome()
                    else:
                        print("Arm task doesn't finished...")
                        self.sm.toHome()

            if rospy.is_shutdown():
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        s = Strategy(True)  # True for simulated mode
    except rospy.ROSInterruptException:
        pass
