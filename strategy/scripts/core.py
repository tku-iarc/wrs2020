#!/usr/bin/env python
import rospy
import sys
import math
from std_msgs.msg import String
from my_state_machine import MyStateMachine
import dynamic_reconfigure.client


class Strategy(object):
  def __init__(self, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(200)
    self.robot = MyStateMachine(sim)
    self.main()

  def main(self):
    while not rospy.is_shutdown():
      if not self.robot.start:
        self.robot.toIdle()
      else:
        self.robot.toMove("ROOMA")

        if rospy.is_shutdown():
          break

        """
        ## Keep Current State Running
        keepState = 'to' + self.robot.current_state.name
        getattr(self.robot, keepState)
        """

        self.rate.sleep()

if __name__ == '__main__':
  try:
      s = Strategy(True) # True is simulated mode
  except rospy.ROSInterruptException:
    pass 
