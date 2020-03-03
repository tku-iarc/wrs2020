#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import numpy as np
import time
import message_filters
import actionlib
import strategy.msg

class Robot(object):

    def __init__(self, sim=False):
        self._ac = actionlib.SimpleActionClient('arm_server', strategy.msg.ArmAction)
        if not sim:
            pass
        else:
            pass

    def call_arm(self, goal):

        if not self._ac.wait_for_server(timeout=rospy.Duration(10.0)): 
            rospy.logerr("Ping action server timed out")
            raise Exception("Ping action server timed out")
        else:
            rospy.loginfo("Ping action server found")

        g = strategy.msg.ArmGoal(cmd=goal)

        self._ac.send_goal(g,
                           active_cb=self.callback_active,
                           feedback_cb=self.callback_feedback,
                           done_cb=self.callback_done)

        # action_state = client.get_state()
        # print(action_state)
        # self._ac.wait_for_result()
        # return self._ac.get_result()

    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

    @property
    def action_state(self):
        return self._ac.get_state()
