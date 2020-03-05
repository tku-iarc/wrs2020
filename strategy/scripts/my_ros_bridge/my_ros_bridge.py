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

        # self._ac.wait_for_result()
        # return self._ac.get_result()

    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def callback_done(self, state, result):
        ## state: 3 is finish?
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

    @property
    def arm_result(self):
        return self._ac.get_result()

    @property
    def arm_state(self):
        ## According actionlib_msgs/GoalStatus
        gsd = {
            0: "PENDING",
            1: "ACTIVE",
            2: "PREEMPTED",
            3: "SUCCEEDED",
            4: "ABORTED",
            5: "REJECTED",
            6: "PREEMPTING",
            7: "RECALLING",
            8: "RECALLED",
            9: "LOST"
        }
        return gsd.get(self._ac.get_state())

    def cancel_arm_task(self, method="all"):
        if method is "all":
            self._ac.cancel_all_goals()
        else:
            self._ac.cancel_goal()
