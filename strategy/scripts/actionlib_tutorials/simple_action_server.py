#! /usr/bin/env python
import rospy
import actionlib
import strategy.msg

class Arm(object):
    _feedback = strategy.msg.ArmFeedback()
    _result = strategy.msg.ArmResult()
    _counter = 0

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, strategy.msg.ArmAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True
        
        self._feedback.status = "STATUS"
        
        rospy.loginfo('%s: Executing, creating fibonacci sequence of cmd: %s with status: %s' % (self._action_name, goal.cmd, self._feedback.status))
        
        if goal.cmd == "Go":
            for i in range(1, 50):
                self._feedback.status = str(i)
                self._as.publish_feedback(self._feedback)
                r.sleep()

                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break

        if goal.cmd == "EJECT":
            for i in range(1, 50):
                self._feedback.status = str(i)
                self._as.publish_feedback(self._feedback)
                r.sleep()
                if i == 5:
                    self._as.set_aborted()
                    print("ABORTED, SORRY")

                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break

        if success:
            self._result.finish = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('arm_server')
    server = Arm(rospy.get_name())
    rospy.spin()