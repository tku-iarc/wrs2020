#! /usr/bin/env python

from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import strategy.msg

client = actionlib.SimpleActionClient('arm_server', strategy.msg.ArmAction)

def fibonacci_client(cmd):

    client.wait_for_server()

    goal = strategy.msg.ArmGoal(cmd)

    client.send_goal(goal,
                     active_cb=callback_active,
                     feedback_cb=callback_feedback,
                     done_cb=callback_done)

    # action_state = client.get_state()
    # print(action_state)

    client.wait_for_result()

    return client.get_result()

def callback_active():
    rospy.loginfo("Action server is processing the goal")

def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))

    # if feedback.status == "10":
    #     goal = strategy.msg.ArmGoal(cmd="EJECT")
    #     client.send_goal(goal,
    #                      active_cb=callback_active,
    #                      feedback_cb=callback_feedback,
    #                      done_cb=callback_done)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client("stocking")
        print("Result: {}".format(result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client("disposing")
        print("Result: {}".format(result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)