#!/usr/bin/env python

"""Use to reset pose of model for simulation."""

import rospy
from gazebo_msgs.msg import ModelState


def set_object(name, pos, ori):
    msg = ModelState()
    msg.model_name = name
    msg.pose.position.x = pos[0]
    msg.pose.position.y = pos[1]
    msg.pose.position.z = pos[2]
    msg.pose.orientation.w = ori[0]
    msg.pose.orientation.x = ori[1]
    msg.pose.orientation.y = ori[2]
    msg.pose.orientation.z = ori[3]
    msg.reference_frame = 'world'
    
    set_mode_pub.publish(msg)
    print msg

    
if __name__ == '__main__':
    rospy.init_node('set_obj')
    print("set_link_state")
    
    set_mode_pub = rospy.Publisher(
        '/gazebo/set_model_state',
        ModelState,
        queue_size=1,
        latch=True
    )

    names = ('lunchbox1', 'lunchbox2', 'lunchbox3', 'lunchbox4', 
             'drink1', 'drink2', 'drink3', 'drink4',
             'riceball1', 'riceball2', 'riceball3', 'riceball4')
    pos  = ((0.09, -0.1625, 0.685),
            (0.09, -0.1625, 0.74),
            (0.09, 0.1625, 0.685),
            (0.09, 0.1625, 0.74),
            (0.23, -0.22, 0.74),
            (0.23, -0.1, 0.74),
            (0.32, -0.1, 0.74),
            (0.32, -0.22, 0.74),
            (0.22, 0.22, 0.67),
            (0.22, 0.1, 0.67),
            (0.31, 0.1, 0.67),
            (0.31, 0.22, 0.67))
    ori  = ((0, 0, 0, 1),
            (0, 0, 0, 1),
            (0, 0, 0, 1),
            (0, 0, 0, 1),
            (0, 0, 0, 0),
            (0, 0, 0, 0),
            (0, 0, 0, 0),
            (0, 0, 0, 0),
            (0.707, 0, 0.707, 0),
            (0.707, 0, 0.707, 0),
            (0.707, 0, 0.707, 0),
            (0.707, 0, 0.707, 0))
    
    for i, name in enumerate(names):
        set_object(name, pos[i], ori[i])
        rospy.sleep(0.1)
