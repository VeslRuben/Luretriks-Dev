#!/usr/bin/env python

import rospy

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

"""
Test version, not in use
"""

class ActionServer():

    def __init__(self):
        self.a_server = actionlib.SimpleActionServer('test_receiver_as', MoveBaseAction, execute_cb=self.execute_cb,
                                                     auto_start=False)
        print('do you even print bro')
        self.a_server.start()
        print('i even print bro')

    def execute_cb(self, goal):
        print(goal)

if __name__ == "__main__":
    rospy.init_node('action_server')
    s = ActionServer()
    rospy.spin()