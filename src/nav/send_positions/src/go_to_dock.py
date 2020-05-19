#!/usr/bin/env python
import time

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import tf


class GoToDock:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.init_pose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.client.wait_for_server()

    def sendGoal(self, pos, orientation):
        """
        Sends goal to action server
        :param pos: position [x, y, z]
        :param orientation: orientation in quaternions [x, y, z, w]
        :return: None
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = pos[0]
        goal.target_pose.pose.position.y = pos[1]
        goal.target_pose.pose.position.z = pos[2]

        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        print "goal sent"
        wait = self.client.wait_for_result()

    def active_cb(self):
        """
        Active callback function
        Not currently in use
        :return: None
        """
        print "active"
        # print "status: {}".format(status)

    def feedback_cb(self, feedback):
        """
        Feedback callback function
        Not currently in use
        :param feedback: feedback from action server
        :return: None
        """
        return
        # print "feedback: {}".format(feedback)

    def done_cb(self, result, yolo):
        """
        Callback function used when goal is reached
        :param result: Result feedback from server
        :param yolo: Not in use
        :return: None
        """
        if result == 0:
            print "not prossest"
        elif result == 1:
            print "beeing prosset"
        elif result == 2:
            print "cancle request"
        elif result == 3:
            print "sucside"
        elif result == 4:
            print "aborted"
        elif result == 5:
            print "rejected"
        elif result == 6:
            print "cancle 2"
        elif result == 7:
            print "canceling"
        elif result == 8:
            print "canceld"
        elif result == 9:
            print "gole lost"

        print "yolo: {}".format(yolo)

    def send_init_pose(self, pos, orientation):
        """
        Publishes initial pose to AMCL via topic initialpose
        :param pos: position in [x, y, z]
        :param orientation: orientation in quaternions [x, y, z, w]
        :return: None
        """
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()

        pose.pose.pose.position.x = pos[0]
        pose.pose.pose.position.y = pos[1]
        pose.pose.pose.position.z = pos[2]

        pose.pose.pose.orientation.x = orientation[0]
        pose.pose.pose.orientation.y = orientation[1]
        pose.pose.pose.orientation.z = orientation[2]
        pose.pose.pose.orientation.w = orientation[3]

        self.init_pose_publisher.publish(pose)


if __name__ == "__main__":
    rospy.init_node('go_to_dock', anonymous=False)
    gtd = GoToDock()
    time.sleep(1)

    # gtd.send_init_pose((-1.56, -0.45, 0), (0, 0, -0.71, 0.69))
    # time.sleep(1)

    q = tf.transformations.quaternion_from_euler(0, 0, 0)

    gtd.sendGoal((-2.6, 10.4, 0), (0, 0, -0.69, 0.72))

    gtd.sendGoal((-1.35, 10.4, 0), (0, 0, -0.69, 0.72))

