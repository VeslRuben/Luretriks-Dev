#!/usr/bin/env python
import rospy
from autodock_test_pkg.msg import TestMessage


def handle_person(data):
    rospy.loginfo("Received person: \n    Name: %s\n    Age: %s", data.test_string, data.test_int)


def subscriber():
    rospy.init_node('test_subscriber_node', anonymous=True)
    rospy.Subscriber('person', TestMessage, handle_person)
    rospy.spin()


if __name__ == '__main__':
    subscriber()
