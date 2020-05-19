#!/usr/bin/env python
from std_msgs.msg import String
import rospy

if __name__ == "__main__":
    """
    sends alive messages to the safety-listener node
    """
    rospy.init_node("safety_publisher", anonymous=True)
    publisher = rospy.Publisher('alive', String, queue_size=10)

    rate = rospy.Rate(1)  # Hz
    while not rospy.is_shutdown():
        publisher.publish("alive")
        rate.sleep()
