#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyHandler:

    def __init__(self, pub):
        self.pub = pub

    def handel_joy(self, msg):
        """
        converts a sensor_msgs/joy to a geometry_msgs/Twist in the range between -0.2 to 0.2
        :param msg: sensor_msgs/joy
        :return: None
        """
        scaler_l = 0.2
        scaler_a = 0.2

        axis = msg.axes
        r_trigger = (axis[2] + 1) / 2
        l_trigger = (axis[5] + 1) / 2
        # linear motion
        forward = r_trigger* scaler_l
        forward -= l_trigger * scaler_l


        # turning motion
        turn = axis[3] * scaler_a

        # lateral shift
        lat = axis[0] * scaler_l

        t = Twist()
        t.linear.x = forward
        t.linear.y = lat
        t.angular.z = turn

        self.pub.publish(t)


if __name__ == "__main__":
    rospy.init_node("joy_to_vel", anonymous=True)
    pub = rospy.Publisher('joy_vel', Twist, queue_size=10)
    handler = JoyHandler(pub)
    rospy.Subscriber("joy", Joy, handler.handel_joy)
    rospy.spin()
