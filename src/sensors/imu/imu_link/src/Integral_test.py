#!/usr/bin/env python
import math

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu

ang = [[], [], []]
lin = [[], [], []]

"""
Not in use
"""

def handle_imu(msg):
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    print (round(math.degrees(r), 2), round(math.degrees(p), 2), round(math.degrees(y), 2))


def handle_imu_raw(msg):
    l = msg.linear_acceleration
    a = msg.angular_velocity
    if len(ang[0]) < 1000:
        ang[0].append(a.x)
        ang[1].append(a.y)
        ang[2].append(a.z)

        lin[0].append(l.x)
        lin[1].append(l.y)
        lin[2].append(l.z)


if __name__ == '__main__':
    rospy.init_node('custom_subscriber', anonymous=True)
    # rospy.Subscriber('imu/data', Imu, handle_imu)
    rospy.Subscriber('imu/data_raw', Imu, handle_imu_raw)
    while not rospy.is_shutdown():
        if len(ang[0]) == 1000:

            amx = np.mean(lin[0])
            amy = np.mean(lin[1])
            amz = np.mean(lin[2])

            lin[0] = lin[0] - amx
            lin[1] = lin[1] - amx
            lin[2] = lin[2] - amx

            b = np.array(lin)
            cv = b.dot(b.T)
            cv = cv / len(lin[0])
            print (cv)
            break


    # rospy.spin()