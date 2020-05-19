#!/usr/bin/env python
import json
import math

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu

covariance = None
offset = None
acc_raw = None

velocity = [0, 0, 0]
trans = [0, 0, 0]
last_time = 0
firist = True

"""
Not in use
"""

def handle_imu_raw(msg):
    global last_time, velocity, acc_raw, firist, trans
    if not firist:
        l = msg.linear_acceleration

        l.x = l.x - acc_raw[0]
        l.y = l.y - acc_raw[1]
        l.z = l.z - acc_raw[2]

        t = msg.header.stamp.to_sec()
        dt = t - last_time
        last_time = t
        v = [(a * dt) + vel for a, vel in zip([l.x, l.y, l.z], velocity)]
        velocity = v

        s = [(ve * dt) + t for ve, t in zip(velocity, trans)]
        trans = s
        print s
    else:
        last_time = msg.header.stamp.to_sec()
        firist = False


if __name__ == '__main__':
    print "hello"
    rospy.init_node('custom_subscriber', anonymous=True)
    # rospy.Subscriber('imu/data', Imu, handle_imu)
    rospy.Subscriber('imu/sensor_raw', Imu, handle_imu_raw)

    with open("/home/ruben/luretriks-dev/src/calibrate/data/calibration_data.json") as cal_file:
        calibration_config = json.load(cal_file)
    covariance = calibration_config['covariance']
    offset = calibration_config['offset']
    acc_raw = offset["acc_raw"]

    while not rospy.is_shutdown():
        pass
