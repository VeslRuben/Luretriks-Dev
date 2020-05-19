#!/usr/bin/env python
import socket
import json
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import numpy as np

hostIp = rospy.get_param("ip", {"ip": socket.gethostname()})
hostIp = hostIp["ip"]

port = rospy.get_param("port", {"port": "5005"})
port = int(port["port"])
print("server started on host: {}, port: {}".format(hostIp, port))

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setblocking(False)
serverAddress = ('192.168.0.103', 5005)
s.bind(serverAddress)
resipients = []

ranges = []
angles = []
redy = True

forward = 0
right = 0
left = 0


def motor_calback(msg):
    """
    formats and saves the latest motor output message
    :param msg: geometry_msgs/Twist
    :return: None
    """
    global forward, right, left
    forward = int(msg.linear.x)
    right = int(msg.linear.y)
    left = int(msg.linear.z)


def handel_laser(data):
    """
    formats and saves the latest laserScan message
    :param data: sensor_msgs/LaseScan
    :return: None
    """
    global ranges, angles, redy, forward, right, left
    redy = False
    ranges = data.ranges
    aMin = data.angle_min
    aMax = data.angle_max
    increment = data.angle_increment
    angles = [x for x in np.arange(aMin, aMax, increment)]
    redy = True


# print(ranges_json)

def main():
    """
    publishes laser scan and motor state with a rate of 20Hz
    :return: None
    """
    global ranges, angles, redy
    rospy.init_node('scan_listener', anonymous=True)
    rospy.Subscriber("scan", LaserScan, handel_laser)
    rospy.Subscriber("roboclaw", Twist, motor_calback)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            message, address = s.recvfrom(1024)
            if address not in resipients:
                resipients.append(address)
                print("Laser pub has Conection from: {}, message: {}".format(address, message))
        except socket.error as e:
            pass
        if redy:
            dictionary = {
                "Lidar": {
                    "Ranges": ranges,
                    "Angles": angles
                },
                "Motor": {
                    "Motor1": forward,
                    "Motor2": forward,
                    "Motor3": right,
                    "Motor4": left
                }
            }
            ranges_json = json.dumps(dictionary)
            for r in resipients:
                s.sendto(ranges_json.encode(), r)

        rate.sleep()


if __name__ == "__main__":
    main()
