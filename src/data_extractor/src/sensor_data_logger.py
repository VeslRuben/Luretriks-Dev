#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField
import serial
import numpy as np
import matplotlib.pyplot as plt
import math

class tbd:
    def __init__(self, serial_port):
        self.serial_port = serial_port

    def get_data(self, expected_params=19):
        while self.serial_port.in_waiting < 1:
            pass
        items = []
        while len(items) is not expected_params:
            data = list(bytearray(self.serial_port.readline()))
            string = ''.join(chr(i) for i in data)
            items = string.strip().split(',')
        raw = [float(s) for s in items]

        imu1 = {"acc": raw[0:3], "gyro": raw[3:6], "mag": raw[6:9]}
        imu2 = {"acc": raw[9:12], "gyro": raw[12:15]}
        imu3 = {"acc": raw[15:18]}
        t = raw[18]
        return imu1, imu2, imu3, t

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    baud_rate = 115200
    with serial.Serial(port, baud_rate, timeout=0.5) as serial_port:
        sensor = tbd(serial_port)

        i = 0
        datalist = []
        datalist2 = []
        datalist3 = []
        while not rospy.is_shutdown():
            if i <= 1000:
                imu1, imu2, imu3, t = sensor.get_data()
                datalist.append(imu1["acc"][0])
                datalist2.append(imu2["acc"][0])
                datalist3.append(imu3["acc"][0])
                i += 1
            else:
                #test_input = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

                newlist = [(x+y+z)/3 for x,y,z in zip(datalist, datalist2, datalist3)]

                window = []
                filter_output = []
                dt = 0.012
                for variabel in newlist:
                    if len(window) > 9:
                        window.pop(0)
                        window.append(variabel)
                    else:
                        window.append(variabel)
                    acc = float(sum(window)) / float(len(window)) + 0.17
                    filter_output.append(acc)

                v0 = 0
                s0 = 0
                velocity_over_time = [0]
                distance_over_time = [0]
                for acc in filter_output:
                    v0 = acc * dt + v0
                    velocity_over_time.append(v0)

                for vel in velocity_over_time:
                    s0 = vel * dt + s0
                    distance_over_time.append(s0)

                plt.figure()
                # plt.plot(range(0, len(datalist3)), datalist3)
                # plt.plot(range(0, len(datalist2)), datalist2)
                # plt.plot(range(0, len(datalist)), datalist)
                #plt.plot(range(0, len(newlist)), newlist)
                plt.plot(range(0, len(filter_output)), filter_output)
                plt.plot(range(0, len(velocity_over_time)), velocity_over_time)
                plt.plot(range(0, len(distance_over_time)), distance_over_time)
                plt.legend(['acc', 'vel', 'distance'])
                plt.grid()
                plt.show()
                rospy.signal_shutdown()