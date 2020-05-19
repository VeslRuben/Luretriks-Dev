#!/usr/bin/env python
import math

import rospy
import tf
from sensor_msgs.msg import Imu
import numpy as np # Ruben
import json


class imuTransformer():

    def __init__(self):
        imuTopic = rospy.get_param("imu_topic")
        name = rospy.get_param('name')
        rospy.init_node(name, anonymous=True)
        print('hallo')
        rospy.Subscriber(imuTopic, Imu, self.imuCallback)
        rospy.Subscriber('imu/data_raw', Imu, self.imu_raw_callback)
        with open("/home/ruben/luretriks-dev/src/calibrate/data/test.json", 'r') as f:
            data = json.load(f)
        self.offset_g = data
        self.initial_quaternion = []
        self.first = True
        self.af_list = []
        self.counter = 0
        self.v0 = [0, 0, 0]
        self.s0 = [0, 0, 0]
        self.raw_acc = None


    def imu_raw_callback(self, msg):
        self.raw_acc = msg.linear_acceleration

    def imuCallback(self, imuData):
        self.counter += 1
        calibrate = False
        #print self.counter
        orientation = imuData.orientation
        linear_acceleration = imuData.linear_acceleration
        if self.first:
            self.first = False
            self.initial_quaternion = orientation
        orientation.x = orientation.x - self.initial_quaternion.x
        orientation.y = orientation.y - self.initial_quaternion.y
        orientation.z = orientation.z - self.initial_quaternion.z
        orientation.w = orientation.w - self.initial_quaternion.w

        if self.raw_acc is not None:
            af = self.ruben_var_her(orientation, self.raw_acc, grav_offset=self.offset_g) # Ruben
            if calibrate:
                if self.counter > 1000:
                    self.af_list.append(af)
                if len(self.af_list) == 5000:
                    afx = []
                    afy = []
                    afz = []
                    for x, y, z in self.af_list:
                        afx.append(x)
                        afy.append(y)
                        afz.append(z)

                    fx_avg = sum(afx)/len(afx)
                    fy_avg = sum(afy)/len(afy)
                    fz_avg = sum(afz)/len(afz)

                    d = {
                        "fx": fx_avg,
                        "fy": fy_avg,
                        "fz": fz_avg
                    }
                    with open("/home/ruben/luretriks-dev/src/calibrate/data/test.json", 'w') as tes:
                        json.dump(d, tes)

            if self.counter > 1000:
                s = self.ruben_var_her_2(af)
            else:
                s = [0, 0, 0]
            br = tf.TransformBroadcaster()
            br.sendTransform((s[0], s[1], 0), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), 'base_link', 'odom')

    def ruben_var_her(self, orientation, raw_acc, grav_offset):
        f_off = [grav_offset['fx'], grav_offset['fy'], grav_offset['fz']]
        qx, qy, qz, qw = [orientation.w, orientation.x, orientation.y, orientation.z]
        ax, ay, az = [raw_acc.x, raw_acc.y, raw_acc.z]
        a_raw = np.array([
            ax,
            ay,
            az
        ]).T
        rot_frame = np.array([
            [2*(qx**2) - 1 + 2*(qy**2), 2*qy*qz + 2*qx*qw, 2*qy*qw - 2*qx*qz],
            [2*qy*qz - 2*qx*qw, 2*(qx**2) - 1 + 2*(qz**2), 2*qz*qw + 2*qx*qy],
            [2*qy*qw + 2*qx*qz, 2*qz*qw - 2*qx*qy, 2*(qx**2) - 1 + 2*(qw**2)]
        ]).reshape(3, 3)

        rot_frame_inv = np.linalg.inv(rot_frame)
        af = rot_frame_inv.dot(a_raw).tolist()
        af = [a-o for a, o in zip(af, f_off)]
        # print rot_frame.dot(np.array([-0.18, 0.28, 10.45]).T)
        return af

    def ruben_var_her_2(self, acc_data, deadband=0.0):
        x, y, z = acc_data
        dt = 0.009  # TODO: make based on timestamp
        if abs(x) < deadband:
            x = 0
        if abs(y) <deadband:
            y = 0
        if abs(z) < deadband:
            z = 0
        v = [a * dt + v0 for a, v0 in zip([x, y, z], self.v0)]
        v = [temp * 0.1 for temp in v]
        #print math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        s = [v1 * dt + s0 for v1, s0 in zip(v, self.s0)]
        self.v0 = v
        self.s0 = s
        return s

    def run(self):
		#while not rospy.is_shutdown():
		#	br = tf.TransformBroadcaster()
		#	br.sendTransform((0, 0, 0), (0, 0, 0, 0), rospy.Time.now(), 'base_link', 'Odom')
		rospy.spin()

if __name__ == "__main__":
	imutrf = imuTransformer()
	imutrf.run()
