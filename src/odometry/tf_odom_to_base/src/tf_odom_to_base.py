#!/usr/bin/env python
import time

import rospy
import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import json


class imuTransformer():
    """
    In dire need of rework. Many different functions were added to this node,
    that are not in use.
    """

    def __init__(self):
        imuTopic = rospy.get_param("imu_topic")
        name = rospy.get_param('name')
        config_file = rospy.get_param('config_file_odom')
        rospy.init_node(name, anonymous=True)
        rospy.Subscriber(imuTopic, Imu, self.imuCallback)
        self.calibrate_reply_publisher = rospy.Publisher('calibrate/reply', String, queue_size=10)
        rospy.Subscriber("calibrate", String, self.handle_config)
        self.initial_quaternion = []
        self.first = True
        with open(config_file, 'r') as cf:
            config_data = json.load(cf)
        self.g_offset = config_data['g_offset']

        self.ready = False
        self.really_ready = False
        self.acc = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.distance = [0, 0, 0]
        self.t0 = 0
        self.timeStart = time.time()
        rospy.Subscriber("imu/sensor_raw", Imu, self.sensor_callback)

    def imuCallback(self, imuData):
        """
        Saves the latest Imu message to memory.
        publishes all frames needed to run slam
        :param imuData: sensor_msgs/Imu
        :return: None
        """
        if self.ready:
            orientation = imuData.orientation
            if self.first:
                self.first = False
                self.initial_quaternion = orientation
                time_stamp = imuData.header.stamp
                self.t0 = time_stamp.to_sec()

            else:
                rotation_matrix = self.createRotationMatrix(orientation)
                NED_acceleration = self.remove_gravity_from_rot_matrix(rotation_matrix, self.acc, self.g_offset)
                time_stamp = imuData.header.stamp
                t = time_stamp.to_sec()
                dt = t - self.t0
                self.t0 = t
                self.vel = [a*dt + v0 for a, v0 in zip(NED_acceleration, self.vel)]
                self.distance = [v*dt + s0 for v, s0 in zip(self.vel, self.distance)]
                #print NED_acceleration
                #print time.time() - self.timeStart

            orientation.x = orientation.x - self.initial_quaternion.x
            orientation.y = orientation.y - self.initial_quaternion.y
            orientation.z = orientation.z - self.initial_quaternion.z
            orientation.w = orientation.w - self.initial_quaternion.w

            roll0 = 0
            pitch0 = 0
            yaw0 = 0
            q0 = tf.transformations.quaternion_from_euler(roll0, pitch0, yaw0)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w))
            yaw_q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            q = tf.transformations.quaternion_from_euler(roll, pitch, 0)
            br = tf.TransformBroadcaster()

            br.sendTransform((0, 0, 0), yaw_q, rospy.Time.now(), 'base_footprint', 'odom')
            br.sendTransform((0, 0, 0.25), q0, rospy.Time.now(), 'base_stabilized', 'base_footprint')
            br.sendTransform((0, 0, 0), q, rospy.Time.now(), 'base_link', 'base_stabilized')

            #br.sendTransform((0, 0, 0), q0, rospy.Time.now(),'base_link', 'base_stabilized')

    def sensor_callback(self, msg):
        """
        Callback function receiving raw IMU-data
        :param msg: IMU-data
        :return: None
        """
        self.acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.ready = True

    def handle_config(self, msg):
        """
        Not currently in use.
        Was intended to be used to remove gravity from NED-matrix
        :param msg: std_msgs/String
        :return:
        """
        name = rospy.get_name()[1:]
        msg = msg.data
        reset_file_name = rospy.get_param('reset_file_odom')
        config_file_name = rospy.get_param('config_file_odom')
        if msg == "reset":
            with open(reset_file_name, 'r') as rf:
                reset_file_data = json.load(rf)
            with open(config_file_name, 'w') as cf:
                json.dump(reset_file_data, cf)
            self.calibrate_reply_publisher.publish(name + "::ok")
        else:
            sender, calibraton = msg.split("::")
            if sender == name:
                rospy.loginfo(name + " config received")
                with open(config_file_name, 'r') as cf:
                    json_calibration = json.loads(calibraton)
                    json_file = json.load(cf)
                    for key in json_calibration.keys():
                        json_file[key] = json_calibration[key]
                with open(config_file_name, 'w') as cf:
                    json.dump(json_file, cf)
                self.calibrate_reply_publisher.publish(name + "::ok")
        with open(config_file_name, 'r') as cf:
            config_data = json.load(cf)
            self.g_offset = config_data['g_offset']

    def createRotationMatrix(self, orientation):
        """
        Creates a rotation matrix from quaternions
        :param orientation: orientation in quaternions [x, y, z, w]
        :return: rotation matrix
        """
        # Extract information from orientation
        qx, qy, qz, qw = [orientation.w, orientation.x, orientation.y, orientation.z]

        rotation_matrix = np.array([
            [2 * (qx ** 2) - 1 + 2 * (qy ** 2), 2 * qy * qz + 2 * qx * qw, 2 * qy * qw - 2 * qx * qz],
            [2 * qy * qz - 2 * qx * qw, 2 * (qx ** 2) - 1 + 2 * (qz ** 2), 2 * qz * qw + 2 * qx * qy],
            [2 * qy * qw + 2 * qx * qz, 2 * qz * qw - 2 * qx * qy, 2 * (qx ** 2) - 1 + 2 * (qw ** 2)]
        ]).reshape(3, 3)

        return rotation_matrix

    def remove_gravity_from_rot_matrix(self, rotation_matrix, raw_acc, grav_offset):
        """
        Removes gravity from the rotation matrix.
        Not currently in use
        :param rotation_matrix: rotation matrix [3x3]
        :param raw_acc: raw acceleration data from accelerometer
        :param grav_offset: gravity offset
        :return: NED-matrix compensated for gravity
        """
        # Need inverse of Rot Matrix
        rotation_matrix_inverse = np.linalg.inv(rotation_matrix)

        # Multiply raw data from accel with inverse of rot matrix
        acc_NED = rotation_matrix_inverse.dot(raw_acc).tolist()

        # Remove gravity from North East Down-matrix
        acc_NED_compensated = [acc - offs for acc, offs in zip(acc_NED, grav_offset)]

        return acc_NED_compensated

    def run(self):
        """
        Runs program
        :return:
        """
        # while not rospy.is_shutdown():
        #	br = tf.TransformBroadcaster()
        #	br.sendTransform((0, 0, 0), (0, 0, 0, 0), rospy.Time.now(), 'base_link', 'Odom')
        rospy.spin()


if __name__ == "__main__":
    time.sleep(5)
    imutrf = imuTransformer()
    imutrf.run()
