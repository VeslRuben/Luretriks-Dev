#!/usr/bin/env python
import time

import rospy
import json
import math
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String


# noinspection DuplicatedCode
class Calibrate:
    def __init__(self):
        rospy.init_node("Calibrate", anonymous=True)
        self.filter_node_name = rospy.get_param("filter_node_name", "filter")
        self.odom_to_base_node_name = rospy.get_param("odom_to_base_node_name", "odom_to_base")
        self.sampels = rospy.get_param("sampels", 1000)
        self.imu_raw_topic = rospy.get_param("imu_raw_topic", 'imu/sensor_raw')
        self.mag_raw_topic = rospy.get_param("mag_raw_topic", 'mag/sensor_raw')
        self.imu_filtered_topic = rospy.get_param("imu_filtered_topic", 'imu/data_raw')
        self.mag_filtered_topic = rospy.get_param("mag_filtered_topic", 'imu/mag')
        self.orientation_topic = rospy.get_param("comp_filter_topic", "imu/data")

        self.replys = {self.filter_node_name: "", self.odom_to_base_node_name: ""}

        rospy.Subscriber("calibrate/reply", String, self.handle_reply)
        self.last_message = ""

        self.calibrate_publisher = rospy.Publisher('calibrate', String, queue_size=10)

        self.sensor_raw = None
        self.sensor_filtered = None
        self.mag_sensor_filtered = None
        self.sensor_g = None

        ##### FLAGS #######
        self.sampling = False

        self.reseseting = False

        self.sample_raw = False
        self.gyro_acc_raw_sampling_done = False

        self.sample_filtered = False
        self.gyro_acc_filtered_sampling_done = False
        self.mag_filtered_sampling_done = False

        self.sample_papa_g = False
        self.orientation_sampling_done = False

        #####################

        self.linear_acceleration_raw_list = [[], [], []]
        self.angular_velocity_raw_list = [[], [], []]

        self.linear_acceleration_list = [[], [], []]
        self.angular_velocity_list = [[], [], []]
        self.mag_vector_list = [[], [], []]

        self.orientation_vector_list = []

    def reset(self):
        """
        sends a message to other nodes to reset calibration files
        :return: None
        """
        self.calibrate_publisher.publish("reset")

    def handle_reply(self, msg):
        """
        registers replies from other nodes
        :param msg: standard_msgs/String
        :return:
        """
        msg = msg.data
        sender, message = msg.split("::")
        try:
            self.replys[sender] = message
        except KeyError as e:
            print(e)

    def handle_gyro_acc_raw(self, msg):
        """
        appends the latest imu message to a list
        :param msg: sensor_msgs/Imu
        :return: None
        """
        if len(self.linear_acceleration_raw_list[0]) < self.sampels:
            l = msg.linear_acceleration
            a = msg.angular_velocity

            self.linear_acceleration_raw_list[0].append(l.x)
            self.linear_acceleration_raw_list[1].append(l.y)
            self.linear_acceleration_raw_list[2].append(l.z)

            self.angular_velocity_raw_list[0].append(a.x)
            self.angular_velocity_raw_list[1].append(a.y)
            self.angular_velocity_raw_list[2].append(a.z)
        else:
            self.gyro_acc_raw_sampling_done = True

    def handle_gyro_acc_filtered(self, msg):
        """
        saves the latest filtered imu message to a list
        :param msg: sensor_msgs/Imu
        :return: None
        """
        if len(self.linear_acceleration_list[0]) < self.sampels:
            l = msg.linear_acceleration
            a = msg.angular_velocity

            self.linear_acceleration_list[0].append(l.x)
            self.linear_acceleration_list[1].append(l.y)
            self.linear_acceleration_list[2].append(l.z)

            self.angular_velocity_list[0].append(a.x)
            self.angular_velocity_list[1].append(a.y)
            self.angular_velocity_list[2].append(a.z)
        else:
            self.gyro_acc_filtered_sampling_done = True

    def handle_mag_filtered(self, msg):
        """
        saves the latest magnetic field message to a list
        :param msg: sensor_msgs/MagneticField
        :return:
        """
        if len(self.mag_vector_list[0]) < self.sampels:
            self.mag_vector_list[0].append(msg.magnetic_field.x)
            self.mag_vector_list[1].append(msg.magnetic_field.y)
            self.mag_vector_list[2].append(msg.magnetic_field.z)
        else:
            self.mag_filtered_sampling_done = True

    def handle_g(self, msg):
        if len(self.orientation_vector_list) < self.sampels:
            self.orientation_vector_list.append(msg.orientation)

        else:
            self.orientation_sampling_done = True

    def calculate_offset(self, data0):
        """
        calculates the static offset in the dataset
        :param data0: dataset
        :return: offset of dataset
        """
        data = data0[:]
        x_mean = np.mean(data[0])
        y_mean = np.mean(data[1])
        z_mean = np.mean(data[2])
        return [x_mean, y_mean, z_mean]

    def calculate_cv(self, data0):
        """
        calculates the covariance of the dataset
        :param data0: dataset
        :return: covariance
        """
        data = data0[:]
        mean_x = np.mean(data[0])
        mean_y = np.mean(data[1])
        mean_z = np.mean(data[2])

        data[0] = data[0] - mean_x
        data[1] = data[1] - mean_y
        data[2] = data[2] - mean_z

        b = np.array(data)
        cv = b.dot(b.T) / self.sampels
        return cv.tolist()

    def createRotationMatrix(self, orientation):
        """
        Creates a rotation matrix from orientation in quaternion
        :param orientation: orientation in quaternions [x, y, z, w]
        :return: Rotation matrix
        """
        # Extract information from orientation
        qx, qy, qz, qw = [orientation.w, orientation.x, orientation.y, orientation.z]

        rotation_matrix = np.array([
            [2 * (qx ** 2) - 1 + 2 * (qy ** 2), 2 * qy * qz + 2 * qx * qw, 2 * qy * qw - 2 * qx * qz],
            [2 * qy * qz - 2 * qx * qw, 2 * (qx ** 2) - 1 + 2 * (qz ** 2), 2 * qz * qw + 2 * qx * qy],
            [2 * qy * qw + 2 * qx * qz, 2 * qz * qw - 2 * qx * qy, 2 * (qx ** 2) - 1 + 2 * (qw ** 2)]
        ]).reshape(3, 3)

        return rotation_matrix

    def return_NED_matrix(self, orientation, raw_acc):
        """
        Returns a NED-matrix
        :param orientation: orientation in quaternions [x, y, z, w]
        :param raw_acc: raw acceleration data from accelerometer
        :return: NED-Matrix
        """
        rot_matrix = self.createRotationMatrix(orientation)

        rot_matrix_inv = np.linalg.inv(rot_matrix)

        acc_NED = rot_matrix_inv.dot(raw_acc)

        return acc_NED

    def createOffset(self, offsetList):
        """
        Creates an offset by taking the average of values from a list
        :param offsetList: list of NED-values
        :return: NED_Offset
        """
        if offsetList is not None:
            afN = []
            afE = []
            afD = []

            for n, e, d in offsetList:
                afN.append(n)
                afE.append(e)
                afD.append(d)

            fN_avg = sum(afN) / len(afN)
            fE_avg = sum(afE) / len(afE)
            fD_avg = sum(afD) / len(afD)

            NED_offset = [fN_avg, fE_avg, fD_avg]

            return NED_offset

    def calibrate_raw(self):
        """
        Calibrates and sends an static offset for the raw Imu data
        :return: boolean if succeeded
        """
        if self.gyro_acc_raw_sampling_done:
            rospy.loginfo("Finished sampling, calculating offsets...")
            self.sampling = False
            self.sensor_raw.unregister()
            raw_gyro_offset = self.calculate_offset(self.angular_velocity_raw_list)
            raw_gyro_offset = json.dumps({"gyro_offset": raw_gyro_offset})
            message = "{}::".format(self.filter_node_name) + raw_gyro_offset
            rospy.loginfo("Sending new config...")
            self.calibrate_publisher.publish(message)
            sucsses = self.reply([self.filter_node_name])
            if sucsses:
                rospy.loginfo("{} has new config for gyro offset".format(self.filter_node_name))
                self.sample_raw = False
                self.sample_filtered = True
                time.sleep(3)
            else:
                rospy.logerr("{} cant update its config, got error as reply".format(self.filter_node_name))
                return False
        elif not self.sampling:
            self.sampling = True
            self.sensor_raw = rospy.Subscriber(self.imu_raw_topic, Imu, self.handle_gyro_acc_raw)
            rospy.loginfo("Calibrating raw sensor data...")
        return True

    def calibrate_filtered(self):
        """
        calibrates and sends a static offset for filtered imu data
        :return: boolean if succeeded
        """
        if self.gyro_acc_filtered_sampling_done and self.mag_filtered_sampling_done:
            rospy.loginfo("Finished sampling, calculating covariances...")
            self.sampling = False
            self.sensor_filtered.unregister()
            self.mag_sensor_filtered.unregister()
            cv_mag = self.calculate_cv(self.mag_vector_list)
            cv_angular = self.calculate_cv(self.angular_velocity_list)
            cv_linear = self.calculate_cv(self.linear_acceleration_list)
            data = {"cv_mag": cv_mag, "cv_angular": cv_angular, "cv_linear": cv_linear}
            message = "{}::".format(self.filter_node_name) + json.dumps(data)
            rospy.loginfo("Sending new config...")
            self.calibrate_publisher.publish(message)
            sucsses = self.reply([self.filter_node_name])
            if sucsses:
                rospy.loginfo("{} has new config for covariance".format(self.filter_node_name))
                self.sample_filtered = False
                self.sample_papa_g = True
                self.gyro_acc_raw_sampling_done = False
                time.sleep(3)
            else:
                rospy.logerr("{} cant update its config, got error as reply".format(self.filter_node_name))
                return False
        elif not self.sampling:
            self.sampling = True
            self.sensor_filtered = rospy.Subscriber(self.imu_filtered_topic, Imu, self.handle_gyro_acc_filtered)
            self.mag_sensor_filtered = rospy.Subscriber(self.mag_filtered_topic, MagneticField,
                                                        self.handle_mag_filtered)
            rospy.loginfo("Calibrating filtered sensor covariance...")
        return True

    def calibrate_papa_g(self):
        """
        calibrate and sends a gravity offset vector
        :return: boolean if succeeded
        """
        if self.gyro_acc_raw_sampling_done and self.sample_papa_g:
            rospy.loginfo("Finished sampling, calculating papa G, aka big G, aka gravity offset...")
            self.sampling = False
            self.sensor_raw.unregister()
            self.sensor_g.unregister()
            acc_list = []
            for x, y, z in zip(self.linear_acceleration_raw_list[0], self.linear_acceleration_raw_list[1],
                               self.linear_acceleration_raw_list[2]):
                acc_list.append([x, y, z])
            ned_matrix = []
            for orientation, acc in zip(self.orientation_vector_list, acc_list):
                ned_matrix.append(self.return_NED_matrix(orientation, acc))
            ned_offest = self.createOffset(ned_matrix)
            g_offset = json.dumps({"g_offset": ned_offest})
            message = "{}::".format(self.odom_to_base_node_name) + g_offset
            rospy.loginfo("Sending new config...")
            self.calibrate_publisher.publish(message)
            sucsses = self.reply([self.odom_to_base_node_name])
            if sucsses:
                rospy.loginfo("{} has new config for covariance".format(self.odom_to_base_node_name))
                self.sample_papa_g = False
                self.sample_raw = False
                rospy.loginfo("Calibration complete!")
                rospy.signal_shutdown("Calibration complete!")
            else:
                rospy.logerr("{} cant update its config, got error as reply".format(self.odom_to_base_node_name))
                return False
        elif not self.sampling:
            self.sampling = True
            self.linear_acceleration_raw_list = [[], [], []]
            self.gyro_acc_raw_sampling_done = False
            self.sensor_raw = rospy.Subscriber(self.imu_raw_topic, Imu, self.handle_gyro_acc_raw)
            self.sensor_g = rospy.Subscriber(self.orientation_topic, Imu, self.handle_g)
            rospy.loginfo("Calibrating papa G, aka big G, aka gravity offset...")
        return True

    def reply(self, fromm):
        """
        checks which nodes has replied
        :param fromm: nodes that should reply
        :return: true if replied ok
        """
        reply = True
        for r in fromm:
            rospy.loginfo("Waiting for reply from {}".format(r))
            done = False
            start = time.time()
            while not rospy.is_shutdown() and not done and not (time.time() - start) > 5:
                if self.replys[r]:
                    done = True
            if self.replys[r] == "ok":
                rospy.loginfo("{} has replied OK".format(r))
                self.replys[r] = ""
                reply = reply and True
            else:
                reply = reply and False
                rospy.logwarn("{} has replied ERROR".format(r))
        return reply

    def run(self):
        """
        Runs a calibration of the IMU
        :return: None
        """
        time.sleep(3)
        self.reset()
        rospy.loginfo("Resetting all nodes")
        self.reseseting = True
        while not rospy.is_shutdown():
            if self.reseseting:
                sucsess = self.reply(self.replys.keys())
                if sucsess:
                    rospy.loginfo("All nodes are reset")
                    self.reseseting = False
                    self.sample_raw = True
                else:
                    rospy.logwarn("Reset failed, retrying")

            if self.sample_raw:
                response = self.calibrate_raw()
                if not response:
                    break

            if self.sample_filtered:
                response = self.calibrate_filtered()
                if not response:
                    break

            if self.sample_papa_g:
                response = self.calibrate_papa_g()
                if not response:
                    break


if __name__ == "__main__":
    c = Calibrate()
    time.sleep(5)
    c.run()
