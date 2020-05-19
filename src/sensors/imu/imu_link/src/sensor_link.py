#!/usr/bin/env python
import time
from multiprocessing import TimeoutError

import rospy
from sensor_msgs.msg import Imu, MagneticField
import serial
import numpy as np


class ImuSensor:
    """
    A class for communicating with an inertial measurement unit over serial
    """

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.last_time = time.time()
        self.timeout = 5

    def get_imu_data(self, expected_params=19):
        """
        Receives IMU-data from the Arduino and extracts and separates it
        :param expected_params: amount of datapoints
        :return: data from imu1, imu2, imu3, timestamp
        """
        while self.serial_port.in_waiting < 1 and not rospy.is_shutdown():
            pass
        items = []
        while len(items) is not expected_params and not rospy.is_shutdown():
            data = list(bytearray(self.serial_port.readline()))
            string = ''.join(chr(i) for i in data)
            items = string.strip().split(',')
        raw = [float(s) for s in items]

        imu1 = {"acc": raw[0:3], "gyro": raw[3:6], "mag": raw[6:9]}
        imu2 = {"acc": raw[9:12], "gyro": raw[12:15]}
        imu3 = {"acc": raw[15:18]}
        t = raw[18]
        return imu1, imu2, imu3, t

    def get_imu_data_single(self, expected_params=10):
        """
        Receives IMU-data from the Arduino and extracts it
        :param expected_params: amount of data-points
        :return: accelerometer_data, gyroscope_data, magnetometer_data, timestamp
        """
        self.last_time = time.time()
        while self.serial_port.in_waiting < 1 and not rospy.is_shutdown():
            if time.time() - self.last_time > self.timeout:
                raise TimeoutError("Arduino timed out")
        items = []
        while len(items) is not expected_params and not rospy.is_shutdown():
            if time.time() - self.last_time > self.timeout:
                raise TimeoutError("Arduino timed out")

            data = list(bytearray(self.serial_port.readline()))
            string = ''.join(chr(i) for i in data)
            items = string.strip().split(',')
        raw = [float(s) for s in items]

        acc_data = raw[0:3]
        gyro_data = raw[3:6]
        mag_data = raw[6:9]
        t = raw[9]

        acc_data = [acc_data[1], -acc_data[0], acc_data[2]]
        gyro_data = [gyro_data[1], -gyro_data[0], gyro_data[2]]
        mag_data = [mag_data[1], -mag_data[0], mag_data[2]]
        return acc_data, gyro_data, mag_data, t


def format_raw(acc_data, gyro_data):
    """
    Formats the data for publishing
    :param acc_data: accelerometer_data
    :param gyro_data: gyroscope_data
    :return: formatted message
    """
    msg = Imu()
    msg.header.frame_id = 'sensor_raw'
    msg.header.stamp = rospy.Time.now()
    msg.angular_velocity.x = gyro_data[0]
    msg.angular_velocity.y = gyro_data[1]
    msg.angular_velocity.z = gyro_data[2]
    msg.linear_acceleration.x = acc_data[0]
    msg.linear_acceleration.y = acc_data[1]
    msg.linear_acceleration.z = acc_data[2]
    return msg


def format_mag(mag_data):
    """
    Formats magnetometer data for publishing
    :param mag_data: magnetometer_data
    :return: formatted message
    """
    msg = MagneticField()
    msg.header.frame_id = 'sensor_mag'
    msg.header.stamp = rospy.Time.now()
    msg.magnetic_field.x = mag_data[0]
    msg.magnetic_field.y = mag_data[1]
    msg.magnetic_field.z = mag_data[2]
    return msg


window_x = []
window_y = []
window_z = []


def running_average_data(imu1, imu2, imu3):
    """
    Takes running average of data from three different IMU's
    :param imu1: acc, gyro, mag
    :param imu2: acc, gyro
    :param imu3: acc
    :return: average_accel, average_gyro, average_mag
    """
    avg_acc = [sum(x) / 3 for x in zip(imu1["acc"], imu2["acc"], imu3["acc"])]
    avg_gyro = [sum(x) / 1 for x in zip(imu1["gyro"])]
    avg_mag = [sum(x) / 1 for x in zip(imu1["mag"])]

    window_x.append(avg_acc[0])
    window_y.append(avg_acc[1])
    window_z.append(avg_acc[2])
    if len(window_x) > 9:
        window_x.pop(0)
        window_y.pop(0)
        window_z.pop(0)

    avg_x = float(sum(window_x)) / float(len(window_x))
    avg_y = float(sum(window_y)) / float(len(window_y))
    avg_z = float(sum(window_z)) / float(len(window_z))

    avg_acc_filt = [avg_x, avg_y, avg_z]

    return avg_acc_filt, avg_gyro, avg_mag


def average_data(imu1, imu2, imu3):
    """
    Takes average of data from three IMU's
    :param imu1: acc, gyro, mag
    :param imu2: acc, gyro
    :param imu3: acc
    :return: averaged acc_data, gyro_data, mag_data
    """
    avg_acc = [sum(x) / 3 for x in zip(imu1["acc"], imu2["acc"], imu3["acc"])]
    avg_gyro = [sum(x) / 1 for x in zip(imu1["gyro"])]
    avg_mag = [sum(x) / 1 for x in zip(imu1["mag"])]

    return avg_acc, avg_gyro, avg_mag


def median_data(imu1, imu2, imu3):
    """
    Takes the median data between three IMU's
    :param imu1: acc, gyro, mag
    :param imu2: acc, gyro
    :param imu3: acc
    :return: median acc_data, gyro_data, mag_data
    """
    med_acc = [np.median([imu1["acc"][0], imu2["acc"][0], imu3["acc"][0]]),
               np.median([imu1["acc"][1], imu2["acc"][1], imu3["acc"][1]]),
               np.median([imu1["acc"][2], imu2["acc"][2], imu3["acc"][2]])]
    med_gyro = imu1["gyro"]
    med_mag = imu1["mag"]

    return med_acc, med_gyro, med_mag


def main():
    """
    Runs the program. Boolean in start to say if data from three IMU's or just a single IMU
    :return: None
    """
    tripleImu = False
    singleImu = True
    rospy.init_node('sensor_publisher_node', anonymous=True)
    port = rospy.get_param("serial_port", "/dev/ttyUSB0")
    baud_rate = rospy.get_param("serial_baud_rate", 115200)

    while not rospy.is_shutdown():
        try:
            with serial.Serial(port, baud_rate, timeout=0.5) as serial_port:
                sensor = ImuSensor(serial_port)
                raw_publisher = rospy.Publisher('imu/sensor_raw', Imu, queue_size=10)
                mag_publisher = rospy.Publisher('imu/sensor_mag', MagneticField, queue_size=10)
                # rate = rospy.Rate(50)
                while not rospy.is_shutdown():
                    if tripleImu:
                        imu1, imu2, imu3, t = sensor.get_imu_data()
                        acc_data, gyro_data, mag_data = running_average_data(imu1, imu2, imu3)
                        raw_publisher.publish(format_raw(acc_data, gyro_data))
                        mag_publisher.publish(format_mag(mag_data))
                    if singleImu:
                        acc_data, gyro_data, mag_data, t = sensor.get_imu_data_single()
                        raw_publisher.publish(format_raw(acc_data, gyro_data))
                        mag_publisher.publish(format_mag(mag_data))

        except TimeoutError as e:
            rospy.logerr(e.message)
        except serial.serialutil.SerialException as e:
            rospy.logfatal(e)
            exit(126)


if __name__ == '__main__':
    '''
    Parameters:
    raw_frame_name (default 'raw_imu_frame')
    mag_frame_name (default 'mag_imu_frame')
    serial_port (default '/dev/ttyUSB0')
    serial_baud_rate (default 115200)
    '''
    main()
