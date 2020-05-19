#!/usr/bin/env python

import rospy
import json
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String


class Filter:

    def __init__(self, b, a=None, k=1):
        """
        Creates a filter object
        :param b: numerator coefficients of the transfer function (coeffs of X)
        :param a: denominator coefficients of the transfer function (coeffs of Y)
        :param k: output gain (default 1)
        """
        if not a:
            a = [1]
        self.b = b
        self.a = a
        self.k = k
        self.input = [0] * len(b)
        self.output = [0] * (len(a) - 1)

    def filter_value(self, x_new):
        """
        Passes a single value through the filter
        TODO: Find a better way to do the calculation than to use pop and insert for self.a[0]
        :param x_new: the value to be passed through the filter
        :return: the filter's output value
        """
        self.input = self._shift_list(self.input, x_new)
        a0 = self.a.pop(0)
        y_new = a0 * (
                sum([b * x for b, x in zip(self.b, self.input)])
                - sum([a * y for a, y in zip(self.a, self.output)])
        )
        self.a.insert(0, a0)
        if len(self.output) > 0:
            self.output = self._shift_list(self.output, y_new)
        return y_new

    def filter_list(self, xn):
        """
        Passes a list of values through the filter
        :param xn: the input vector
        :return: the filter output vector
        """
        y = []
        for x in xn:
            y.append(self.filter_value(x))
        return y

    def clear(self):
        """
        Clear the filter's stored input and output list
        :return: None
        """
        self.input = [0] * len(self.input)
        self.output = [0] * len(self.output)

    def _shift_list(self, lst, val):
        """
        Removes the last value in a list an puts in a value in the first position
        :param lst: list
        :param val: value
        :return: shifted list
        """
        lst.pop()
        lst.insert(0, val)
        return lst


class MultiChannelFilter:

    def __init__(self, channels, b, a=None, k=1):
        self.channels = channels
        self.filters = []
        for i in range(self.channels):
            self.filters.append(Filter(b, a=a, k=k))

    def filter_values(self, values):
        """
        Filters values
        :param values: values to filter
        :return: filtered values
        """
        filtered_values = [0] * self.channels
        if len(values) == self.channels:
            for i in range(self.channels):
                filtered_values[i] = self.filters[i].filter_value(values[i])
        else:
            filtered_values = None
        return filtered_values


class SensorFilter:

    def __init__(self, acc_num, gyro_num, mag_num,
                 acc_offset=None, gyro_offset=None, mag_offset=None,
                 acc_cv=None, gyro_cv=None, mag_cv=None):
        if mag_cv is None:
            mag_cv = [0] * 9
        if gyro_cv is None:
            gyro_cv = [0] * 9
        if acc_cv is None:
            acc_cv = [0] * 9
        if mag_offset is None:
            mag_offset = [0, 0, 0]
        if gyro_offset is None:
            gyro_offset = [0, 0, 0]
        if acc_offset is None:
            acc_offset = [0, 0, 0]

        self.mag_cv = mag_cv[0] + mag_cv[1] + mag_cv[2]
        self.gyro_cv = gyro_cv[0] + gyro_cv[1] + gyro_cv[2]
        self.acc_cv = acc_cv[0] + acc_cv[1] + acc_cv[2]
        self.mag_offset = mag_offset
        self.gyro_offset = gyro_offset
        self.acc_offset = acc_offset
        self.acc_filter = MultiChannelFilter(3, acc_num)
        self.gyro_filter = MultiChannelFilter(3, gyro_num)
        self.mag_filter = MultiChannelFilter(3, mag_num)
        self.raw_publisher = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        self.mag_publisher = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        self.calibrate_reply_publisher = rospy.Publisher('calibrate/reply', String, queue_size=10)

    def handle_sensor_raw(self, msg):
        """
        Filters the raw imu data and publishes the filtered data
        :param msg: imu-data
        :return: None
        """
        gyro_raw = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        acc_raw = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [g - o for g, o in zip(gyro_raw, self.gyro_offset)]
        acc = [a - o for a, o in zip(acc_raw, self.acc_offset)]
        gyro_data = self.gyro_filter.filter_values(gyro)
        acc_data = self.acc_filter.filter_values(acc)
        new_msg = self.format_raw(acc_data, gyro_data)
        self.raw_publisher.publish(new_msg)

    def handle_sensor_mag(self, msg):
        """
        Filters the raw magnetometer data and publishes the filtered data
        :param msg: Magnetometer-data
        :return: None
        """
        mag_raw = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
        mag = [m - o for m, o in zip(mag_raw, self.mag_offset)]
        mag_data = self.mag_filter.filter_values(mag)
        new_msg = self.format_mag(mag_data)
        self.mag_publisher.publish(new_msg)

    def handle_config(self, msg):
        """
        Receives and updates the config from the calibrate-script
        :param msg: std_msgs/String
        :return: None
        """
        name = rospy.get_name()[1:]
        msg = msg.data
        config_file_name = rospy.get_param('config_file_filter')
        reset_file_name = rospy.get_param('reset_file_filter')

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

            self.gyro_offset = config_data['gyro_offset']
            mag_cv = config_data['cv_mag']
            gyro_cv = config_data['cv_angular']
            acc_cv = config_data['cv_linear']
            self.mag_cv = mag_cv[0] + mag_cv[1] + mag_cv[2]
            self.gyro_cv = gyro_cv[0] + gyro_cv[1] + gyro_cv[2]
            self.acc_cv = acc_cv[0] + acc_cv[1] + acc_cv[2]

    def format_raw(self, acc_data, gyro_data):
        """
        Formats ROS IMU-message
        :param acc_data: accelerometer data
        :param gyro_data: gyroscope data
        :return: formated message
        """
        msg = Imu()
        msg.header.frame_id = 'imu_raw'
        msg.header.stamp = rospy.Time.now()
        msg.angular_velocity.x = gyro_data[0]
        msg.angular_velocity.y = gyro_data[1]
        msg.angular_velocity.z = gyro_data[2]
        msg.angular_velocity_covariance = self.gyro_cv
        msg.linear_acceleration.x = acc_data[0]
        msg.linear_acceleration.y = acc_data[1]
        msg.linear_acceleration.z = acc_data[2]
        msg.linear_acceleration_covariance = self.acc_cv
        return msg

    def format_mag(self, mag_data):
        """
        Formats magnetometer-message
        :param mag_data: magnetometer data
        :return: formated message
        """
        msg = MagneticField()
        msg.header.frame_id = 'imu_mag'
        msg.header.stamp = rospy.Time.now()
        msg.magnetic_field.x = mag_data[0]
        msg.magnetic_field.y = mag_data[1]
        msg.magnetic_field.z = mag_data[2]
        msg.magnetic_field_covariance = self.mag_cv
        return msg


def main():
    """
    Runs the filter
    :return:
    """
    rospy.init_node('imu_publisher_node', anonymous=True)
    filter_file = rospy.get_param("filter_file")
    calibration_file = rospy.get_param("config_file_filter")
    with open(filter_file) as json_file:
        filter_config = json.load(json_file)
    acc_filter = filter_config['accelerometer']
    gyro_filter = filter_config['gyroscope']
    mag_filter = filter_config['magnetometer']

    with open(calibration_file) as cal_file:
        calibration_config = json.load(cal_file)
    sensor_filter = SensorFilter(
        acc_filter['num'],
        gyro_filter['num'],
        mag_filter['num'],
        gyro_offset=calibration_config['gyro_offset'],
        acc_cv=calibration_config['cv_linear'],
        gyro_cv=calibration_config['cv_angular'],
        mag_cv=calibration_config['cv_mag']
    )
    rospy.Subscriber('imu/sensor_raw', Imu, sensor_filter.handle_sensor_raw)
    rospy.Subscriber('imu/sensor_mag', MagneticField, sensor_filter.handle_sensor_mag)
    rospy.Subscriber("calibrate", String, sensor_filter.handle_config)
    rospy.spin()


if __name__ == '__main__':
    main()
