#!/usr/bin/env python
import datetime

import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
import json

file_name = "/home/ruben/luretriks-dev/logs/log_{}.txt".format(str(datetime.datetime.now()))

saving = False

data_defalt = {"imu/data": [], "imu/raw": [], "mag": [], "Pose2D": [], "motors": [], "slam_pose": [], "kalman": [],
               "cmd_vel": [], "motor_output": [], "odom": [], "amcl_pose": [], "globla_path": []}
data = dict(data_defalt)


def save():
    """
    saves logged data to disk
    :return:
    """
    print "saving"
    global saving, data, data_defalt
    saving = True
    data_to_save = data.copy()
    data = {"imu/data": [], "imu/raw": [], "mag": [], "Pose2D": [], "motors": [], "slam_pose": [], "kalman": [],
            "cmd_vel": [], "motor_output": [], "odom": [], "amcl_pose": [], "globla_path": []}
    saving = False
    with open(file_name, "a") as f:
        f.write(json.dumps(data_to_save) + "\n")


def format_time(stamp):
    """
    formats time to seconds
    :param stamp: rospy.time
    :return: time i seconds
    """
    return stamp.secs + stamp.nsecs * (10 ** -9)


def imuCallback(imuData):
    """
    formats and saves IMU data from the comp-filter to memory.
    :param imuData: sensor_msgs/Imu
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = imuData.header.stamp
        collected_data["time"] = format_time(stamp)

        orientation = imuData.orientation
        collected_data["orientation"] = [orientation.x, orientation.y, orientation.z, orientation.w]

        angular_velocity = imuData.angular_velocity
        collected_data["angular_velocity"] = [angular_velocity.x, angular_velocity.y, angular_velocity.z]

        linear_acceleration = imuData.linear_acceleration
        collected_data["linear_acceleration"] = [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]

        data["imu/data"].append(collected_data)


def scan_match_calback(msg):
    """
    formats and saves position from scan matcher to memory
    :param msg: geometry_msgs/Pose_2d
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        collected_data["pose"] = [msg.x, msg.y, msg.theta]
        stamp = rospy.Time.now()
        collected_data["time"] = format_time(stamp)

        data["Pose2D"].append(collected_data)


def handle_slam_pose(msg):
    """
    formats and saves pose from slam to memory.
    :param msg: geometry_msgs/PoseStamped
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = msg.header.stamp
        collected_data["time"] = format_time(stamp)
        pose = msg.pose
        collected_data["covariance"] = pose.covariance
        pose = pose.pose
        collected_data["position"] = [pose.position.x, pose.position.y, pose.position.z]
        collected_data["orientation"] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        data["slam_pose"].append(collected_data)


def handle_sensor_mag(msg):
    """
    formats and saves magnetometer data to memory.
    :param msg: sensor_msgs/MagneticField
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = msg.header.stamp
        collected_data["time"] = format_time(stamp)
        magnetic_field = msg.magnetic_field
        collected_data["magnetic_field"] = [magnetic_field.x, magnetic_field.y, magnetic_field.z]

        data["mag"].append(collected_data)


def handle_sensor_raw(msg):
    """
    formats and saves raw Imu data to memory
    :param msg: sensor_msgs/IMU
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = msg.header.stamp
        collected_data["time"] = format_time(stamp)

        orientation = msg.orientation
        collected_data["orientation"] = [orientation.x, orientation.y, orientation.z, orientation.w]

        angular_velocity = msg.angular_velocity
        collected_data["angular_velocity"] = [angular_velocity.x, angular_velocity.y, angular_velocity.z]

        linear_acceleration = msg.linear_acceleration
        collected_data["linear_acceleration"] = [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]

        data["imu/raw"].append(collected_data)


def motor_calback(msg):
    """
    formats and saves motor speed commands to memory
    :param msg: geometry_msgs/Twist
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        forward_left_motor = msg.linear.x
        forward_right_motor = msg.linear.x
        right_turning_motor = msg.linear.y
        left_turning_motor = msg.linear.z
        stamp = rospy.Time.now()
        collected_data["time"] = format_time(stamp)

        collected_data["fl"] = forward_left_motor
        collected_data["fr"] = forward_right_motor
        collected_data["rt"] = right_turning_motor
        collected_data["lt"] = left_turning_motor

        data["motors"].append(collected_data)


def handle_kalman(msg):
    """
    formats and saves velocity data to memory
    :param msg: geometry_msgs/Twist
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = rospy.Time.now()
        collected_data["time"] = format_time(stamp)
        x = msg.linear.x
        y = msg.linear.y
        n = msg.angular.z
        collected_data["x"] = x
        collected_data["y"] = y
        collected_data["n"] = n
        data["kalman"].append(collected_data)


def handle_cmd_vel(msg):
    """
    formats and saves desired velocity to memory
    :param msg: geometry_msgs/Twist
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = rospy.Time.now()
        collected_data["time"] = format_time(stamp)
        x = msg.linear.x
        y = msg.linear.y
        n = msg.angular.z
        collected_data["vx"] = x
        collected_data["vy"] = y
        collected_data["vr"] = n
        data["cmd_vel"].append(collected_data)


def handle_motor_out(msg):
    """
      formats and saves motor speed commands to memory
      :param msg: geometry_msgs/Twist
      :return: None
      """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = rospy.Time.now()
        collected_data["time"] = format_time(stamp)
        x = msg.linear.x
        y = msg.linear.y
        n = msg.angular.z
        collected_data["vx"] = x
        collected_data["vy"] = y
        collected_data["vr"] = n
        data["motor_output"].append(collected_data)


def handle_odom(msg):
    """
    formats and saves odometry data to memory
    :param msg: nav_msgs/Odometry
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = msg.header.stamp
        collected_data["time"] = format_time(stamp)
        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y
        n = msg.twist.twist.angular.z
        collected_data["vx"] = x
        collected_data["vy"] = y
        collected_data["vr"] = n

        pose = msg.pose.pose
        collected_data["position"] = [pose.position.x, pose.position.y, pose.position.z]
        collected_data["orientation"] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        data["odom"].append(collected_data)


def handle_amcl(msg):
    """
    formats and saves the pose from AMCL to memory
    :param msg: geometry_msgs/PoseWitCovarianceStamped
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = msg.header.stamp
        collected_data["time"] = format_time(stamp)
        pose = msg.pose
        collected_data["covariance"] = pose.covariance
        pose = pose.pose
        collected_data["position"] = [pose.position.x, pose.position.y, pose.position.z]
        collected_data["orientation"] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        data["amcl_pose"].append(collected_data)


def handle_path(msg):
    """
    formats and saves the global path to memory
    :param msg: geometry_msgs/Path
    :return: None
    """
    global data, saving
    if not saving:
        collected_data = {}
        stamp = msg.header.stamp
        collected_data["time"] = format_time(stamp)
        path = []
        for p in msg.poses:
            pos = p.pose.position
            ori = p.pose.orientation
            d = {"position": [pos.x, pos.y, pos.z], "orientation": [ori.x, ori.y, ori.z, ori.w]}
            path.append(d)
        collected_data["path"] = path
        data["globla_path"].append(collected_data)


def main():
    global last_time, data
    with open(file_name, "w+") as f:
        f.write("")
    rospy.init_node("logger", anonymous=True)
    rospy.Subscriber("imu/data", Imu, imuCallback)
    rospy.Subscriber("pose2D", Pose2D, scan_match_calback)
    rospy.Subscriber("roboclaw", Twist, motor_calback)
    rospy.Subscriber("poseupdate", PoseWithCovarianceStamped, handle_slam_pose)
    rospy.Subscriber("kalman", Twist, handle_kalman)
    rospy.Subscriber("cmd_vel", Twist, handle_cmd_vel)
    rospy.Subscriber("motor_output", Twist, handle_motor_out)
    rospy.Subscriber("odom", Odometry, handle_odom)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, handle_amcl)
    rospy.Subscriber("move_base/TebLocalPlannerROS/global_plan", Path, handle_path)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        save()
        rate.sleep()
    save()


if __name__ == "__main__":
    main()
