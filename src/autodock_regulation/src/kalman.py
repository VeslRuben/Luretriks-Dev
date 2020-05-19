#!/usr/bin/env python
import csv
from filterpy.kalman import KalmanFilter
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from math import cos, sin
import numpy as np


class Jallman:
    def __init__(self):

        with open('/home/ruben/luretriks-dev/src/autodock_regulation/config/force_curve.csv') as f:
            reader = csv.reader(f)
            force_curve_list = list(reader)
            force_curve_list = [[float(x), float(y)] for x, y in force_curve_list]
            self.force_curve = {}
            for x in force_curve_list:
                self.force_curve[x[0]] = x[1]

        with open('/home/ruben/luretriks-dev/src/autodock_regulation/config/physical_params.csv') as f:
            reader = csv.reader(f)
            self.physical_params = list(reader)
            self.physical_params = [float(x) for x in self.physical_params[0]]

        with open('/home/ruben/luretriks-dev/src/autodock_regulation/config/system.csv') as f:
            reader = csv.reader(f)
            system = list(reader)
            system = [[float(y) for y in x] for x in system]
            self.A = np.array([system[0], system[1], system[2]])
            self.B = np.array([system[3], system[4], system[5]])
            self.C = np.array([system[6], system[7], system[8]])

        self.pub = rospy.Publisher("kalman", Twist, queue_size=10)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

        self.last_pose = [[0, 0, 0], [0, 0, 0]]
        self.vel_samples = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.vel_i = 0
        self.last_ang = 0
        self.last_pos_m = np.array([[0], [0], [1]])
        self.lastime = rospy.Time.now()
        self.first = True

        self.last_vel = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.last_poses = [[[0, 0, 0], [0, 0, 0, 0]], [[0, 0, 0], [0, 0, 0, 0]], [[0, 0, 0], [0, 0, 0, 0]],
                           [[0, 0, 0], [0, 0, 0, 0]], [[0, 0, 0], [0, 0, 0, 0]]]

        self.u = np.array([[0], [0], [0]])
        self.filter = KalmanFilter(dim_x=3, dim_z=3, dim_u=3)
        self.filter.x = np.array([[0], [0], [0]])
        self.filter.F = self.A
        self.filter.H = self.C
        self.filter.P *= 1.
        self.filter.R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * 100
        self.Q = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * 20.
        self.filter.Q = self.Q

    def handel_pose(self, msg):
        """
        Takes in a pose message and transforms it in to body coordinates then adds that to a global list
        :param msg: geometry_msgs/PoseStamped
        :return: None
        """
        position = msg.pose.position
        orientation = msg.pose.orientation
        _, _, ang = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w))
        ang2 = ang
        ang3 = self.last_ang
        pos = np.array([[position.x], [position.y], [1]])
        rot_m0 = np.array([[cos(-self.last_ang), -sin(-self.last_ang), 0],
                           [sin(-self.last_ang), cos(-self.last_ang), 0],
                           [0, 0, 1]])
        trans_b_to_w = (-1 * rot_m0.dot(self.last_pos_m)).tolist()
        rot_m = np.array([[cos(-self.last_ang), -sin(-self.last_ang), trans_b_to_w[0][0]],
                          [sin(-self.last_ang), cos(-self.last_ang), trans_b_to_w[1][0]],
                          [0, 0, 1]])
        new_pos = rot_m.dot(pos)
        dt = (msg.header.stamp - self.lastime).to_sec()
        ca = (ang2 - ang3)
        if abs(ca) > 3.14:
            if ang < 0:
                ang2 += 2 * np.pi
            else:
                ang3 += 2 * np.pi
        vr = (ang2 - ang3) / dt
        v = new_pos / dt
        v[2] = vr
        self.last_ang = ang
        self.last_pos_m = pos
        self.lastime = msg.header.stamp
        if not self.first:
            self.vel_samples.append(v)
            self.vel_samples.pop(0)
            self.vel_i += 1
        else:
            self.first = False

        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation = tf.transformations.euler_from_quaternion(
            (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        self.last_pose = [position, orientation]

    def constrain(self, value):
        """
        Constrain a value to +- 120
        :param value: number
        :return: constrained number
        """
        if value > 120:
            value = 120
        elif value < -120:
            value = -120
        return value

    def handel_force(self, msg):
        """
        Takes in a force vector and updates the forceparameter of the kalman filter
        :param msg: geometry_msgs/Twist
        :return: None
        """
        f = msg.linear
        # [forward_motors, rigt_motor, left_motor]
        force = [self.constrain(f.x), self.constrain(f.y), self.constrain(f.z)]
        forward = self.force_curve[round(force[0])]
        right = self.force_curve[round(force[1])]
        left = self.force_curve[round(force[2])]
        r = self.physical_params[2]
        fi = self.physical_params[3]
        x_force = forward * 2
        y_force = right - left
        n_force = r * (-right * cos(fi) - left * cos(fi))
        self.u = np.array([[x_force], [y_force], [n_force]])

    def publish(self, vel):
        """
        formats and publishes a nav_msgs/Odometry
        :param vel: velocity of the robot in body coordinates
        :return: None
        """
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]
        odom.twist.twist.angular.z = vel[2]

        self.odom_pub.publish(odom)

    def run(self):
        """
        Runs the main loop of the program
        :return: None
        """
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.vel_i >= 5:
                vel = [0, 0, 0]
                for x, y, r in self.vel_samples:
                    vel[0] += x
                    vel[1] += y
                    vel[2] += r
                vel = np.array(vel).reshape((3, -1)) / len(self.vel_samples)
                self.filter.update(vel)
                self.vel_i = 0
            self.filter.predict(self.u, self.B, self.A, self.Q)
            t = Twist()
            t.linear.x = self.filter.x[0]
            t.linear.y = self.filter.x[1]
            t.angular.z = self.filter.x[2]
            self.pub.publish(t)
            self.publish([self.filter.x[0], self.filter.x[1], self.filter.x[2]])


if __name__ == "__main__":
    rospy.init_node('jallman', anonymous=True)
    j = Jallman()
    pose_topic = rospy.get_param("pose_topic", "slam_out_pose")
    rospy.Subscriber(pose_topic, PoseStamped, j.handel_pose)
    rospy.Subscriber("roboclaw", Twist, j.handel_force)
    j.run()
