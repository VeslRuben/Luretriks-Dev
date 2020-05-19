#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf


class Odom:
    def __init__(self):
        self.pose_i = 0
        self.poses = [[[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]],
                      [[0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0]]]
        self.lase_pose = [[0, 0, 0], [0, 0, 0]]

        rospy.Subscriber("slam_out_pose", PoseStamped, self.handel_pose)

        self.pub = rospy.Publisher("odom", Odometry, queue_size=10)

    def handel_pose(self, msg):
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation = tf.transformations.euler_from_quaternion(
            (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        if self.pose_i < 5:
            self.poses[self.pose_i] = [position, orientation]
        self.pose_i += 1

    def average(self, lst):
        sum_p = [0, 0, 0]
        sum_o = [0, 0, 0]
        for l in lst:
            sum_p = [x + y for x, y in zip(sum_p, l[0])]
            sum_o = [x + y for x, y in zip(sum_o, l[1])]
        p = [float(x) / float(len(lst)) for x in sum_p]
        o = [float(x) / float(len(lst)) for x in sum_o]
        return p, o

    def diff(self, pose, last_pose):
        return [p - l_p for p, l_p in zip(pose[0], last_pose[0])], [o - l_o for o, l_o in zip(pose[1], last_pose[1])]

    def publish(self, vel, pose):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = pose[0][0]
        odom.pose.pose.position.y = pose[0][1]
        odom.pose.pose.position.z = pose[0][2]

        q = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vel[0][0]
        odom.twist.twist.linear.y = vel[0][1]
        odom.twist.twist.linear.z = vel[0][2]

        odom.twist.twist.angular.x = vel[1][0]
        odom.twist.twist.angular.y = vel[1][1]
        odom.twist.twist.angular.z = vel[1][2]

        self.pub.publish(odom)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            a_pose = self.average(self.poses)
            vel = self.diff(a_pose, self.lase_pose)
            self.lase_pose = a_pose

            self.publish(vel, a_pose)

            if self.pose_i >= 5:
                self.pose_i = 0


if __name__ == "__main__":
    rospy.init_node("odom", anonymous=True)
    o = Odom()
    o.run()
