#!/usr/bin/env python
import csv

import rospy
from autodock_test_pkg.msg import TestMessage
from geometry_msgs.msg import PoseStamped, Twist


def main():
    rospy.init_node('test_publisher_node', anonymous=True)
    pub_p = rospy.Publisher('swag', PoseStamped, queue_size=10)  # Topic to publish, message format, queue_size
    pub2 = rospy.Publisher('yolo', Twist, queue_size=10)  # Topic to publish, message format, queue_size
    rate = rospy.Rate(130)  # 1 Hz refresh rate

    with open('/home/ruben/luretriks-dev/src/autodock_test_pkg/test_orient.csv') as f:
        reader = csv.reader(f)
        ori = list(reader)
        ori = [[float(y) for y in x] for x in ori]
    with open('/home/ruben/luretriks-dev/src/autodock_test_pkg/test_pose.csv') as f:
        reader = csv.reader(f)
        pose = list(reader)
        pose = [[float(y) for y in x] for x in pose]
    with open('/home/ruben/luretriks-dev/src/autodock_test_pkg/motor_vector.csv') as f:
        reader = csv.reader(f)
        motor = list(reader)
        motor = [[float(y) for y in x] for x in motor]
        print len(motor)
    i = 0
    while not rospy.is_shutdown():
        if i % 11 == 0:
            pose_m = PoseStamped()
            pose_m.header.stamp = rospy.Time.now()
            pose_m.pose.position.x = pose[i/11][0]
            pose_m.pose.position.y = pose[i/11][1]
            pose_m.pose.position.z = pose[i/11][2]
            pose_m.pose.orientation.x = ori[i/11][0]
            pose_m.pose.orientation.y = ori[i/11][1]
            pose_m.pose.orientation.z = ori[i/11][2]
            pose_m.pose.orientation.w = ori[i/11][3]
            pub_p.publish(pose_m)
        t = Twist()
        t.linear.x = motor[i][0]
        t.linear.y = motor[i][1]
        t.linear.z = motor[i][2]
        pub2.publish(t)
        i += 1
        rate.sleep()


if __name__ == '__main__':
    main()
