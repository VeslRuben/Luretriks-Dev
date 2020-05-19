#!/usr/bin/env python
import time
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String


class Watcher:

    def __init__(self):
        self.t0 = time.time()
        self.timeout = True

        self.mode = "manual"
        self.idle = False

        self.new_joy_msg = False
        self.vel_from_joy = Twist()

        self.vel_from_cmd_vel = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.new_vel_msg = False

        self.scaler = 50
        self.a_button_last_state = 0
        self.b_button_last_state = 0
        self.y_button_last_state = 0

        self.motor_publisher = rospy.Publisher('pid', Twist, queue_size=10)
        self.roboclaw_publisher = rospy.Publisher('roboclaw', Twist, queue_size=10)

    def handle_joy(self, msg):
        """
        Gets the button presses from the xbox controller.
        Sends manual or autonomous control
        :param msg: sensor_msgs/joy
        :return: None
        """
        buttons = msg.buttons

        y_button = buttons[2]

        if y_button and not self.y_button_last_state:
            if self.mode == "manual":
                self.mode = "auto"
            else:
                self.mode = "manual"

        self.y_button_last_state = y_button

    def handle_cmd_vel(self, msg):
        """
        saves the latest velocity message from the nav stack
        :param msg: geometry_msgs/Twist
        :return: None
        """
        vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.vel_from_cmd_vel.append(vel)
        self.vel_from_cmd_vel.pop(0)
        if vel[0] == 0 and vel[1] == 0 and vel[2] == 0:
            for i in range(5):
                self.vel_from_cmd_vel.append(vel)
                self.vel_from_cmd_vel.pop(0)
        self.new_vel_msg = True

    def handle_joy_vel(self, msg):
        """
        saves the latest velocity message from the joy_to_vel node
        :param msg: geometry_msgs/Twist
        :return: None
        """
        self.vel_from_joy = msg
        self.new_joy_msg = True

    def handel_timeout(self, msg):
        """
        resets timer if a new message from safety_publisher is received
        :param msg: standard_msgs/String
        :return: None
        """
        if msg.data == "alive":
            self.t0 = time.time()

    def handle_sys_cmd(self, msg):
        """
        changes operating mode depending on the command from the GUI
        Currently not in use since TCP has not been established with the GUI
        :param msg: standard_msgs/String
        :return:
        """
        if msg.data == "manual":
            self.mode = "manual"
        elif msg.data == "auto":
            self.mode = "auto"
        elif msg.data == "idle":
            self.idle = True
        elif msg.data == "run":
            self.idle = False

    def main(self):
        """
        forwards velocity messages depending on the operating mode of the robot
        or sends zero velocity if no connection to pc
        :return: None
        """
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            rate.sleep()

            if time.time() - self.t0 > 2:
                self.timeout = True
            else:
                self.timeout = False

            if self.timeout:
                msg = Twist()
                msg.linear.x = 0
                msg.linear.y = 0
                msg.linear.z = 0
                self.roboclaw_publisher.publish(msg)
                rospy.logwarn("Timeout, stoping all motors")
            elif self.idle:
                msg = Twist()
                msg.linear.x = 0
                msg.linear.y = 0
                msg.angular.z = 0
                self.motor_publisher.publish(msg)
            elif self.mode == "manual":
                if self.new_joy_msg:
                    self.motor_publisher.publish(self.vel_from_joy)
                    self.new_joy_msg = False
            elif self.mode == "auto":
                if self.new_vel_msg:
                    vel = [0, 0, 0]
                    for x, y, r in self.vel_from_cmd_vel:
                        vel[0] += x
                        vel[1] += y
                        vel[2] += r
                    vel = [float(c) / len(self.vel_from_cmd_vel) for c in vel]
                    t = Twist()
                    t.linear.x = vel[0]
                    t.linear.y = vel[1]
                    t.angular.z = vel[2]
                    self.motor_publisher.publish(t)
                    self.new_vel_msg = False


if __name__ == '__main__':
    rospy.init_node("safety_watcher", anonymous=True)
    watch = Watcher()
    rospy.Subscriber("joy", Joy, watch.handle_joy)
    rospy.Subscriber("cmd_vel", Twist, watch.handle_cmd_vel)
    rospy.Subscriber("joy_vel", Twist, watch.handle_joy_vel)
    rospy.Subscriber("ctrl_mode", String, watch.handle_sys_cmd)
    rospy.Subscriber("alive", String, watch.handel_timeout)
    watch.main()
