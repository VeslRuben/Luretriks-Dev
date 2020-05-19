#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class PID:

    def __init__(self, p, i, d, rate=20., out_lim=None, e_sum_lim=None):
        """
        :param p: P gain
        :param i: I gain
        :param d: D gain
        :param rate: Rate in seconds
        :param out_lim: maximum output
        :param e_sum_lim: maximum error sum value
        """
        if e_sum_lim is None:
            e_sum_lim = [-10., 10.]
        if out_lim is None:
            out_lim = [-50., 50.]
        self.out_lim = out_lim
        self.p = float(p)
        self.i = float(i)
        self.d = float(d)
        self.e0 = 0.
        self.e_sum = 0.
        self.t0 = 0.
        self.rate = rate
        self.e_sum_lim = e_sum_lim

    def pass_value(self, r, m, dt=None):
        """
        calculates the output from the controller
        :param r: setpoint
        :param m: measurement
        :param dt: time pased in sec
        :return: PID output
        """
        if dt is None:
            dt = 1. / float(self.rate)

        e = r - m
        de = e - self.e0
        self.e_sum += e
        self.e_sum = self._constrain(self.e_sum, self.e_sum_lim)
        self.e0 = e

        p = self.p * e
        i = self.i * self.e_sum * dt
        d = self.d * de / dt
        pid = self._constrain(p + i + d, self.out_lim)

        return pid

    def _constrain(self, v, c):
        """
        constrains the value.
        :param v: value to constrain
        :param c: constrain limit
        :return: constrained value
        """
        v = max(v, c[0])
        v = min(v, c[1])
        return float(v)


class Regulator:

    def __init__(self, pid_forward, pid_side, pid_turn, publisher_topic='roboclaw'):
        self.pid_forward = pid_forward
        self.pid_side = pid_side
        self.pid_turn = pid_turn
        self.publisher = rospy.Publisher(publisher_topic, Twist, queue_size=10)
        self.r = {'vx': 0, 'vy': 0, 'vr': 0}
        self.m = {'vx': 0, 'vy': 0, 'vr': 0}

    def handle_vel(self, msg):
        """
        saves the latest velocity message
        :param msg: geometry_msgs/Twist
        :return: None
        """
        self.r['vx'] = msg.linear.x
        self.r['vy'] = msg.linear.y
        self.r['vr'] = msg.angular.z

    def handle_measurement(self, msg):
        """
        saves the latest measurements from the motors
        :param msg: geometry_msgs/Twist
        :return: None
        """
        self.m['vx'] = msg.linear.x
        self.m['vy'] = msg.linear.y
        self.m['vr'] = msg.angular.z

    def constrain(self, value):
        """
        constrains the number to +- 100.
        :param value: number
        :return: constraind number
        """
        if value > 100:
            value = 100
        elif value < -100:
            value = -100
        return value

    def comp_for_bacwords(self, speed):
        """
        compensate for motors not being as efficient in reverse direction.
        :param speed: motor speed
        :return: speed * 1.3 if negaive
        """
        if speed < 0:
            return speed * 1.3
        else:
            return speed

    def run(self, dt=None):
        """
        runs the regulator
        :param dt: time pased
        :return: None
        """
        forward = self.pid_forward.pass_value(self.r['vx'], self.m['vx'], dt=dt)
        side = self.pid_side.pass_value(self.r['vy'], self.m['vy'], dt=dt)
        turn = self.pid_turn.pass_value(self.r['vr'], self.m['vr'], dt=dt)

        r, l = 0, 0
        r -= turn
        l -= turn
        r += side
        l -= side

        forward = self.constrain(forward)
        r = self.constrain(r)
        l = self.constrain(l)

        msg = Twist()
        msg.linear.x = self.comp_for_bacwords(forward)
        msg.linear.y = self.comp_for_bacwords(r)
        msg.linear.z = self.comp_for_bacwords(l)

        self.publisher.publish(msg)


def main():
    r = 20.  # frequency of rospy.Rate
    rate = rospy.Rate(int(r))
    pid_forward = PID(500., 500., 5., rate=r)
    pid_side = PID(500., 400., 5., rate=r)
    pid_turn = PID(500., 400., 5., rate=r)
    regulator = Regulator(pid_forward, pid_side, pid_turn)
    rospy.Subscriber('pid', Twist, regulator.handle_vel)
    rospy.Subscriber('kalman', Twist, regulator.handle_measurement)

    while not rospy.is_shutdown():
        rate.sleep()
        regulator.run()


if __name__ == '__main__':
    rospy.init_node('regulator', anonymous=True)
    main()
