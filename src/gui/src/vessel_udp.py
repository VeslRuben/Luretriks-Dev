#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import socket
import json
from threading import Thread


class UdpServer:

    def __init__(self, host, port, buffer_size=1024):
        self.addr = (host, port)
        self.buffer_size = buffer_size
        self.available = {'kalman': True, 'roboclaw': True}
        self.data = {
            'kalman': {'speedX': 0, 'speedY': 0, 'speedTheta': 0},
            'motors': {'forwardLeft': 0, 'forwardRight': 0, 'turnLeft': 0, 'turnRight': 0}
        }
        self.socket = None
        self.client_list = []
        self.shutdown = False

    def kalman_callback(self, msg):
        """
        formats and saves the latest vessel speed message
        :param msg: geometry_msgs/Twist
        :return: None
        """
        if self.available['kalman']:
            self.available['kalman'] = False
            self.data['kalman']['speedX'] = msg.linear.x
            self.data['kalman']['speedY'] = msg.linear.y
            self.data['kalman']['speedTheta'] = msg.angular.z
            self.available['kalman'] = True

    def roboclaw_callback(self, msg):
        """
        formats and saves the latest motor speed message
        :param msg: geometry_msgs/Twist
        :return: None
        """
        if self.available['roboclaw']:
            self.available['roboclaw'] = False
            self.data['motors']['forwardLeft'] = msg.linear.x
            self.data['motors']['forwardRight'] = msg.linear.x
            self.data['motors']['turnRight'] = msg.linear.y
            self.data['motors']['turnLeft'] = msg.linear.z
            self.available['roboclaw'] = True

    def set_socket(self, socket):
        self.socket = socket
        self.socket.settimeout(1)

    def accept_client(self):
        """
        registers new address to send to
        :return: None
        """
        while not self.shutdown:
            if self.socket is not None:
                try:
                    msg, client_addr = s.recvfrom(self.buffer_size)
                    if client_addr not in self.client_list:
                        self.client_list.append(client_addr)
                        print("vessel pub has Conection from: {}, message: {}".format(client_addr, msg))
                except socket.error as e:
                    pass


if __name__ == '__main__':
    rospy.init_node('vessel_udp_publisher', anonymous=True)
    server = UdpServer('192.168.0.103', 5006)
    welcome_thread = Thread(target=server.accept_client)
    rate = rospy.Rate(4)
    rospy.Subscriber("roboclaw", Twist, server.roboclaw_callback)
    rospy.Subscriber("kalman", Twist, server.kalman_callback)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(server.addr)
    server.set_socket(s)
    welcome_thread.start()
    while not rospy.is_shutdown():
        rate.sleep()

        while not server.available['kalman']:
            pass
        server.available['kalman'] = False
        while not server.available['roboclaw']:
            pass
        server.available['roboclaw'] = False
        data = json.dumps(server.data).encode('utf-8')
        for c in server.client_list:
            s.sendto(data, c)

        server.available['kalman'] = True
        server.available['roboclaw'] = True
    server.shutdown = True