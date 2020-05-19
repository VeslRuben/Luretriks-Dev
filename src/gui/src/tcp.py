#!/usr/bin/env python

import socket
import json
import rospy
import thread
from std_msgs.msg import String

"""
This is a script that is incomplete, and should be finished to actually enable communication from the GUI to the program
to change driving modes etc. 
"""

class TcpServer:

    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.shutdown = False
        self.publisher = rospy.Publisher('ctrl_mode', String, queue_size=10)

    def parse_msg(self, msg):
        """
        {topic: system_cmd, data: {recipient: ..., parameter_name: ...., value: ...}}
        """
        data = msg.decode('utf-8')
        message = String.data = data
        self.publisher.publish(message)

    def main(self):
        rate = rospy.Rate(5)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as soc:
            soc.bind(self.addr)
            while not rospy.is_shutdown():
                soc.listen(5)
                connection, address = soc.accept()
                with connection:
                    connection.settimeout(None)
                    while connection:
                        data = connection.recv(1024)
                        if not data:
                            break
                        self.parse_msg(data)
                        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tcp', anonymous=True)
    t = TcpServer('192.168.0.103', 5015)
    t.main()
