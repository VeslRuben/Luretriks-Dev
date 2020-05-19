#!/usr/bin/env python
import json
import socket

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import copy
import base64


class MapToGui:
    def __init__(self):
        self.rate = rospy.Rate(5)
        self.ready = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serverAddress = ('192.168.0.103', 5007)
        self.sock.bind(serverAddress)
        self.sock.setblocking(False)
        self.resipients = []

        self.map = None
        self.map_gray = None
        self.map_meta_data = None

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def crope_img(self, im, meta_data, scale=2.0, gray_copy=None):
        """
        recursively crops the image so only gray pixels are removed.
        will stop trying to crop the image if white or gray pixels are getting lost, updates the map metadata.
        :param im: Image
        :param meta_data: meta data for the image
        :param scale: how much each pass should try to remove
        :param gray_copy: if cropping a picture in color put in gray scale image here to reduce computational time
        :return: cropped image, updated meta data
        """
        meta_data = copy.deepcopy(meta_data)
        if not gray_copy is None:
            crop_im = gray_copy
        else:
            crop_im = im
        center_h = int(meta_data.height / 2)
        center_w = int(meta_data.width / 2)
        dim_h = int(meta_data.height / scale)
        dim_w = int(meta_data.width / scale)
        crop = True
        for x in range(dim_h):
            if not crop_im[int(center_h - dim_h / scale) + x, int(center_w - dim_w / scale)] == 155.0:
                crop = False
                break
            if not crop_im[int(center_h - dim_h / scale) + x, int(center_w + dim_w / scale)] == 155.0:
                crop = False
                break
        if crop:
            for x in range(dim_w):
                if not crop_im[int(center_h - dim_h / scale), int(center_w - dim_w / scale) + x] == 155.0:
                    crop = False
                    break
                if not crop_im[int(center_h + dim_h / scale), int(center_w - dim_w / scale) + x] == 155.0:
                    crop = False
                    break

        if crop:
            im = im[int(center_h - dim_h / scale):int(center_h + dim_h / scale),
                 int(center_w - dim_w / scale):int(center_w + dim_w / scale)]
            meta_data.height = meta_data.height / scale
            meta_data.width = meta_data.width / scale
            meta_data.origin.position.x = meta_data.origin.position.x / scale
            meta_data.origin.position.y = meta_data.origin.position.y / scale
            if not gray_copy is None:
                gray_copy = gray_copy[int(center_h - dim_h / scale):int(center_h + dim_h / scale),
                            int(center_w - dim_w / scale):int(center_w + dim_w / scale)]
            im, meta_data = self.crope_img(im, meta_data, scale, gray_copy)

        return im, meta_data

    def recolore_img(self, im):
        """
        recolors the image to the right gray scale 0-255
        :param im: Image
        :return: Image in right scale
        """
        im = np.where(im == 100, 255, im)
        im = np.where(im == -1, 100, im)
        return 255 - im

    def handle_o_grid(self, msg):
        """
        recieve the map and save it to memory
        :param msg: nav_msgs/OccupancyGrid
        :return: None
        """
        im = np.array(msg.data)
        im = np.reshape(im, (msg.info.width, msg.info.height))
        im = self.recolore_img(im)
        im = im.astype(np.float32)
        im, msg.info = self.crope_img(im, msg.info)
        im_c = cv2.cvtColor(im, cv2.COLOR_GRAY2RGB)

        self.ready = False
        self.map = im_c
        self.map_gray = im
        self.map_meta_data = msg.info
        self.ready = True

    def calc_robot_pos(self, transform, map_meta_data):
        """
        finds the pixel on the map where the robot is located
        :param transform: position of robot in map coordinate frame
        :param map_meta_data: meta data of the map
        :return: Position of robot
        """
        x_offset = int(0 - map_meta_data.origin.position.x / map_meta_data.resolution)
        y_offset = int(0 - map_meta_data.origin.position.y / map_meta_data.resolution)
        t = transform.transform.translation
        return [int(t.x / map_meta_data.resolution + x_offset), int(t.y / map_meta_data.resolution + y_offset),
                int(t.z)]

    def send(self, im):
        """
        send a udp message wit an image of the map containing the robot
        :param im: image with robot drawn on
        :return: None
        """
        nco = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        im = cv2.imencode(".jpg", im, nco)[1]
        im = base64.b64encode(im)
        for r in self.resipients:
            self.sock.sendto(im, r)

    def main(self):
        """
        main loop of the program
        :return:
        """
        while not rospy.is_shutdown():
            try:
                message, address = self.sock.recvfrom(1024)
                if address not in self.resipients:
                    self.resipients.append(address)
                    print("map to gui has Conection from: {}, message: {}".format(address, message))
            except socket.error as e:
                pass

            if self.ready:
                self.rate.sleep()
                try:
                    base_link_trans = self.tfBuffer.lookup_transform("map", 'base_link', rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(e)
                    continue
                pose = self.calc_robot_pos(base_link_trans, self.map_meta_data)
                im = cv2.circle(self.map.copy(), (pose[0], pose[1]), 5, (0, 255, 0))
                #im, _ = self.crope_img(im, self.map_meta_data, 1.5, self.map_gray)
                im = im[850:512+1024,650:512+700,:]
                im = cv2.resize(im, (800, 600))
                self.send(im)


if __name__ == "__main__":
    rospy.init_node('test_yplp', anonymous=True)
    m = MapToGui()
    rospy.Subscriber('map', OccupancyGrid, m.handle_o_grid)
    m.main()
    # rospy.spin()
