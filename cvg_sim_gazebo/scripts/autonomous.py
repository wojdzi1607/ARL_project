#!/usr/bin/env python
import rospy

import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import cv2


class Autonomous:
    def __init__(self):
        # Aruco detect
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/detected_markers", Image, queue_size=1)
        self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
        self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.callback_bottom_image)
        # Autonomous fly
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Time
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1. / 20.), self.timer_callback)
        # Global vars
        self.command = Twist()
        self.corners = None
        self.x = 0

    def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0 / 20.):
            self.pub(self.last_published)

    def callback_bottom_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        markers_img, ids_list = self.detect_aruco(cv_image)

        if ids_list is None:
            self.id_pub.publish(ids_list)
        else:
            ids_str = ''.join(str(e) for e in ids_list)
            self.id_pub.publish(ids_str)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def detect_aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the aruco markers and display its aruco id.

        self.corners = rejected
        self.autonomous_flying(self.corners)

        return output, ids

    def autonomous_flying(self, corners):
        self.x += 1
        self.pubTakeoff.publish(Empty())

        # print(corners)

        for marker in corners:
            x_center = (marker[0, 0, 0] + (marker[0, 1, 0] - marker[0, 0, 0]) / 2) / 640
            y_center = (marker[0, 0, 1] + (marker[0, 3, 1] - marker[0, 0, 1]) / 2) / 360
            # print(x_center, y_center)
            if x_center < 0.5 and y_center < 0.5:
                print('forward left')
                self.command.linear.x = -0.1
                self.command.linear.y = 0.1
            if x_center > 0.5 and y_center < 0.5:
                print('forward right')
                self.command.linear.x = -0.1
                self.command.linear.y = -0.1
            if x_center < 0.5 and y_center > 0.5:
                print('backward left')
                self.command.linear.x = -0.5
                self.command.linear.y = 0.1
            if x_center > 0.5 and y_center > 0.5:
                print('backward right')
                self.command.linear.x = -0.5
                self.command.linear.y = -0.1
            print(x_center, y_center)

        if self.x < 100:
            self.command.linear.x = 0
            self.command.linear.y = 0
            self.command.linear.z = 1
            self.command.angular.x = 0
            self.command.angular.y = 0
            self.command.angular.z = 0
        else:
            # self.command.linear.x = -0.5
            # self.command.linear.y = 0
            self.command.linear.z = 0
            # self.command.angular.x = 0
            # self.command.angular.y = 0
            # self.command.angular.z = 0
        self.pub.publish(self.command)


if __name__ == "__main__":
    rospy.init_node("autonomous_node")
    f = Autonomous()
    rospy.spin()
