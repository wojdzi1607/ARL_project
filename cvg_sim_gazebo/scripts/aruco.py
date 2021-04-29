#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class Image_convert_pub:
    def __init__(self):
        self.pub = rospy.Publisher('/ardrone/corners', numpy_msg(Floats), queue_size=10)
        self.image_pub = rospy.Publisher("/detected_markers", Image, queue_size=1)
        self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw", Image, self.callback)
        # self.pub_cor = rospy.Publisher('/ardrone/corners', Int32MultiArray, queue_size=10)
        self.corners = None

    def callback(self, data):
        x = np.array(self.corners)
        self.pub.publish(np.array(x, dtype=np.float32))

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
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.corners = corners
        print(type(corners))
        output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the aruco markers and display its aruco id.
        return output, ids


if __name__ == "__main__":
    rospy.init_node("aruco_node")
    ic = Image_convert_pub()
    rospy.spin()
