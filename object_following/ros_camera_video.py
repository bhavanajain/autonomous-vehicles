import os, sys
from os.path import dirname, abspath

likely_lib_path = os.path.join(
    dirname(dirname(dirname(abspath(__file__)))), "lib", "p27_site_packages"
)
if os.path.exists(likely_lib_path):
    sys.path.insert(0, likely_lib_path)

import roslib
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
from imutils import paths
import numpy as np
import imutils

# Define the codec and create VideoWriter object
frame_width = 640
frame_height = 480
fourcc = cv2.VideoWriter_fourcc(*"XVID")
video_writer = cv2.VideoWriter("output.avi", fourcc, 20, (frame_width, frame_height))

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/mako_1/mako_1/image_raw", Image, self.callback
        )

    def callback(self, data):
        try:
            global video_writer
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            video_writer.write(cv_image)

        except CvBridgeError as e:
            global video_writer
            video_writer.release()
            cv2.destroyAllWindows()
            print(e)


rospy.init_node("image_converter", anonymous=True)
ic = image_converter()

try:
    rospy.spin()
except KeyboardInterrupt:
    global video_writer
    video_writer.release()
    cv2.destroyAllWindows()

    print("Shutting down")
