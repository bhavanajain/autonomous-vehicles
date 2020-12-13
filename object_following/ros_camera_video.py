import os, sys
lib_path = "/home/bhavana*/.local/lib/python2.7/site-packages"
if os.path.exists(lib_path):
    sys.path.insert(0, lib_path)

import argparse
from imutils.video import FPS
from imutils.video import VideoStream
import imutils
import numpy as np
from imutils import paths
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import rospy
import roslib


class object_tracker:
    def __init__(self, tracker_type):
        self.bridge = CvBridge()
        # subscribe to the camera feed
        self.image_sub = rospy.Subscriber(
            "/mako_1/mako_1/image_raw", Image, self.callback
        )
        self.tracker_type = tracker_type

        # define tracker object based on cv version and tracker type
        (major, minor) = cv2.__version__.split(".")[:2]
        if int(major) == 3 and int(minor) < 3:
            self.tracker = cv2.Tracker_create(tracker_type.upper())
        else:
            OPENCV_OBJECT_TRACKERS = {
                "csrt": cv2.TrackerCSRT_create,
                "kcf": cv2.TrackerKCF_create,
                "boosting": cv2.TrackerBoosting_create,
                "mil": cv2.TrackerMIL_create,
                "tld": cv2.TrackerTLD_create,
                "medianflow": cv2.TrackerMedianFlow_create,
                "mosse": cv2.TrackerMOSSE_create
            }
            self.tracker = OPENCV_OBJECT_TRACKERS[tracker_type]()
        self.initBB = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = imutils.resize(cv_image, width=500)
            (H, W) = frame.shape[:2]

            if self.initBB is not None:
                (success, box) = self.tracker.update(frame)
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    cv2.rectangle(frame, (x, y), (x + w, y + h),
                                  (0, 255, 0), 2)
                self.fps.update()
                self.fps.stop()
                info = [
                    ("Tracker", self.tracker_type),
                    ("Success", "Yes" if success else "No"),
                    ("FPS", "{:.2f}".format(self.fps.fps())),
                ]
                for (i, (k, v)) in enumerate(info):
                    text = "{}: {}".format(k, v)
                    cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("ROS Camera Feed", frame)
            key = cv2.waitKey(1) & 0xFF

            # if the 's' key is selected, we are going to "select" a bounding
            # box to track
            if key == ord("s"):
                self.initBB = cv2.selectROI("ROS Camera Feed", frame, fromCenter=False,
                                            showCrosshair=True)
                self.tracker.init(frame, self.initBB)
                self.fps = FPS().start()

            # if the `q` key was pressed, break from the loop
            elif key == ord("q"):
                sys.exit(0)

        except CvBridgeError as e:
            print(e)


parser = argparse.ArgumentParser()
parser.add_argument("-t", "--tracker", type=str, default="kcf",
                help="OpenCV object tracker type")
args = vars(parser.parse_args())

rospy.init_node("object_tracker", anonymous=True)
ic = object_tracker(args["tracker"])

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
    print("Shutting down")
