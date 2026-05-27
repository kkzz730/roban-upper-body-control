#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

bridge = CvBridge()
saved = False

def callback(msg):
    global saved
    if saved:
        return

    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        save_path = os.path.expanduser(
            "~/roban_motion_control/week1/images/realsense_color_test.jpg"
        )
        cv2.imwrite(save_path, img)
        print("Saved robot camera image to:", save_path)
        saved = True
        rospy.signal_shutdown("image saved")
    except Exception as e:
        print("cv_bridge error:", e)

if __name__ == "__main__":
    rospy.init_node("save_one_realsense_color_image")
    rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size=1)
    rospy.spin()
