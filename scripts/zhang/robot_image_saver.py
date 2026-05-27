#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


IMAGE_TOPIC = "/camera/color/image_raw"
SAVE_PATH = "/home/lemon/roban_motion_control/week4/images/latest_realsense.jpg"
SAVE_HZ = 2.0


class RobotImageSaver(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_image = None
        self.last_save_time = 0.0

        save_dir = os.path.dirname(SAVE_PATH)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        self.sub = rospy.Subscriber(
            IMAGE_TOPIC,
            Image,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("Subscribed image topic: %s", IMAGE_TOPIC)
        rospy.loginfo("Saving latest image to: %s", SAVE_PATH)
        rospy.loginfo("Save frequency: %.1f Hz", SAVE_HZ)

    def image_callback(self, msg):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return

        now = time.time()
        if now - self.last_save_time < 1.0 / SAVE_HZ:
            return

        ok = cv2.imwrite(SAVE_PATH, image_bgr)
        self.last_save_time = now

        if ok:
            rospy.loginfo("Saved image: %s", SAVE_PATH)
        else:
            rospy.logwarn("Failed to save image: %s", SAVE_PATH)


def main():
    rospy.init_node("robot_image_saver", anonymous=True)
    RobotImageSaver()
    rospy.spin()


if __name__ == "__main__":
    main()
