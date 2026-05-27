#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pose_detector import detect_upper_body_angles, draw_pose_result_overlay
from smoothing_filter import AngleSmoother


IMAGE_TOPIC = "/camera/color/image_raw"
OUTPUT_TOPIC = "/upper_body_pose_angles"

# Low frequency publishing for safety.
PUBLISH_HZ = 2.0

# Minimum landmark visibility / confidence threshold.
MIN_CONFIDENCE = 0.6

# Save overlay image for experiment record.
OVERLAY_SAVE_PATH = "/home/lemon/roban_motion_control/week4/pose_result_overlay.jpg"


class RealtimePosePublisher(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_msg_time = None

        self.smoother = AngleSmoother(alpha=0.3)

        self.pub = rospy.Publisher(
            OUTPUT_TOPIC,
            String,
            queue_size=10
        )

        self.sub = rospy.Subscriber(
            IMAGE_TOPIC,
            Image,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("Subscribed image topic: %s", IMAGE_TOPIC)
        rospy.loginfo("Publishing pose angles topic: %s", OUTPUT_TOPIC)
        rospy.loginfo("Publish frequency: %.1f Hz", PUBLISH_HZ)

    def image_callback(self, msg):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return

        self.latest_image = image_bgr
        self.latest_msg_time = rospy.Time.now()

    def build_invalid_result(self, reason):
        return {
            "visible": False,
            "confidence": 0.0,
            "left_arm_raise_angle": None,
            "left_elbow_angle": None,
            "right_arm_raise_angle": None,
            "right_elbow_angle": None,
            "reason": reason
        }

    def build_publish_result(self, pose_result):
        """
        Convert raw pose result to robot-side JSON format.

        The robot controller only needs:
        visible, confidence,
        left_arm_raise_angle, left_elbow_angle,
        right_arm_raise_angle, right_elbow_angle,
        reason.
        """
        if not pose_result.get("visible", False):
            return {
                "visible": False,
                "confidence": float(pose_result.get("confidence", 0.0)),
                "left_arm_raise_angle": None,
                "left_elbow_angle": None,
                "right_arm_raise_angle": None,
                "right_elbow_angle": None,
                "reason": pose_result.get("reason", "invalid pose")
            }

        smoothed = self.smoother.update_result(pose_result)

        return {
            "visible": True,
            "confidence": float(smoothed.get("confidence", 0.0)),
            "left_arm_raise_angle": smoothed.get(
                "smoothed_left_arm_raise_angle",
                smoothed.get("left_arm_raise_angle")
            ),
            "left_elbow_angle": smoothed.get(
                "smoothed_left_elbow_angle",
                smoothed.get("left_elbow_angle")
            ),
            "right_arm_raise_angle": smoothed.get(
                "smoothed_right_arm_raise_angle",
                smoothed.get("right_arm_raise_angle")
            ),
            "right_elbow_angle": smoothed.get(
                "smoothed_right_elbow_angle",
                smoothed.get("right_elbow_angle")
            ),
            "reason": smoothed.get("reason", "ok")
        }

    def run_once(self):
        if self.latest_image is None:
            result = self.build_invalid_result("no image received")
            self.publish_result(result)
            return

        pose_result = detect_upper_body_angles(
            self.latest_image,
            min_visibility=MIN_CONFIDENCE
        )

        publish_result = self.build_publish_result(pose_result)
        self.publish_result(publish_result)

        try:
            overlay_dir = os.path.dirname(OVERLAY_SAVE_PATH)
            if overlay_dir and not os.path.exists(overlay_dir):
                os.makedirs(overlay_dir)

            overlay = draw_pose_result_overlay(self.latest_image, pose_result)
            cv2.imwrite(OVERLAY_SAVE_PATH, overlay)
        except Exception as e:
            rospy.logwarn("Failed to save overlay image: %s", str(e))

    def publish_result(self, result):
        text = json.dumps(result, ensure_ascii=False)
        self.pub.publish(String(data=text))
        rospy.loginfo("pose json: %s", text)


def main():
    rospy.init_node("realtime_pose_publisher", anonymous=True)

    node = RealtimePosePublisher()
    rate = rospy.Rate(PUBLISH_HZ)

    while not rospy.is_shutdown():
        node.run_once()
        rate.sleep()


if __name__ == "__main__":
    main()
