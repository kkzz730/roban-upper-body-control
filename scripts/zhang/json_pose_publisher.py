#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rospy
import json

from std_msgs.msg import String


JSON_PATH = "/home/lemon/roban_motion_control/week4/logs/upper_body_pose_angles.json"
OUTPUT_TOPIC = "/upper_body_pose_angles"
PUBLISH_HZ = 20.0


class JsonPosePublisher(object):
    def __init__(self):
        self.pub = rospy.Publisher(
            OUTPUT_TOPIC,
            String,
            queue_size=10
        )

        self.last_text = None
        self.last_mtime = 0.0
        self.last_logged_text = None
        self.last_log_time = 0.0

        rospy.loginfo("Reading pose JSON from: %s", JSON_PATH)
        rospy.loginfo("Publishing ROS topic: %s", OUTPUT_TOPIC)
        rospy.loginfo("Publish frequency: %.1f Hz", PUBLISH_HZ)

    def build_invalid_json(self, reason):
        return {
            "visible": False,
            "confidence": 0.0,
            "left_arm_raise_angle": None,
            "left_elbow_angle": None,
            "right_arm_raise_angle": None,
            "right_elbow_angle": None,
            "reason": reason
        }

    def read_json_text(self):
        if not os.path.exists(JSON_PATH):
            return json.dumps(
                self.build_invalid_json("json file not found"),
                ensure_ascii=False
            )

        try:
            mtime = os.path.getmtime(JSON_PATH)

            with open(JSON_PATH, "r") as f:
                data = json.load(f)

            text = json.dumps(data, ensure_ascii=False)
            self.last_text = text
            self.last_mtime = mtime
            return text

        except Exception as e:
            rospy.logwarn("Failed to read JSON: %s", str(e))

            if self.last_text is not None:
                return self.last_text

            return json.dumps(
                self.build_invalid_json("json read failed"),
                ensure_ascii=False
            )

    def run(self):
        rate = rospy.Rate(PUBLISH_HZ)

        while not rospy.is_shutdown():
            text = self.read_json_text()
            self.pub.publish(String(data=text))
            now = time.time()
            if text != self.last_logged_text or now - self.last_log_time > 1.0:
                rospy.loginfo("publish pose json: %s", text)
                self.last_logged_text = text
                self.last_log_time = now
            rate.sleep()


def main():
    rospy.init_node("json_pose_publisher", anonymous=True)
    node = JsonPosePublisher()
    node.run()


if __name__ == "__main__":
    main()
