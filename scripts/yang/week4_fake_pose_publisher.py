#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
from std_msgs.msg import String

def main():
    rospy.init_node("week4_fake_pose_publisher")

    pub = rospy.Publisher("/upper_body_pose_angles", String, queue_size=10)
    rate = rospy.Rate(0.5)

    samples = [
    {
        "visible": True,
        "confidence": 0.99,
        "left_arm_raise_angle": 90.0,
        "left_elbow_angle": 120.0,
        "right_arm_raise_angle": 90.0,
        "right_elbow_angle": 120.0
    },
    {
        "visible": True,
        "confidence": 0.98,
        "left_arm_raise_angle": 30.0,
        "left_elbow_angle": 60.0,
        "right_arm_raise_angle": 30.0,
        "right_elbow_angle": 60.0
    }
]

    idx = 0

    print("Week4 fake pose publisher started.")
    print("Publishing JSON to /upper_body_pose_angles")

    while not rospy.is_shutdown():
        msg = String()
        msg.data = json.dumps(samples[idx % len(samples)])
        rospy.loginfo("publish pose: %s", msg.data)
        pub.publish(msg)
        idx += 1
        rate.sleep()

if __name__ == "__main__":
    main()
