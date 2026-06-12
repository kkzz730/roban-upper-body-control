#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json
import time

import rospy
from std_msgs.msg import String


DEFAULT_TOPIC = "/upper_body_pose_angles"
DEFAULT_FAKE_POSE_HZ = 30.0

SAMPLES = [
    {
        "visible": True,
        "confidence": 0.95,
        "left_arm_raise_angle": 20.0,
        "left_elbow_angle": 160.0,
        "right_arm_raise_angle": 20.0,
        "right_elbow_angle": 160.0,
        "reason": "fake_high_rate",
    },
    {
        "visible": True,
        "confidence": 0.96,
        "left_arm_raise_angle": 80.0,
        "left_elbow_angle": 100.0,
        "right_arm_raise_angle": 80.0,
        "right_elbow_angle": 100.0,
        "reason": "fake_high_rate",
    },
    {
        "visible": True,
        "confidence": 0.94,
        "left_arm_raise_angle": 45.0,
        "left_elbow_angle": 130.0,
        "right_arm_raise_angle": 65.0,
        "right_elbow_angle": 120.0,
        "reason": "fake_high_rate",
    },
]


def main():
    parser = argparse.ArgumentParser(description="Publish high-rate fake upper-body pose JSON.")
    parser.add_argument("--hz", type=float, default=DEFAULT_FAKE_POSE_HZ, help="Publish frequency.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Pose JSON output topic.")
    parser.add_argument("--sample-period", type=float, default=2.0, help="Seconds to hold each fake pose.")
    args = parser.parse_args()

    rospy.init_node("fake_pose_high_rate_publisher", anonymous=True)
    pub = rospy.Publisher(args.topic, String, queue_size=10)
    rate = rospy.Rate(max(1.0, args.hz))

    print("Fake high-rate pose publisher started.")
    print("Publishing JSON to %s at %.1f Hz" % (args.topic, args.hz))

    start = time.time()
    while not rospy.is_shutdown():
        elapsed = time.time() - start
        index = int(elapsed / max(0.1, args.sample_period)) % len(SAMPLES)
        text = json.dumps(SAMPLES[index])
        pub.publish(String(data=text))
        rospy.loginfo("publish pose: %s", text)
        rate.sleep()


if __name__ == "__main__":
    main()
