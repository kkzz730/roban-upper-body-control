#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lejulib import *
import rospy
import json
import time
from std_msgs.msg import String

# Stable base frame from previous verified Roban action scripts.
BASE_FRAME = [0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0]

# Conservative upper-body joint indices based on previous action-frame tests.
LEFT_SHOULDER_IDX = 13
LEFT_ELBOW_IDX = 14
RIGHT_SHOULDER_IDX = 16
RIGHT_ELBOW_IDX = 17

ACTION_INTERVAL = 1.5
last_action_time = 0.0
busy = False

def map_range(x, in_min, in_max, out_min, out_max):
    x = max(min(x, in_max), in_min)
    return out_min + (x - in_min) * (out_max - out_min) / float(in_max - in_min)

def safe_angle(x, low=-80.0, high=80.0):
    return max(low, min(high, x))

def generate_upper_body_frame(pose):
    frame = BASE_FRAME[:]

    # If left-arm fields are not provided yet, keep left arm safe.
    left_raise = pose.get("left_arm_raise_angle", 0.0)
    left_elbow = pose.get("left_elbow_angle", 30.0)

    right_raise = pose.get("right_arm_raise_angle", 0.0)
    right_elbow = pose.get("right_elbow_angle", 30.0)

    left_shoulder_delta = map_range(left_raise, 0, 120, 0, 35)
    left_elbow_delta = map_range(left_elbow, 30, 170, 0, 30)

    right_shoulder_delta = map_range(right_raise, 0, 120, 0, 35)
    right_elbow_delta = map_range(right_elbow, 30, 170, 0, 30)

    frame[LEFT_SHOULDER_IDX] = safe_angle(BASE_FRAME[LEFT_SHOULDER_IDX] + left_shoulder_delta)
    frame[LEFT_ELBOW_IDX] = safe_angle(BASE_FRAME[LEFT_ELBOW_IDX] - left_elbow_delta)

    frame[RIGHT_SHOULDER_IDX] = safe_angle(BASE_FRAME[RIGHT_SHOULDER_IDX] - right_shoulder_delta)
    frame[RIGHT_ELBOW_IDX] = safe_angle(BASE_FRAME[RIGHT_ELBOW_IDX] + right_elbow_delta)

    return frame

def callback(msg):
    global last_action_time, busy

    now = time.time()

    if busy:
        print("Controller busy. Skip current pose.")
        return

    if now - last_action_time < ACTION_INTERVAL:
        print("Update too frequent. Skip current pose.")
        return

    try:
        pose = json.loads(msg.data)
        print("Received pose:", pose)

        if not pose.get("visible", False):
            print("Pose not visible. Skip robot motion.")
            return

        if pose.get("confidence", 0.0) < 0.6:
            print("Low confidence. Skip robot motion.")
            return

        target_frame = generate_upper_body_frame(pose)

        print("Generated upper-body frame:", target_frame)
        print("Frame length:", len(target_frame))

        frames = [
            (BASE_FRAME, 600, 100),
            (target_frame, 900, 500),
            (BASE_FRAME, 900, 0)
        ]

        busy = True
        client_action.action(frames)
        last_action_time = time.time()
        busy = False

    except Exception as err:
        busy = False
        serror(err)

def main():
    node_initial()
    print("Week4 upper body controller started.")
    print("Subscribing topic: /upper_body_pose_angles")
    rospy.Subscriber("/upper_body_pose_angles", String, callback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
