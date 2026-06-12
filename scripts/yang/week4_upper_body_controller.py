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

# Safety filter parameters
MIN_CONFIDENCE = 0.85

# Limit maximum angle change per accepted control step.
MAX_ANGLE_STEP = 15.0

# Execute after one valid frame. Smoothness is handled by step limiting.
REQUIRED_STABLE_FRAMES = 1

# Last accepted pose for step limiting.
last_valid_pose = None
stable_frame_count = 0

last_action_time = 0.0
busy = False

def map_range(x, in_min, in_max, out_min, out_max):
    x = max(min(x, in_max), in_min)
    return out_min + (x - in_min) * (out_max - out_min) / float(in_max - in_min)

def safe_angle(x, low=-80.0, high=80.0):
    return max(low, min(high, x))





def get_pose_angles(pose):
    return [
        pose.get("left_arm_raise_angle", None),
        pose.get("left_elbow_angle", None),
        pose.get("right_arm_raise_angle", None),
        pose.get("right_elbow_angle", None)
    ]


def has_none_angle(pose):
    angles = get_pose_angles(pose)
    return any(angle is None for angle in angles)


def limit_angle_step(prev_pose, current_pose):
    if prev_pose is None:
        return current_pose

    limited_pose = dict(current_pose)

    angle_fields = [
        "left_arm_raise_angle",
        "left_elbow_angle",
        "right_arm_raise_angle",
        "right_elbow_angle"
    ]

    for field in angle_fields:
        prev_value = prev_pose.get(field, None)
        curr_value = current_pose.get(field, None)

        if prev_value is None or curr_value is None:
            continue

        diff = curr_value - prev_value

        if diff > MAX_ANGLE_STEP:
            limited_pose[field] = prev_value + MAX_ANGLE_STEP
            print(
                "Limit angle step: %s prev=%.2f curr=%.2f limited=%.2f"
                % (field, prev_value, curr_value, limited_pose[field])
            )
        elif diff < -MAX_ANGLE_STEP:
            limited_pose[field] = prev_value - MAX_ANGLE_STEP
            print(
                "Limit angle step: %s prev=%.2f curr=%.2f limited=%.2f"
                % (field, prev_value, curr_value, limited_pose[field])
            )

    return limited_pose


def generate_upper_body_frame(pose):
    frame = BASE_FRAME[:]

    left_raise = pose.get("left_arm_raise_angle", 0.0)
    left_elbow = pose.get("left_elbow_angle", 180.0)

    right_raise = pose.get("right_arm_raise_angle", 0.0)
    right_elbow = pose.get("right_elbow_angle", 180.0)

    # MediaPipe elbow angle: close to 180 means straight arm.
    # Convert it to elbow bending amount.
    left_elbow_bend = 180.0 - left_elbow
    right_elbow_bend = 180.0 - right_elbow

    # Conservative mapping for safety.
    # Arm raise: 0~100 deg human angle -> 0~25 deg robot shoulder delta.
    # Elbow bend: 0~100 deg human bend -> 0~25 deg robot elbow delta.
    left_shoulder_delta = map_range(left_raise, 0, 100, 0, 25)
    right_shoulder_delta = map_range(right_raise, 0, 100, 0, 25)

    left_elbow_delta = map_range(left_elbow_bend, 0, 100, 0, 25)
    right_elbow_delta = map_range(right_elbow_bend, 0, 100, 0, 25)

    frame[LEFT_SHOULDER_IDX] = safe_angle(
        BASE_FRAME[LEFT_SHOULDER_IDX] + left_shoulder_delta,
        -80.0,
        80.0
    )
    frame[LEFT_ELBOW_IDX] = safe_angle(
        BASE_FRAME[LEFT_ELBOW_IDX] - left_elbow_delta,
        -80.0,
        80.0
    )

    frame[RIGHT_SHOULDER_IDX] = safe_angle(
        BASE_FRAME[RIGHT_SHOULDER_IDX] - right_shoulder_delta,
        -80.0,
        80.0
    )
    frame[RIGHT_ELBOW_IDX] = safe_angle(
        BASE_FRAME[RIGHT_ELBOW_IDX] + right_elbow_delta,
        -80.0,
        80.0
    )

    print(
        "Mapping debug: "
        "L_raise=%.2f L_elbow=%.2f L_bend=%.2f "
        "R_raise=%.2f R_elbow=%.2f R_bend=%.2f"
        % (
            left_raise,
            left_elbow,
            left_elbow_bend,
            right_raise,
            right_elbow,
            right_elbow_bend
        )
    )

    print(
        "Delta debug: "
        "L_shoulder_delta=%.2f L_elbow_delta=%.2f "
        "R_shoulder_delta=%.2f R_elbow_delta=%.2f"
        % (
            left_shoulder_delta,
            left_elbow_delta,
            right_shoulder_delta,
            right_elbow_delta
        )
    )

    return frame

def callback(msg):
    global last_action_time, busy, last_valid_pose, stable_frame_count

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
            stable_frame_count = 0
            return

        if pose.get("confidence", 0.0) < MIN_CONFIDENCE:
            print("Low confidence. Skip robot motion.")
            stable_frame_count = 0
            return

        if has_none_angle(pose):
            print("Angle field is None. Skip robot motion.")
            stable_frame_count = 0
            return

        pose_for_control = limit_angle_step(last_valid_pose, pose)

        stable_frame_count += 1
        print("Stable frame count:", stable_frame_count)

        if stable_frame_count < REQUIRED_STABLE_FRAMES:
            print("Waiting for more stable frames. Skip current pose.")
            last_valid_pose = pose_for_control
            return

        last_valid_pose = pose_for_control

        target_frame = generate_upper_body_frame(pose_for_control)

        print("Generated upper-body frame:", target_frame)
        print("Frame length:", len(target_frame))

        frames = [
            (BASE_FRAME, 800, 1200),
            (target_frame, 1200, 2500),
            (BASE_FRAME, 1200, 1500),
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
