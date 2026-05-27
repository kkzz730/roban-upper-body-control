# -*- coding: utf-8 -*-

import os
import time
import json
import subprocess
import cv2

from pose_detector import detect_upper_body_angles, draw_pose_result_overlay
from smoothing_filter import AngleSmoother


ROBOT_HOST = "lemon@10.129.44.37"

ROBOT_IMAGE_PATH = "/home/lemon/roban_motion_control/week4/images/latest_realsense.jpg"
LOCAL_IMAGE_PATH = "images/week4/latest_realsense_from_robot.jpg"

LOCAL_OVERLAY_PATH = "images/week4/pose_result_overlay.jpg"
LOCAL_JSON_PATH = "logs/week4/upper_body_pose_angles.json"
LOCAL_LOG_PATH = "logs/week4/mac_pose_json_sender_log.txt"

ROBOT_JSON_PATH = "/home/lemon/roban_motion_control/week4/logs/upper_body_pose_angles.json"

LOOP_HZ = 1.0
MIN_VISIBILITY = 0.6


def run_cmd(cmd):
    return subprocess.call(cmd, shell=True)


def ensure_dirs():
    os.makedirs(os.path.dirname(LOCAL_IMAGE_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(LOCAL_OVERLAY_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(LOCAL_JSON_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(LOCAL_LOG_PATH), exist_ok=True)


def pull_image_from_robot():
    cmd = "scp %s:%s %s" % (
        ROBOT_HOST,
        ROBOT_IMAGE_PATH,
        LOCAL_IMAGE_PATH
    )
    return run_cmd(cmd) == 0


def push_json_to_robot():
    cmd = "scp %s %s:%s" % (
        LOCAL_JSON_PATH,
        ROBOT_HOST,
        ROBOT_JSON_PATH
    )
    return run_cmd(cmd) == 0


def build_publish_json(smoothed):
    if not smoothed.get("visible", False):
        return {
            "visible": False,
            "confidence": float(smoothed.get("confidence", 0.0)),
            "left_arm_raise_angle": None,
            "left_elbow_angle": None,
            "right_arm_raise_angle": None,
            "right_elbow_angle": None,
            "reason": smoothed.get("reason", "invalid pose")
        }

    return {
        "visible": True,
        "confidence": float(smoothed.get("confidence", 0.0)),
        "left_arm_raise_angle": smoothed.get("smoothed_left_arm_raise_angle"),
        "left_elbow_angle": smoothed.get("smoothed_left_elbow_angle"),
        "right_arm_raise_angle": smoothed.get("smoothed_right_arm_raise_angle"),
        "right_elbow_angle": smoothed.get("smoothed_right_elbow_angle"),
        "reason": smoothed.get("reason", "ok")
    }


def main():
    ensure_dirs()
    smoother = AngleSmoother(alpha=0.3)

    print("Mac pose JSON sender started.")
    print("Pull image from:", ROBOT_HOST + ":" + ROBOT_IMAGE_PATH)
    print("Push JSON to:", ROBOT_HOST + ":" + ROBOT_JSON_PATH)
    print("Loop frequency: %.1f Hz" % LOOP_HZ)

    with open(LOCAL_LOG_PATH, "a", encoding="utf-8") as log_file:
        while True:
            start = time.time()

            ok = pull_image_from_robot()
            if not ok:
                print("Failed to pull image from robot.")
                time.sleep(1.0 / LOOP_HZ)
                continue

            image = cv2.imread(LOCAL_IMAGE_PATH)
            if image is None:
                print("Failed to read local image:", LOCAL_IMAGE_PATH)
                time.sleep(1.0 / LOOP_HZ)
                continue

            raw_result = detect_upper_body_angles(
                image,
                min_visibility=MIN_VISIBILITY
            )
            smoothed = smoother.update_result(raw_result)
            publish_json = build_publish_json(smoothed)

            overlay = draw_pose_result_overlay(image, raw_result)
            cv2.imwrite(LOCAL_OVERLAY_PATH, overlay)

            with open(LOCAL_JSON_PATH, "w", encoding="utf-8") as f:
                json.dump(publish_json, f, ensure_ascii=False, indent=2)

            push_ok = push_json_to_robot()

            line = json.dumps(publish_json, ensure_ascii=False)
            print(line)
            log_file.write(line + "\n")
            log_file.flush()

            if not push_ok:
                print("Warning: failed to push JSON to robot.")

            elapsed = time.time() - start
            sleep_time = max(0.0, 1.0 / LOOP_HZ - elapsed)
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
