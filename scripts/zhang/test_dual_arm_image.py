# -*- coding: utf-8 -*-

import os
import sys
import json
import cv2

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CURRENT_DIR)

from pose_detector import detect_upper_body_angles, draw_pose_result_overlay
from smoothing_filter import AngleSmoother


IMAGE_PATH = "images/week4/input_realsense.jpg"
OUTPUT_PATH = "images/week4/pose_result_overlay.jpg"
JSON_PATH = "logs/week4/dual_arm_pose_result.json"
LOG_PATH = "logs/week4/dual_arm_terminal_output.txt"


def main():
    image = cv2.imread(IMAGE_PATH)
    if image is None:
        print("Failed to read image: %s" % IMAGE_PATH)
        sys.exit(1)

    result = detect_upper_body_angles(image, min_visibility=0.6)

    smoother = AngleSmoother(alpha=0.3)
    smoothed = smoother.update_result(result)

    overlay = draw_pose_result_overlay(image, result)

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(JSON_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)

    cv2.imwrite(OUTPUT_PATH, overlay)

    with open(JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(smoothed, f, ensure_ascii=False, indent=2)

    output_text = json.dumps(smoothed, ensure_ascii=False, indent=2)
    print(output_text)

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write(output_text + "\n")
        f.write("overlay_path: %s\n" % OUTPUT_PATH)
        f.write("json_path: %s\n" % JSON_PATH)


if __name__ == "__main__":
    main()
