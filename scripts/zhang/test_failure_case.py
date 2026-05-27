# -*- coding: utf-8 -*-

import os
import sys
import json
import cv2

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CURRENT_DIR)

from pose_detector import detect_upper_body_angles, draw_pose_result_overlay


IMAGE_PATH = "images/week3/input_realsense.jpg"
OUTPUT_PATH = "images/week3/failure_case.jpg"
JSON_PATH = "logs/week3/failure_case.json"
LOG_PATH = "logs/week3/failure_case_note.txt"


def main():
    image = cv2.imread(IMAGE_PATH)
    if image is None:
        print("Failed to read image: %s" % IMAGE_PATH)
        sys.exit(1)

    # 故意设置极高阈值，用来模拟“置信度过滤后不可用于控制”的情况
    result = detect_upper_body_angles(image, min_visibility=1.01)

    overlay = draw_pose_result_overlay(image, result)
    cv2.imwrite(OUTPUT_PATH, overlay)

    os.makedirs(os.path.dirname(JSON_PATH), exist_ok=True)

    with open(JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    note_lines = [
        "failure / low-confidence test",
        "method: set min_visibility=1.01 to verify confidence filtering logic",
        "visible: %s" % result["visible"],
        "confidence: %.6f" % result["confidence"],
        "reason: %s" % result["reason"],
        "right_arm_raise_angle: %s" % result["right_arm_raise_angle"],
        "right_elbow_angle: %s" % result["right_elbow_angle"],
        "",
        "conclusion:",
        "When visible=False, angle fields are None and should not be used for robot control."
    ]

    text = "\n".join(note_lines)
    print(text)

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write(text + "\n")


if __name__ == "__main__":
    main()
