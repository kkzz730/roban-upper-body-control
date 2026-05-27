# -*- coding: utf-8 -*-

import os
import cv2

from pose_detector import detect_right_arm_keypoints, draw_right_arm_overlay
from angle_calculator import (
    calculate_right_elbow_angle,
    calculate_right_arm_raise_angle
)


IMAGE_PATH = os.path.expanduser(
    "~/roban_motion_control/week2/images/realsense_color_test.jpg"
)

OUTPUT_PATH = os.path.expanduser(
    "~/roban_motion_control/week2/images/pose_result_overlay.jpg"
)

LOG_PATH = os.path.expanduser(
    "~/roban_motion_control/week2/logs/pose_angle_output.txt"
)


def draw_angle_text(image, keypoints, right_elbow_angle, right_arm_raise_angle):
    output = image.copy()

    if right_elbow_angle is not None:
        cv2.putText(
            output,
            "right_elbow_angle: %.1f" % right_elbow_angle,
            (20, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2
        )

    if right_arm_raise_angle is not None:
        cv2.putText(
            output,
            "right_arm_raise_angle: %.1f" % right_arm_raise_angle,
            (20, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2
        )

    return output


def main():
    image = cv2.imread(IMAGE_PATH)

    if image is None:
        print("Failed to read image:", IMAGE_PATH)
        return

    keypoints = detect_right_arm_keypoints(image)

    right_elbow_angle = None
    right_arm_raise_angle = None

    if keypoints["visible"]:
        right_elbow_angle = calculate_right_elbow_angle(
            keypoints["right_shoulder"],
            keypoints["right_elbow"],
            keypoints["right_wrist"]
        )

        right_arm_raise_angle = calculate_right_arm_raise_angle(
            keypoints["right_shoulder"],
            keypoints["right_elbow"]
        )

    overlay = draw_right_arm_overlay(image, keypoints)
    overlay = draw_angle_text(
        overlay,
        keypoints,
        right_elbow_angle,
        right_arm_raise_angle
    )

    cv2.imwrite(OUTPUT_PATH, overlay)

    lines = [
        "image_path: %s" % IMAGE_PATH,
        "visible: %s" % keypoints["visible"],
        "reason: %s" % keypoints["reason"],
        "confidence: %.4f" % keypoints["confidence"],
        "right_shoulder: %s" % keypoints["right_shoulder"],
        "right_elbow: %s" % keypoints["right_elbow"],
        "right_wrist: %s" % keypoints["right_wrist"],
        "right_elbow_angle: %s" % (
            "%.4f" % right_elbow_angle if right_elbow_angle is not None else "None"
        ),
        "right_arm_raise_angle: %s" % (
            "%.4f" % right_arm_raise_angle if right_arm_raise_angle is not None else "None"
        ),
        "overlay_path: %s" % OUTPUT_PATH,
    ]

    for line in lines:
        print(line)

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
