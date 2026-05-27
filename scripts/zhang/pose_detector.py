# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp

from angle_calculator import (
    calculate_right_elbow_angle,
    calculate_right_arm_raise_angle
)


mp_pose = mp.solutions.pose


def _landmark_to_pixel(landmark, image_width, image_height):
    """
    MediaPipe normalized landmark -> pixel coordinate.
    """
    x = int(landmark.x * image_width)
    y = int(landmark.y * image_height)
    visibility = float(landmark.visibility)
    return [x, y, visibility]


def detect_upper_body_angles(image_bgr, min_visibility=0.5):
    """
    输入：OpenCV BGR 图像
    输出：右臂关键点与角度信息 dict

    当检测失败或置信度过低时：
    visible=False，角度字段为 None，避免后续机器人控制误用低质量数据。
    """
    image_height, image_width = image_bgr.shape[:2]

    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    with mp_pose.Pose(
        static_image_mode=True,
        model_complexity=1,
        enable_segmentation=False,
        min_detection_confidence=0.5
    ) as pose:
        result = pose.process(image_rgb)

    if not result.pose_landmarks:
        return {
            "visible": False,
            "confidence": 0.0,
            "right_shoulder": None,
            "right_elbow": None,
            "right_wrist": None,
            "right_arm_raise_angle": None,
            "right_elbow_angle": None,
            "reason": "no pose landmarks"
        }

    landmarks = result.pose_landmarks.landmark

    right_shoulder_lm = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    right_elbow_lm = landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW]
    right_wrist_lm = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]

    right_shoulder = _landmark_to_pixel(
        right_shoulder_lm, image_width, image_height
    )
    right_elbow = _landmark_to_pixel(
        right_elbow_lm, image_width, image_height
    )
    right_wrist = _landmark_to_pixel(
        right_wrist_lm, image_width, image_height
    )

    confidence = min(
        right_shoulder[2],
        right_elbow[2],
        right_wrist[2]
    )

    visible = confidence >= min_visibility

    if not visible:
        return {
            "visible": False,
            "confidence": confidence,
            "right_shoulder": right_shoulder[:2],
            "right_elbow": right_elbow[:2],
            "right_wrist": right_wrist[:2],
            "right_arm_raise_angle": None,
            "right_elbow_angle": None,
            "reason": "low visibility"
        }

    right_elbow_angle = calculate_right_elbow_angle(
        right_shoulder[:2],
        right_elbow[:2],
        right_wrist[:2]
    )

    right_arm_raise_angle = calculate_right_arm_raise_angle(
        right_shoulder[:2],
        right_elbow[:2]
    )

    return {
        "visible": True,
        "confidence": confidence,
        "right_shoulder": right_shoulder[:2],
        "right_elbow": right_elbow[:2],
        "right_wrist": right_wrist[:2],
        "right_arm_raise_angle": right_arm_raise_angle,
        "right_elbow_angle": right_elbow_angle,
        "reason": "ok"
    }


def draw_pose_result_overlay(image_bgr, result):
    """
    绘制右肩、右肘、右腕关键点、连线和角度文本。
    """
    output = image_bgr.copy()

    if not result["visible"]:
        cv2.putText(
            output,
            "pose invalid: %s, conf=%.3f" % (
                result.get("reason", "unknown"),
                result.get("confidence", 0.0)
            ),
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )
        return output

    shoulder = tuple(result["right_shoulder"])
    elbow = tuple(result["right_elbow"])
    wrist = tuple(result["right_wrist"])

    cv2.circle(output, shoulder, 6, (0, 255, 0), -1)
    cv2.circle(output, elbow, 6, (0, 255, 0), -1)
    cv2.circle(output, wrist, 6, (0, 255, 0), -1)

    cv2.line(output, shoulder, elbow, (255, 0, 0), 2)
    cv2.line(output, elbow, wrist, (255, 0, 0), 2)

    cv2.putText(
        output,
        "visible: %s conf: %.3f" % (
            result["visible"],
            result["confidence"]
        ),
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    cv2.putText(
        output,
        "elbow_angle: %.1f" % result["right_elbow_angle"],
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    cv2.putText(
        output,
        "raise_angle: %.1f" % result["right_arm_raise_angle"],
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    return output
