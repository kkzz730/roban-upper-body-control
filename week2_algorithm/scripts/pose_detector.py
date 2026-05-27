# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp


mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils


def _landmark_to_pixel(landmark, image_width, image_height):
    """
    MediaPipe 归一化坐标 -> 图像像素坐标
    """
    x = int(landmark.x * image_width)
    y = int(landmark.y * image_height)
    visibility = float(landmark.visibility)
    return [x, y, visibility]


def detect_right_arm_keypoints(image_bgr, min_visibility=0.5):
    """
    输入：OpenCV BGR 图像
    输出：右肩、右肘、右腕关键点坐标

    返回格式：
    {
        "visible": True/False,
        "right_shoulder": [x, y],
        "right_elbow": [x, y],
        "right_wrist": [x, y],
        "confidence": 0.86,
        "reason": "ok"
    }
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
            "right_shoulder": None,
            "right_elbow": None,
            "right_wrist": None,
            "confidence": 0.0,
            "reason": "no pose landmarks"
        }

    landmarks = result.pose_landmarks.landmark

    right_shoulder_lm = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    right_elbow_lm = landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW]
    right_wrist_lm = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]

    right_shoulder = _landmark_to_pixel(right_shoulder_lm, image_width, image_height)
    right_elbow = _landmark_to_pixel(right_elbow_lm, image_width, image_height)
    right_wrist = _landmark_to_pixel(right_wrist_lm, image_width, image_height)

    confidence = min(
        right_shoulder[2],
        right_elbow[2],
        right_wrist[2]
    )

    visible = confidence >= min_visibility

    return {
        "visible": visible,
        "right_shoulder": right_shoulder[:2],
        "right_elbow": right_elbow[:2],
        "right_wrist": right_wrist[:2],
        "confidence": confidence,
        "reason": "ok" if visible else "low visibility"
    }


def draw_right_arm_overlay(image_bgr, keypoints):
    """
    在图像上画右肩、右肘、右腕和连线
    """
    output = image_bgr.copy()

    if not keypoints["visible"]:
        cv2.putText(
            output,
            "Right arm not visible: %s" % keypoints["reason"],
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )
        return output

    shoulder = tuple(keypoints["right_shoulder"])
    elbow = tuple(keypoints["right_elbow"])
    wrist = tuple(keypoints["right_wrist"])

    cv2.circle(output, shoulder, 6, (0, 255, 0), -1)
    cv2.circle(output, elbow, 6, (0, 255, 0), -1)
    cv2.circle(output, wrist, 6, (0, 255, 0), -1)

    cv2.line(output, shoulder, elbow, (255, 0, 0), 2)
    cv2.line(output, elbow, wrist, (255, 0, 0), 2)

    cv2.putText(
        output,
        "conf: %.2f" % keypoints["confidence"],
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv2.putText(
        output,
        "S%s E%s W%s" % (
            keypoints["right_shoulder"],
            keypoints["right_elbow"],
            keypoints["right_wrist"]
        ),
        (20, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2
    )

    return output
