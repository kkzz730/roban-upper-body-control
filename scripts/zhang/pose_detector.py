# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp

from angle_calculator import angle_between


mp_pose = mp.solutions.pose


def _landmark_to_pixel(landmark, image_width, image_height):
    x = int(landmark.x * image_width)
    y = int(landmark.y * image_height)
    visibility = float(landmark.visibility)
    return [x, y, visibility]


def calculate_arm_raise_angle(shoulder, elbow):
    import math
    import numpy as np

    upper_arm = np.array(
        [elbow[0] - shoulder[0], elbow[1] - shoulder[1]],
        dtype=float
    )
    vertical_down = np.array([0.0, 1.0], dtype=float)

    denom = np.linalg.norm(upper_arm) * np.linalg.norm(vertical_down)
    if denom < 1e-6:
        return None

    cos_angle = np.dot(upper_arm, vertical_down) / denom
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


def _build_invalid_result(reason, confidence=0.0):
    return {
        "visible": False,
        "confidence": confidence,
        "left_shoulder": None,
        "left_elbow": None,
        "left_wrist": None,
        "right_shoulder": None,
        "right_elbow": None,
        "right_wrist": None,
        "left_arm_raise_angle": None,
        "left_elbow_angle": None,
        "right_arm_raise_angle": None,
        "right_elbow_angle": None,
        "reason": reason
    }


def _result_from_mediapipe(image_bgr, result, min_visibility=0.6):
    image_height, image_width = image_bgr.shape[:2]

    if not result.pose_landmarks:
        return _build_invalid_result("no pose landmarks", 0.0)

    landmarks = result.pose_landmarks.landmark

    left_shoulder = _landmark_to_pixel(
        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER],
        image_width,
        image_height
    )
    left_elbow = _landmark_to_pixel(
        landmarks[mp_pose.PoseLandmark.LEFT_ELBOW],
        image_width,
        image_height
    )
    left_wrist = _landmark_to_pixel(
        landmarks[mp_pose.PoseLandmark.LEFT_WRIST],
        image_width,
        image_height
    )

    right_shoulder = _landmark_to_pixel(
        landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER],
        image_width,
        image_height
    )
    right_elbow = _landmark_to_pixel(
        landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW],
        image_width,
        image_height
    )
    right_wrist = _landmark_to_pixel(
        landmarks[mp_pose.PoseLandmark.RIGHT_WRIST],
        image_width,
        image_height
    )

    confidence = min(
        left_shoulder[2],
        left_elbow[2],
        left_wrist[2],
        right_shoulder[2],
        right_elbow[2],
        right_wrist[2]
    )

    if confidence < min_visibility:
        return {
            "visible": False,
            "confidence": confidence,
            "left_shoulder": left_shoulder[:2],
            "left_elbow": left_elbow[:2],
            "left_wrist": left_wrist[:2],
            "right_shoulder": right_shoulder[:2],
            "right_elbow": right_elbow[:2],
            "right_wrist": right_wrist[:2],
            "left_arm_raise_angle": None,
            "left_elbow_angle": None,
            "right_arm_raise_angle": None,
            "right_elbow_angle": None,
            "reason": "low visibility"
        }

    left_elbow_angle = angle_between(
        left_shoulder[:2],
        left_elbow[:2],
        left_wrist[:2]
    )
    right_elbow_angle = angle_between(
        right_shoulder[:2],
        right_elbow[:2],
        right_wrist[:2]
    )

    left_arm_raise_angle = calculate_arm_raise_angle(
        left_shoulder[:2],
        left_elbow[:2]
    )
    right_arm_raise_angle = calculate_arm_raise_angle(
        right_shoulder[:2],
        right_elbow[:2]
    )

    return {
        "visible": True,
        "confidence": confidence,
        "left_shoulder": left_shoulder[:2],
        "left_elbow": left_elbow[:2],
        "left_wrist": left_wrist[:2],
        "right_shoulder": right_shoulder[:2],
        "right_elbow": right_elbow[:2],
        "right_wrist": right_wrist[:2],
        "left_arm_raise_angle": left_arm_raise_angle,
        "left_elbow_angle": left_elbow_angle,
        "right_arm_raise_angle": right_arm_raise_angle,
        "right_elbow_angle": right_elbow_angle,
        "reason": "ok"
    }


def detect_upper_body_angles(image_bgr, min_visibility=0.6):
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    with mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        enable_segmentation=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as pose:
        result = pose.process(image_rgb)

    return _result_from_mediapipe(image_bgr, result, min_visibility)


class UpperBodyPoseDetector(object):
    """
    Persistent MediaPipe Pose detector for video stream.
    Reusing this object reduces frame-to-frame jitter.
    """
    def __init__(self, min_detection_confidence=0.6, min_tracking_confidence=0.6):
        self.pose = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

    def detect(self, image_bgr, min_visibility=0.6):
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        result = self.pose.process(image_rgb)
        return _result_from_mediapipe(image_bgr, result, min_visibility)

    def close(self):
        self.pose.close()


def draw_pose_result_overlay(image_bgr, result):
    output = image_bgr.copy()

    if not result.get("visible", False):
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

    left_shoulder = tuple(result["left_shoulder"])
    left_elbow = tuple(result["left_elbow"])
    left_wrist = tuple(result["left_wrist"])

    right_shoulder = tuple(result["right_shoulder"])
    right_elbow = tuple(result["right_elbow"])
    right_wrist = tuple(result["right_wrist"])

    cv2.circle(output, left_shoulder, 6, (0, 255, 0), -1)
    cv2.circle(output, left_elbow, 6, (0, 255, 0), -1)
    cv2.circle(output, left_wrist, 6, (0, 255, 0), -1)
    cv2.line(output, left_shoulder, left_elbow, (255, 0, 0), 2)
    cv2.line(output, left_elbow, left_wrist, (255, 0, 0), 2)

    cv2.circle(output, right_shoulder, 6, (0, 255, 0), -1)
    cv2.circle(output, right_elbow, 6, (0, 255, 0), -1)
    cv2.circle(output, right_wrist, 6, (0, 255, 0), -1)
    cv2.line(output, right_shoulder, right_elbow, (255, 0, 0), 2)
    cv2.line(output, right_elbow, right_wrist, (255, 0, 0), 2)

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
        "L raise: %.1f  L elbow: %.1f" % (
            result["left_arm_raise_angle"],
            result["left_elbow_angle"]
        ),
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        output,
        "R raise: %.1f  R elbow: %.1f" % (
            result["right_arm_raise_angle"],
            result["right_elbow_angle"]
        ),
        (20, 115),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    return output
