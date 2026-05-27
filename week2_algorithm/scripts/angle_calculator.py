# -*- coding: utf-8 -*-

import math
import numpy as np


def angle_between(a, b, c):
    """
    计算 ∠ABC，a/b/c 为二维点 [x, y]
    返回角度，单位：degree
    """
    ba = np.array([a[0] - b[0], a[1] - b[1]], dtype=float)
    bc = np.array([c[0] - b[0], c[1] - b[1]], dtype=float)

    denom = np.linalg.norm(ba) * np.linalg.norm(bc)
    if denom < 1e-6:
        return None

    cos_angle = np.dot(ba, bc) / denom
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    return math.degrees(math.acos(cos_angle))


def calculate_right_elbow_angle(right_shoulder, right_elbow, right_wrist):
    """
    右肘弯曲角：∠肩-肘-腕
    """
    return angle_between(right_shoulder, right_elbow, right_wrist)


def calculate_right_arm_raise_angle(right_shoulder, right_elbow):
    """
    右臂抬起角。
    简化定义：以上臂向量 shoulder->elbow 与图像竖直向下方向的夹角表示。
    图像坐标中 y 轴向下，所以竖直向下向量为 [0, 1]。
    """
    upper_arm = np.array(
        [right_elbow[0] - right_shoulder[0],
         right_elbow[1] - right_shoulder[1]],
        dtype=float
    )

    vertical_down = np.array([0.0, 1.0], dtype=float)

    denom = np.linalg.norm(upper_arm) * np.linalg.norm(vertical_down)
    if denom < 1e-6:
        return None

    cos_angle = np.dot(upper_arm, vertical_down) / denom
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    return math.degrees(math.acos(cos_angle))


if __name__ == "__main__":
    # 简单自测
    shoulder = [100, 100]
    elbow = [100, 200]
    wrist = [150, 250]

    elbow_angle = calculate_right_elbow_angle(shoulder, elbow, wrist)
    raise_angle = calculate_right_arm_raise_angle(shoulder, elbow)

    print("right_elbow_angle:", elbow_angle)
    print("right_arm_raise_angle:", raise_angle)
