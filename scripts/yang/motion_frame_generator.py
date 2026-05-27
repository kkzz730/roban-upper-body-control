# -*- coding: utf-8 -*-

from angle_mapper import map_right_arm_angles
from safety_filter import safe_joint_angle

BASE_FRAME = [0,0,0,0,0,0,0,0,0,0,0,0,0,-61,-18,0,61,18,0,0,0,0]

RIGHT_SHOULDER_IDX = 16
RIGHT_ELBOW_IDX = 17


def generate_right_arm_frame(right_arm_raise_angle, right_elbow_angle):
    mapped = map_right_arm_angles(right_arm_raise_angle, right_elbow_angle)

    frame = BASE_FRAME[:]

    frame[RIGHT_SHOULDER_IDX] = safe_joint_angle(
        frame[RIGHT_SHOULDER_IDX] - mapped["shoulder_delta"],
        -80, 80
    )

    frame[RIGHT_ELBOW_IDX] = safe_joint_angle(
        frame[RIGHT_ELBOW_IDX] + mapped["elbow_delta"],
        -80, 80
    )

    return frame


if __name__ == "__main__":
    frame = generate_right_arm_frame(62.7, 52.6)
    print(frame)
    print("length:", len(frame))
