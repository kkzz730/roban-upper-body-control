# -*- coding: utf-8 -*-

def map_range(x, in_min, in_max, out_min, out_max):
    """Map a human angle value to a Roban joint angle range."""
    x = max(min(x, in_max), in_min)
    return out_min + (x - in_min) * (out_max - out_min) / float(in_max - in_min)


def map_right_arm_angles(right_arm_raise_angle, right_elbow_angle):
    """
    Conservative mapping for week 3.
    """
    shoulder_delta = map_range(right_arm_raise_angle, 0, 120, 0, 35)
    elbow_delta = map_range(right_elbow_angle, 30, 170, 0, 30)

    return {
        "shoulder_delta": shoulder_delta,
        "elbow_delta": elbow_delta
    }


if __name__ == "__main__":
    print(map_right_arm_angles(62.7, 52.6))
