# -*- coding: utf-8 -*-


def smooth_angle(prev_angle, new_angle, alpha=0.3):
    """
    Exponential moving average for angle smoothing.
    """
    if prev_angle is None:
        return new_angle

    if new_angle is None:
        return prev_angle

    return alpha * new_angle + (1.0 - alpha) * prev_angle


class AngleSmoother(object):
    """
    Smooth multiple upper-body angle fields.
    """
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.prev_values = {}

    def update(self, name, new_angle):
        prev_angle = self.prev_values.get(name, None)
        smoothed = smooth_angle(prev_angle, new_angle, self.alpha)
        self.prev_values[name] = smoothed
        return smoothed

    def update_result(self, pose_result):
        """
        Input: result dict from detect_upper_body_angles().
        Output: result dict with smoothed angle fields.

        If visible=False, smoothed angle fields are None.
        """
        result = dict(pose_result)

        angle_fields = [
            "left_arm_raise_angle",
            "left_elbow_angle",
            "right_arm_raise_angle",
            "right_elbow_angle"
        ]

        if not result.get("visible", False):
            for field in angle_fields:
                result["smoothed_" + field] = None
            return result

        for field in angle_fields:
            result["smoothed_" + field] = self.update(
                field,
                result.get(field)
            )

        return result


if __name__ == "__main__":
    # Simulated angle sequence with sudden changes.
    raw_frames = [
        {
            "left_arm_raise_angle": 60.0,
            "left_elbow_angle": 52.0,
            "right_arm_raise_angle": 61.0,
            "right_elbow_angle": 53.0
        },
        {
            "left_arm_raise_angle": 62.0,
            "left_elbow_angle": 54.0,
            "right_arm_raise_angle": 62.0,
            "right_elbow_angle": 54.0
        },
        {
            "left_arm_raise_angle": 61.0,
            "left_elbow_angle": 53.0,
            "right_arm_raise_angle": 61.5,
            "right_elbow_angle": 53.5
        },
        {
            "left_arm_raise_angle": 78.0,
            "left_elbow_angle": 70.0,
            "right_arm_raise_angle": 79.0,
            "right_elbow_angle": 71.0
        },
        {
            "left_arm_raise_angle": 63.0,
            "left_elbow_angle": 55.0,
            "right_arm_raise_angle": 63.5,
            "right_elbow_angle": 55.5
        },
    ]

    smoother = AngleSmoother(alpha=0.3)

    print(
        "frame, "
        "raw_L_raise, smooth_L_raise, "
        "raw_L_elbow, smooth_L_elbow, "
        "raw_R_raise, smooth_R_raise, "
        "raw_R_elbow, smooth_R_elbow"
    )

    for i, frame in enumerate(raw_frames):
        pose_result = {"visible": True}
        pose_result.update(frame)

        smoothed = smoother.update_result(pose_result)

        print(
            "%d, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f" % (
                i,
                frame["left_arm_raise_angle"],
                smoothed["smoothed_left_arm_raise_angle"],
                frame["left_elbow_angle"],
                smoothed["smoothed_left_elbow_angle"],
                frame["right_arm_raise_angle"],
                smoothed["smoothed_right_arm_raise_angle"],
                frame["right_elbow_angle"],
                smoothed["smoothed_right_elbow_angle"]
            )
        )
