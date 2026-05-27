# -*- coding: utf-8 -*-


def smooth_angle(prev_angle, new_angle, alpha=0.3):
    """
    指数滑动平均滤波。

    prev_angle: 上一次平滑后的角度
    new_angle: 当前原始角度
    alpha: 平滑系数，越大越跟手，越小越平稳
    """
    if prev_angle is None:
        return new_angle

    if new_angle is None:
        return prev_angle

    return alpha * new_angle + (1.0 - alpha) * prev_angle


class AngleSmoother(object):
    """
    多角度平滑器，用于同时平滑 right_arm_raise_angle 和 right_elbow_angle。
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
        输入 detect_upper_body_angles() 的输出 dict。
        返回加入 smoothed_right_arm_raise_angle 和 smoothed_right_elbow_angle 的新 dict。
        """
        result = dict(pose_result)

        if not result.get("visible", False):
            result["smoothed_right_arm_raise_angle"] = None
            result["smoothed_right_elbow_angle"] = None
            return result

        result["smoothed_right_arm_raise_angle"] = self.update(
            "right_arm_raise_angle",
            result.get("right_arm_raise_angle")
        )

        result["smoothed_right_elbow_angle"] = self.update(
            "right_elbow_angle",
            result.get("right_elbow_angle")
        )

        return result


if __name__ == "__main__":
    # 模拟几帧角度，包含一次明显突变，用于验证平滑效果
    raw_raise_angles = [60.0, 62.0, 61.0, 78.0, 63.0, 62.5, 62.0]
    raw_elbow_angles = [52.0, 54.0, 53.0, 70.0, 55.0, 53.5, 52.8]

    smoother = AngleSmoother(alpha=0.3)

    print("frame, raw_raise, smooth_raise, raw_elbow, smooth_elbow")

    for i, (raise_angle, elbow_angle) in enumerate(
        zip(raw_raise_angles, raw_elbow_angles)
    ):
        pose_result = {
            "visible": True,
            "right_arm_raise_angle": raise_angle,
            "right_elbow_angle": elbow_angle
        }

        smoothed = smoother.update_result(pose_result)

        print(
            "%d, %.2f, %.2f, %.2f, %.2f" % (
                i,
                raise_angle,
                smoothed["smoothed_right_arm_raise_angle"],
                elbow_angle,
                smoothed["smoothed_right_elbow_angle"]
            )
        )
