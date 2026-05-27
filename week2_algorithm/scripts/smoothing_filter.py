# -*- coding: utf-8 -*-

def smooth_angle(prev_angle, new_angle, alpha=0.3):
    """
    指数滑动平均滤波。

    prev_angle: 上一帧平滑后的角度
    new_angle: 当前帧原始角度
    alpha: 平滑系数，越大越跟手，越小越平稳

    返回：平滑后的角度
    """
    if prev_angle is None:
        return new_angle

    if new_angle is None:
        return prev_angle

    return alpha * new_angle + (1.0 - alpha) * prev_angle


class AngleSmoother(object):
    """
    用于同时平滑多个角度。
    """
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.prev_values = {}

    def update(self, name, new_angle):
        prev_angle = self.prev_values.get(name, None)
        smoothed = smooth_angle(prev_angle, new_angle, self.alpha)
        self.prev_values[name] = smoothed
        return smoothed


if __name__ == "__main__":
    # 简单自测：模拟一组抖动角度
    raw_angles = [50, 55, 52, 70, 54, 53, 52]

    smoother = AngleSmoother(alpha=0.3)

    print("frame, raw_angle, smoothed_angle")
    for i, angle in enumerate(raw_angles):
        smoothed = smoother.update("right_elbow_angle", angle)
        print("%d, %.2f, %.2f" % (i, angle, smoothed))
