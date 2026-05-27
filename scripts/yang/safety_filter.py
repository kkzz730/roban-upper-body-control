# -*- coding: utf-8 -*-

def clamp(x, low, high):
    return max(low, min(high, x))


def limit_delta(current, target, max_delta=5.0):
    """Limit angle change per control step."""
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


def safe_joint_angle(angle, low=-80.0, high=80.0):
    return clamp(angle, low, high)


if __name__ == "__main__":
    print("clamp:", clamp(90, -40, 40))
    print("limit_delta:", limit_delta(0, 30, 5))
