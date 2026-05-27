# Roban Upper Body Motion Control

Human pose retargeting based upper-body motion control for Roban humanoid robot.

## Team Division

- Yang Kaichen: robot-side mapping, safety filtering, 22-joint action frame generation and real robot testing.
- Zhang Yixuan: pose detection, keypoint extraction, right arm angle calculation, confidence filtering and smoothing.

## Main Interface

Pose module output:

{
    "visible": true,
    "confidence": 0.99,
    "right_arm_raise_angle": 62.7,
    "right_elbow_angle": 52.6
}

Robot module output:

- Roban 22-dimensional action frame
- Executed through client_action.action()
