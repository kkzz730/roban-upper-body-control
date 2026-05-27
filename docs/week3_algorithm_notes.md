# Week 3 Algorithm Notes - Zhang Yixuan

## 1. Task Goal

This week focuses on the algorithm-side interface for the Roban upper-body motion control project. The goal is to package the previous single-image pose recognition workflow into a reusable function interface.

The algorithm module does not directly control the Roban robot. It receives an OpenCV BGR image and outputs right-arm keypoints, confidence, and two angle values.

## 2. Input and Output

Input:

- image_bgr: OpenCV BGR image
- Source can be images/week3/input_realsense.jpg or future ROS image frames converted by cv_bridge.

Valid output example:

- visible: True
- confidence: 0.9996
- right_shoulder: [302, 304]
- right_elbow: [203, 355]
- right_wrist: [226, 227]
- right_arm_raise_angle: 62.7447
- right_elbow_angle: 52.5580
- reason: ok

Invalid or low-confidence output example:

- visible: False
- right_arm_raise_angle: None
- right_elbow_angle: None
- reason: low visibility

When visible is False, the angle fields should not be used for robot control.

## 3. Implemented Files

- scripts/zhang/angle_calculator.py: computes right elbow angle and right arm raise angle.
- scripts/zhang/pose_detector.py: provides detect_upper_body_angles(image_bgr).
- scripts/zhang/test_pose_image.py: tests the pose interface on a saved Roban camera image.
- scripts/zhang/smoothing_filter.py: applies exponential moving average smoothing.
- scripts/zhang/test_failure_case.py: verifies low-confidence filtering behavior.

## 4. Experiment Result

Valid pose result:

- visible: True
- confidence: 0.9995885491371155
- right_shoulder: [302, 304]
- right_elbow: [203, 355]
- right_wrist: [226, 227]
- right_arm_raise_angle: 62.74467162505693
- right_elbow_angle: 52.558041865112145

## 5. Smoothing Result

A simulated sudden angle jump was smoothed by the EMA filter. For example, when raw values jumped to 78.00 and 70.00, the smoothed values only changed to 65.90 and 57.90.

## 6. Failure Case Handling

A low-confidence filtering test was added by setting min_visibility=1.01. The result was:

- visible: False
- reason: low visibility
- right_arm_raise_angle: None
- right_elbow_angle: None

This verifies that invalid pose results will not be passed to the robot control module.

## 7. Interface Agreement

Zhang Yixuan's algorithm-side module outputs pose angles and confidence only. Yang Kaichen's robot-side module will handle angle mapping, safety limits, 22-dimensional action frames, and real robot execution.
