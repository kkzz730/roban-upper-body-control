# Week 4 Stream-Based Upper Body Imitation Plan

## 1. Background

The original fourth-week pipeline used Roban to save the latest camera image as a JPG file. The Mac pulled this image through scp, ran MediaPipe pose recognition, generated upper-body angle JSON, and sent the JSON back to Roban.

This pipeline worked, but it had high latency and was not a direct video stream solution.

## 2. Updated Pipeline

The updated solution uses Roban's RealSense camera video stream directly:

Roban RealSense camera
-> web_video_server HTTP video stream
-> Mac OpenCV VideoCapture
-> MediaPipe Pose recognition
-> upper-body angle calculation
-> EMA smoothing
-> JSON result upload to Roban
-> json_pose_publisher.py publishes /upper_body_pose_angles
-> week4_upper_body_controller.py maps angles to a 22-dimensional Roban motion frame

## 3. Why MediaPipe Runs on Mac

Roban uses ROS Kinetic and Python2. Its Python2 environment supports rospy, cv_bridge, and OpenCV, but does not support MediaPipe. Its Python3 environment is also not suitable for directly running this ROS image processing pipeline.

Therefore, the Mac runs OpenCV and MediaPipe, while Roban keeps ROS topic publishing and robot action execution.

## 4. Main Scripts

- scripts/zhang/test_roban_video_stream.py: tests whether Mac can read Roban's HTTP video stream.
- scripts/zhang/mac_stream_pose_json_sender.py: reads Roban video stream, runs pose recognition, smooths angles, and sends JSON back to Roban.
- scripts/zhang/pose_detector.py: detects upper-body landmarks and calculates left/right arm angles.
- scripts/zhang/smoothing_filter.py: smooths angle outputs.
- scripts/zhang/json_pose_publisher.py: runs on Roban and publishes JSON to /upper_body_pose_angles.
- scripts/yang/week4_upper_body_controller.py: subscribes /upper_body_pose_angles, maps angles to a 22-dimensional Roban upper-body frame, and sends safe low-frequency robot actions.

## 5. Stability Improvements

The controller was improved to avoid random arm shaking:

- Confidence threshold is set to 0.85.
- MediaPipe Pose detector is reused across video frames.
- EMA smoothing alpha is reduced to 0.12.
- Elbow mapping uses elbow_bend = 180 - elbow_angle.
- Per-step angle change is limited by MAX_ANGLE_STEP = 15.0.
- The controller no longer sends BASE_FRAME before every action, avoiding repeated retract-expand motion.

## 6. Final Result

The final system can read Roban's video stream on Mac, recognize upper-body poses, generate stable left/right arm angles, send JSON back to Roban, publish /upper_body_pose_angles, and drive the robot upper body through a 22-dimensional frame.

The robot action is low-frequency and safety filtered. When pose is not visible or confidence is too low, the controller skips robot motion.
