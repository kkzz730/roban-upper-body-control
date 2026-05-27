# Week 1 Notes

## Goal
Verify Roban camera input and action execution interface.

## Camera Input
The stable camera topic was confirmed as:

- /camera/color/image_raw
- Type: sensor_msgs/Image
- Publisher: /camera/realsense2_camera_manager
- Average frequency: about 29.9 Hz

The script `save_one_image.py` subscribes to the ROS image topic, converts it to OpenCV BGR format using cv_bridge, and saves one RealSense color image.

## Motion Interface
The existing Roban action interface was checked through:

- client_action.action()
- client_action.custom_action()

The file `week1_custom_shake.py` was created from the original shake-head action script. It modifies the motion amplitude conservatively and verifies that a customized action frame can be executed by Roban.

## Note
Robot IP, screenshots, and raw logs are not stored in this repository.
