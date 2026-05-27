# -*- coding: utf-8 -*-

import json
import os
from motion_frame_generator import generate_right_arm_frame

def main():
    json_path = os.path.join("test_data", "sample_pose_output.json")

    with open(json_path, "r") as f:
        pose = json.load(f)

    if not pose.get("visible", False):
        print("Pose not visible. Skip motion generation.")
        return

    if pose.get("confidence", 0.0) < 0.6:
        print("Low confidence. Skip motion generation.")
        return

    frame = generate_right_arm_frame(
        pose["right_arm_raise_angle"],
        pose["right_elbow_angle"]
    )

    print("Input pose:", pose)
    print("Generated Roban frame:", frame)
    print("Frame length:", len(frame))

if __name__ == "__main__":
    main()
