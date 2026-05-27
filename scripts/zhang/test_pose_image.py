# -*- coding: utf-8 -*-

import os
import sys
import json
import argparse
import cv2

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CURRENT_DIR)

from pose_detector import detect_upper_body_angles, draw_pose_result_overlay


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image",
        default="images/week3/input_realsense.jpg",
        help="input image path"
    )
    parser.add_argument(
        "--output",
        default="images/week3/pose_result_overlay.jpg",
        help="output overlay image path"
    )
    parser.add_argument(
        "--json",
        default="logs/week3/pose_result.json",
        help="output json path"
    )
    parser.add_argument(
        "--log",
        default="logs/week3/pose_terminal_output.txt",
        help="terminal output log path"
    )
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        print("Failed to read image: %s" % args.image)
        sys.exit(1)

    result = detect_upper_body_angles(image)
    overlay = draw_pose_result_overlay(image, result)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    os.makedirs(os.path.dirname(args.json), exist_ok=True)
    os.makedirs(os.path.dirname(args.log), exist_ok=True)

    cv2.imwrite(args.output, overlay)

    with open(args.json, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    output_text = json.dumps(result, ensure_ascii=False, indent=2)
    print(output_text)

    with open(args.log, "w", encoding="utf-8") as f:
        f.write(output_text + "\n")
        f.write("overlay_path: %s\n" % args.output)
        f.write("json_path: %s\n" % args.json)


if __name__ == "__main__":
    main()
